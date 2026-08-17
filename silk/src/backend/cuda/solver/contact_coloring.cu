#include "backend/cuda/solver/contact_coloring.cuh"

#include <cassert>
#include <cstdint>
#include <cub/cub.cuh>
#include <cuda/atomic>
#include <cuda/devices>
#include <cuda/std/array>
#include <cuda/std/bit>
#include <cuda/std/utility>
#include <cuda/warp>
#include <stdexcept>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

namespace {

constexpr int BLOCK_DIM = 128;
constexpr int ACTIVE_CONTACTS_PER_THREAD = 8;

// A bitset of colors. Index from right to left (LSB to MSB).
// if bits[i] is set, it means color i has occurred. color 0 is a special value
// indicating no color and is always set.
template <int N>
class ColorFlags {
  static constexpr uint32_t MAX_COLOR = 32u * N;

  // Chunk of bits indexed from right to left.
  // i.e. data_[0] = least significant chunk.
  ctd::array<uint32_t, N> data_;

 public:
  __device__ constexpr ColorFlags() {
    // offset 0 implies no color and should always be set.
    data_[0] = 1u;
    for (int i = 1; i < N; ++i) {
      data_[i] = 0u;
    }
  }

  __device__ void set(uint16_t color) {
    assert(color < MAX_COLOR);

    uint16_t chunk = color / 32u;
    uint16_t remain = color % 32u;

    data_[chunk] |= (1u << remain);
  }

  // return the minimal valid color that never occured. 0 means no color left.
  __device__ uint16_t first_valid_color() const {
    constexpr uint32_t FULL = 0xFFFFFFFF;

    uint16_t color = 0u;
#pragma unroll
    for (int i = N - 1; i >= 0; --i) {
      if (data_[i] != FULL) {
        color = i * 32u + ctd::countr_one(data_[i]);
      }
    }

    return color;
  }
};

template <int N>
__global__ void greedy_color(ctd::span<const FrictionContact> contacts,
                             ctd::span<const int> vertex_offsets,
                             ctd::span<const int> incident_contacts,
                             ctd::span<const int> active_contacts,
                             ctd::span<uint16_t> colors, bool* out_of_color) {
  if (*out_of_color) {
    return;
  }

  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  int active_begin = tid * ACTIVE_CONTACTS_PER_THREAD;
  if (active_begin >= active_contacts.size()) {
    return;
  }

  for (int i = 0; i < ACTIVE_CONTACTS_PER_THREAD; ++i) {
    int active_idx = active_begin + i;
    if (active_idx >= active_contacts.size()) {
      break;
    }

    int contact_id = active_contacts[active_idx];

    // loop over contacts incident to vertices of the current contact.
    // no need to skip itself since it's always 0 (no color).
    ColorFlags<N> flags;
    const FrictionContact& contact = contacts[contact_id];
#pragma unroll
    for (int slot = 0; slot < 4; ++slot) {
      if (contact.gamma(slot) == 0.0f) {
        continue;
      }
      int vertex = contact.state_offset(slot) / 3;
      int start = vertex_offsets[vertex];
      int end = vertex_offsets[vertex + 1];
      for (int j = start; j < end; ++j) {
        uint16_t adj_color = colors[incident_contacts[j]];
        flags.set(adj_color);
      }
    }

    uint16_t color = flags.first_valid_color();
    if (color == 0) {
      cu::atomic_ref<bool> a{*out_of_color};
      a = true;
    }

    colors[contact_id] = color;
  }
}

// Detect color conflict and add contact back to active list for recoloring.
__global__ void resolve_conflicts(ctd::span<const FrictionContact> contacts,
                                  ctd::span<const int> vertex_offsets,
                                  ctd::span<const int> incident_contacts,
                                  ctd::span<uint16_t> colors,
                                  ctd::span<const int> active_contacts,
                                  ctd::span<int> active_contacts_next,
                                  int* active_contacts_counter_next,
                                  const bool* out_of_color) {
  using WReduce = cub::WarpReduce<int>;
  using WScan = cub::WarpScan<int>;

  if (*out_of_color) {
    return;
  }

  __shared__ union {
    typename WReduce::TempStorage reduce[BLOCK_DIM / 32];
    typename WScan::TempStorage scan[BLOCK_DIM / 32];
  } warp_temp;

  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  int active_begin = tid * ACTIVE_CONTACTS_PER_THREAD;
  // A local vector of contact ids needs recoloring.
  int recolor_contact_ids[ACTIVE_CONTACTS_PER_THREAD];
  int local_recoloring_num = 0;

  for (int i = 0; i < ACTIVE_CONTACTS_PER_THREAD; ++i) {
    int active_idx = active_begin + i;
    if (active_idx >= active_contacts.size()) {
      break;
    }

    // Loop all adjacent contacts.
    int contact_id = active_contacts[active_idx];
    const FrictionContact& contact = contacts[contact_id];
    uint16_t color = colors[contact_id];
    bool need_recoloring = false;
#pragma unroll
    for (int slot = 0; slot < 4; ++slot) {
      if (contact.gamma(slot) == 0.0f) {
        continue;
      }
      int vertex = contact.state_offset(slot) / 3;
      int start = vertex_offsets[vertex];
      int end = vertex_offsets[vertex + 1];
      for (int j = start; j < end; ++j) {
        int adjacent_contact_id = incident_contacts[j];
        uint16_t adj_color = colors[incident_contacts[j]];
        // We choose to recolor contact with smaller index.
        if (contact_id < adjacent_contact_id && color == adj_color) {
          need_recoloring = true;
          break;
        }
      }
      if (need_recoloring) {
        break;
      }
    }

    // Reset the color so next round of greedy coloring don't get confused.
    if (need_recoloring) {
      colors[contact_id] = 0u;
      recolor_contact_ids[local_recoloring_num] = contact_id;
      ++local_recoloring_num;
    }
  }

  int lid = threadIdx.x % 32;  // lane id
  int wid = threadIdx.x / 32;  // warp id

  int warp_recoloring_num =
      WReduce(warp_temp.reduce[wid]).Sum(local_recoloring_num);
  if (warp_recoloring_num == 0) {
    return;
  }

  // starting offset for all thread in current warp.
  int active_contacts_offset = 0;
  if (lid == 0) {
    cu::atomic_ref<int> a{*active_contacts_counter_next};
    active_contacts_offset = a.fetch_add(warp_recoloring_num);
  }
  active_contacts_offset =
      cu::device::warp_shuffle_idx(active_contacts_offset, 0);

  int offset = 0;
  WScan(warp_temp.scan[wid]).ExclusiveSum(local_recoloring_num, offset);

  for (int i = 0; i < local_recoloring_num; ++i) {
    active_contacts_next[active_contacts_offset + offset + i] =
        recolor_contact_ids[i];
  }
}

__global__ void zero_int(int* value) {
  int tid = blockDim.x * blockIdx.x + threadIdx.x;
  if (tid == 0) {
    *value = 0;
  }
}

bool color_graph(ctd::span<const FrictionContact> contacts,
                 ctd::span<const int> vertex_offsets,
                 ctd::span<const int> incident_contacts,
                 ctd::span<uint16_t> colors, CudaRuntime rt) {
  assert(contacts.size() > 0);

  // init.
  int num = contacts.size();
  auto out_of_color = alloc<bool>(rt, 1, false);
  auto active_contacts_counter = alloc<int>(rt, 1, num);
  auto active_contacts_counter_next = alloc<int>(rt, 1, num);
  auto active_contacts = alloc<int>(rt, num);
  auto active_contacts_next = alloc<int>(rt, num);
  auto init_buffers = [contacts = active_contacts.data(),
                       contacts_next = active_contacts_next.data(),
                       colors] __device__(int i) {
    contacts[i] = i;
    contacts_next[i] = i;
    colors[i] = 0;
  };
  cub::DeviceFor::Bulk(num, init_buffers, rt.stream.get());

  // Loops till all contacts has the correct color.
  // 1. greedy color in parallel.
  // 2. resolve conflicts.
  int color_n = 1;
  while (scalar_load(active_contacts_counter.data(), rt) != 0) {
    for (int i = 0; i < 10; ++i) {
      int host_active_contacts_counter =
          scalar_load(active_contacts_counter.data(), rt);
      if (host_active_contacts_counter == 0) {
        break;
      }

      ctd::span<const int> active_contacts_span(active_contacts.data(),
                                                host_active_contacts_counter);
      int active_thread_num = div_round_up(host_active_contacts_counter,
                                           ACTIVE_CONTACTS_PER_THREAD);
      int grid_num = div_round_up(active_thread_num, BLOCK_DIM);

      if (color_n == 1) {
        greedy_color<1><<<grid_num, BLOCK_DIM, 0, rt.stream.get()>>>(
            contacts, vertex_offsets, incident_contacts, active_contacts_span,
            colors, out_of_color.data());
      } else if (color_n == 4) {
        greedy_color<4><<<grid_num, BLOCK_DIM, 0, rt.stream.get()>>>(
            contacts, vertex_offsets, incident_contacts, active_contacts_span,
            colors, out_of_color.data());
      } else if (color_n == 16) {
        greedy_color<16><<<grid_num, BLOCK_DIM, 0, rt.stream.get()>>>(
            contacts, vertex_offsets, incident_contacts, active_contacts_span,
            colors, out_of_color.data());
      } else {
        greedy_color<64><<<grid_num, BLOCK_DIM, 0, rt.stream.get()>>>(
            contacts, vertex_offsets, incident_contacts, active_contacts_span,
            colors, out_of_color.data());
      }

      zero_int<<<1, 1, 0, rt.stream.get()>>>(
          active_contacts_counter_next.data());
      grid_num = div_round_up(active_thread_num, BLOCK_DIM);
      resolve_conflicts<<<grid_num, BLOCK_DIM, 0, rt.stream.get()>>>(
          contacts, vertex_offsets, incident_contacts, colors,
          active_contacts_span, active_contacts_next,
          active_contacts_counter_next.data(), out_of_color.data());

      ctd::swap(active_contacts, active_contacts_next);
      ctd::swap(active_contacts_counter, active_contacts_counter_next);
    }

    // color_n is the number of 32-bit color words. Increase the palette and
    // restart, or give up after exhausting the 64-word palette.
    rt.stream.sync();
    if (scalar_load(out_of_color.data(), rt)) {
      if (color_n == 64) {
        return false;
      }

      // reset and restart;
      cub::DeviceFor::Bulk(num, init_buffers, rt.stream.get());
      scalar_write(active_contacts_counter.data(), num, rt);
      scalar_write(active_contacts_counter_next.data(), num, rt);
      scalar_write(out_of_color.data(), false, rt);
      color_n *= 4;
      continue;
    }
  }

  return true;
}

}  // namespace

std::pair<cu::device_buffer<int>, std::vector<int>> color_contacts(
    ctd::span<const FrictionContact> contacts,
    ctd::span<const int> vertex_offsets, ctd::span<const int> incident_contacts,
    CudaRuntime rt) {
  // color contacts.
  int num = contacts.size();
  auto unsorted_colors = alloc<uint16_t>(rt, num);
  bool success = color_graph(contacts, vertex_offsets, incident_contacts,
                             unsorted_colors, rt);
  if (!success) {
    throw std::runtime_error("Too many colors");
  }

  // radix sort color and contacts.
  auto sorted_colors = alloc<uint16_t>(rt, num);
  auto unsorted_contact_ids = alloc<int>(rt, num);
  auto sorted_contact_ids = alloc<int>(rt, num);

  cub::DeviceFor::Bulk(
      num,
      [data = unsorted_contact_ids.data()] __device__(int i) { data[i] = i; },
      rt.stream.get());

  size_t radix_temp_size;
  cub::DeviceRadixSort::SortPairs(
      nullptr, radix_temp_size, unsorted_colors.data(), sorted_colors.data(),
      unsorted_contact_ids.data(), sorted_contact_ids.data(), num, 0,
      8 * sizeof(uint16_t), rt.stream.get());
  auto radix_temp = alloc<char>(rt, radix_temp_size);

  cub::DeviceRadixSort::SortPairs(
      radix_temp.data(), radix_temp_size, unsorted_colors.data(),
      sorted_colors.data(), unsorted_contact_ids.data(),
      sorted_contact_ids.data(), num, 0, 8 * sizeof(uint16_t), rt.stream.get());

  // Fill contact ranges.
  uint16_t max_color = scalar_load(sorted_colors.data() + num - 1, rt);
  auto group_ranges = alloc<int>(rt, max_color + 1);

  auto fill_ranges =
      [ranges = ctd::span<int>{group_ranges},
       colors = ctd::span<uint16_t>{sorted_colors}] __device__(int i) {
        uint16_t color = colors[i];
        assert(color != 0);

        if (i == 0 || color != colors[i - 1]) {
          ranges[color - 1] = i;
        }

        if (i == colors.size() - 1) {
          ranges.back() = colors.size();
        }
      };
  cub::DeviceFor::Bulk(num, fill_ranges, rt.stream.get());
  rt.stream.sync();

  std::vector<int> host_ranges = vec_like_to_host<int>(group_ranges, rt);
  return {std::move(sorted_contact_ids), std::move(host_ranges)};
}

}  // namespace silk::cuda

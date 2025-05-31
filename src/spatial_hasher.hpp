#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <span>
#include <vector>

#include "common_types.hpp"

struct CellInfo {
  uint32_t time = 0;   // time stamp
  uint32_t start = 0;  // cell start
  uint32_t size = 0;   // cell size
  uint32_t fill = 0;   // the current empty slot
};

struct CellMinMax {
  using Vec3i32 = Eigen::Matrix<int32_t, 1, 3>;
  Vec3i32 min;
  Vec3i32 max;
};

template <typename T>
class SpatialHasher {
  std::vector<CellInfo> cell_infos_;
  std::vector<uint32_t> active_cell_hashes;
  uint32_t cell_primitives_count_ = 0;
  uint32_t time = 0;
  std::vector<T> cells_;

  inline uint32_t spatial_hash(int32_t i, int32_t j, int32_t k) const {
    assert(hash_map_size != 0);

    return static_cast<uint32_t>((73856093 * i) ^ (19349663 * j) ^
                                 (83492791 * k)) %
           hash_map_size;
  }

  inline CellMinMax get_cell_minmax(const Bbox& bbox) const {
    return {.min = (bbox.min / cell_size).array().floor().cast<int32_t>(),
            .max = (bbox.max / cell_size).array().floor().cast<int32_t>()};
  }

  void populate_cell_info(const Bbox& bbox) {
    assert(cell_size != 0);
    assert(!cell_infos_.empty());

    auto [cell_min, cell_max] = get_cell_minmax(bbox);

    for (int32_t i = cell_min[0]; i <= cell_max[0]; i++) {
      for (int32_t j = cell_min[1]; j <= cell_max[1]; j++) {
        for (int32_t k = cell_min[2]; k <= cell_max[2]; k++) {
          uint32_t h = spatial_hash(i, j, k);
          auto& info = cell_infos_[h];

          // if time stamp is not the current one, this cell info is outdated
          if (info.time != time) {
            info = {.time = time, .start = 0, .size = 0, .fill = 0};
            active_cell_hashes.push_back(h);
          }

          cell_infos_[h].size++;
          cell_primitives_count_++;
        }
      }
    }
  }

  void batch_populate_cell_info(std::span<const Bbox> bboxes) {
    assert(hash_map_size != 0);
    assert(!bboxes.empty());

    if (cell_infos_.size() != hash_map_size) {
      cell_infos_.resize(hash_map_size);
      std::ranges::fill(cell_infos_, CellInfo{0, 0, 0, 0});
    }
    active_cell_hashes.clear();
    cell_primitives_count_ = 0;

    // calculate cell primative count
    for (auto& bbox : bboxes) {
      populate_cell_info(bbox);
    }

    // calculate cell start
    uint32_t start = 0;
    // TODO: maybe scanning cell_infos_ directly is faster due to cache
    // locality, need profiling
    for (uint32_t h : active_cell_hashes) {
      auto& info = cell_infos_[h];
      info.start = start;
      info.fill = start;
      start += info.size;
    }
  }

  void cell_insert(const T& value, const Bbox& bbox) {
    assert(cell_size != 0);
    assert(!cells_.empty());
    assert(!cell_infos_.empty());

    auto [cell_min, cell_max] = get_cell_minmax(bbox);

    for (int32_t i = cell_min[0]; i <= cell_max[0]; i++) {
      for (int32_t j = cell_min[1]; j <= cell_max[1]; j++) {
        for (int32_t k = cell_min[2]; k <= cell_max[2]; k++) {
          uint32_t h = spatial_hash(i, j, k);
          auto& info = cell_infos_[h];
          cells_[info.fill] = value;
          info.fill++;
        }
      }
    }
  }

  void batch_cell_insert(std::span<const T> values,
                         std::span<const Bbox> bboxes) {
    assert(!values.empty());
    assert(values.size() == bboxes.size());
    assert(!cell_infos_.empty());

    cells_.resize(cell_primitives_count_);
    for (uint32_t i = 0; i < values.size(); i++) {
      cell_insert(values[i], bboxes[i]);
    }
  }

 public:
  float cell_size = 1;
  uint32_t hash_map_size = 1999;

  void clear() {
    cell_infos_ = {};
    active_cell_hashes = {};
    cell_primitives_count_ = 0;
    time = 0;
    cells_ = {};
  }

  void update(std::span<const T> values, std::span<const Bbox> bboxes) {
    assert(!values.empty());
    assert(values.size() == bboxes.size());

    time++;
    batch_populate_cell_info(bboxes);
    batch_cell_insert(values, bboxes);
  }

  std::vector<T> query_neighbors(const Bbox& bbox) const {
    assert(cell_size != 0);
    assert(!cell_infos_.empty());
    assert(!cells_.empty());

    auto [cell_min, cell_max] = get_cell_minmax(bbox);

    std::vector<T> neighbors;
    for (int32_t i = cell_min[0] - 1; i <= cell_max[0] + 1; i++) {
      for (int32_t j = cell_min[1] - 1; j <= cell_max[1] + 1; j++) {
        for (int32_t k = cell_min[2] - 1; k <= cell_max[2] + 1; k++) {
          uint32_t h = spatial_hash(i, j, k);
          auto& info = cell_infos_[h];
          for (uint32_t idx = info.start; idx < info.start + info.size; idx++) {
            const T& value = cells_[idx];
            if (std::ranges::find(neighbors, value) == neighbors.end()) {
              neighbors.push_back(value);
            }
          }
        }
      }
    }

    return neighbors;
  }
};

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>

#include "bbox.hpp"

namespace silk {

struct SpatialCellInfo {
  uint32_t time = 0;  // time stamp
  int start = 0;      // cell start
  int size = 0;       // cell size
  int fill = 0;       // the current empty slot
};

struct SpatialCellMinMax {
  Eigen::Vector3i min;
  Eigen::Vector3i max;
};

template <typename T>
class SpatialHasher {
  std::vector<SpatialCellInfo> cell_infos_;
  std::vector<uint32_t> active_cell_hashes;
  int cell_primitives_count_ = 0;
  uint32_t time = 0;  // time will wrap around when overflow
  std::vector<T> cells_;

  inline uint32_t spatial_hash(int i, int j, int k) const {
    assert(hash_map_size != 0);

    return static_cast<uint32_t>((73856093 * i) ^ (19349663 * j) ^
                                 (83492791 * k)) %
           hash_map_size;
  }

  inline SpatialCellMinMax get_cell_minmax(const Bbox& bbox) const {
    return {.min = (bbox.min / cell_size).array().floor().cast<int>(),
            .max = (bbox.max / cell_size).array().floor().cast<int>()};
  }

  void populate_cell_info(const Bbox& bbox) {
    assert(cell_size != 0);
    assert(!cell_infos_.empty());

    auto [cell_min, cell_max] = get_cell_minmax(bbox);

    for (int i = cell_min[0]; i <= cell_max[0]; i++) {
      for (int j = cell_min[1]; j <= cell_max[1]; j++) {
        for (int k = cell_min[2]; k <= cell_max[2]; k++) {
          uint32_t h = spatial_hash(i, j, k);
          SpatialCellInfo& info = cell_infos_[h];

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

  void batch_populate_cell_info(const std::vector<Bbox>& bboxes) {
    assert(hash_map_size != 0);
    assert(!bboxes.empty());

    if (cell_infos_.size() != hash_map_size) {
      cell_infos_.resize(hash_map_size);
      std::fill(cell_infos_.begin(), cell_infos_.end(),
                SpatialCellInfo{0, 0, 0, 0});
    }
    active_cell_hashes.clear();
    cell_primitives_count_ = 0;

    // calculate cell primative count
    for (const Bbox& bbox : bboxes) {
      populate_cell_info(bbox);
    }

    // calculate cell start
    int start = 0;
    // TODO: maybe scanning cell_infos_ directly is faster due to cache
    // locality, need profiling
    for (uint32_t h : active_cell_hashes) {
      SpatialCellInfo& info = cell_infos_[h];
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

    for (int i = cell_min[0]; i <= cell_max[0]; i++) {
      for (int j = cell_min[1]; j <= cell_max[1]; j++) {
        for (int k = cell_min[2]; k <= cell_max[2]; k++) {
          uint32_t h = spatial_hash(i, j, k);
          SpatialCellInfo& info = cell_infos_[h];
          cells_[info.fill] = value;
          info.fill++;
        }
      }
    }
  }

  void batch_cell_insert(const std::vector<T>& values,
                         const std::vector<Bbox>& bboxes) {
    assert(!values.empty());
    assert(values.size() == bboxes.size());
    assert(!cell_infos_.empty());

    cells_.resize(cell_primitives_count_);
    for (int i = 0; i < values.size(); i++) {
      cell_insert(values[i], bboxes[i]);
    }
  }

 public:
  float cell_size = 1;
  int hash_map_size = 1999;

  void clear() {
    cell_infos_ = {};
    active_cell_hashes = {};
    cell_primitives_count_ = 0;
    time = 0;
    cells_ = {};
  }

  void update(const std::vector<T>& values, const std::vector<Bbox>& bboxes) {
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
    for (int i = cell_min[0] - 1; i <= cell_max[0] + 1; i++) {
      for (int j = cell_min[1] - 1; j <= cell_max[1] + 1; j++) {
        for (int k = cell_min[2] - 1; k <= cell_max[2] + 1; k++) {
          uint32_t h = spatial_hash(i, j, k);
          const SpatialCellInfo& info = cell_infos_[h];
          for (int idx = info.start; idx < info.start + info.size; idx++) {
            const T& value = cells_[idx];
            if (std::find(neighbors.begin(), neighbors.end(), value) ==
                neighbors.end()) {
              neighbors.push_back(value);
            }
          }
        }
      }
    }

    return neighbors;
  }
};

}  // namespace silk

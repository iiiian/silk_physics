#pragma once

#include <array>
#include <span>
#include <vector>

#include "backend/cpu/collision/bbox.hpp"

namespace silk::cpu {

struct BboxSOAView {
  std::array<const float*, 3> min;
  std::array<const float*, 3> max;

  BboxSOAView subspan(int start) const {
    BboxSOAView result = *this;
    for (int axis = 0; axis < 3; ++axis) {
      result.min[axis] += start;
      result.max[axis] += start;
    }
    return result;
  }
};

struct BboxSOAStore {
  void resize(int bbox_num) {
    bbox_num_ = bbox_num;
    min_x_.resize(bbox_num);
    min_y_.resize(bbox_num);
    min_z_.resize(bbox_num);
    max_x_.resize(bbox_num);
    max_y_.resize(bbox_num);
    max_z_.resize(bbox_num);
  }

  void set(int index, const Bbox& bbox) {
    min_x_[index] = bbox.min[0];
    min_y_[index] = bbox.min[1];
    min_z_[index] = bbox.min[2];
    max_x_[index] = bbox.max[0];
    max_y_[index] = bbox.max[1];
    max_z_[index] = bbox.max[2];
  }

  BboxSOAView view(int start = 0) const {
    BboxSOAView v;
    v.min[0] = min_x_.data() + start;
    v.min[1] = min_y_.data() + start;
    v.min[2] = min_z_.data() + start;
    v.max[0] = max_x_.data() + start;
    v.max[1] = max_y_.data() + start;
    v.max[2] = max_z_.data() + start;
    return v;
  }

 private:
  int bbox_num_ = 0;
  std::vector<float> min_x_;
  std::vector<float> min_y_;
  std::vector<float> min_z_;
  std::vector<float> max_x_;
  std::vector<float> max_y_;
  std::vector<float> max_z_;
};

/// @brief Test bbox overlaps using SAP.
/// @param query_bbox Query Bbox.
/// @param candidate_bboxes  Candidate bboxes
/// @param candidate_proxies Candidate KDTree proxies.
/// @param axis SAP axis.
/// @param overlap_proxies Output buffer for overlapped proxies.
/// @return Overlap proxy count.
int sap_bbox_overlaps(const Bbox& query_bbox,
                      const BboxSOAView& candidate_bboxes,
                      std::span<const int> candidate_proxies, int axis,
                      std::vector<int>& overlap_proxies);

}  // namespace silk::cpu

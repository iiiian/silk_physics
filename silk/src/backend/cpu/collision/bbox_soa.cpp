#include "backend/cpu/collision/bbox_soa.hpp"

#include <bit>

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "backend/cpu/collision/bbox_soa.cpp"

#include <hwy/foreach_target.h>
#include <hwy/highway.h>

HWY_BEFORE_NAMESPACE();
namespace silk::cpu::HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;

template <int axis0>
int sap_bbox_overlaps_target_axis(const Bbox& query_bbox,
                                  const BboxSOAView& candidate_bboxes,
                                  std::span<const int> candidate_proxies,
                                  std::vector<int>& overlap_proxies) {
  hn::ScalableTag<float> d;
  using Vec = hn::Vec<decltype(d)>;
  int lane_num = hn::Lanes(d);

  // clang-format off
  // Test collider a against collider group b.
  // Let SAP axis be 0 and aux axis be 1,2. Require
  // a_min_0 <= b.max_0 && 
  // b_min_0 <= a.max_0 &&
  // a_min_1 <= b.max_1 && 
  // b_min_1 <= a.max_1 &&
  // a_min_2 <= b.max_2 &&
  // b_min_2 <= a.max_2 &&
  // clang-format on

  constexpr int axis1 = (axis0 + 1) % 3;
  constexpr int axis2 = (axis0 + 2) % 3;

  Vec a_min_0 = hn::Set(d, query_bbox.min[axis0]);
  Vec a_max_0 = hn::Set(d, query_bbox.max[axis0]);
  Vec a_min_1 = hn::Set(d, query_bbox.min[axis1]);
  Vec a_max_1 = hn::Set(d, query_bbox.max[axis1]);
  Vec a_min_2 = hn::Set(d, query_bbox.min[axis2]);
  Vec a_max_2 = hn::Set(d, query_bbox.max[axis2]);

  int overlap_num = 0;
  int candidate_num = candidate_proxies.size();
  if (overlap_proxies.size() < candidate_num) {
    overlap_proxies.resize(candidate_num);
  }
  int i = 0;
  for (; i + lane_num <= candidate_num; i += lane_num) {
    // Test main axis.
    Vec b_min_0 = hn::LoadU(d, candidate_bboxes.min[axis0] + i);
    Vec b_max_0 = hn::LoadU(d, candidate_bboxes.max[axis0] + i);
    auto main_mask = hn::Le(b_min_0, a_max_0);
    main_mask = hn::And(main_mask, hn::Le(a_min_0, b_max_0));

    // Test aux axis.
    Vec b_min_1 = hn::LoadU(d, candidate_bboxes.min[axis1] + i);
    Vec b_max_1 = hn::LoadU(d, candidate_bboxes.max[axis1] + i);
    Vec b_min_2 = hn::LoadU(d, candidate_bboxes.min[axis2] + i);
    Vec b_max_2 = hn::LoadU(d, candidate_bboxes.max[axis2] + i);
    auto aux_mask = hn::Le(a_min_1, b_max_1);
    aux_mask = hn::And(aux_mask, hn::Le(b_min_1, a_max_1));
    aux_mask = hn::And(aux_mask, hn::Le(a_min_2, b_max_2));
    aux_mask = hn::And(aux_mask, hn::Le(b_min_2, a_max_2));
    aux_mask = hn::And(aux_mask, main_mask);
    uint64_t overlap_bits = hn::BitsFromMask(d, aux_mask);
    while (overlap_bits != 0) {
      int lane = std::countr_zero(overlap_bits);
      overlap_proxies[overlap_num] = candidate_proxies[i + lane];
      ++overlap_num;
      overlap_bits &= overlap_bits - 1;
    }

    // Early return if tail does not overlap on SAP axis.
    if (!hn::AllTrue(d, main_mask)) {
      return overlap_num;
    }
  }

  // Trailing batch.
  for (; i < candidate_num; ++i) {
    if (query_bbox.max[axis0] < candidate_bboxes.min[axis0][i]) {
      break;
    }
    if (query_bbox.min[axis0] <= candidate_bboxes.max[axis0][i] &&
        query_bbox.min[axis1] <= candidate_bboxes.max[axis1][i] &&
        candidate_bboxes.min[axis1][i] <= query_bbox.max[axis1] &&
        query_bbox.min[axis2] <= candidate_bboxes.max[axis2][i] &&
        candidate_bboxes.min[axis2][i] <= query_bbox.max[axis2]) {
      overlap_proxies[overlap_num] = candidate_proxies[i];
      ++overlap_num;
    }
  }

  return overlap_num;
}

int sap_bbox_overlaps_target(const Bbox& query_bbox,
                             const BboxSOAView& candidate_bboxes,
                             std::span<const int> candidate_proxies, int axis,
                             std::vector<int>& overlap_proxies) {
  switch (axis) {
    case 0:
      return sap_bbox_overlaps_target_axis<0>(
          query_bbox, candidate_bboxes, candidate_proxies, overlap_proxies);
    case 1:
      return sap_bbox_overlaps_target_axis<1>(
          query_bbox, candidate_bboxes, candidate_proxies, overlap_proxies);
    default:
      return sap_bbox_overlaps_target_axis<2>(
          query_bbox, candidate_bboxes, candidate_proxies, overlap_proxies);
  }
}

}  // namespace silk::cpu::HWY_NAMESPACE
HWY_AFTER_NAMESPACE();

#if HWY_ONCE
namespace silk::cpu {

HWY_EXPORT(sap_bbox_overlaps_target);

int sap_bbox_overlaps(const Bbox& query_bbox,
                      const BboxSOAView& candidate_bboxes,
                      std::span<const int> candidate_proxies, int axis,
                      std::vector<int>& overlap_proxies) {
  return HWY_DYNAMIC_DISPATCH(sap_bbox_overlaps_target)(
      query_bbox, candidate_bboxes, candidate_proxies, axis, overlap_proxies);
}

}  // namespace silk::cpu
#endif

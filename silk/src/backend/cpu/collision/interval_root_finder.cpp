// Interval-based root finding for continuous collision detection (CPU).
#include "backend/cpu/collision/interval_root_finder.hpp"

#include <algorithm>
#include <map>
#include <optional>
#include <vector>

namespace silk::cpu {

using IntervalBox = std::array<Interval, 3>;

struct VFFuncCoefficients {
  float c0[3];  /// v_t0-f0_t0.
  float c1[3];  /// v_t1-f0_t1.
  float c2[3];  /// f0_t0-f1_t0.
  float c3[3];  /// f0_t1-f1_t1.
  float c4[3];  /// f0_t0-f2_t0.
  float c5[3];  /// f0_t1-f2_t1.
};

struct EEFuncCoefficients {
  float c0[3];  /// ea0_t0-eb0_t0.
  float c1[3];  /// ea0_t1-eb0_t1.
  float c2[3];  /// ea1_t0-eb1_t0.
  float c3[3];  /// ea1_t1-eb1_t1.
  float c4[3];  /// eb0_t0-eb1_t0.
  float c5[3];  /// eb0_t1-eb1_t1.
};

// Fast 1D bound using only start/end values in the unit box.
template <bool is_vertex_face>
bool eval_unit_bbox_1d(const Eigen::Vector3f &a_t0, const Eigen::Vector3f &b_t0,
                       const Eigen::Vector3f &c_t0, const Eigen::Vector3f &d_t0,
                       const Eigen::Vector3f &a_t1, const Eigen::Vector3f &b_t1,
                       const Eigen::Vector3f &c_t1, const Eigen::Vector3f &d_t1,
                       float eps, float ms, int dim, bool &bbox_in_eps,
                       float &tol) {
  float minv;
  float maxv;
  if constexpr (is_vertex_face) {
    Eigen::Array<float, 6, 1> A;
    A(0) = a_t0(dim);
    A(1) = a_t0(dim);
    A(2) = a_t0(dim);
    A(3) = a_t1(dim);
    A(4) = a_t1(dim);
    A(5) = a_t1(dim);

    Eigen::Array<float, 6, 1> B;
    B(0) = b_t0(dim);
    B(1) = c_t0(dim);
    B(2) = d_t0(dim);
    B(3) = b_t1(dim);
    B(4) = c_t1(dim);
    B(5) = d_t1(dim);

    const Eigen::Array<float, 6, 1> D = A - B;
    minv = D.minCoeff();
    maxv = D.maxCoeff();
  } else {
    Eigen::Array<float, 8, 1> A;
    A(0) = a_t0(dim);
    A(1) = a_t0(dim);
    A(2) = b_t0(dim);
    A(3) = b_t0(dim);
    A(4) = a_t1(dim);
    A(5) = a_t1(dim);
    A(6) = b_t1(dim);
    A(7) = b_t1(dim);

    Eigen::Array<float, 8, 1> B;
    B(0) = c_t0(dim);
    B(1) = d_t0(dim);
    B(2) = c_t0(dim);
    B(3) = d_t0(dim);
    B(4) = c_t1(dim);
    B(5) = d_t1(dim);
    B(6) = c_t1(dim);
    B(7) = d_t1(dim);

    const Eigen::Array<float, 8, 1> D = A - B;
    minv = D.minCoeff();
    maxv = D.maxCoeff();
  }

  tol = maxv - minv;  // Real width of the co-domain interval in this dim.
  bbox_in_eps = false;
  const float eps_and_ms = eps + ms;

  if (minv > eps_and_ms || maxv < -eps_and_ms) {
    return false;
  }

  if (minv >= -eps_and_ms && maxv <= eps_and_ms) {
    bbox_in_eps = true;
  }

  return true;
}

bool eval_vf_bbox_1d(const std::array<Interval, 3> &tuv,
                     const VFFuncCoefficients &coeff, float eps_and_ms, int dim,
                     bool &bbox_in_eps, float &tol) {
  // clang-format off
  // Let vertex v at time t0 be v_t0 and face vertices be f0_t0, f1_t0, f2_t0.
  // Let vertex v at time t be v and face vertices be f0, f1, f2.
  // Likewise for time t1.
  //
  // Raw L1 VF distance function_vf formula:
  //   function_vf = (vertex position) - (face position)
  //               = vtx - ((f1 - f0) * u + (f2 - f0) * v + f0)
  //               = (vtx - f0) + (f0 - f1) * u + (f0 - f2) * v.
  // Where
  //   vtx - f0 = ((v_t1 - f0_t1) - (v_t0 - f0_t0)) * t + (v_t0 - f0_t0);
  //   f0 - f1 = ((f0_t1 - f1_t1) - (f0_t0 - f1_t0)) * t + (f0_t0 - f1_t0);
  //   f0 - f2 = ((f0_t1 - f2_t1) - (f0_t0 - f2_t0)) * t + (f0_t0 - f2_t0);
  // Above coefficients are pre-computed and stored in VFBoxCoefficients.
  // clang-format on
  float t_lo = tuv[0].lower;
  float t_up = tuv[0].upper;
  float u_lo = tuv[1].lower;
  float u_up = tuv[1].upper;
  float v_lo = tuv[2].lower;
  float v_up = tuv[2].upper;
  float c0 = coeff.c0[dim];
  float c1 = coeff.c1[dim];
  float c2 = coeff.c2[dim];
  float c3 = coeff.c3[dim];
  float c4 = coeff.c4[dim];
  float c5 = coeff.c5[dim];

  auto vf_bounds_at_t = [&](float t, float &vf_min, float &vf_max) {
    float v_minus_f0 = (c1 - c0) * t + c0;
    float f0_minus_f1 = (c3 - c2) * t + c2;
    float f0_minus_f2 = (c5 - c4) * t + c4;

    if (u_up + v_up <= 1.0f) {
      float f0_minus_f1_u_lo = f0_minus_f1 * u_lo;
      float f0_minus_f1_u_up = f0_minus_f1 * u_up;
      float f0_minus_f2_v_lo = f0_minus_f2 * v_lo;
      float f0_minus_f2_v_up = f0_minus_f2 * v_up;
      vf_min = v_minus_f0 + std::min(f0_minus_f1_u_lo, f0_minus_f1_u_up) +
               std::min(f0_minus_f2_v_lo, f0_minus_f2_v_up);
      vf_max = v_minus_f0 + std::max(f0_minus_f1_u_lo, f0_minus_f1_u_up) +
               std::max(f0_minus_f2_v_lo, f0_minus_f2_v_up);
      return;
    }

    vf_min = std::numeric_limits<float>::infinity();
    vf_max = -std::numeric_limits<float>::infinity();
    auto include_vf_boundary_point = [&](float u, float v) {
      float vf_value = v_minus_f0 + f0_minus_f1 * u + f0_minus_f2 * v;
      vf_min = std::min(vf_min, vf_value);
      vf_max = std::max(vf_max, vf_value);
    };

    // Triangle parameter is valid only if u+v <= 1.0.
    // By clipping the uv parameter box, we could have;
    // - 1-4 corners.
    // - 1-2 diagonal intersection.

    // Test all four corners.
    if (u_lo + v_lo <= 1.0f) {
      include_vf_boundary_point(u_lo, v_lo);
    }
    if (u_lo + v_up <= 1.0f) {
      include_vf_boundary_point(u_lo, v_up);
    }
    if (u_up + v_lo <= 1.0f) {
      include_vf_boundary_point(u_up, v_lo);
    }
    if (u_up + v_up <= 1.0f) {
      include_vf_boundary_point(u_up, v_up);
    }
    // Test all four diagonal intersections.
    float v_at_u_lo = 1.0f - u_lo;
    if (v_at_u_lo >= v_lo && v_at_u_lo <= v_up) {
      include_vf_boundary_point(u_lo, v_at_u_lo);
    }
    float v_at_u_up = 1.0f - u_up;
    if (v_at_u_up >= v_lo && v_at_u_up <= v_up) {
      include_vf_boundary_point(u_up, v_at_u_up);
    }
    float u_at_v_lo = 1.0f - v_lo;
    if (u_at_v_lo >= u_lo && u_at_v_lo <= u_up) {
      include_vf_boundary_point(u_at_v_lo, v_lo);
    }
    float u_at_v_up = 1.0f - v_up;
    if (u_at_v_up >= u_lo && u_at_v_up <= u_up) {
      include_vf_boundary_point(u_at_v_up, v_up);
    }
  };

  float vf_min_at_t_lo;
  float vf_max_at_t_lo;
  float vf_min_at_t_up;
  float vf_max_at_t_up;
  vf_bounds_at_t(t_lo, vf_min_at_t_lo, vf_max_at_t_lo);
  vf_bounds_at_t(t_up, vf_min_at_t_up, vf_max_at_t_up);
  float minv = std::min(vf_min_at_t_lo, vf_min_at_t_up);
  float maxv = std::max(vf_max_at_t_lo, vf_max_at_t_up);
  bbox_in_eps = false;
  if (minv > eps_and_ms || maxv < -eps_and_ms) {
    return false;
  }
  tol = maxv - minv;
  if (minv >= -eps_and_ms && maxv <= eps_and_ms) {
    bbox_in_eps = true;
  }
  return true;
}

bool eval_ee_bbox_1d(const std::array<Interval, 3> &tuv,
                     const EEFuncCoefficients &coeff, float eps_and_ms, int dim,
                     bool &bbox_in_eps, float &tol) {
  // clang-format off
  // Let edge a vertex 0 at time t0 be ea0_t0, vertex 1 be ea1_t1.
  // Let edge a vertex 0 at time t be ea0, vertex 1 be ea1.
  // Likewise for edge b and time t1.
  //
  // Raw L1 EE distance function_ee formula:
  //   function_ee = (edge a collision point) - (edge b collision point)
  //               = ((ea1 - ea0) * u + ea0 ) - ((eb1 - eb0) * v + eb0 )
  //               = (ea0 - eb0) + (ea1 - ea0) * u + (eb0 - eb1) * v.
  // Where
  //   ea0 - eb0 =  ((ea0_t1 - eb0_t1) - (ea0_t0 - eb0_t0)) * t + (ea0_t0 - eb0_t0);
  //   ea1 - ea0 =  ((ea1_t1 - ea0_t1) - (ea1_t0 - ea0_t0)) * t + (ea1_t0 - ea0_t0);
  //   eb0 - eb1 =  ((eb0_t1 - eb1_t1) - (eb0_t0 - eb1_t0)) * t + (eb0_t0 - eb1_t0);
  // Above coefficients are pre-computed and stored in EEBoxCoefficients.
  // clang-format on
  float t_lo = tuv[0].lower;
  float t_up = tuv[0].upper;
  float u_lo = tuv[1].lower;
  float u_up = tuv[1].upper;
  float v_lo = tuv[2].lower;
  float v_up = tuv[2].upper;
  float c0 = coeff.c0[dim];
  float c1 = coeff.c1[dim];
  float c2 = coeff.c2[dim];
  float c3 = coeff.c3[dim];
  float c4 = coeff.c4[dim];
  float c5 = coeff.c5[dim];

  auto ee_bounds_at_t = [&](float t, float &ee_min, float &ee_max) {
    float ea0_minus_eb0 = (c1 - c0) * t + c0;
    float ea1_minus_ea0 = (c3 - c2) * t + c2;
    float eb0_minus_eb1 = (c5 - c4) * t + c4;
    float ea1_minus_ea0_u_lo = ea1_minus_ea0 * u_lo;
    float ea1_minus_ea0_u_up = ea1_minus_ea0 * u_up;
    float eb0_minus_eb1_v_lo = eb0_minus_eb1 * v_lo;
    float eb0_minus_eb1_v_up = eb0_minus_eb1 * v_up;
    ee_min = ea0_minus_eb0 + std::min(ea1_minus_ea0_u_lo, ea1_minus_ea0_u_up) +
             std::min(eb0_minus_eb1_v_lo, eb0_minus_eb1_v_up);
    ee_max = ea0_minus_eb0 + std::max(ea1_minus_ea0_u_lo, ea1_minus_ea0_u_up) +
             std::max(eb0_minus_eb1_v_lo, eb0_minus_eb1_v_up);
  };

  float ee_min_at_t_lo;
  float ee_max_at_t_lo;
  float ee_min_at_t_up;
  float ee_max_at_t_up;
  ee_bounds_at_t(t_lo, ee_min_at_t_lo, ee_max_at_t_lo);
  ee_bounds_at_t(t_up, ee_min_at_t_up, ee_max_at_t_up);
  float minv = std::min(ee_min_at_t_lo, ee_min_at_t_up);
  float maxv = std::max(ee_max_at_t_lo, ee_max_at_t_up);
  bbox_in_eps = false;
  if (minv > eps_and_ms || maxv < -eps_and_ms) {
    return false;
  }
  tol = maxv - minv;
  if (minv >= -eps_and_ms && maxv <= eps_and_ms) {
    bbox_in_eps = true;
  }
  return true;
}

// Check if the hex in (t,u,v) maps near the origin; compute per-axis widths.
// Uses 8-corner evaluation; when fully inside, further bisection can stop.
template <bool is_vertex_face, bool is_unit_tuv>
bool origin_in_bbox_eval(
    const std::array<Interval, 3> &tuv, const Eigen::Vector3f &a_t0,
    const Eigen::Vector3f &b_t0, const Eigen::Vector3f &c_t0,
    const Eigen::Vector3f &d_t0, const Eigen::Vector3f &a_t1,
    const Eigen::Vector3f &b_t1, const Eigen::Vector3f &c_t1,
    const Eigen::Vector3f &d_t1, const Eigen::Array3f &eps, float ms,
    bool &bbox_in_eps, Eigen::Array3f &tolerance,
    const VFFuncCoefficients &vf_coefficients,
    const EEFuncCoefficients &ee_coefficients) {
  bool xyz_bbox_in_eps[3];
  bbox_in_eps = false;
  if constexpr (is_unit_tuv) {
    for (int dim = 0; dim < 3; dim++) {
      if (!eval_unit_bbox_1d<is_vertex_face>(
              a_t0, b_t0, c_t0, d_t0, a_t1, b_t1, c_t1, d_t1, eps(dim), ms, dim,
              xyz_bbox_in_eps[dim], tolerance(dim))) {
        return false;
      }
    }
  } else {
    if constexpr (is_vertex_face) {
      for (int dim = 0; dim < 3; dim++) {
        if (!eval_vf_bbox_1d(tuv, vf_coefficients, eps(dim) + ms, dim,
                             xyz_bbox_in_eps[dim], tolerance(dim))) {
          return false;
        }
      }
    } else {
      for (int dim = 0; dim < 3; dim++) {
        if (!eval_ee_bbox_1d(tuv, ee_coefficients, eps(dim) + ms, dim,
                             xyz_bbox_in_eps[dim], tolerance(dim))) {
          return false;
        }
      }
    }
  }

  bbox_in_eps = xyz_bbox_in_eps[0] && xyz_bbox_in_eps[1] && xyz_bbox_in_eps[2];
  return true;
}

// Pick the dimension with the largest relative width above its tolerance.
int find_next_split(const Eigen::Array3f &widths, const Eigen::Array3f &tols) {
  Eigen::Array3f tmp =
      (widths > tols)
          .select(widths / tols, -std::numeric_limits<float>::infinity());
  int max_index;
  tmp.maxCoeff(&max_index);
  return max_index;
}

bool split_and_push(const std::array<Interval, 3> &tuv, int split_i,
                    std::function<void(const std::array<Interval, 3> &)> push,
                    bool is_vertex_face) {
  std::pair<Interval, Interval> halves = tuv[split_i].bisect();

  // Guard against degenerately small intervals causing infinite splitting.
  constexpr float SPLITING_LOWER_BOUND = 1e-12f;
  if (halves.first.upper < SPLITING_LOWER_BOUND) {
    return true;
  }

  std::array<Interval, 3> tmp = tuv;

  if (split_i == 0 || !is_vertex_face) {
    // split time or edge uv
    tmp[split_i] = halves.second;
    push(tmp);
    tmp[split_i] = halves.first;
    push(tmp);
  } else {
    // Enforce triangle domain constraint u + v ≤ 1 when splitting.
    const Interval &other = (split_i == 1) ? tuv[2] : tuv[1];
    if (halves.second.lower + other.lower <= 1.0f) {
      tmp[split_i] = halves.second;
      push(tmp);
    }
    if (halves.first.lower + other.lower <= 1.0f) {
      tmp[split_i] = halves.first;
      push(tmp);
    }
  }

  return false;  // no overflow
}

CCDResult make_ccd_result(const std::array<Interval, 3> &tuv, float tolerance) {
  // Pack the time interval and termination tolerance into CCDResult.
  CCDResult c;
  c.t = {tuv[0].lower, tuv[0].upper};
  c.tolerance = tolerance;
  c.use_small_ms = false;
  c.small_ms_t = {0.0f, 0.0f};  // not use

  return c;
}

Eigen::Array3f width(const std::array<Interval, 3> &x) {
  return Eigen::Array3f(x[0].upper - x[0].lower, x[1].upper - x[1].lower,
                        x[2].upper - x[2].lower);
}

// Bucket DFS over (t,u,v) with box tests and targeted splitting.
// Each stack is keyed by t.lower, and stacks are processed chronologically.
//
// Based on narrowphase benchmark, >95% of CPU time is dominated by queries with
// deep traversal level (10+). However, compared to u and v the refinement level
// of time is low. A dedicated DFS traversal stack for each t lower bound keeps
// traversal chronological. The resulting speedup is 3-7x.
template <bool is_vertex_face>
std::optional<CCDResult> interval_root_finder_bucket_DFS(
    const Eigen::Vector3f &a_t0, const Eigen::Vector3f &b_t0,
    const Eigen::Vector3f &c_t0, const Eigen::Vector3f &d_t0,
    const Eigen::Vector3f &a_t1, const Eigen::Vector3f &b_t1,
    const Eigen::Vector3f &c_t1, const Eigen::Vector3f &d_t1,
    const std::array<Interval, 3> &iset, const Eigen::Array3f &tol,
    float co_domain_tolerance, const Eigen::Array3f &err, float ms,
    long max_iter, bool is_unit_interval) {
  // DFS traversal stack bucket. Keyed by t lower bound.
  std::map<float, std::vector<std::pair<IntervalBox, int>>> t_stacks;

  IntervalBox current = iset;
  int tree_level = 0;
  bool has_current = true;

  int iter_count = 0;

  // Precompute coefficients for VF/EE L1 distance function.
  VFFuncCoefficients vf_coefficients;
  EEFuncCoefficients ee_coefficients;
  if constexpr (is_vertex_face) {
    for (int dim = 0; dim < 3; ++dim) {
      vf_coefficients.c0[dim] = a_t0(dim) - b_t0(dim);
      vf_coefficients.c1[dim] = a_t1(dim) - b_t1(dim);
      vf_coefficients.c2[dim] = b_t0(dim) - c_t0(dim);
      vf_coefficients.c3[dim] = b_t1(dim) - c_t1(dim);
      vf_coefficients.c4[dim] = b_t0(dim) - d_t0(dim);
      vf_coefficients.c5[dim] = b_t1(dim) - d_t1(dim);
    }
  } else {
    for (int dim = 0; dim < 3; ++dim) {
      ee_coefficients.c0[dim] = a_t0(dim) - c_t0(dim);
      ee_coefficients.c1[dim] = a_t1(dim) - c_t1(dim);
      ee_coefficients.c2[dim] = b_t0(dim) - a_t0(dim);
      ee_coefficients.c3[dim] = b_t1(dim) - a_t1(dim);
      ee_coefficients.c4[dim] = c_t0(dim) - d_t0(dim);
      ee_coefficients.c5[dim] = c_t1(dim) - d_t1(dim);
    }
  }

  while (has_current || !t_stacks.empty()) {
    auto bucket = t_stacks.end();
    if (!has_current) {
      // Get top of the stack with smallest t lower bound.
      bucket = t_stacks.begin();
      auto &stack = bucket->second;
      auto entry = std::move(stack.back());
      stack.pop_back();
      current = std::move(entry.first);
      tree_level = entry.second;
    }
    has_current = false;
    ++iter_count;

    // True if bbox evaluation intersects the root region.
    bool bbox_in_zero;
    // True if bbox evaluation is entirely inside the root region.
    bool bbox_in_eps;
    // Per-axis width of the bbox evaluation image.
    Eigen::Array3f bbox_eval_tolerance;
    if (is_unit_interval) {
      bbox_in_zero = origin_in_bbox_eval<is_vertex_face, true>(
          current, a_t0, b_t0, c_t0, d_t0, a_t1, b_t1, c_t1, d_t1, err, ms,
          bbox_in_eps, bbox_eval_tolerance, vf_coefficients, ee_coefficients);
      is_unit_interval = false;
    } else {
      bbox_in_zero = origin_in_bbox_eval<is_vertex_face, false>(
          current, a_t0, b_t0, c_t0, d_t0, a_t1, b_t1, c_t1, d_t1, err, ms,
          bbox_in_eps, bbox_eval_tolerance, vf_coefficients, ee_coefficients);
    }

    // current interval does not contain function root. skip.
    if (!bbox_in_zero) {
      if (bucket != t_stacks.end() && bucket->second.empty()) {
        t_stacks.erase(bucket);
      }
      continue;
    }

    // The box might contain a root; check if it is small enough to accept.
    bool is_co_domain_tol_small_enough =
        (bbox_eval_tolerance <= co_domain_tolerance).all();
    bool is_interval_small_enough =
        is_co_domain_tol_small_enough || bbox_in_eps;

    // The bucket is the smallest pending t.lower, so this is chronological.
    if (is_interval_small_enough) {
      return make_ccd_result(current, co_domain_tolerance);
    }

    // All boxes with a smaller t.lower are exhausted, so current is the
    // earliest conservative fallback when the iteration limit is reached.
    if (max_iter > 0 && iter_count > max_iter) {
      float true_tolerance =
          std::max(co_domain_tolerance, bbox_eval_tolerance.maxCoeff());
      return make_ccd_result(current, true_tolerance);
    }

    // find the next dimension to split
    Eigen::Array3f widths = width(current);
    int split_dimension = find_next_split(widths, tol);
    auto push = [&t_stacks, tree_level](const IntervalBox &box) {
      t_stacks[box[0].lower].emplace_back(box, tree_level + 1);
    };
    bool overflow =
        split_and_push(current, split_dimension, push, is_vertex_face);
    if (overflow) {
      float true_tolerance =
          std::max(co_domain_tolerance, bbox_eval_tolerance.maxCoeff());
      return make_ccd_result(current, true_tolerance);
    }
    // Keep an emptied bucket alive through evaluation and splitting so
    // children with the same t.lower reuse its vector allocation.
    if (bucket != t_stacks.end() && bucket->second.empty()) {
      t_stacks.erase(bucket);
    }
  }

  return std::nullopt;
}

Eigen::Array3f get_numerical_error(const Eigen::Vector3f &abs_max,
                                   const bool is_vertex_face) {
  // Single-precision minimum-separation filters for the cached affine
  // evaluation graph. See test/narrowphase/derive_numerical_filter.py.
  constexpr float EE_FILTER = 3.814698e-06;
  constexpr float VF_FILTER = 4.053116e-06;

  Eigen::Vector3f gamma = abs_max.cwiseMax(1.0f);
  float filter = is_vertex_face ? VF_FILTER : EE_FILTER;
  return filter * gamma.array();
}

// ------------------------------------------------------------------------
// Template instantiation
// ------------------------------------------------------------------------

template std::optional<CCDResult> interval_root_finder_bucket_DFS<true>(
    const Eigen::Vector3f &a_t0, const Eigen::Vector3f &b_t0,
    const Eigen::Vector3f &c_t0, const Eigen::Vector3f &d_t0,
    const Eigen::Vector3f &a_t1, const Eigen::Vector3f &b_t1,
    const Eigen::Vector3f &c_t1, const Eigen::Vector3f &d_t1,
    const std::array<Interval, 3> &iset, const Eigen::Array3f &tol,
    float co_domain_tolerance, const Eigen::Array3f &err, float ms,
    long max_iter, bool is_unit_interval);

template std::optional<CCDResult> interval_root_finder_bucket_DFS<false>(
    const Eigen::Vector3f &a_t0, const Eigen::Vector3f &b_t0,
    const Eigen::Vector3f &c_t0, const Eigen::Vector3f &d_t0,
    const Eigen::Vector3f &a_t1, const Eigen::Vector3f &b_t1,
    const Eigen::Vector3f &c_t1, const Eigen::Vector3f &d_t1,
    const std::array<Interval, 3> &iset, const Eigen::Array3f &tol,
    float co_domain_tolerance, const Eigen::Array3f &err, float ms,
    long max_iter, bool is_unit_interval);

}  // namespace silk::cpu

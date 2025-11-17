// Interval-based root finding for continuous collision detection (CPU).
#include "backend/cuda/collision/interval_root_finder.hpp"

#include <algorithm>
#include <optional>

#include "common/logger.hpp"

namespace silk::cuda {

/// Convert (t,u,v) box into arrays of the 8 hex-vertex parameter samples.
void tuv_to_array(const std::array<Interval, 3> &tuv,
                  Eigen::Array<float, 8, 1> &t, Eigen::Array<float, 8, 1> &u,
                  Eigen::Array<float, 8, 1> &v) {
  // t order: 0,0,0,0,1,1,1,1
  // u order: 0,0,1,1,0,0,1,1
  // v order: 0,1,0,1,0,1,0,1
  float t0 = tuv[0].lower;
  float t1 = tuv[0].upper;
  float u0 = tuv[1].lower;
  float u1 = tuv[1].upper;
  float v0 = tuv[2].lower;
  float v1 = tuv[2].upper;
  t = {t0, t0, t0, t0, t1, t1, t1, t1};
  u = {u0, u0, u1, u1, u0, u0, u1, u1};
  v = {v0, v1, v0, v1, v0, v1, v0, v1};
}

/// Evaluate edge–edge point difference over parameterized motion.
Eigen::Array<float, 8, 1> function_ee(const float &ea0_t0, const float &ea1_t0,
                                      const float &eb0_t0, const float &eb1_t0,
                                      const float &ea0_t1, const float &ea1_t1,
                                      const float &eb0_t1, const float &eb1_t1,
                                      const Eigen::Array<float, 8, 1> &t,
                                      const Eigen::Array<float, 8, 1> &u,
                                      const Eigen::Array<float, 8, 1> &v) {
  Eigen::Array<float, 8, 1> ea0 = (ea0_t1 - ea0_t0) * t + ea0_t0;
  Eigen::Array<float, 8, 1> ea1 = (ea1_t1 - ea1_t0) * t + ea1_t0;
  Eigen::Array<float, 8, 1> eb0 = (eb0_t1 - eb0_t0) * t + eb0_t0;
  Eigen::Array<float, 8, 1> eb1 = (eb1_t1 - eb1_t0) * t + eb1_t0;

  Eigen::Array<float, 8, 1> va = (ea1 - ea0) * u + ea0;
  Eigen::Array<float, 8, 1> vb = (eb1 - eb0) * v + eb0;

  return va - vb;
}

/// Evaluate vertex–face point difference over parameterized motion.
Eigen::Array<float, 8, 1> function_vf(const float &v_t0, const float &f0_t0,
                                      const float &f1_t0, const float &f2_t0,
                                      const float &v_t1, const float &f0_t1,
                                      const float &f1_t1, const float &f2_t1,
                                      const Eigen::Array<float, 8, 1> &t,
                                      const Eigen::Array<float, 8, 1> &u,
                                      const Eigen::Array<float, 8, 1> &v) {
  Eigen::Array<float, 8, 1> vtx = (v_t1 - v_t0) * t + v_t0;
  Eigen::Array<float, 8, 1> f0 = (f0_t1 - f0_t0) * t + f0_t0;
  Eigen::Array<float, 8, 1> f1 = (f1_t1 - f1_t0) * t + f1_t0;
  Eigen::Array<float, 8, 1> f2 = (f2_t1 - f2_t0) * t + f2_t0;
  Eigen::Array<float, 8, 1> pt = (f1 - f0) * u + (f2 - f0) * v + f0;

  return vtx - pt;
}

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

// Vectorized bound returning the true per-axis co-domain width.
// Returns overlap with [-eps-ms, eps+ms]; sets bbox_in_eps when fully inside.
template <bool is_vertex_face>
bool eval_bbox_1d(Eigen::Array<float, 8, 1> &t, Eigen::Array<float, 8, 1> &u,
                  Eigen::Array<float, 8, 1> &v, const Eigen::Vector3f &a_t0,
                  const Eigen::Vector3f &b_t0, const Eigen::Vector3f &c_t0,
                  const Eigen::Vector3f &d_t0, const Eigen::Vector3f &a_t1,
                  const Eigen::Vector3f &b_t1, const Eigen::Vector3f &c_t1,
                  const Eigen::Vector3f &d_t1, float eps, float ms, int dim,
                  bool &bbox_in_eps, float &tol) {
  Eigen::Array<float, 8, 1> vs;
  if constexpr (is_vertex_face) {
    vs = function_vf(a_t0(dim), b_t0(dim), c_t0(dim), d_t0(dim), a_t1(dim),
                     b_t1(dim), c_t1(dim), d_t1(dim), t, u, v);
  } else {
    vs = function_ee(a_t0(dim), b_t0(dim), c_t0(dim), d_t0(dim), a_t1(dim),
                     b_t1(dim), c_t1(dim), d_t1(dim), t, u, v);
  }

  float minv = vs.minCoeff();
  float maxv = vs.maxCoeff();

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

// Check if the hex in (t,u,v) maps near the origin; compute per-axis widths.
// Uses 8-corner evaluation; when fully inside, further bisection can stop.
template <bool is_vertex_face, bool is_unit_tuv>
bool origin_in_bbox_eval(
    const std::array<Interval, 3> &tuv, const Eigen::Vector3f &a_t0,
    const Eigen::Vector3f &b_t0, const Eigen::Vector3f &c_t0,
    const Eigen::Vector3f &d_t0, const Eigen::Vector3f &a_t1,
    const Eigen::Vector3f &b_t1, const Eigen::Vector3f &c_t1,
    const Eigen::Vector3f &d_t1, const Eigen::Array3f &eps, float ms,
    bool &bbox_in_eps, Eigen::Array3f &tolerance) {
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
    Eigen::Array<float, 8, 1> t, u, v;
    tuv_to_array(tuv, t, u, v);

    for (int dim = 0; dim < 3; dim++) {
      if (!eval_bbox_1d<is_vertex_face>(t, u, v, a_t0, b_t0, c_t0, d_t0, a_t1,
                                        b_t1, c_t1, d_t1, eps(dim), ms, dim,
                                        xyz_bbox_in_eps[dim], tolerance(dim))) {
        return false;
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
    SPDLOG_ERROR("overflow occured when splitting intervals!");
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

// Lower bound of the time-of-impact for a (t,u,v) box.
float get_toi(const std::array<Interval, 3> &tuv) { return tuv[0].lower; }

CCDResult make_ccd_result(const std::array<Interval, 3> &tuv, float tolerance) {
  // Pack the interval box and termination tolerance into CCDResult.
  CCDResult c;
  c.t = {tuv[0].lower, tuv[0].upper};
  c.u = {tuv[1].lower, tuv[1].upper};
  c.v = {tuv[2].lower, tuv[2].upper};
  c.tolerance = tolerance;
  c.use_small_ms = false;
  c.small_ms_t = {0.0f, 0.0f};  // not use

  return c;
}

Eigen::Array3f width(const std::array<Interval, 3> &x) {
  return Eigen::Array3f(x[0].upper - x[0].lower, x[1].upper - x[1].lower,
                        x[2].upper - x[2].lower);
}

// Breadth-first search over (t,u,v) with box tests and targeted splitting.
template <bool is_vertex_face>
std::optional<CCDResult> interval_root_finder_BFS(
    const Eigen::Vector3f &a_t0, const Eigen::Vector3f &b_t0,
    const Eigen::Vector3f &c_t0, const Eigen::Vector3f &d_t0,
    const Eigen::Vector3f &a_t1, const Eigen::Vector3f &b_t1,
    const Eigen::Vector3f &c_t1, const Eigen::Vector3f &d_t1,
    const std::array<Interval, 3> &iset, const Eigen::Array3f &tol,
    float co_domain_tolerance, const Eigen::Array3f &err, float ms,
    long max_iter, bool is_unit_interval) {
  // Stack of (interval box, tree level). Boxes are sorted by level then TOI.
  std::vector<std::pair<std::array<Interval, 3>, int>> istack;
  istack.emplace_back(iset, 0);

  int iter_count = 0;
  int tree_level = 0;

  // A root may be deferred if a potentially earlier root exists.
  std::optional<std::array<Interval, 3>> skipped_candidate;

  // The first root at a level is treated as the earliest candidate.
  bool is_earliest_candidate = true;
  std::array<Interval, 3> earliest_candidate;
  Eigen::Array3f earliest_bbox_eval_tolerance;

  for (int stack_idx = 0; stack_idx < istack.size(); ++stack_idx) {
    // Reached a new tree level; sort by TOI to prioritize earlier boxes.
    if (istack[stack_idx].second != tree_level) {
      auto cmp = [](const std::pair<std::array<Interval, 3>, int> &i1,
                    const std::pair<std::array<Interval, 3>, int> &i2) {
        return i1.first[0].lower < i2.first[0].lower;
      };
      std::sort(istack.data() + stack_idx, istack.data() + istack.size(), cmp);

      ++tree_level;
      is_earliest_candidate = true;
    }

    // current intervals
    std::array<Interval, 3> current = istack[stack_idx].first;

    // if this box is later than skipped toi, skip.
    if (skipped_candidate && get_toi(current) >= get_toi(*skipped_candidate)) {
      continue;
    }

    iter_count++;

    // True if bbox evaluation intersects the root region.
    bool bbox_in_zero;
    // True if bbox evaluation is entirely inside the root region.
    bool bbox_in_eps;
    // Per-axis width of the bbox evaluation image.
    Eigen::Array3f bbox_eval_tolerance;
    if (is_unit_interval) {
      bbox_in_zero = origin_in_bbox_eval<is_vertex_face, true>(
          current, a_t0, b_t0, c_t0, d_t0, a_t1, b_t1, c_t1, d_t1, err, ms,
          bbox_in_eps, bbox_eval_tolerance);
      is_unit_interval = false;
    } else {
      bbox_in_zero = origin_in_bbox_eval<is_vertex_face, false>(
          current, a_t0, b_t0, c_t0, d_t0, a_t1, b_t1, c_t1, d_t1, err, ms,
          bbox_in_eps, bbox_eval_tolerance);
    }

    // current interval does not contain function root. skip.
    if (!bbox_in_zero) {
      continue;
    }

    // The box might contain a root; check if it is small enough to accept.

    bool is_co_domain_tol_small_enough =
        (bbox_eval_tolerance <= co_domain_tolerance).all();
    bool is_interval_small_enough =
        is_co_domain_tol_small_enough || bbox_in_eps;

    // Earliest acceptable root found; terminate.
    if (is_earliest_candidate && is_interval_small_enough) {
      return make_ccd_result(current, co_domain_tolerance);
    }

    // Acceptable but not earliest; record and skip.
    if (is_interval_small_enough) {
      if (skipped_candidate) {
        if (get_toi(*skipped_candidate) > get_toi(current)) {
          skipped_candidate = current;
        }
      } else {
        skipped_candidate = current;
      }
      continue;
    }

    // Not small enough but earliest; remember if bounded by max_iter.
    if (is_earliest_candidate) {
      is_earliest_candidate = false;

      if (max_iter > 0) {
        earliest_candidate = current;
        earliest_bbox_eval_tolerance = bbox_eval_tolerance;
      }
    }

    // Max-iter reached: return earliest candidate with a safe tolerance.
    if (max_iter > 0 && iter_count > max_iter) {
      float true_tolerance = std::max(co_domain_tolerance,
                                      earliest_bbox_eval_tolerance.maxCoeff());
      return make_ccd_result(earliest_candidate, true_tolerance);
    }

    // find the next dimension to split
    Eigen::Array3f widths = width(current);
    int split_dimension = find_next_split(widths, tol);
    auto push = [&istack, &tree_level](const std::array<Interval, 3> &i) {
      istack.emplace_back(i, tree_level + 1);
    };
    bool overflow =
        split_and_push(current, split_dimension, push, is_vertex_face);
    if (overflow) {
      SPDLOG_ERROR("overflow occured when splitting intervals!");

      float true_tolerance = std::max(co_domain_tolerance,
                                      earliest_bbox_eval_tolerance.maxCoeff());
      return make_ccd_result(current, true_tolerance);
    }
  }

  if (skipped_candidate) {
    return make_ccd_result(*skipped_candidate, co_domain_tolerance);
  }

  return std::nullopt;
}

Eigen::Array3f get_numerical_error(const Eigen::Vector3f &abs_max,
                                   const bool is_vertex_face) {
  // Cubic filters empirically chosen for EE/VF; scaled by clamped magnitude.
  constexpr float EE_FILTER = 3.814698e-06;
  constexpr float VF_FILTER = 4.053116e-06;

  Eigen::Vector3f delta = abs_max.cwiseMin(1.0f);
  float filter = is_vertex_face ? VF_FILTER : EE_FILTER;
  return filter * delta.array().pow(3);
}

// ------------------------------------------------------------------------
// Template instantiation
// ------------------------------------------------------------------------

template std::optional<CCDResult> interval_root_finder_BFS<true>(
    const Eigen::Vector3f &a_t0, const Eigen::Vector3f &b_t0,
    const Eigen::Vector3f &c_t0, const Eigen::Vector3f &d_t0,
    const Eigen::Vector3f &a_t1, const Eigen::Vector3f &b_t1,
    const Eigen::Vector3f &c_t1, const Eigen::Vector3f &d_t1,
    const std::array<Interval, 3> &iset, const Eigen::Array3f &tol,
    float co_domain_tolerance, const Eigen::Array3f &err, float ms,
    long max_iter, bool is_unit_interval);

template std::optional<CCDResult> interval_root_finder_BFS<false>(
    const Eigen::Vector3f &a_t0, const Eigen::Vector3f &b_t0,
    const Eigen::Vector3f &c_t0, const Eigen::Vector3f &d_t0,
    const Eigen::Vector3f &a_t1, const Eigen::Vector3f &b_t1,
    const Eigen::Vector3f &c_t1, const Eigen::Vector3f &d_t1,
    const std::array<Interval, 3> &iset, const Eigen::Array3f &tol,
    float co_domain_tolerance, const Eigen::Array3f &err, float ms,
    long max_iter, bool is_unit_interval);

}  // namespace silk::cuda

#include "backend/cpu/collision/ccd.hpp"

#include <Eigen/Core>
#include <array>
#include <cassert>
#include <limits>
#include <optional>

#include "backend/cpu/collision/interval.hpp"
#include "backend/cpu/collision/interval_root_finder.hpp"

namespace silk::cpu {

constexpr float CCD_MAX_TIME_TOL = std::numeric_limits<float>::infinity();
constexpr float CCD_MAX_COORD_TOL = std::numeric_limits<float>::infinity();

// Compute axis-aligned bounds for four points.
std::array<Eigen::Vector3f, 2> bbd_4_pts(const Eigen::Vector3f &p0,
                                         const Eigen::Vector3f &p1,
                                         const Eigen::Vector3f &p2,
                                         const Eigen::Vector3f &p3) {
  return {{p0.cwiseMin(p1).cwiseMin(p2).cwiseMin(p3),
           p0.cwiseMax(p1).cwiseMax(p2).cwiseMax(p3)}};
}

// Calculate the maximum per-axis span across two AABBs and their separation.
float get_max_axis_diff(const std::array<Eigen::Vector3f, 2> &b1,
                        const std::array<Eigen::Vector3f, 2> &b2) {
  return std::max({
      (b1[1] - b1[0]).maxCoeff(),
      (b2[1] - b2[0]).maxCoeff(),
      (b2[0] - b1[1]).cwiseAbs().maxCoeff(),
      (b1[0] - b2[1]).cwiseAbs().maxCoeff(),
  });
}

float max_linf_4(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2,
                 const Eigen::Vector3f &p3, const Eigen::Vector3f &p4,
                 const Eigen::Vector3f &p1e, const Eigen::Vector3f &p2e,
                 const Eigen::Vector3f &p3e, const Eigen::Vector3f &p4e) {
  return std::max({(p1e - p1).lpNorm<Eigen::Infinity>(),
                   (p2e - p2).lpNorm<Eigen::Infinity>(),
                   (p3e - p3).lpNorm<Eigen::Infinity>(),
                   (p4e - p4).lpNorm<Eigen::Infinity>()});
}

/// @brief Saturating division helper.
/// @param a Numerator.
/// @param b Denominator.
/// @param max_val Upper clamp when `b == 0`.
/// @return `a/b` if `b != 0`, otherwise `max_val`.
float clamp_div(float a, float b, float max_val) {
  if (b == 0) {
    return max_val;
  } else {
    return std::min(a / b, max_val);
  }
}

constexpr float DEFAULT_CCD_DISTANCE_TOL = 1e-6;

/**
 * @brief Map a distance tolerance to {t,u,v} tolerances for vertex–face CCD.
 */
Eigen::Array3f compute_vertex_face_tolerances(
    const Eigen::Vector3f &v_t0, const Eigen::Vector3f &f0_t0,
    const Eigen::Vector3f &f1_t0, const Eigen::Vector3f &f2_t0,
    const Eigen::Vector3f &v_t1, const Eigen::Vector3f &f0_t1,
    const Eigen::Vector3f &f1_t1, const Eigen::Vector3f &f2_t1,
    float distance_tolerance = DEFAULT_CCD_DISTANCE_TOL) {
  const Eigen::Vector3f p000 = v_t0 - f0_t0;
  const Eigen::Vector3f p001 = v_t0 - f2_t0;
  const Eigen::Vector3f p011 = v_t0 - (f1_t0 + f2_t0 - f0_t0);
  const Eigen::Vector3f p010 = v_t0 - f1_t0;
  const Eigen::Vector3f p100 = v_t1 - f0_t1;
  const Eigen::Vector3f p101 = v_t1 - f2_t1;
  const Eigen::Vector3f p111 = v_t1 - (f1_t1 + f2_t1 - f0_t1);
  const Eigen::Vector3f p110 = v_t1 - f1_t1;

  float dl = 3 * max_linf_4(p000, p001, p011, p010, p100, p101, p111, p110);
  float edge0_length =
      3 * max_linf_4(p000, p100, p101, p001, p010, p110, p111, p011);
  float edge1_length =
      3 * max_linf_4(p000, p100, p110, p010, p001, p101, p111, p011);

  return Eigen::Array3f(
      clamp_div(distance_tolerance, dl, CCD_MAX_TIME_TOL),
      clamp_div(distance_tolerance, edge0_length, CCD_MAX_COORD_TOL),
      clamp_div(distance_tolerance, edge1_length, CCD_MAX_COORD_TOL));
}

/**
 * @brief Map a distance tolerance to {t,u,v} tolerances for edge–edge CCD.
 */
Eigen::Array3f compute_edge_edge_tolerances(
    const Eigen::Vector3f &ea0_t0, const Eigen::Vector3f &ea1_t0,
    const Eigen::Vector3f &eb0_t0, const Eigen::Vector3f &eb1_t0,
    const Eigen::Vector3f &ea0_t1, const Eigen::Vector3f &ea1_t1,
    const Eigen::Vector3f &eb0_t1, const Eigen::Vector3f &eb1_t1,
    float distance_tolerance = DEFAULT_CCD_DISTANCE_TOL) {
  Eigen::Vector3f p000 = ea0_t0 - eb0_t0;
  Eigen::Vector3f p001 = ea0_t0 - eb1_t0;
  Eigen::Vector3f p010 = ea1_t0 - eb0_t0;
  Eigen::Vector3f p011 = ea1_t0 - eb1_t0;
  Eigen::Vector3f p100 = ea0_t1 - eb0_t1;
  Eigen::Vector3f p101 = ea0_t1 - eb1_t1;
  Eigen::Vector3f p110 = ea1_t1 - eb0_t1;
  Eigen::Vector3f p111 = ea1_t1 - eb1_t1;

  float dl = 3 * max_linf_4(p000, p001, p011, p010, p100, p101, p111, p110);
  float edge0_length =
      3 * max_linf_4(p000, p100, p101, p001, p010, p110, p111, p011);
  float edge1_length =
      3 * max_linf_4(p000, p100, p110, p010, p001, p101, p111, p011);

  return Eigen::Array3f(
      clamp_div(distance_tolerance, dl, CCD_MAX_TIME_TOL),
      clamp_div(distance_tolerance, edge0_length, CCD_MAX_COORD_TOL),
      clamp_div(distance_tolerance, edge1_length, CCD_MAX_COORD_TOL));
}

template <bool is_vertex_face>
std::optional<CCDResult> CCD(
    const Eigen::Vector3f &a_t0, const Eigen::Vector3f &b_t0,
    const Eigen::Vector3f &c_t0, const Eigen::Vector3f &d_t0,
    const Eigen::Vector3f &a_t1, const Eigen::Vector3f &b_t1,
    const Eigen::Vector3f &c_t1, const Eigen::Vector3f &d_t1,
    const Eigen::Array3f &err, float ms, float tolerance, long max_itr,
    bool no_zero_toi) {
  // Compute co-domain tolerance.
  Eigen::Array3f tol;
  if constexpr (is_vertex_face) {
    tol = compute_vertex_face_tolerances(a_t0, b_t0, c_t0, d_t0, a_t1, b_t1,
                                         c_t1, d_t1, tolerance);
  } else {
    tol = compute_edge_edge_tolerances(a_t0, b_t0, c_t0, d_t0, a_t1, b_t1, c_t1,
                                       d_t1, tolerance);
  }

  // Initialize unit domain for t, u, and v.
  Interval zero_to_one{0.0f, 1.0f};
  std::array<Interval, 3> iset = {{
      zero_to_one,
      zero_to_one,
      zero_to_one,
  }};

  // Breadth-first interval root finding in (t,u,v).
  auto result = interval_root_finder_BFS<is_vertex_face>(
      a_t0, b_t0, c_t0, d_t0, a_t1, b_t1, c_t1, d_t1, iset, tol, tolerance, err,
      ms, max_itr, true);

  if (!no_zero_toi) {
    return result;
  }

  if (!result || result->t(0) > 0.0f) {
    return result;
  }

  // Avoid returning t=0 by refining minimum separation if needed.
  constexpr int MS_REFINE_ITER = 1;
  for (int i = 0; i < MS_REFINE_ITER; ++i) {
    ms *= 0.5f;
    auto refine_result = interval_root_finder_BFS<is_vertex_face>(
        a_t0, b_t0, c_t0, d_t0, a_t1, b_t1, c_t1, d_t1, iset, tol, tolerance,
        err, ms, max_itr, true);

    if (!refine_result) {
      result->use_small_ms = true;
      result->small_ms_t = {1.0f, 1.0f};
      return result;
    } else if (refine_result->t(0) > 0.0f) {
      result->use_small_ms = true;
      result->small_ms_t = refine_result->t;
      return result;
    }
  }

  result->use_small_ms = true;
  result->small_ms_t = {0.0f, 0.0f};
  return result;
}

std::optional<CCDResult> edge_edge_ccd(
    const Eigen::Vector3f &ea0_t0, const Eigen::Vector3f &ea1_t0,
    const Eigen::Vector3f &eb0_t0, const Eigen::Vector3f &eb1_t0,
    const Eigen::Vector3f &ea0_t1, const Eigen::Vector3f &ea1_t1,
    const Eigen::Vector3f &eb0_t1, const Eigen::Vector3f &eb1_t1,
    const Eigen::Array3f &err, float ms, float tolerance, long max_itr,
    bool no_zero_toi) {
  return CCD</*is_vertex_face=*/false>(ea0_t0, ea1_t0, eb0_t0, eb1_t0, ea0_t1,
                                       ea1_t1, eb0_t1, eb1_t1, err, ms,
                                       tolerance, max_itr, no_zero_toi);
}

std::optional<CCDResult> vertex_face_ccd(
    const Eigen::Vector3f &v_t0, const Eigen::Vector3f &f0_t0,
    const Eigen::Vector3f &f1_t0, const Eigen::Vector3f &f2_t0,
    const Eigen::Vector3f &v_t1, const Eigen::Vector3f &f0_t1,
    const Eigen::Vector3f &f1_t1, const Eigen::Vector3f &f2_t1,
    const Eigen::Array3f &err, float ms, float tolerance, long max_itr,
    bool no_zero_toi) {
  return CCD</*is_vertex_face=*/true>(v_t0, f0_t0, f1_t0, f2_t0, v_t1, f0_t1,
                                      f1_t1, f2_t1, err, ms, tolerance, max_itr,
                                      no_zero_toi);
}

}  // namespace silk::cpu

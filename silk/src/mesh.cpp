#include "mesh.hpp"

#include <igl/edges.h>
#include <igl/facet_components.h>
#include <igl/internal_angles.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>

#include <Eigen/Core>

#include "logger.hpp"

namespace silk {

/**
 * @brief Validates basic schema requirements for mesh data.
 *
 * Ensures the input data has valid pointers, correct array sizing (multiples of
 * 3 for XYZ coordinates and triangle indices), and meets minimum geometry
 * requirements.
 *
 * @param mc Input mesh configuration to validate
 * @param vnum Minimal vertex count
 * @param fnum Minimal face count
 * @return true if schema is valid, false otherwise
 */
bool check_schema(const MeshConfig& mc, int min_vnum, int min_fnum) {
  if (mc.verts.data == nullptr || mc.faces.data == nullptr) {
    SPDLOG_WARN("Invalid mesh: null pointer(s) in verts or faces.");
    return false;
  }

  // Vertex data is packed as [x0,y0,z0,x1,y1,z1,...], faces as
  // [i0,j0,k0,i1,j1,k1,...].
  if (mc.verts.size % 3 != 0 || mc.faces.size % 3 != 0) {
    SPDLOG_WARN(
        "Invalid mesh: verts.num ({}) and faces.num ({}) must be multiples of "
        "3.",
        mc.verts.size, mc.faces.size);
    return false;
  }

  int vnum = mc.verts.size / 3;
  int fnum = mc.faces.size / 3;
  if (vnum < min_vnum) {
    SPDLOG_WARN("Invalid mesh: need at least {} vertices, got {}.", min_vnum,
                vnum);
    return false;
  }
  if (fnum < min_fnum) {
    SPDLOG_WARN("Invalid mesh: need at least {} faces, got {}.", min_fnum,
                fnum);
    return false;
  }

  return true;
}

/**
 * @brief Validates that all vertex positions are finite numbers.
 *
 * @param V Vertex position matrix (#vertices × 3)
 * @return true if all positions are finite, false if any NaN/Inf detected
 */
bool check_finite_positions(const RMatrixX3f& V) {
  if (!V.allFinite()) {
    SPDLOG_WARN("Invalid mesh: vertex positions contain NaN/Inf.");
    return false;
  }
  return true;
}

/**
 * @brief Validates face indexing integrity.
 *
 * Ensures all face indices are within valid vertex range [0, vnum) and that
 * no triangle has repeated vertex indices.
 *
 * @param F Face index matrix (#faces × 3)
 * @param vnum Total number of vertices for bounds checking
 * @return true if indexing is valid, false if out-of-bounds or degenerate faces
 * found
 */
bool check_indexing(const RMatrixX3i& F, int vnum) {
  int min_idx = F.minCoeff();
  int max_idx = F.maxCoeff();
  if (min_idx < 0 || max_idx >= vnum) {
    SPDLOG_WARN(
        "Invalid mesh: face indices out of range [0, {}). min={}, max={}", vnum,
        min_idx, max_idx);
    return false;
  }

  // Check for degenerate triangles with repeated vertex indices.
  for (int f = 0; f < F.rows(); ++f) {
    int a = F(f, 0);
    int b = F(f, 1);
    int c = F(f, 2);
    if (a == b || b == c || c == a) {
      SPDLOG_WARN(
          "Invalid mesh: triangle {} has repeated vertex indices ({}, {}, {}).",
          f, a, b, c);
      return false;
    }
  }

  return true;
}

/**
 * @brief Validates triangle angle quality via angle check.
 *
 * Ensures all triangle internal angles fall within [min_degree,
 * 180-min_degree].
 *
 * @param V Vertex position matrix (#vertices × 3)
 * @param F Face index matrix (#faces × 3)
 * @param min_degree Minimum allowed angle in degrees (default: 0.5°)
 * @return true if all angles are within acceptable bounds, false otherwise
 */
bool check_triangle_angles_min(const RMatrixX3f& V, const RMatrixX3i& F,
                               float min_degree = 0.5f) {
  static constexpr float PI = 3.14159265358979323846f;

  Eigen::MatrixXf K;  // #faces × 3 matrix of internal angles in radians
  igl::internal_angles(V, F, K);

  float min_rad = min_degree * PI / 180.0f;
  float max_rad = PI - min_rad;  // Complement angle threshold
  float K_min = K.minCoeff();
  float K_max = K.maxCoeff();
  if (K_min < min_rad || K_max > max_rad) {
    SPDLOG_WARN(
        "Invalid mesh: poor triangle angle quality. Min angle {:.2f}°, Max "
        "angle "
        "{:.2f}°.",
        180.0f * K_min / PI, 180.0f * K_max / PI);
    return false;
  }

  return true;
}

/**
 * @brief Validates edge manifoldness.
 *
 * @param F Face index matrix (#faces × 3)
 * @return true if mesh is edge-manifold, false otherwise
 */
bool check_edge_manifold(const RMatrixX3i& F) {
  if (!igl::is_edge_manifold(F)) {
    SPDLOG_WARN(
        "Invalid mesh: non-manifold edges detected "
        "(igl::is_edge_manifold=false).");
    return false;
  }
  return true;
}

/**
 * @brief Validates vertex manifoldness.
 *
 * @param F Face index matrix (#faces × 3)
 * @return true if mesh is vertex-manifold, false otherwise
 */
bool check_vertex_manifold(const RMatrixX3i& F) {
  if (!igl::is_vertex_manifold(F)) {
    SPDLOG_WARN(
        "Invalid mesh: non-manifold vertices detected "
        "(igl::is_vertex_manifold=false).");
    return false;
  }
  return true;
}

/**
 * @brief Validates that the mesh forms a single connected component.
 *
 * @param F Face index matrix (#faces × 3)
 * @return true if mesh has exactly one connected component, false otherwise
 */
bool check_single_component(const RMatrixX3i& F) {
  Eigen::VectorXi C;  // Per-face component labels
  int num = igl::facet_components(F, C);
  if (num != 1) {
    SPDLOG_WARN(
        "Invalid mesh: expected a single connected component, found {}.", num);
    return false;
  }
  return true;
}

std::optional<TriMesh> make_cloth_mesh(MeshConfig mesh_config) {
  if (!check_schema(mesh_config, 3, 1)) {
    return std::nullopt;
  }
  int vnum = mesh_config.verts.size / 3;
  int fnum = mesh_config.faces.size / 3;

  RMatrixX3f V = Eigen::Map<const RMatrixX3f>(mesh_config.verts.data, vnum, 3);
  RMatrixX3i F = Eigen::Map<const RMatrixX3i>(mesh_config.faces.data, fnum, 3);

  if (!check_finite_positions(V)) {
    return std::nullopt;
  }
  if (!check_indexing(F, vnum)) {
    return std::nullopt;
  }
  if (!check_triangle_angles_min(V, F)) {
    return std::nullopt;
  }
  if (!check_edge_manifold(F)) {
    return std::nullopt;
  }
  if (!check_vertex_manifold(F)) {
    return std::nullopt;
  }
  if (!check_single_component(F)) {
    return std::nullopt;
  }

  TriMesh m;
  m.V = std::move(V);
  igl::edges(F, m.E);
  m.F = std::move(F);

  m.avg_edge_length = 0.0f;
  for (int i = 0; i < m.E.rows(); ++i) {
    int idx0 = m.E(i, 0);
    int idx1 = m.E(i, 1);
    m.avg_edge_length += (m.V.row(idx0) - m.V.row(idx1)).norm();
  }
  m.avg_edge_length /= static_cast<float>(m.E.rows());

  return m;
}

std::optional<TriMesh> make_obstacle_mesh(MeshConfig mesh_config) {
  if (!check_schema(mesh_config, 1, 0)) {
    return std::nullopt;
  }
  int vnum = mesh_config.verts.size / 3;
  int fnum = mesh_config.faces.size / 3;

  RMatrixX3f V = Eigen::Map<const RMatrixX3f>(mesh_config.verts.data, vnum, 3);
  RMatrixX3i F = Eigen::Map<const RMatrixX3i>(mesh_config.faces.data, fnum, 3);

  if (!check_finite_positions(V)) {
    return std::nullopt;
  }
  if (!check_indexing(F, vnum)) {
    return std::nullopt;
  }

  TriMesh m;
  m.V = std::move(V);
  igl::edges(F, m.E);
  m.F = std::move(F);

  m.avg_edge_length = 0.0f;
  for (int i = 0; i < m.E.rows(); ++i) {
    int idx0 = m.E(i, 0);
    int idx1 = m.E(i, 1);
    m.avg_edge_length += (m.V.row(idx0) - m.V.row(idx1)).norm();
  }
  m.avg_edge_length /= static_cast<float>(m.E.rows());

  return m;
}

}  // namespace silk

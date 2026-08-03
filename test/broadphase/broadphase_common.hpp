#pragma once

#include <Eigen/Core>
#include <array>
#include <filesystem>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace silk::broadphase_benchmark {

using Pair = std::pair<int, int>;

struct Box {
  Eigen::Vector3f min;
  Eigen::Vector3f max;
  std::array<int, 3> vertex_ids = {-1, -1, -1};
  int id = -1;  /// primitive id.
};

enum class QueryKind { EE, VF };

/// Broadphase query.
/// If kind == EE, group_a is edge bboxes and group_b is empty.
/// If kind == VF, group_a is vertex bboxes and group_b is face bboxes.
struct QueryInput {
  QueryKind kind = QueryKind::EE;
  std::span<const Box> group_a;
  std::span<const Box> group_b;
  std::span<const Pair> required_pairs;
};

/// A single test case. Represent one time step in one scene.
struct TestCase {
  std::string scene;
  int timestep = -1;
  Eigen::MatrixXd vertices_t0;
  Eigen::MatrixXd vertices_t1;
  std::vector<Box> vertex_boxes;
  std::vector<Box> edge_boxes;
  std::vector<Box> face_boxes;
  std::vector<Pair> required_ee;
  std::vector<Pair> required_vf;
};

/// File and spec related to a test scene.
struct SceneFiles {
  std::string name;
  int first_timestep = 0;
  /// Path to .ply storing vertex position.
  std::vector<std::filesystem::path> frames;
  /// Path to json EE collision cadidate ground truth.
  std::vector<std::filesystem::path> boxes_ee;
  /// Path to json VF collision cadidate ground truth.
  std::vector<std::filesystem::path> boxes_vf;
};

/// Resolve dataset files.
std::vector<SceneFiles> resolve_scene_files(
    const std::filesystem::path& data_root);

/// Load test data for a single time step.
TestCase load_case(const SceneFiles& scene, int timestep_index);

/// Make EE/VF briadphase query.
QueryInput make_query(const TestCase& test_case, QueryKind kind);

inline bool is_boxes_overlaping(const Box& a, const Box& b) {
  return (a.min.array() <= b.max.array()).all() &&
         (b.min.array() <= a.max.array()).all();
}

inline bool is_sharing_vertex(const Box& a, const Box& b) {
  for (int va : a.vertex_ids) {
    if (va < 0) {
      continue;
    }
    for (int vb : b.vertex_ids) {
      if (va == vb) {
        return true;
      }
    }
  }
  return false;
}

/// Ensure EE pair is sorted in ascending order.
inline Pair normalized_pair(Pair pair, QueryKind kind) {
  if (kind == QueryKind::EE && pair.second < pair.first) {
    std::swap(pair.first, pair.second);
  }
  return pair;
}

}  // namespace silk::broadphase_benchmark

#include "narrowphase_common.hpp"

#include <igl/read_triangle_mesh.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../broadphase/cpu_adapters.hpp"
#include "backend/cpu/collision/interval_root_finder.hpp"

namespace silk::narrowphase_benchmark {

namespace {

using broadphase_benchmark::Pair;

struct PairHash {
  std::size_t operator()(const Pair& pair) const {
    return std::hash<int>{}(pair.first) ^ (std::hash<int>{}(pair.second) << 1);
  }
};

using ExpectedMap = std::unordered_map<Pair, bool, PairHash>;

ExpectedMap load_expected(const SceneFiles& scene_files,
                          const broadphase_benchmark::TestCase& test_case,
                          QueryKind kind, int timestep_index,
                          std::vector<Pair>* dataset_pairs) {
  const std::filesystem::path& boxes_path =
      kind == QueryKind::EE ? scene_files.boxes_ee[timestep_index]
                            : scene_files.boxes_vf[timestep_index];
  std::filesystem::path label_path = boxes_path.parent_path().parent_path() /
                                     "mma_bool" /
                                     (std::to_string(test_case.timestep) +
                                      query_name(kind) + "_mma_bool.json");
  std::ifstream boxes_input(boxes_path);
  if (!boxes_input) {
    throw std::runtime_error("Unable to load pairs from " +
                             boxes_path.string());
  }

  nlohmann::json pairs = nlohmann::json::parse(boxes_input);
  if (pairs.empty()) {
    return {};
  }
  std::ifstream labels_input(label_path);
  if (!labels_input) {
    throw std::runtime_error("Unable to load labels for " +
                             boxes_path.string());
  }
  std::vector<bool> labels =
      nlohmann::json::parse(labels_input).get<std::vector<bool>>();
  if (pairs.size() != labels.size()) {
    throw std::runtime_error("Pair and label counts differ in " +
                             boxes_path.string());
  }

  int vertex_num = test_case.vertex_boxes.size();
  int edge_num = test_case.edge_boxes.size();
  int face_offset = vertex_num + edge_num;
  ExpectedMap expected;
  expected.reserve(pairs.size());
  if (dataset_pairs != nullptr) {
    dataset_pairs->reserve(pairs.size());
  }
  for (int i = 0; i < pairs.size(); ++i) {
    Pair pair{pairs[i][0].get<int>(), pairs[i][1].get<int>()};
    if (kind == QueryKind::EE) {
      pair.first -= vertex_num;
      pair.second -= vertex_num;
      if (pair.second < pair.first) {
        std::swap(pair.first, pair.second);
      }
    } else {
      if (pair.first >= face_offset) {
        std::swap(pair.first, pair.second);
      }
      pair.second -= face_offset;
    }
    if (dataset_pairs != nullptr) {
      dataset_pairs->push_back(pair);
    }
    expected.emplace(pair, labels[i]);
  }
  return expected;
}

Eigen::Vector3f vertex(const Eigen::MatrixXd& vertices, int vertex_id) {
  return vertices.row(vertex_id).cast<float>();
}

Query make_query(const broadphase_benchmark::TestCase& test_case,
                 QueryKind kind, Pair pair, const Eigen::Array3f& err,
                 int query_index, const ExpectedMap& expected) {
  const broadphase_benchmark::Box& primitive_a =
      kind == QueryKind::EE ? test_case.edge_boxes[pair.first]
                            : test_case.vertex_boxes[pair.first];
  const broadphase_benchmark::Box& primitive_b =
      kind == QueryKind::EE ? test_case.edge_boxes[pair.second]
                            : test_case.face_boxes[pair.second];

  Query query;
  query.err = err;
  query.timestep = test_case.timestep;
  query.timestep_query = query_index;
  for (int frame = 0; frame < 2; ++frame) {
    const Eigen::MatrixXd& vertices =
        frame == 0 ? test_case.vertices_t0 : test_case.vertices_t1;
    int offset = frame * 4;
    if (kind == QueryKind::EE) {
      query.vertices[offset] = vertex(vertices, primitive_a.vertex_ids[0]);
      query.vertices[offset + 1] = vertex(vertices, primitive_a.vertex_ids[1]);
      query.vertices[offset + 2] = vertex(vertices, primitive_b.vertex_ids[0]);
      query.vertices[offset + 3] = vertex(vertices, primitive_b.vertex_ids[1]);
    } else {
      query.vertices[offset] = vertex(vertices, primitive_a.vertex_ids[0]);
      for (int i = 0; i < 3; ++i) {
        query.vertices[offset + i + 1] =
            vertex(vertices, primitive_b.vertex_ids[i]);
      }
    }
  }
  auto found = expected.find(pair);
  if (found != expected.end()) {
    query.expected = found->second;
    query.expected_known = true;
  }
  return query;
}

}  // namespace

std::vector<SceneFiles> resolve_scene_files(
    const std::filesystem::path& data_root) {
  return broadphase_benchmark::resolve_scene_files(data_root);
}

SceneBounds load_scene_bounds(const SceneFiles& scene_files) {
  Eigen::Vector3d scene_min =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d scene_max =
      Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  bool found_vertex = false;
  for (const std::filesystem::path& frame : scene_files.frames) {
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    if (!igl::read_triangle_mesh(frame.string(), vertices, faces)) {
      throw std::runtime_error("Unable to read scene frame " + frame.string());
    }
    if (vertices.rows() == 0) {
      continue;
    }
    scene_min = scene_min.cwiseMin(vertices.colwise().minCoeff().transpose());
    scene_max = scene_max.cwiseMax(vertices.colwise().maxCoeff().transpose());
    found_vertex = true;
  }
  if (!found_vertex) {
    throw std::runtime_error("Scene contains no vertices: " + scene_files.name);
  }

  SceneBounds result;
  for (int axis = 0; axis < 3; ++axis) {
    result.min(axis) = std::nextafter(static_cast<float>(scene_min(axis)),
                                      -std::numeric_limits<float>::infinity());
    result.max(axis) = std::nextafter(static_cast<float>(scene_max(axis)),
                                      std::numeric_limits<float>::infinity());
  }
  return result;
}

QueryBatch load_batch(const SceneFiles& scene_files, QueryKind kind,
                      const SceneBounds& scene_bounds, int timestep_index,
                      broadphase_benchmark::CpuAdapter* broadphase) {
  broadphase_benchmark::TestCase test_case =
      broadphase_benchmark::load_case(scene_files, timestep_index);

  Eigen::Vector3f abs_max =
      scene_bounds.min.cwiseAbs().cwiseMax(scene_bounds.max.cwiseAbs());
  Eigen::Array3f err = cpu::get_numerical_error(abs_max, kind == QueryKind::VF);
  std::vector<Pair> pairs;
  ExpectedMap expected =
      load_expected(scene_files, test_case, kind, timestep_index,
                    broadphase == nullptr ? &pairs : nullptr);

  if (broadphase != nullptr) {
    broadphase_benchmark::QueryKind broadphase_kind =
        kind == QueryKind::EE ? broadphase_benchmark::QueryKind::EE
                              : broadphase_benchmark::QueryKind::VF;
    broadphase_benchmark::QueryInput input =
        broadphase_benchmark::make_query(test_case, broadphase_kind);
    broadphase->prepare(input);
    broadphase->build();
    broadphase->clear_output();
    broadphase->query();
    broadphase->materialize_output();
    std::span<const Pair> broadphase_pairs = broadphase->output();
    pairs.assign(broadphase_pairs.begin(), broadphase_pairs.end());
  }

  QueryBatch batch{.scene = scene_files.name, .kind = kind, .timestep_num = 1};
  batch.queries.reserve(pairs.size());
  for (int i = 0; i < pairs.size(); ++i) {
    batch.queries.push_back(
        make_query(test_case, kind, pairs[i], err, i, expected));
  }
  return batch;
}

std::string query_name(QueryKind kind) {
  return kind == QueryKind::EE ? "ee" : "vf";
}

float compute_minimum_separation(const Query&, QueryKind) { return 0.0f; }

}  // namespace silk::narrowphase_benchmark

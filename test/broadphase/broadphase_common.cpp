#include "broadphase_common.hpp"

#include <igl/edges.h>
#include <igl/read_triangle_mesh.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace silk::broadphase_benchmark {

namespace {

// Metadata for test scene files.
struct DataSetDesc {
  const char* name;
  int first_timestep;
  int timestep_num;
  const char* frame_prefix;
  bool pad_frame_number;
};

constexpr std::array DATASET = {
    DataSetDesc{"armadillo-rollers", 0, 400, "", false},
    DataSetDesc{"cloth-ball", 0, 93, "cloth_ball", false},
    DataSetDesc{"cloth-funnel", 1, 499, "", true},
    DataSetDesc{"n-body-simulation", 0, 75, "balls16_", false},
    DataSetDesc{"puffer-ball", 0, 130, "", false},
    DataSetDesc{"rod-twist", 0, 4000, "", false},
};

std::string frame_filename(const DataSetDesc& scene, int timestep) {
  std::string number = scene.pad_frame_number ? std::format("{:03}", timestep)
                                              : std::to_string(timestep);
  return std::string(scene.frame_prefix) + number + ".ply";
}

/// conservative round down fp64 to fp32.
float round_down(double value) {
  float result = static_cast<float>(value);
  return std::nextafter(result, -std::numeric_limits<float>::infinity());
}

/// conservative round up fp64 to fp32.
float round_up(double value) {
  float result = static_cast<float>(value);
  return std::nextafter(result, std::numeric_limits<float>::infinity());
}

/// @brief Make conservative bbox for point, edge, or triangle.
/// @param vertices_t0 Position at t0 stored in (vert num x 3) matrix.
/// @param vertices_t1 Position at t0 stored in (vert num x 3) matrix.
/// @param vertex_ids Vertex id.
/// @param vertex_num Vertex num, 1 for point, 2 for edge, 3 for triangle.
/// @param id Primitive id.
Box make_box(const Eigen::MatrixXd& vertices_t0,
             const Eigen::MatrixXd& vertices_t1,
             const std::array<int, 3>& vertex_ids, int vertex_num, int id) {
  Eigen::Vector3d min =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d max =
      Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (int i = 0; i < vertex_num; ++i) {
    int vertex_id = vertex_ids[i];
    min = min.cwiseMin(vertices_t0.row(vertex_id).transpose());
    min = min.cwiseMin(vertices_t1.row(vertex_id).transpose());
    max = max.cwiseMax(vertices_t0.row(vertex_id).transpose());
    max = max.cwiseMax(vertices_t1.row(vertex_id).transpose());
  }

  Box box;
  for (int axis = 0; axis < 3; ++axis) {
    box.min(axis) = round_down(min(axis));
    box.max(axis) = round_up(max(axis));
  }
  box.vertex_ids = vertex_ids;
  box.id = id;
  return box;
}

/// @brief Load broadphase ground truth collision pair from file.
/// @param path Path to ground truth json.
/// @param kind EE or VF.
/// @param vertex_num Vertex num.
/// @param edge_num Edge num.
std::vector<Pair> load_pairs(const std::filesystem::path& path, QueryKind kind,
                             int vertex_num, int edge_num) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Unable to open " + path.string());
  }

  nlohmann::json json = nlohmann::json::parse(input);
  std::vector<Pair> pairs;
  // Ground truth json stores candidate primitive id pair. For example,
  // [ [32, 11], [573, 2] ]
  //
  // Primitive id is layouted as:
  // [vertices] [edges] [faces]
  //  0  ...V-1
  //            V...V+E-1
  //                   V+E...V+E+F-1
  // We index point/edge/face separately, so offsets are subtracted.
  if (kind == QueryKind::EE) {
    for (const auto& item : json) {
      int a = item.at(0).get<int>();
      int b = item.at(1).get<int>();
      a -= vertex_num;
      b -= vertex_num;
      if (b < a) {
        std::swap(a, b);
      }
      pairs.emplace_back(a, b);
    }
  } else {
    int face_offset = vertex_num + edge_num;
    for (const auto& item : json) {
      int a = item.at(0).get<int>();
      int b = item.at(1).get<int>();
      if (a >= face_offset) {
        std::swap(a, b);
      }
      b -= face_offset;
      pairs.emplace_back(a, b);
    }
  }
  std::sort(pairs.begin(), pairs.end());
  return pairs;
}

}  // namespace

std::vector<SceneFiles> resolve_scene_files(
    const std::filesystem::path& data_root) {
  if (!std::filesystem::is_directory(data_root)) {
    throw std::runtime_error("Dataset directory does not exist: " +
                             data_root.string());
  }

  std::vector<SceneFiles> scenes;
  scenes.reserve(DATASET.size());
  for (const DataSetDesc& desc : DATASET) {
    const std::filesystem::path scene_root = data_root / desc.name;
    const std::filesystem::path frames_root = scene_root / "frames";
    const std::filesystem::path boxes_root = scene_root / "boxes";
    SceneFiles scene{.name = desc.name, .first_timestep = desc.first_timestep};
    scene.frames.reserve(desc.timestep_num + 1);
    scene.boxes_ee.reserve(desc.timestep_num);
    scene.boxes_vf.reserve(desc.timestep_num);
    for (int i = 0; i <= desc.timestep_num; ++i) {
      const int timestep = desc.first_timestep + i;
      scene.frames.push_back(frames_root / frame_filename(desc, timestep));
    }
    for (int i = 0; i < desc.timestep_num; ++i) {
      const int timestep = desc.first_timestep + i;
      scene.boxes_ee.push_back(boxes_root /
                               (std::to_string(timestep) + "ee.json"));
      scene.boxes_vf.push_back(boxes_root /
                               (std::to_string(timestep) + "vf.json"));
      if (!std::filesystem::is_regular_file(scene.frames[i]) ||
          !std::filesystem::is_regular_file(scene.frames[i + 1]) ||
          !std::filesystem::is_regular_file(scene.boxes_ee[i]) ||
          !std::filesystem::is_regular_file(scene.boxes_vf[i])) {
        throw std::runtime_error("Incomplete dataset timestep " + scene.name +
                                 "/" + std::to_string(timestep));
      }
    }
    scenes.push_back(std::move(scene));
  }
  return scenes;
}

TestCase load_case(const SceneFiles& scene, int timestep_index) {
  const int timestep = scene.first_timestep + timestep_index;
  Eigen::MatrixXd vertices_t0;
  Eigen::MatrixXd vertices_t1;
  Eigen::MatrixXi faces_t0;
  Eigen::MatrixXi faces_t1;
  if (!igl::read_triangle_mesh(scene.frames[timestep_index].string(),
                               vertices_t0, faces_t0) ||
      !igl::read_triangle_mesh(scene.frames[timestep_index + 1].string(),
                               vertices_t1, faces_t1)) {
    throw std::runtime_error("Unable to read frames for " + scene.name + "/" +
                             std::to_string(timestep));
  }
  if (vertices_t0.rows() != vertices_t1.rows() ||
      faces_t0.rows() != faces_t1.rows() ||
      faces_t0.cols() != faces_t1.cols() ||
      (faces_t0.array() != faces_t1.array()).any()) {
    throw std::runtime_error("Mesh topology changed in " + scene.name + "/" +
                             std::to_string(timestep));
  }

  Eigen::MatrixXi edges;
  igl::edges(faces_t0, edges);

  TestCase result;
  result.scene = scene.name;
  result.timestep = timestep;
  result.vertex_boxes.reserve(vertices_t0.rows());
  result.edge_boxes.reserve(edges.rows());
  result.face_boxes.reserve(faces_t0.rows());

  for (int i = 0; i < vertices_t0.rows(); ++i) {
    result.vertex_boxes.push_back(
        make_box(vertices_t0, vertices_t1, {i, -1, -1}, 1, i));
  }
  for (int i = 0; i < edges.rows(); ++i) {
    result.edge_boxes.push_back(make_box(vertices_t0, vertices_t1,
                                         {edges(i, 0), edges(i, 1), -1}, 2, i));
  }
  for (int i = 0; i < faces_t0.rows(); ++i) {
    result.face_boxes.push_back(
        make_box(vertices_t0, vertices_t1,
                 {faces_t0(i, 0), faces_t0(i, 1), faces_t0(i, 2)}, 3, i));
  }

  result.required_ee = load_pairs(scene.boxes_ee[timestep_index], QueryKind::EE,
                                  vertices_t0.rows(), edges.rows());
  result.required_vf = load_pairs(scene.boxes_vf[timestep_index], QueryKind::VF,
                                  vertices_t0.rows(), edges.rows());
  return result;
}

QueryInput make_query(const TestCase& test_case, QueryKind kind) {
  if (kind == QueryKind::EE) {
    return {kind, test_case.edge_boxes, {}, test_case.required_ee};
  }
  return {kind, test_case.vertex_boxes, test_case.face_boxes,
          test_case.required_vf};
}

}  // namespace silk::broadphase_benchmark

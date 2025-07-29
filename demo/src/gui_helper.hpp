#pragma once

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

#include <Eigen/Core>
#include <cstdint>
#include <silk/silk.hpp>
#include <vector>

enum class SilkObjectType : int { None = 0, Cloth = 1, Obstacle = 2 };

class Object {
 public:
  std::string name;

  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> V;
  Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> F;
  polyscope::SurfaceMesh* mesh;
  int vert_num = 0;
  int edge_num = 0;
  int face_num = 0;

  SilkObjectType type = SilkObjectType::None;
  uint32_t silk_handle = 0;

  std::unordered_set<int> pin_group;
  std::vector<int> pin_index;
  silk::ClothConfig cloth_config;
  silk::CollisionConfig collision_config;

  bool pinned_changed = true;
  bool physical_config_changed = true;
  bool collision_config_changed = true;

  glm::mat4x4 old_transformation;
};

enum class UIMode { Normal, Paint, Sim };
// clang-format off
// clang-format on

struct Context {
  UIMode ui_mode = UIMode::Normal;
  int selection = -1;
  std::vector<Object> objects;
  silk::GlobalConfig global_config;
  silk::World silk_world;
};

class IWidget {
 public:
  virtual ~IWidget() = default;
  virtual void draw() = 0;
};

void update_mesh_stat(Object& obj);

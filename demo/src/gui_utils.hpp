#pragma once

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

#include <Eigen/Core>
#include <optional>
#include <silk/silk.hpp>
#include <vector>

#include "eigen_alias.hpp"
#include "object.hpp"

enum class SilkObjectType : int { Cloth = 1, Obstacle = 2 };
enum class UIMode { Normal, Paint, Sim };

struct Mesh {
  Vert verts;
  Face faces;
};

std::optional<Mesh> load_mesh_from_file(const std::string& path);

struct Context {
  // pIObject contains silk handle so it must be destroyed after silk world.
  // So silk_world must be declared before objects.
  silk::World silk_world;

  UIMode ui_mode = UIMode::Normal;
  int selection = -1;
  std::vector<pIObject> objects;
  silk::GlobalConfig global_config;
};

class IWidget {
 public:
  virtual ~IWidget() = default;
  virtual void draw() = 0;
};

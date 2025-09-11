#include "obstacle.hpp"

#include <polyscope/surface_mesh.h>
#include <spdlog/spdlog.h>

#include <utility>

#include "../polyscope_silk_interop.hpp"

namespace py = polyscope;

std::optional<Obstacle> Obstacle::try_make_obstacle(silk::World* world,
                                                    std::string name, Vert V,
                                                    Face F) {
  if (V.rows() == 0) {
    return std::nullopt;
  }

  auto mesh = py::registerSurfaceMesh(name, V, F);
  if (!mesh) {
    return std::nullopt;
  }
  mesh->setSelectionMode(polyscope::MeshSelectionMode::VerticesOnly);
  mesh->setEdgeWidth(1.0);
  mesh->setEdgeColor({0.0, 0.0, 0.0});

  // For an N×3 vertex matrix, take column-wise extrema and transpose to 3×1.
  Eigen::Vector3f max = V.colwise().maxCoeff().transpose();
  Eigen::Vector3f min = V.colwise().minCoeff().transpose();
  float mesh_scale = (max - min).norm();

  Obstacle o;
  o.name = std::move(name);
  o.mesh = mesh;
  o.V = std::move(V);
  o.F = std::move(F);
  o.scale = mesh_scale;
  o.world = world;
  o.silk_handle = 0;
  o.collision_config = {};
  o.collision_config_changed = false;

  return o;
}

Obstacle::Obstacle(Obstacle&& other) noexcept {
  swap(other);
  other.clear();
}

Obstacle::~Obstacle() {
  if (mesh) {
    py::removeSurfaceMesh(name);
  }

  if (silk_handle != 0 && world) {
    silk::Result r = world->remove_obstacle(silk_handle);
    if (!r) {
      spdlog::error("Fail to remove silk obstacle. Error: {}", r.to_string());
    }
  }
}

Obstacle& Obstacle::operator=(Obstacle&& other) noexcept {
  swap(other);
  other.clear();
  return *this;
}

void Obstacle::swap(Obstacle& other) noexcept {
  std::swap(name, other.name);
  std::swap(mesh, other.mesh);
  std::swap(V, other.V);
  std::swap(F, other.F);
  std::swap(scale, other.scale);
  std::swap(world, other.world);
  std::swap(silk_handle, other.silk_handle);
  std::swap(collision_config, other.collision_config);
  std::swap(collision_config_changed, other.collision_config_changed);
}

void Obstacle::clear() noexcept {
  mesh = nullptr;
  V = {};
  F = {};
  world = nullptr;
  silk_handle = 0;
}

std::string Obstacle::get_name() const { return name; }

const polyscope::SurfaceMesh* Obstacle::get_mesh() const { return mesh; }

float Obstacle::get_object_scale() const { return scale; }

uint32_t Obstacle::get_silk_handle() const { return silk_handle; }

ObjectStat Obstacle::get_stat() const {
  return {static_cast<int>(V.rows()), static_cast<int>(F.rows())};
}

void Obstacle::draw() { draw_collision_config(); }

bool Obstacle::init_sim() {
  if (silk_handle == 0) {
    silk::MeshConfig mesh_config;
    mesh_config.verts.data = V.data();
    mesh_config.verts.size = V.size();
    mesh_config.faces.data = F.data();
    mesh_config.faces.size = F.size();

    silk::Result r =
        world->add_obstacle(collision_config, mesh_config, silk_handle);
    if (!r) {
      spdlog::error("Fail to init obstacle {}. Error: {}", name, r.to_string());
      return false;
    }
    return true;
  }

  return update_silk_obstacle();
}

bool Obstacle::sim_step_pre() { return update_silk_obstacle(); }

bool Obstacle::sim_step_post() { return true; }

bool Obstacle::exit_sim() { return true; }

void Obstacle::handle_pick(const polyscope::PickResult&, bool, int) {
  // Obstacles ignore picking
}

void Obstacle::draw_collision_config() {
  ImGui::SeparatorText("Collision Setting");

  silk::CollisionConfig& c = this->collision_config;

  if (ImGui::Checkbox("Is Collision On", &c.is_collision_on)) {
    collision_config_changed = true;
  }
  if (ImGui::InputInt("Collision Group", &c.group)) {
    collision_config_changed = true;
  }
  if (ImGui::InputFloat("Restitution", &c.restitution)) {
    collision_config_changed = true;
  }
  if (ImGui::InputFloat("Friction", &c.friction)) {
    collision_config_changed = true;
  }
}

bool Obstacle::update_silk_obstacle() {
  if (collision_config_changed) {
    silk::Result r =
        world->set_obstacle_collision_config(silk_handle, collision_config);
    if (!r) {
      spdlog::error("Fail to update obstacle {} collision config. Error: {}",
                    name, r.to_string());
      return false;
    }
    collision_config_changed = false;
  }

  auto& position = mesh->vertexPositions;
  position.ensureHostBufferPopulated();
  silk::ConstSpan<float> s = make_const_span_from_position(position);
  silk::Result r = world->set_obstacle_position(silk_handle, s);
  if (!r) {
    spdlog::error("Fail to update obstacle {} position. Error: {}", name,
                  r.to_string());
    return false;
  }

  return true;
}

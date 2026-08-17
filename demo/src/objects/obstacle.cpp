#include "obstacle.hpp"

#include <polyscope/surface_mesh.h>
#include <spdlog/spdlog.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <span>
#include <utility>

#include "../eigen_alias.hpp"
#include "../gui_utils.hpp"
#include "../polyscope_silk_interop.hpp"
#include "../position_cache.hpp"
#include "../transform.hpp"
#include "draw_utils.hpp"

namespace py = polyscope;

std::optional<Obstacle> Obstacle::make_obstacle(silk::World* world,
                                                std::string name, Vert V,
                                                Face F) {
  if (!world) {
    return std::nullopt;
  }

  if (V.rows() == 0 || F.rows() == 0) {
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
  o.name_ = std::move(name);
  o.mesh_ = mesh;
  o.V_ = std::move(V);
  o.F_ = std::move(F);
  o.mesh_scale_ = mesh_scale;
  o.world_ = world;
  o.silk_handle_ = 0;
  o.collision_config_ = {};
  o.cache_ = {};
  o.collision_config_changed_ = false;
  o.position_ = glm::vec3(0.0f);
  o.rotation_ = glm::vec3(0.0f);
  o.angular_velocity_ = glm::vec3(0.0f);
  o.rotation_pivot_ = glm::vec3(o.V_.col(0).mean(), o.V_.col(1).mean(),
                                o.V_.col(2).mean());
  o.scale_ = 1.0f;
  o.transform_changed_ = false;
  o.drag_position_changed_ = false;

  return o;
}

std::optional<Obstacle> Obstacle::make_obstacle(
    silk::World* world, const config::ObstacleObject& obj) {
  auto mesh = load_mesh_from_file(obj.mesh);
  if (!mesh) {
    return std::nullopt;
  }

  AffineTransformer transformer{obj.transform};
  transformer.apply(mesh->verts);

  auto obstacle = make_obstacle(world, obj.name, std::move(mesh->verts),
                                std::move(mesh->faces));
  if (!obstacle) {
    return std::nullopt;
  }

  obstacle->collision_config_.is_collision_on = obj.collision.enabled;
  obstacle->collision_config_.is_self_collision_on =
      obj.collision.self_collision;
  obstacle->collision_config_.group = obj.collision.group;
  obstacle->collision_config_.friction = obj.collision.friction;
  obstacle->collision_config_.restitution = obj.collision.restitution;
  obstacle->angular_velocity_ = glm::radians(glm::vec3(
      obj.angular_velocity_deg_s[0], obj.angular_velocity_deg_s[1],
      obj.angular_velocity_deg_s[2]));

  return obstacle;
}

Obstacle::Obstacle(Obstacle&& other) noexcept {
  swap(other);
  other.clear();
}

Obstacle::~Obstacle() {
  if (mesh_) {
    py::removeSurfaceMesh(name_);
  }

  if (silk_handle_ != 0) {
    silk::Result r = world_->remove_obstacle(silk_handle_);
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
  std::swap(name_, other.name_);
  std::swap(mesh_, other.mesh_);
  std::swap(V_, other.V_);
  std::swap(F_, other.F_);
  std::swap(mesh_scale_, other.mesh_scale_);
  std::swap(world_, other.world_);
  std::swap(silk_handle_, other.silk_handle_);
  std::swap(collision_config_, other.collision_config_);
  std::swap(cache_, other.cache_);
  std::swap(collision_config_changed_, other.collision_config_changed_);
  std::swap(position_, other.position_);
  std::swap(rotation_, other.rotation_);
  std::swap(angular_velocity_, other.angular_velocity_);
  std::swap(rotation_pivot_, other.rotation_pivot_);
  std::swap(scale_, other.scale_);
  std::swap(transform_changed_, other.transform_changed_);
  std::swap(drag_position_changed_, other.drag_position_changed_);
}

void Obstacle::clear() noexcept {
  mesh_ = nullptr;
  V_ = {};
  F_ = {};
  world_ = nullptr;
  silk_handle_ = 0;
  cache_.clear();
}

std::string Obstacle::get_name() const { return name_; }

const polyscope::SurfaceMesh* Obstacle::get_mesh() const { return mesh_; }

const Face& Obstacle::get_faces() const { return F_; }

float Obstacle::get_object_scale() const { return mesh_scale_; }

uint32_t Obstacle::get_silk_handle() const { return silk_handle_; }

ObjectStat Obstacle::get_stat() const {
  return {static_cast<int>(V_.rows()), static_cast<int>(F_.rows())};
}

const PositionCache& Obstacle::get_cache() const { return cache_; }

PositionCache& Obstacle::get_cache() { return cache_; }

void Obstacle::draw() {
  draw_collision_config(collision_config_, collision_config_changed_);

  draw_transform_widget(position_, rotation_, scale_, transform_changed_);
  if (transform_changed_) {
    AffineTransformer transformer{position_, rotation_, scale_};
    mesh_->setTransform(transformer.get_glm_affine());
    transform_changed_ = false;
  }
}

bool Obstacle::init_sim() {
  cache_.clear();

  // apply transformation
  mesh_->vertexPositions.ensureHostBufferPopulated();
  AffineTransformer transformer{position_, rotation_, scale_};
  transformer.apply(mesh_->vertexPositions.data);
  mesh_->vertexPositions.markHostBufferUpdated();
  mesh_->resetTransform();

  std::span<const float> vert_span =
      make_const_span_from_position(mesh_->vertexPositions);

  if (silk_handle_ == 0) {
    silk::MeshConfig mesh_config;
    mesh_config.verts = vert_span;
    mesh_config.faces = std::span<const int>(F_.data(), F_.size());

    silk::Result r =
        world_->add_obstacle(collision_config_, mesh_config, silk_handle_);
    if (!r) {
      spdlog::error("Fail to init obstacle {}. Error: {}", name_,
                    r.to_string());
      return false;
    }
    return true;
  }

  silk::Result r = world_->set_obstacle_position(silk_handle_, vert_span);
  if (!r) {
    spdlog::error("Fail to set obstacle {} init position. Error: {}", name_,
                  r.to_string());
    return false;
  }

  return true;
}

bool Obstacle::sim_step_pre(float dt) {
  if (collision_config_changed_) {
    silk::Result r =
        world_->set_obstacle_collision_config(silk_handle_, collision_config_);
    if (!r) {
      spdlog::error("Fail to update obstacle {} collision config. Error: {}",
                    name_, r.to_string());
      return false;
    }
    collision_config_changed_ = false;
  }

  if (angular_velocity_ != glm::vec3(0.0f)) {
    mesh_->vertexPositions.ensureHostBufferPopulated();
    for (glm::vec3& vertex : mesh_->vertexPositions.data) {
      vertex -= rotation_pivot_;
    }
    AffineTransformer transformer{glm::vec3(0.0f), dt * angular_velocity_,
                                  1.0f};
    transformer.apply(mesh_->vertexPositions.data);
    for (glm::vec3& vertex : mesh_->vertexPositions.data) {
      vertex += rotation_pivot_;
    }
    mesh_->vertexPositions.markHostBufferUpdated();
    drag_position_changed_ = true;
  }

  if (drag_position_changed_) {
    mesh_->vertexPositions.ensureHostBufferPopulated();
    std::span<const float> vert_span =
        make_const_span_from_position(mesh_->vertexPositions);
    silk::Result r = world_->set_obstacle_position(silk_handle_, vert_span);
    if (!r) {
      spdlog::error("Fail to update obstacle {} drag position. Error: {}",
                    name_, r.to_string());
      return false;
    }
  }

  return true;
}

bool Obstacle::sim_step_post(float current_time) {
  if (cache_.empty() || drag_position_changed_) {
    mesh_->vertexPositions.ensureHostBufferPopulated();
    std::span<const float> vert_span =
        make_const_span_from_position(mesh_->vertexPositions);
    Eigen::Map<const Vert> vert{vert_span.data(), (int)vert_span.size() / 3, 3};
    cache_.emplace_back(current_time, vert);

    drag_position_changed_ = false;
  }
  return true;
}

bool Obstacle::exit_sim() {
  mesh_->updateVertexPositions(V_);
  AffineTransformer transformer{position_, rotation_, scale_};
  mesh_->setTransform(transformer.get_glm_affine());
  return true;
}

void Obstacle::handle_pick(const polyscope::PickResult&, bool, int) {
  // Obstacles ignore picking
}

void Obstacle::handle_drag(const glm::vec3& shift) {
  mesh_->vertexPositions.ensureHostBufferAllocated();
  for (int i = 0; i < V_.rows(); ++i) {
    mesh_->vertexPositions.data[i] += shift;
  }
  mesh_->vertexPositions.markHostBufferUpdated();

  drag_position_changed_ = true;
}

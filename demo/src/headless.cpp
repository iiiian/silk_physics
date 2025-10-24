#include "headless.hpp"

#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cassert>
#include <cstdint>
#include <optional>
#include <silk/silk.hpp>
#include <string>
#include <utility>
#include <vector>

#include "alembic_writer.hpp"
#include "gui_utils.hpp"
#include "object.hpp"
#include "transform.hpp"

silk::ClothConfig to_cloth_config(const config::ClothParams& params) {
  silk::ClothConfig cloth;
  cloth.elastic_stiffness = static_cast<float>(params.elastic_stiffness);
  cloth.bending_stiffness = static_cast<float>(params.bending_stiffness);
  cloth.density = static_cast<float>(params.density);
  cloth.damping = static_cast<float>(params.damping);
  return cloth;
}

silk::CollisionConfig to_collision_config(const config::Collision& collision) {
  silk::CollisionConfig cfg;
  cfg.is_collision_on = collision.enabled;
  cfg.is_self_collision_on = collision.self_collision;
  cfg.group = collision.group;
  cfg.restitution = static_cast<float>(collision.restitution);
  cfg.friction = static_cast<float>(collision.friction);
  return cfg;
}

class HeadlessCloth : public IObject {
 private:
  silk::World* world_;
  std::string name_;
  Vert verts_;
  Face faces_;
  silk::ClothConfig cloth_config_;
  silk::CollisionConfig collision_config_;
  uint32_t handle_;
  PositionCache cache_;

 public:
  static std::optional<HeadlessCloth> make_headless_cloth(
      silk::World* world, const config::ClothObject& config) {
    assert(world != nullptr);

    auto mesh = load_mesh_from_file(config.mesh);
    if (!mesh) {
      spdlog::error("Failed to load cloth mesh '{}' for headless simulation.",
                    config.mesh);
      return std::nullopt;
    }

    if (mesh->verts.rows() == 0 || mesh->faces.rows() == 0) {
      spdlog::error("Empty cloth mesh '{}' for headless simulation.",
                    config.mesh);
      return std::nullopt;
    }

    AffineTransformer transformer{config.transform};
    transformer.apply(mesh->verts);

    HeadlessCloth cloth;
    cloth.world_ = world;
    cloth.name_ = config.name;
    cloth.verts_ = std::move(mesh->verts);
    cloth.faces_ = std::move(mesh->faces);
    cloth.cloth_config_ = to_cloth_config(config.cloth);
    cloth.collision_config_ = to_collision_config(config.collision);
    cloth.handle_ = 0;
    cloth.cache_ = {};
    cloth.cache_.emplace_back(0.0f, cloth.verts_);

    return cloth;
  }

  ~HeadlessCloth() override {
    if (world_ && handle_ != 0) {
      silk::Result r = world_->remove_cloth(handle_);
      if (!r) {
        spdlog::warn(
            "Failed to remove cloth '{}' during destruction. Error: {}", name_,
            r.to_string());
      }
    }
  }

  HeadlessCloth(const HeadlessCloth&) = delete;

  HeadlessCloth& operator=(const HeadlessCloth&) = delete;

  HeadlessCloth(HeadlessCloth&& other) noexcept {
    swap(other);
    other.handle_ = 0;
  }

  HeadlessCloth& operator=(HeadlessCloth&& other) noexcept {
    if (this != &other) {
      swap(other);
      other.handle_ = 0;
    }
    return *this;
  }

  std::string get_name() const override { return name_; }

  const polyscope::SurfaceMesh* get_mesh() const override { return nullptr; }

  const Face& get_faces() const override { return faces_; }

  // dummy
  float get_object_scale() const override { return 1.0f; }

  uint32_t get_silk_handle() const override { return handle_; }

  // dummy
  ObjectStat get_stat() const override { return ObjectStat{}; }

  const PositionCache& get_cache() const override { return cache_; }

  PositionCache& get_cache() override { return cache_; }

  // dummy
  void draw() override {}

  bool init_sim() override {
    silk::MeshConfig mesh_config;
    mesh_config.verts.data = verts_.data();
    mesh_config.verts.size = verts_.size();
    mesh_config.faces.data = faces_.data();
    mesh_config.faces.size = faces_.size();

    // Empty pin
    silk::ConstSpan<int> pin_index;

    silk::Result r = world_->add_cloth(cloth_config_, collision_config_,
                                       mesh_config, pin_index, handle_);
    if (!r) {
      spdlog::error(
          "Failed to add cloth '{}' to headless simulation. Error: {}", name_,
          r.to_string());
      return false;
    }

    return true;
  }

  // dummy
  bool sim_step_pre() override { return true; }

  bool sim_step_post(float current_time) override {
    Vert buffer;
    buffer.resize(verts_.rows(), 3);

    silk::Span<float> position_span;
    position_span.data = buffer.data();
    position_span.size = buffer.size();

    silk::Result r = world_->get_cloth_position(handle_, position_span);
    if (!r) {
      spdlog::error("Failed to fetch cloth '{}' positions. Error: {}", name_,
                    r.to_string());
      return false;
    }

    cache_.emplace_back(current_time, buffer);
    return true;
  }

  // dummy
  bool exit_sim() override { return true; }

  // dummy
  void handle_pick(const polyscope::PickResult&, bool, int) override {}

  // dummy
  void handle_drag(const glm::vec3&) override {}

 private:
  HeadlessCloth() = default;

  void swap(HeadlessCloth& other) noexcept {
    std::swap(world_, other.world_);
    std::swap(name_, other.name_);
    std::swap(verts_, other.verts_);
    std::swap(faces_, other.faces_);
    std::swap(cloth_config_, other.cloth_config_);
    std::swap(collision_config_, other.collision_config_);
    std::swap(handle_, other.handle_);
    std::swap(cache_, other.cache_);
  }
};

class HeadlessObstacle : public IObject {
 private:
  silk::World* world_;
  std::string name_;
  Vert verts_;
  Face faces_;
  silk::CollisionConfig collision_config_;
  uint32_t handle_ = 0;
  PositionCache cache_;

 public:
  static std::optional<HeadlessObstacle> make_headless_obstacle(
      silk::World* world, const config::ObstacleObject& config) {
    assert(world != nullptr);

    auto mesh = load_mesh_from_file(config.mesh);
    if (!mesh) {
      spdlog::error(
          "Failed to load obstacle mesh '{}' for headless simulation.",
          config.mesh);
      return std::nullopt;
    }

    if (mesh->verts.rows() == 0 || mesh->faces.rows() == 0) {
      spdlog::error("Obstacle '{}' has empty geometry; skipping.", config.name);
      return std::nullopt;
    }

    AffineTransformer transformer{config.transform};
    transformer.apply(mesh->verts);

    HeadlessObstacle obstacle;
    obstacle.world_ = world;
    obstacle.name_ = config.name;
    obstacle.verts_ = std::move(mesh->verts);
    obstacle.faces_ = std::move(mesh->faces);
    obstacle.collision_config_ = to_collision_config(config.collision);
    obstacle.handle_ = 0;
    obstacle.cache_ = {};
    obstacle.cache_.emplace_back(0.0f, obstacle.verts_);

    return obstacle;
  }

  ~HeadlessObstacle() override {
    if (world_ && handle_ != 0) {
      silk::Result r = world_->remove_obstacle(handle_);
      if (!r) {
        spdlog::warn(
            "Failed to remove obstacle '{}' during destruction. Error: {}",
            name_, r.to_string());
      }
    }
  }

  HeadlessObstacle(const HeadlessObstacle&) = delete;

  HeadlessObstacle& operator=(const HeadlessObstacle&) = delete;

  HeadlessObstacle(HeadlessObstacle&& other) noexcept {
    swap(other);
    other.handle_ = 0;
  }

  HeadlessObstacle& operator=(HeadlessObstacle&& other) noexcept {
    if (this != &other) {
      swap(other);
      other.handle_ = 0;
    }
    return *this;
  }

  std::string get_name() const override { return name_; }

  // dummy
  const polyscope::SurfaceMesh* get_mesh() const override { return nullptr; }

  const Face& get_faces() const override { return faces_; }

  // dummy
  float get_object_scale() const override { return 1.0f; }

  uint32_t get_silk_handle() const override { return handle_; }

  // dummy
  ObjectStat get_stat() const override { return ObjectStat{}; }

  const PositionCache& get_cache() const override { return cache_; }

  PositionCache& get_cache() override { return cache_; }

  // dummy
  void draw() override {}

  bool init_sim() override {
    silk::MeshConfig mesh_config;
    mesh_config.verts.data = verts_.data();
    mesh_config.verts.size = verts_.size();
    mesh_config.faces.data = faces_.data();
    mesh_config.faces.size = faces_.size();

    silk::Result r =
        world_->add_obstacle(collision_config_, mesh_config, handle_);
    if (!r) {
      spdlog::error(
          "Failed to add obstacle '{}' to headless simulation. Error: {}",
          name_, r.to_string());
      return false;
    }

    return true;
  }

  // dummy
  bool sim_step_pre() override { return true; }

  // dummy
  bool sim_step_post(float current_time) override { return true; }

  // dummy
  bool exit_sim() override { return true; }

  // dummy
  void handle_pick(const polyscope::PickResult&, bool, int) override {}

  // dummy
  void handle_drag(const glm::vec3&) override {}

 private:
  HeadlessObstacle() = default;

  void swap(HeadlessObstacle& other) noexcept {
    std::swap(world_, other.world_);
    std::swap(name_, other.name_);
    std::swap(verts_, other.verts_);
    std::swap(faces_, other.faces_);
    std::swap(collision_config_, other.collision_config_);
    std::swap(handle_, other.handle_);
    std::swap(cache_, other.cache_);
  }
};

silk::GlobalConfig make_global_config(const config::Global& global_cfg) {
  silk::GlobalConfig cfg{};
  cfg.dt = static_cast<float>(global_cfg.dt);
  cfg.max_outer_iteration = global_cfg.max_outer_iteration;
  cfg.max_inner_iteration = global_cfg.max_inner_iteration;
  cfg.acceleration_x = static_cast<float>(global_cfg.acceleration[0]);
  cfg.acceleration_y = static_cast<float>(global_cfg.acceleration[1]);
  cfg.acceleration_z = static_cast<float>(global_cfg.acceleration[2]);
  return cfg;
}

void headless_run(const SimConfig& sim_config, const std::string& out_path) {
  silk::World world;

  std::vector<pIObject> objects;

  for (const auto& cloth_cfg : sim_config.cloths) {
    auto cloth = HeadlessCloth::make_headless_cloth(&world, cloth_cfg);
    if (!cloth) {
      continue;
    }
    objects.push_back(std::make_unique<HeadlessCloth>(std::move(*cloth)));
  }

  for (const auto& obstacle_cfg : sim_config.obstacles) {
    auto obstacle =
        HeadlessObstacle::make_headless_obstacle(&world, obstacle_cfg);
    if (!obstacle) {
      continue;
    }
    objects.push_back(std::make_unique<HeadlessObstacle>(std::move(*obstacle)));
  }

  if (objects.empty()) {
    spdlog::error("Headless simulation aborted: no valid objects were loaded.");
    return;
  }

  for (auto& object : objects) {
    if (!object->init_sim()) {
      spdlog::error("Headless simulation aborted while initializing '{}'.",
                    object->get_name());
      return;
    }
  }

  silk::GlobalConfig global_cfg = make_global_config(sim_config.global);
  silk::Result r = world.set_global_config(global_cfg);
  if (!r) {
    spdlog::error("Failed to apply global config. Error: {}", r.to_string());
    return;
  }

  int total_steps = sim_config.global.total_steps;
  spdlog::info("Headless simulation start. Total time {}s. Total steps {}",
               total_steps * global_cfg.dt, total_steps);
  for (int step = 0; step < total_steps; ++step) {
    for (auto& object : objects) {
      if (!object->sim_step_pre()) {
        spdlog::error("Headless simulation aborted during pre-step for '{}'.",
                      object->get_name());
        return;
      }
    }

    silk::Result step_result = world.solver_step();
    if (!step_result) {
      spdlog::error("Solver step {} failed. Error: {}", step,
                    step_result.to_string());
      return;
    }

    float current_time = (step + 1) * global_cfg.dt;
    spdlog::info("Finish step {}. Current time {}s", step, current_time);
    for (auto& object : objects) {
      if (!object->sim_step_post(current_time)) {
        spdlog::error("Headless simulation aborted during post-step for '{}'.",
                      object->get_name());
        return;
      }
    }
  }

  for (auto& object : objects) {
    if (!object->exit_sim()) {
      spdlog::warn("Object '{}' reported errors while exiting simulation.",
                   object->get_name());
    }
  }

  if (!write_scene(out_path, objects)) {
    spdlog::error("Failed to export headless simulation to '{}'.", out_path);
  } else {
    spdlog::info("Headless simulation exported to '{}'.", out_path);
  }
}

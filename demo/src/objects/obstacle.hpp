#pragma once

#include <Eigen/Core>
#include <glm/glm.hpp>
#include <optional>

#include "../config.hpp"
#include "../eigen_alias.hpp"
#include "../gui_utils.hpp"
#include "../object.hpp"
#include "../position_cache.hpp"
#include "silk/silk.hpp"

class Obstacle : public IObject {
 private:
  // polyscope
  std::string name_;
  polyscope::SurfaceMesh* mesh_;

  // mesh
  Vert V_;
  Face F_;
  float mesh_scale_;

  // silk
  silk::World* world_;
  uint32_t silk_handle_;
  silk::CollisionConfig collision_config_;

  PositionCache cache_;

  // transform
  glm::vec3 position_;
  glm::vec3 rotation_;
  float scale_;

  bool collision_config_changed_;
  bool transform_changed_;
  bool drag_position_changed_;

 public:
  static std::optional<Obstacle> make_obstacle(silk::World* world,
                                               std::string name, Vert V,
                                               Face F);
  static std::optional<Obstacle> make_obstacle(
      silk::World* world, const config::ObstacleObject& obj);

  // default ctor is private, use factory function try_make_obstacle instead.
  Obstacle(const Obstacle&) = delete;
  Obstacle(Obstacle&& other) noexcept;
  ~Obstacle();
  Obstacle& operator=(const Obstacle&) = delete;
  Obstacle& operator=(Obstacle&& other) noexcept;

  void swap(Obstacle& other) noexcept;
  void clear() noexcept;

  std::string get_name() const override;
  const polyscope::SurfaceMesh* get_mesh() const override;
  const Face& get_faces() const override;
  float get_object_scale() const override;
  uint32_t get_silk_handle() const override;
  ObjectStat get_stat() const override;
  const PositionCache& get_cache() const override;
  PositionCache& get_cache() override;

  void draw() override;

  bool init_sim() override;
  bool sim_step_pre() override;
  bool sim_step_post(float current_time) override;
  bool exit_sim() override;

  void handle_pick(const polyscope::PickResult& pick, bool add_to_selection,
                   int pick_radius) override;
  void handle_drag(const glm::vec3& shift) override;

 private:
  Obstacle() = default;
};

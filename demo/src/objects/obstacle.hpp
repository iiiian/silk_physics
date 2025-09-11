#pragma once

#include <Eigen/Core>
#include <optional>

#include "../gui_helper.hpp"
#include "../object_interface.hpp"
#include "silk/silk.hpp"

// to codex:
// - unlike cloth, obstacle has no pin. the whole obstalce is pinned
// - obstacle only has collision config
// - obstacle gnore pick
// - obstacle silk update pass position of the whole mesh
class Obstacle : public IObject {
 private:
  // polyscope
  std::string name;
  polyscope::SurfaceMesh* mesh;

  // mesh
  Vert V;
  Face F;
  float scale;

  // silk
  silk::World* world;
  uint32_t silk_handle;
  silk::CollisionConfig collision_config;

  bool collision_config_changed;

 public:
  static std::optional<Obstacle> try_make_obstacle(silk::World* world,
                                                   std::string name, Vert V,
                                                   Face F);

  // default ctor is private, use factory function try_make_obstacle instead.
  Obstacle(const Obstacle&) = delete;
  Obstacle(Obstacle&& other) noexcept;

  ~Obstacle();

  Obstacle& operator=(const Obstacle&) = delete;
  Obstacle& operator=(Obstacle&& other) noexcept;

  void swap(Obstacle& other) noexcept;
  void clear() noexcept;

  // getters
  std::string get_name() const override;
  const polyscope::SurfaceMesh* get_mesh() const override;
  float get_object_scale() const override;
  uint32_t get_silk_handle() const override;
  ObjectStat get_stat() const override;

  // draw per-object imgui controls
  void draw() override;

  // simulation hooks
  bool init_sim() override;
  bool sim_step_pre() override;
  bool sim_step_post() override;
  bool exit_sim() override;

  // picking/selection hook: widget forwards raw pick + add/remove intent
  void handle_pick(const polyscope::PickResult& pick, bool add_to_selection,
                   int pick_radius) override;

 private:
  Obstacle() = default;

  void draw_collision_config();
  bool update_silk_obstacle();
};

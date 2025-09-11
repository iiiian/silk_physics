#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <optional>
#include <vector>

#include "../gui_helper.hpp"
#include "../object_interface.hpp"
#include "silk/silk.hpp"

class Cloth : public IObject {
 private:
  // polyscope
  std::string name;
  polyscope::SurfaceMesh* mesh;

  // mesh
  Vert V;
  Face F;
  std::vector<std::vector<int>> adj;
  float scale;

  // silk
  silk::World* world;
  uint32_t silk_handle;
  silk::ClothConfig cloth_config;
  silk::CollisionConfig collision_config;
  std::unordered_set<int> pin_group;
  std::vector<int> pin_index;

  bool pin_index_changed;
  bool cloth_config_changed;
  bool collision_config_changed;

 public:
  static std::optional<Cloth> try_make_cloth(silk::World* world,
                                             std::string name, Vert V, Face F);

  // default ctor is private, use factory function try_make_cloth instead.
  Cloth(const Cloth&) = delete;
  Cloth(Cloth&& other) noexcept;

  ~Cloth();

  Cloth& operator=(const Cloth&) = delete;
  Cloth& operator=(Cloth&& other) noexcept;

  void swap(Cloth& other) noexcept;
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
  Cloth() = default;

  void draw_cloth_config();
  void draw_collision_config();

  void update_pin_index();
  std::vector<float> gather_pin_position() const;
  bool update_silk_cloth();
};

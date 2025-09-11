#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <optional>
#include <vector>

#include "../gui_utils.hpp"
#include "../object.hpp"
#include "silk/silk.hpp"

class Cloth : public IObject {
 private:
  // polyscope
  std::string name_;
  polyscope::SurfaceMesh* mesh_;

  // mesh
  Vert V_;
  Face F_;
  std::vector<std::vector<int>> adjacency_list_;
  float mesh_scale_;

  // silk
  silk::World* world_;
  uint32_t silk_handle_;
  silk::ClothConfig cloth_config_;
  silk::CollisionConfig collision_config_;
  std::unordered_set<int> pin_group_;
  std::vector<int> pin_index_;

  // transform
  glm::vec3 position_;
  glm::vec3 rotation_;
  float scale_;

  bool pin_index_changed_;
  bool cloth_config_changed_;
  bool collision_config_changed_;
  bool transform_changed_;

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

  void handle_pick(const polyscope::PickResult& pick, bool add_to_selection,
                   int pick_radius) override;
  void handle_drag(const glm::vec3& shift) override;

 private:
  Cloth() = default;

  void update_pin_index();
  std::vector<float> gather_pin_position() const;
};

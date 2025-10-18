#include "cloth.hpp"

#include <igl/adjacency_list.h>
#include <igl/edges.h>
#include <polyscope/surface_mesh.h>
#include <spdlog/spdlog.h>

#include <array>
#include <glm/glm.hpp>
#include <queue>

#include "../glm_utils.hpp"
#include "../gui_utils.hpp"
#include "../polyscope_silk_interop.hpp"
#include "draw_utils.hpp"

namespace py = polyscope;

std::optional<Cloth> Cloth::make_cloth(silk::World* world, std::string name,
                                       Vert V, Face F) {
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

  std::vector<std::vector<int>> adj;
  igl::adjacency_list(F, adj);

  Cloth c;
  c.name_ = name;
  c.mesh_ = mesh;
  c.V_ = std::move(V);
  c.F_ = std::move(F);
  c.adjacency_list_ = std::move(adj);
  c.mesh_scale_ = mesh_scale;
  c.world_ = world;
  c.silk_handle_ = 0;
  c.cloth_config_ = {};
  c.collision_config_ = {};
  c.pin_group_ = {};
  c.pin_index_ = {};
  c.pin_index_changed_ = false;
  c.cloth_config_changed_ = false;
  c.collision_config_changed_ = false;
  c.position_ = glm::vec3(0.0f);
  c.rotation_ = glm::vec3(0.0f);
  c.scale_ = 1.0f;
  c.transform_changed_ = false;

  return c;
}

std::optional<Cloth> Cloth::make_cloth(silk::World* world,
                                       const config::ClothObject& obj) {
  auto mesh = load_mesh_from_file(obj.mesh);
  if (!mesh) {
    return std::nullopt;
  }

  auto cloth = make_cloth(world, obj.name, std::move(mesh->verts),
                          std::move(mesh->faces));
  if (!cloth) {
    return std::nullopt;
  }

  cloth->cloth_config_.elastic_stiffness = obj.cloth.elastic_stiffness;
  cloth->cloth_config_.bending_stiffness = obj.cloth.bending_stiffness;
  cloth->cloth_config_.density = obj.cloth.density;
  cloth->cloth_config_.damping = obj.cloth.damping;
  cloth->collision_config_.is_collision_on = obj.collision.enabled;
  cloth->collision_config_.is_self_collision_on = obj.collision.self_collision;
  cloth->collision_config_.group = obj.collision.group;
  cloth->collision_config_.friction = obj.collision.friction;
  cloth->collision_config_.restitution = obj.collision.restitution;

  // TODO: improve config type to avoid this akward translation.
  auto arr_to_glm = [](const std::array<double, 3>& arr) -> glm::vec3 {
    glm::vec3 v;
    v[0] = arr[0];
    v[1] = arr[1];
    v[2] = arr[2];

    return v;
  };

  cloth->position_ = arr_to_glm(obj.transform.translation);
  cloth->rotation_ = arr_to_glm(obj.transform.rotation_euler_deg);

  auto& s = obj.transform.scale;
  cloth->scale_ = std::max(s[0], std::max(s[1], s[2]));

  return cloth;
}

Cloth::Cloth(Cloth&& other) noexcept {
  swap(other);
  other.clear();
}

Cloth::~Cloth() {
  if (mesh_) {
    py::removeSurfaceMesh(name_);
  }

  if (silk_handle_ != 0 && world_) {
    silk::Result r = world_->remove_cloth(silk_handle_);
    if (!r) {
      spdlog::error("Fail to remove silk cloth. Error: {}", r.to_string());
    }
  }
}

Cloth& Cloth::operator=(Cloth&& other) noexcept {
  swap(other);
  other.clear();

  return *this;
}

void Cloth::swap(Cloth& other) noexcept {
  std::swap(name_, other.name_);
  std::swap(mesh_, other.mesh_);
  std::swap(V_, other.V_);
  std::swap(F_, other.F_);
  std::swap(adjacency_list_, other.adjacency_list_);
  std::swap(mesh_scale_, other.mesh_scale_);
  std::swap(world_, other.world_);
  std::swap(silk_handle_, other.silk_handle_);
  std::swap(cloth_config_, other.cloth_config_);
  std::swap(collision_config_, other.collision_config_);
  std::swap(pin_group_, other.pin_group_);
  std::swap(pin_index_, other.pin_index_);
  std::swap(pin_index_changed_, other.pin_index_changed_);
  std::swap(cloth_config_changed_, other.cloth_config_changed_);
  std::swap(collision_config_changed_, other.collision_config_changed_);
  std::swap(position_, other.position_);
  std::swap(rotation_, other.rotation_);
  std::swap(scale_, other.scale_);
  std::swap(transform_changed_, other.transform_changed_);
}

void Cloth::clear() noexcept {
  mesh_ = nullptr;
  V_ = {};
  F_ = {};
  adjacency_list_ = {};
  world_ = nullptr;
  silk_handle_ = 0;
}

std::string Cloth::get_name() const { return name_; }

const polyscope::SurfaceMesh* Cloth::get_mesh() const { return mesh_; }

float Cloth::get_object_scale() const { return mesh_scale_; }

uint32_t Cloth::get_silk_handle() const { return silk_handle_; }

ObjectStat Cloth::get_stat() const {
  return {static_cast<int>(V_.rows()), static_cast<int>(F_.rows())};
}

void Cloth::draw() {
  draw_cloth_config(cloth_config_, cloth_config_changed_);
  draw_collision_config(collision_config_, collision_config_changed_);

  draw_transform_widget(position_, rotation_, scale_, transform_changed_);
  if (transform_changed_) {
    mesh_->setTransform(build_transformation(position_, rotation_, scale_));
  }
}

bool Cloth::init_sim() {
  // apply transformation
  mesh_->vertexPositions.ensureHostBufferPopulated();
  glm::mat4 T = build_transformation(position_, rotation_, scale_);
  for (auto& v : mesh_->vertexPositions.data) {
    v = transform_vertex(T, v);
  }
  mesh_->vertexPositions.markHostBufferUpdated();
  mesh_->resetTransform();

  silk::ConstSpan<float> vert_span =
      make_const_span_from_position(mesh_->vertexPositions);

  if (silk_handle_ != 0 && transform_changed_) {
    silk::Result r = world_->remove_cloth(silk_handle_);
    if (!r) {
      spdlog::error("Fail to remove cloth {} during init sim. Error: {}", name_,
                    r.to_string());
      return false;
    }
    silk_handle_ = 0;
  }

  if (silk_handle_ != 0) {
    return true;
  }

  update_pin_index();

  silk::MeshConfig mesh_config;
  mesh_config.verts = vert_span;
  mesh_config.faces.data = F_.data();
  mesh_config.faces.size = F_.size();

  silk::Result r = world_->add_cloth(cloth_config_, collision_config_,
                                     mesh_config, pin_index_, silk_handle_);
  if (!r) {
    spdlog::error("Fail to init cloth {}. Error: {}", name_, r.to_string());
    return false;
  }

  pin_index_changed_ = false;
  cloth_config_changed_ = false;
  collision_config_changed_ = false;
  transform_changed_ = false;

  return true;
}

bool Cloth::sim_step_pre() {
  if (pin_index_changed_) {
    update_pin_index();
    silk::Result r = world_->set_cloth_pin_index(silk_handle_, pin_index_);
    if (!r) {
      spdlog::error("Fail to update cloth {} pin index. Error: {}", name_,
                    r.to_string());
      return false;
    }
  }
  if (cloth_config_changed_) {
    silk::Result r = world_->set_cloth_config(silk_handle_, cloth_config_);
    if (!r) {
      spdlog::error("Fail to update cloth {} config. Error: {}", name_,
                    r.to_string());
      return false;
    }
  }
  if (collision_config_changed_) {
    silk::Result r =
        world_->set_cloth_collision_config(silk_handle_, collision_config_);
    if (!r) {
      spdlog::error("Fail to update cloth {} collision config. Error: {}",
                    name_, r.to_string());
      return false;
    }
  }

  pin_index_changed_ = false;
  cloth_config_changed_ = false;
  collision_config_changed_ = false;

  // update pin position
  if (pin_group_.empty()) {
    return true;
  }
  auto pin_position = gather_pin_position();
  silk::Result r = world_->set_cloth_pin_position(silk_handle_, pin_position);
  if (!r) {
    spdlog::error("Fail to update cloth {} pin position. Error: {}", name_,
                  r.to_string());
    return false;
  }

  return true;
}

bool Cloth::sim_step_post() {
  mesh_->vertexPositions.ensureHostBufferAllocated();
  silk::Span<float> position = make_span_from_position(mesh_->vertexPositions);
  silk::Result r = world_->get_cloth_position(silk_handle_, position);
  if (!r) {
    spdlog::error("Fail to update cloth {} position. Error: {}", name_,
                  r.to_string());
    return false;
  }
  mesh_->vertexPositions.markHostBufferUpdated();

  return true;
}

bool Cloth::exit_sim() {
  mesh_->vertexPositions.ensureHostBufferAllocated();
  mesh_->updateVertexPositions(V_);
  mesh_->setTransform(build_transformation(position_, rotation_, scale_));
  mesh_->vertexPositions.markHostBufferUpdated();

  return true;
}

void Cloth::handle_pick(const polyscope::PickResult& pick,
                        bool add_to_selection, int pick_radius) {
  py::SurfaceMeshPickResult result = mesh_->interpretPickResult(pick);

  // BFS traverse the adjacency list to find all vertices selected/removed
  // by paint brush.
  // std::vector<bool> is a bit array
  std::vector<bool> visited(V_.rows(), false);
  std::queue<std::pair<int, int>> queue;  // (vertex, depth)
  std::vector<int> selected;

  int root = result.index;
  visited[root] = 1;
  queue.emplace(root, false);

  while (!queue.empty()) {
    auto [vert_idx, depth] = queue.front();
    queue.pop();
    selected.push_back(vert_idx);

    if (depth >= pick_radius) {
      continue;
    }

    if (add_to_selection) {
      if (pin_group_.insert(vert_idx).second) {
        pin_index_changed_ = true;
      }
    } else {
      if (pin_group_.erase(vert_idx) == 1) {
        pin_index_changed_ = true;
      }
    }

    for (int neighbor : adjacency_list_[vert_idx]) {
      if (!visited[neighbor]) {
        visited[neighbor] = 1;
        queue.emplace(neighbor, depth + 1);
      }
    }
  }

  // Update vertex color quantity to visualize current selection as yellow.
  // Use the mesh base surface color for unselected vertices to preserve
  // appearance.
  const glm::vec3 base_color = mesh_->getSurfaceColor();
  const glm::vec3 selected_color = {1.0f, 1.0f, 0.0f};  // yellow

  // Build a full color array; this is O(nV) but simple and robust.
  int vert_num = V_.rows();
  std::vector<glm::vec3> colors(vert_num, base_color);
  for (int idx : pin_group_) {
    colors[idx] = selected_color;
  }

  // Replace existing quantity (if any), or remove when empty selection.
  mesh_->removeQuantity("pin_highlight");
  if (!pin_group_.empty()) {
    auto* q = mesh_->addVertexColorQuantity("pin_highlight", colors);
    q->setEnabled(true);
  }
}

void Cloth::update_pin_index() {
  pin_index_.clear();
  for (int idx : pin_group_) {
    pin_index_.push_back(idx);
  }
}

std::vector<float> Cloth::gather_pin_position() const {
  auto& position = mesh_->vertexPositions;
  position.ensureHostBufferPopulated();

  std::vector<float> pin_position;
  for (int idx : pin_index_) {
    pin_position.push_back(position.data[idx][0]);
    pin_position.push_back(position.data[idx][1]);
    pin_position.push_back(position.data[idx][2]);
  }

  return pin_position;
}

void Cloth::handle_drag(const glm::vec3& shift) {
  if (pin_group_.empty()) {
    return;  // No pins to shift
  }

  mesh_->vertexPositions.ensureHostBufferAllocated();
  for (int idx : pin_group_) {
    mesh_->vertexPositions.data[idx] += shift;
  }
  mesh_->vertexPositions.markHostBufferUpdated();
}

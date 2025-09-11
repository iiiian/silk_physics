#include "cloth.hpp"

#include <igl/adjacency_list.h>
#include <igl/edges.h>
#include <polyscope/surface_mesh.h>
#include <spdlog/spdlog.h>

#include <cstring>
#include <glm/glm.hpp>
#include <queue>

#include "../polyscope_silk_interop.hpp"

namespace py = polyscope;

std::optional<Cloth> Cloth::try_make_cloth(silk::World* world, std::string name,
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
  c.name = name;
  c.mesh = mesh;
  c.V = std::move(V);
  c.F = std::move(F);
  c.adj = std::move(adj);
  c.scale = mesh_scale;
  c.world = world;
  c.silk_handle = 0;
  c.cloth_config = {};
  c.collision_config = {};
  c.pin_group = {};
  c.pin_index = {};
  c.pin_index_changed = false;
  c.cloth_config_changed = false;
  c.collision_config_changed = false;

  return c;
}

Cloth::Cloth(Cloth&& other) noexcept {
  swap(other);
  other.clear();
}

Cloth::~Cloth() {
  if (mesh) {
    py::removeSurfaceMesh(name);
  }

  if (silk_handle != 0 && world) {
    silk::Result r = world->remove_cloth(silk_handle);
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
  std::swap(name, other.name);
  std::swap(mesh, other.mesh);
  std::swap(V, other.V);
  std::swap(F, other.F);
  std::swap(adj, other.adj);
  std::swap(scale, other.scale);
  std::swap(world, other.world);
  std::swap(silk_handle, other.silk_handle);
  std::swap(cloth_config, other.cloth_config);
  std::swap(collision_config, other.collision_config);
  std::swap(pin_group, other.pin_group);
  std::swap(pin_index, other.pin_index);
  std::swap(pin_index_changed, other.pin_index_changed);
  std::swap(cloth_config_changed, other.cloth_config_changed);
  std::swap(collision_config_changed, other.collision_config_changed);
}

void Cloth::clear() noexcept {
  mesh = nullptr;
  V = {};
  F = {};
  adj = {};
  world = nullptr;
  silk_handle = 0;
}

std::string Cloth::get_name() const { return name; }

const polyscope::SurfaceMesh* Cloth::get_mesh() const { return mesh; }

float Cloth::get_object_scale() const { return scale; }

uint32_t Cloth::get_silk_handle() const { return silk_handle; }

ObjectStat Cloth::get_stat() const {
  return {static_cast<int>(V.rows()), static_cast<int>(F.rows())};
}

void Cloth::draw() {
  draw_cloth_config();
  draw_collision_config();
}

bool Cloth::init_sim() {
  // if this object is not yet present in silk engine,
  // add it.
  if (silk_handle == 0) {
    update_pin_index();

    silk::MeshConfig mesh_config;
    mesh_config.verts.data = V.data();
    mesh_config.verts.size = V.size();
    mesh_config.faces.data = F.data();
    mesh_config.faces.size = F.size();

    silk::Result r = world->add_cloth(cloth_config, collision_config,
                                      mesh_config, pin_index, silk_handle);
    if (!r) {
      spdlog::error("Fail to init cloth {}. Error: {}", name, r.to_string());
      return false;
    }

    pin_index_changed = false;
    cloth_config_changed = false;
    collision_config_changed = false;

    return true;
  }

  return update_silk_cloth();
}

bool Cloth::sim_step_pre() { return update_silk_cloth(); }

bool Cloth::sim_step_post() {
  mesh->vertexPositions.ensureHostBufferAllocated();
  silk::Span<float> position = make_span_from_position(mesh->vertexPositions);
  silk::Result r = world->get_cloth_position(silk_handle, position);
  if (!r) {
    spdlog::error("Fail to update cloth {} position. Error: {}", name,
                  r.to_string());
    return false;
  }
  mesh->vertexPositions.markHostBufferUpdated();

  return true;
}

bool Cloth::exit_sim() {
  // resume initial cloth position
  mesh->vertexPositions.ensureHostBufferAllocated();
  silk::Span<float> position = make_span_from_position(mesh->vertexPositions);
  size_t copy_size = V.size() * sizeof(float);
  memcpy(position.data, V.data(), copy_size);
  mesh->vertexPositions.markHostBufferUpdated();

  return true;
}

void Cloth::handle_pick(const polyscope::PickResult& pick,
                        bool add_to_selection, int pick_radius) {
  py::SurfaceMeshPickResult result = mesh->interpretPickResult(pick);

  // BFS traverse the adjacency list to find all vertices selected/removed
  // by paint brush.
  // std::vector<bool> is a bit array
  std::vector<bool> visited(V.rows(), false);
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
      if (pin_group.insert(vert_idx).second) {
        pin_index_changed = true;
      }
    } else {
      if (pin_group.erase(vert_idx) == 1) {
        pin_index_changed = true;
      }
    }

    for (int neighbor : adj[vert_idx]) {
      if (!visited[neighbor]) {
        visited[neighbor] = 1;
        queue.emplace(neighbor, depth + 1);
      }
    }
  }

  // Update vertex color quantity to visualize current selection as yellow.
  // Use the mesh base surface color for unselected vertices to preserve
  // appearance.
  const glm::vec3 base_color = mesh->getSurfaceColor();
  const glm::vec3 selected_color = {1.0f, 1.0f, 0.0f};  // yellow

  // Build a full color array; this is O(nV) but simple and robust.
  int vert_num = V.rows();
  std::vector<glm::vec3> colors(vert_num, base_color);
  for (int idx : pin_group) {
    colors[idx] = selected_color;
  }

  // Replace existing quantity (if any), or remove when empty selection.
  mesh->removeQuantity("pin_highlight");
  if (!pin_group.empty()) {
    auto* q = mesh->addVertexColorQuantity("pin_highlight", colors);
    q->setEnabled(true);
  }
}

void Cloth::draw_cloth_config() {
  ImGui::SeparatorText("Cloth Setting");

  silk::ClothConfig& c = this->cloth_config;

  if (ImGui::InputFloat("Elastic Stiffness", &c.elastic_stiffness)) {
    cloth_config_changed = true;
  }
  if (ImGui::InputFloat("Bending Stiffness", &c.bending_stiffness)) {
    cloth_config_changed = true;
  }
  if (ImGui::InputFloat("Density", &c.density)) {
    cloth_config_changed = true;
  }
  if (ImGui::InputFloat("Damping", &c.damping)) {
    cloth_config_changed = true;
  }
}

void Cloth::draw_collision_config() {
  ImGui::SeparatorText("Collision Setting");

  silk::CollisionConfig& c = this->collision_config;

  if (ImGui::Checkbox("Is Collision On", &c.is_collision_on)) {
    collision_config_changed = true;
  }
  if (ImGui::Checkbox("Is Self Collision On", &c.is_self_collision_on)) {
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

void Cloth::update_pin_index() {
  pin_index.clear();
  for (int idx : pin_group) {
    pin_index.push_back(idx);
  }
}

std::vector<float> Cloth::gather_pin_position() const {
  auto& position = mesh->vertexPositions;
  position.ensureHostBufferPopulated();

  std::vector<float> pin_position;
  for (int idx : pin_index) {
    pin_position.push_back(position.data[idx][0]);
    pin_position.push_back(position.data[idx][1]);
    pin_position.push_back(position.data[idx][2]);
  }

  return pin_position;
}

bool Cloth::update_silk_cloth() {
  if (pin_index_changed) {
    update_pin_index();
    silk::Result r = world->set_cloth_pin_index(silk_handle, pin_index);
    if (!r) {
      spdlog::error("Fail to update cloth {} pin index. Error: {}", name,
                    r.to_string());
      return false;
    }
  }
  if (cloth_config_changed) {
    silk::Result r = world->set_cloth_config(silk_handle, cloth_config);
    if (!r) {
      spdlog::error("Fail to update cloth {} config. Error: {}", name,
                    r.to_string());
      return false;
    }
  }
  if (collision_config_changed) {
    silk::Result r =
        world->set_cloth_collision_config(silk_handle, collision_config);
    if (!r) {
      spdlog::error("Fail to update cloth {} collision config. Error: {}", name,
                    r.to_string());
      return false;
    }
  }

  pin_index_changed = false;
  cloth_config_changed = false;
  collision_config_changed = false;

  // update pin position
  if (pin_group.empty()) {
    return true;
  }
  auto pin_position = gather_pin_position();
  silk::Result r = world->set_cloth_pin_position(silk_handle, pin_position);
  if (!r) {
    spdlog::error("Fail to update cloth {} pin position. Error: {}", name,
                  r.to_string());
    return false;
  }

  return true;
}

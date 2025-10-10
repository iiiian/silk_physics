#pragma once
#include <array>
#include <optional>
#include <string>
#include <vector>
#include <silk/silk.hpp>

namespace cli {

struct Transform {
  bool enabled = false;
  std::optional<std::array<double,3>> translation;
  std::optional<std::array<double,3>> rotation_euler_deg;
  std::optional<double>                scale_uniform;
  std::optional<std::array<double,3>>  scale_xyz;
};

enum class ObjectType { Cloth, Obstacle };

struct ObjectConfig {
  ObjectType type;
  std::string name;
  std::string mesh_path;

  silk::CollisionConfig collision{};
  bool has_collision = false;

  silk::ClothConfig cloth{};
  bool has_cloth = false;

  std::vector<int>   pin_indices;
  std::vector<float> pin_positions;

  std::optional<Transform> transform;
};

struct GlobalStop {
  int    total_steps = -1;
  double max_time    = -1.0;
};

struct SimConfig {
  silk::GlobalConfig global{};
  GlobalStop         stop{};
  bool               headless = false;
  std::vector<ObjectConfig> objects;
};

bool load_sim_config(const std::string& json_path, SimConfig& out, std::string& err);

} // namespace cli

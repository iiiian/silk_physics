#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace config {

//********************************/
//*          Global             */
//********************************/
struct Global {
  float dt = 0.01;
  int max_outer_iteration = 100;
  int max_inner_iteration = 50;
  std::array<float, 3> acceleration{0.0, 0.0, -9.8};
  int total_steps = 100;
};

//********************************/
//*          Collision           */
//********************************/
struct Collision {
  bool enabled = true;
  bool self_collision = false;
  int group = 0;
  float restitution = 0.0;
  float friction = 0.5;
};

//********************************/
//*          Transform           */
//********************************/
struct Transform {
  std::array<float, 3> translation{0.0, 0.0, 0.0};
  std::array<float, 3> rotation_euler_deg{0.0, 0.0, 0.0};
  std::array<float, 3> scale{1.0, 1.0, 1.0};
};

//********************************/
//*         ClothParams          */
//********************************/
struct ClothParams {
  float elastic_stiffness = 100.0;
  float bending_stiffness = 0.0001;
  float density = 0.1;
  float damping = 0.01;
};

//********************************/
//*        Pin Selection         */
//********************************/
struct PinBBox {
  std::array<float, 3> min{0.0, 0.0, 0.0};
  std::array<float, 3> max{0.0, 0.0, 0.0};
};

struct PinSelection {
  std::optional<PinBBox> bbox;
};

//********************************/
//*          Object Base         */
//********************************/

enum class ObjectType { Cloth, Obstacle };

struct ObjectBase {
  ObjectType type = ObjectType::Cloth;
  std::string name;
  std::string mesh;
  Collision collision;
  Transform transform;
};

//******** Cloth Object **********/
struct ClothObject : public ObjectBase {
  ClothParams cloth;
  std::optional<PinSelection> pin_selection;
  ClothObject() { type = ObjectType::Cloth; }
};

//******* Obstacle Object *******/
struct ObstacleObject : public ObjectBase {
  ObstacleObject() { type = ObjectType::Obstacle; }
};

}  // namespace config

struct SimConfig {
  config::Global global;
  std::vector<config::ClothObject> cloths;
  std::vector<config::ObstacleObject> obstacles;
};

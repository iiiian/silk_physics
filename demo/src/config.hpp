#pragma once

#include <array>
#include <string>
#include <vector>

namespace config {

//********************************/
//*          Global             */
//********************************/
struct Global {
  double dt = 0.01;
  int max_outer_iteration = 100;
  int max_inner_iteration = 50;
  std::array<double, 3> acceleration{0.0, 0.0, -9.8};
  int total_steps = 100;
  double max_time = 0.0;
  bool headless = false;
};

//********************************/
//*          Collision           */
//********************************/
struct Collision {
  bool enabled = true;
  bool self_collision = false;
  int group = 0;
  double restitution = 0.0;
  double friction = 0.5;
  double thickness = 0.0;
};

//********************************/
//*          Transform           */
//********************************/
struct Transform {
  std::array<double, 3> translation{0.0, 0.0, 0.0};
  std::array<double, 3> rotation_euler_deg{0.0, 0.0, 0.0};
  std::array<double, 3> scale{1.0, 1.0, 1.0};
};

//********************************/
//*         ClothParams          */
//********************************/
struct ClothParams {
  double elastic_stiffness = 100.0;
  double bending_stiffness = 0.0001;
  double density = 0.1;
  double damping = 0.01;
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

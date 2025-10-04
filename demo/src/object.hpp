#pragma once

#include <polyscope/pick.h>
#include <polyscope/surface_mesh.h>

#include <cstdint>
#include <glm/glm.hpp>
#include <memory>
#include <string>

#include "position_cache.hpp"

struct ObjectStat {
  int vert_num = 0;
  int face_num = 0;
};

class IObject {
 public:
  virtual ~IObject() = default;

  // getters
  virtual std::string get_name() const = 0;
  virtual const polyscope::SurfaceMesh* get_mesh() const = 0;
  virtual const Face& get_faces() const = 0;
  virtual float get_object_scale() const = 0;
  virtual uint32_t get_silk_handle() const = 0;
  virtual ObjectStat get_stat() const = 0;
  virtual const PositionCache& get_cache() const = 0;
  virtual PositionCache& get_cache() = 0;

  // draw per-object imgui controls
  virtual void draw() = 0;

  // simulation hooks
  virtual bool init_sim() = 0;
  virtual bool sim_step_pre() = 0;
  virtual bool sim_step_post(float current_time) = 0;
  virtual bool exit_sim() = 0;

  // picking/selection hook: widget forwards raw pick + add/remove intent
  virtual void handle_pick(const polyscope::PickResult& pick,
                           bool add_to_selection, int pick_radius) = 0;
  // position shift for dragging during simulation
  virtual void handle_drag(const glm::vec3& shift) = 0;
};

using pIObject = std::unique_ptr<IObject>;

#pragma once

#include <Eigen/Core>

#include "cloth_solver_data.hpp"
#include "handle.hpp"
#include "manager.hpp"
#include "mesh.hpp"
#include "object_collider.hpp"
#include "obstacle_position.hpp"
#include "pin.hpp"
#include "silk/silk.hpp"
#include "solver_state.hpp"

namespace silk {

template <typename T>
inline constexpr bool always_false_v = false;

// This is a table recording component handles of an entity.
// The actual component is stored in registry
struct Entity {
  Handle self;
  Handle cloth_config;
  Handle cloth_static_solver_data;
  Handle cloth_dynamic_solver_data;
  Handle solver_state;
  Handle collision_config;
  Handle tri_mesh;
  Handle pin;
  Handle obstacle_position;
  Handle object_collider;
};

class Registry {
 private:
  // For ECS Component type, this template is specialized with two members:
  // handle_ptr:
  //  of type Handle Entity::*, points to component handle inside Entity.
  // manager_ptr:
  //  of type Manager<T> Registry::*, points to manager inside Registry.
  template <typename T>
  struct ComponentTraits {
    static_assert(always_false_v<T>, "This type is not an ECS component!!");
  };

  Manager<Entity> entity;
  Manager<ClothConfig> cloth_config;
  Manager<CollisionConfig> collision_config;
  Manager<TriMesh> tri_mesh;
  Manager<Pin> pin;
  Manager<ClothStaticSolverData> cloth_static_solver_data;
  Manager<ClothDynamicSolverData> cloth_dynamic_solver_data;
  Manager<SolverState> solver_state;
  Manager<ObstaclePosition> obstacle_position;
  Manager<ObjectCollider> object_collider;

 public:
  // get component of an entity
  template <typename T>
  T* get(const Entity& entity) {
    Manager<T>& manager = this->*ComponentTraits<T>::manager_ptr;
    Handle h = entity.*ComponentTraits<T>::handle_ptr;
    return manager.get(h);
  }

  // get component of an entity
  template <typename T>
  const T* get(const Entity& entity) const {
    const Manager<T>& manager = this->*ComponentTraits<T>::manager_ptr;
    Handle h = entity.*ComponentTraits<T>::handle_ptr;
    return manager.get(h);
  }

  // get component of an entity
  template <typename T>
  T* get(const Entity* entity) {
    if (!entity) {
      return nullptr;
    }
    return get<T>(*entity);
  }

  // get component of an entity
  template <typename T>
  const T* get(const Entity* entity) const {
    if (!entity) {
      return nullptr;
    }
    return get<T>(*entity);
  }

  // get component of all entities
  template <typename T>
  std::vector<T>& get_all() {
    Manager<T>& manager = this->*ComponentTraits<T>::manager_ptr;
    return manager.data();
  }

  // get component of all entities
  template <typename T>
  const std::vector<T>& get_all() const {
    const Manager<T>& manager = this->*ComponentTraits<T>::manager_ptr;
    return manager.data();
  }

  // remove component of an entity.
  template <typename T>
  void remove(Entity& entity) {
    Manager<T>& manager = this->*ComponentTraits<T>::manager_ptr;
    manager.remove(entity.*ComponentTraits<T>::handle_ptr);
    entity.*ComponentTraits<T>::handle_ptr = {};
  }

  // remove component of an entity.
  template <typename T>
  void remove(Entity* entity) {
    if (!entity) {
      return;
    }
    remove<T>(*entity);
  }

  // set component of an entity.
  template <typename T>
  T* set(Entity& entity, T&& component) {
    remove<T>(entity);

    Manager<T>& manager = this->*ComponentTraits<T>::manager_ptr;
    Handle new_handle = manager.add(std::forward<T>(component));
    assert(!new_handle.is_empty());

    entity.*ComponentTraits<T>::handle_ptr = new_handle;

    // newly add component always locates at the end of the data vector
    return &manager.data().back();
  }

  // set component of an entity.
  template <typename T>
  T* set(Entity* entity, T&& component) {
    if (!entity) {
      return nullptr;
    }
    return set<T>(*entity, std::forward<T>(component));
  }

  // get entity
  Entity* get_entity(Handle entity_handle) {
    return this->entity.get(entity_handle);
  }

  // get entity
  const Entity* get_entity(Handle entity_handle) const {
    return this->entity.get(entity_handle);
  }

  // get all entities
  std::vector<Entity>& get_all_entities() { return this->entity.data(); }

  // get all entities
  const std::vector<Entity>& get_all_entities() const {
    return this->entity.data();
  }

  // delete entity and all its components
  void remove_entity(Handle entity_handle) {
    Entity* entity = this->entity.get(entity_handle);
    if (!entity) {
      return;
    }

    remove<ClothConfig>(entity);
    remove<CollisionConfig>(entity);
    remove<TriMesh>(entity);
    remove<Pin>(entity);
    remove<ClothStaticSolverData>(entity);
    remove<ClothDynamicSolverData>(entity);
    remove<SolverState>(entity);
    remove<ObstaclePosition>(entity);
    remove<ObjectCollider>(entity);
    this->entity.remove(entity_handle);
  }

  // add new entity
  std::pair<Handle, Entity*> add_entity() {
    Handle h = this->entity.add(Entity{});
    if (h.is_empty()) {
      return std::make_pair(h, nullptr);
    }
    Entity* entity = this->entity.get(h);
    return std::make_pair(h, entity);
  }

  // clear the entire registry, nuke all entities and components
  void clear() {
    entity.clear();
    cloth_config.clear();
    collision_config.clear();
    tri_mesh.clear();
    pin.clear();
    cloth_static_solver_data.clear();
    cloth_dynamic_solver_data.clear();
    solver_state.clear();
    obstacle_position.clear();
    object_collider.clear();
  }
};

#define ECS_SPECIALIZE_COMPONENT_TRAIT(type, name)                            \
  template <>                                                                 \
  struct Registry::ComponentTraits<type> {                                    \
    static constexpr Handle Entity::* handle_ptr = &Entity::name;             \
    static constexpr Manager<type> Registry::* manager_ptr = &Registry::name; \
  };

ECS_SPECIALIZE_COMPONENT_TRAIT(ClothConfig, cloth_config)
ECS_SPECIALIZE_COMPONENT_TRAIT(CollisionConfig, collision_config)
ECS_SPECIALIZE_COMPONENT_TRAIT(TriMesh, tri_mesh)
ECS_SPECIALIZE_COMPONENT_TRAIT(Pin, pin)
ECS_SPECIALIZE_COMPONENT_TRAIT(ClothStaticSolverData, cloth_static_solver_data)
ECS_SPECIALIZE_COMPONENT_TRAIT(ClothDynamicSolverData,
                               cloth_dynamic_solver_data)
ECS_SPECIALIZE_COMPONENT_TRAIT(SolverState, solver_state)
ECS_SPECIALIZE_COMPONENT_TRAIT(ObstaclePosition, obstacle_position)
ECS_SPECIALIZE_COMPONENT_TRAIT(ObjectCollider, object_collider)

}  // namespace silk

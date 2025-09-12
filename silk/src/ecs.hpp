/**
 * @file ecs.hpp
 * @brief Entity Component System implementation for Silk physics simulation.
 *
 * Provides a type-safe ECS registry that manages entities and their associated
 * components. Uses compile-time reflection via ComponentTraits to map component
 * types to their storage locations.
 */

#pragma once

#include <Eigen/Core>

#include "cloth_solver_data.hpp"
#include "handle.hpp"
#include "manager.hpp"
#include "mesh.hpp"
#include "object_collider.hpp"
#include "object_state.hpp"
#include "obstacle_position.hpp"
#include "pin.hpp"
#include "silk/silk.hpp"

namespace silk {

// Template helper to force static_assert failure for unsupported component
// types.
template <typename T>
inline constexpr bool always_false_v = false;

/**
 * @brief Entity record containing handles to all possible components.
 *
 * Acts as an index into the Registry's component managers. Each handle
 * either points to a valid component or is empty.
 * The actual component data is stored in the Registry's Manager instances.
 */
struct Entity {
  Handle self;
  Handle cloth_config;
  Handle cloth_topology;
  Handle cloth_solver_context;
  Handle solver_state;
  Handle collision_config;
  Handle tri_mesh;
  Handle pin;
  Handle obstacle_position;
  Handle object_collider;
};

/**
 * @brief Central ECS registry managing all entities and components.
 *
 * Provides type-safe component access through compile-time reflection.
 * Components are stored in separate Manager instances, with entities
 * holding handles that reference into these managers.
 */
class Registry {
 private:
  /**
   * @brief Compile-time traits mapping component types to storage locations.
   *
   * Specialized for each component type via ECS_SPECIALIZE_COMPONENT_TRAIT.
   * Provides two pointers:
   * - handle_ptr: Points to the Handle field in Entity for this component
   * - manager_ptr: Points to the Manager<T> field in Registry for this
   * component
   *
   * Unspecialized template triggers static_assert for unsupported types.
   */
  template <typename T>
  struct ComponentTraits {
    static_assert(always_false_v<T>, "This type is not an ECS component!!");
  };

  Manager<Entity> entity;
  Manager<ClothConfig> cloth_config;
  Manager<CollisionConfig> collision_config;
  Manager<TriMesh> tri_mesh;
  Manager<Pin> pin;
  Manager<ClothTopology> cloth_topology;
  Manager<ClothSolverContext> cloth_solver_context;
  Manager<ObjectState> solver_state;
  Manager<ObstaclePosition> obstacle_position;
  Manager<ObjectCollider> object_collider;

 public:
  /**
   * @brief Retrieve component of specified type from entity.
   * @param entity Entity to query for component
   * @return Pointer to component or nullptr if not present
   */
  template <typename T>
  T* get(const Entity& entity) {
    Manager<T>& manager = this->*ComponentTraits<T>::manager_ptr;
    Handle h = entity.*ComponentTraits<T>::handle_ptr;
    return manager.get(h);
  }

  /**
   * @brief Retrieve const component of specified type from entity.
   * @param entity Entity to query for component
   * @return Const pointer to component or nullptr if not present
   */
  template <typename T>
  const T* get(const Entity& entity) const {
    const Manager<T>& manager = this->*ComponentTraits<T>::manager_ptr;
    Handle h = entity.*ComponentTraits<T>::handle_ptr;
    return manager.get(h);
  }

  /**
   * @brief Retrieve component from entity pointer with null safety.
   * @param entity Pointer to entity (may be null)
   * @return Pointer to component or nullptr if entity is null or component
   * absent
   */
  template <typename T>
  T* get(const Entity* entity) {
    if (!entity) {
      return nullptr;
    }
    return get<T>(*entity);
  }

  /**
   * @brief Retrieve const component from entity pointer with null safety.
   * @param entity Pointer to entity (may be null)
   * @return Const pointer to component or nullptr if entity is null or
   * component absent
   */
  template <typename T>
  const T* get(const Entity* entity) const {
    if (!entity) {
      return nullptr;
    }
    return get<T>(*entity);
  }

  /**
   * @brief Access all components of specified type across all entities.
   * @return Reference to vector containing all components of type T
   */
  template <typename T>
  std::vector<T>& get_all() {
    Manager<T>& manager = this->*ComponentTraits<T>::manager_ptr;
    return manager.data();
  }

  /**
   * @brief Access all components of specified type (const version).
   * @return Const reference to vector containing all components of type T
   */
  template <typename T>
  const std::vector<T>& get_all() const {
    const Manager<T>& manager = this->*ComponentTraits<T>::manager_ptr;
    return manager.data();
  }

  /**
   * @brief Remove component from entity and invalidate handle.
   * @param entity Entity to remove component from
   */
  template <typename T>
  void remove(Entity& entity) {
    Manager<T>& manager = this->*ComponentTraits<T>::manager_ptr;
    manager.remove(entity.*ComponentTraits<T>::handle_ptr);
    entity.*ComponentTraits<T>::handle_ptr = {};
  }

  /**
   * @brief Remove component from entity pointer with null safety.
   * @param entity Pointer to entity (may be null)
   */
  template <typename T>
  void remove(Entity* entity) {
    if (!entity) {
      return;
    }
    remove<T>(*entity);
  }

  /**
   * @brief Set/replace component on entity
   * @param entity Entity to attach component to
   * @param component Component data to move
   * @return Pointer to newly created component
   */
  template <typename T>
  T* set(Entity& entity, T&& component) {
    remove<T>(entity);

    Manager<T>& manager = this->*ComponentTraits<T>::manager_ptr;
    Handle new_handle = manager.add(std::forward<T>(component));
    assert(!new_handle.is_empty());

    entity.*ComponentTraits<T>::handle_ptr = new_handle;

    // Newly added component always locates at the end of the data vector.
    return &manager.data().back();
  }

  /**
   * @brief Set/replace component on entity pointer with null safety.
   * @param entity Pointer to entity (may be null)
   * @param component Component data to move
   * @return Pointer to newly created component or nullptr if entity is null
   */
  template <typename T>
  T* set(Entity* entity, T&& component) {
    if (!entity) {
      return nullptr;
    }
    return set<T>(*entity, std::forward<T>(component));
  }

  /**
   * @brief Retrieve entity by handle.
   * @param entity_handle Handle to the entity
   * @return Pointer to entity or nullptr if handle is invalid
   */
  Entity* get_entity(Handle entity_handle) {
    return this->entity.get(entity_handle);
  }

  /**
   * @brief Retrieve entity by handle (const version).
   * @param entity_handle Handle to the entity
   * @return Const pointer to entity or nullptr if handle is invalid
   */
  const Entity* get_entity(Handle entity_handle) const {
    return this->entity.get(entity_handle);
  }

  /** @brief Access all entities in the registry. */
  std::vector<Entity>& get_all_entities() { return this->entity.data(); }

  /** @brief Access all entities in the registry (const version). */
  const std::vector<Entity>& get_all_entities() const {
    return this->entity.data();
  }

  /**
   * @brief Remove entity and all its associated components.
   * @param entity_handle Handle to the entity to remove
   */
  void remove_entity(Handle entity_handle) {
    Entity* entity = this->entity.get(entity_handle);
    if (!entity) {
      return;
    }

    remove<ClothConfig>(entity);
    remove<CollisionConfig>(entity);
    remove<TriMesh>(entity);
    remove<Pin>(entity);
    remove<ClothTopology>(entity);
    remove<ClothSolverContext>(entity);
    remove<ObjectState>(entity);
    remove<ObstaclePosition>(entity);
    remove<ObjectCollider>(entity);
    this->entity.remove(entity_handle);
  }

  /**
   * @brief Create a new entity with no components.
   * @return Pair of (handle, entity pointer) or (empty handle, nullptr) on
   * failure
   */
  std::pair<Handle, Entity*> add_entity() {
    Handle h = this->entity.add(Entity{});
    if (h.is_empty()) {
      return std::make_pair(h, nullptr);
    }
    Entity* entity = this->entity.get(h);
    entity->self = h;
    return std::make_pair(h, entity);
  }

  /**
   * @brief Clear entire registry, removing all entities and components.
   */
  void clear() {
    entity.clear();
    cloth_config.clear();
    collision_config.clear();
    tri_mesh.clear();
    pin.clear();
    cloth_topology.clear();
    cloth_solver_context.clear();
    solver_state.clear();
    obstacle_position.clear();
    object_collider.clear();
  }
};

/**
 * @brief Macro to specialize ComponentTraits for a component type.
 * @param type Component type (e.g., ClothConfig)
 * @param name Field name in Entity and Registry (e.g., cloth_config)
 */
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
ECS_SPECIALIZE_COMPONENT_TRAIT(ClothTopology, cloth_topology)
ECS_SPECIALIZE_COMPONENT_TRAIT(ClothSolverContext, cloth_solver_context)
ECS_SPECIALIZE_COMPONENT_TRAIT(ObjectState, solver_state)
ECS_SPECIALIZE_COMPONENT_TRAIT(ObstaclePosition, obstacle_position)
ECS_SPECIALIZE_COMPONENT_TRAIT(ObjectCollider, object_collider)

}  // namespace silk

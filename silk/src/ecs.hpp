/**
 * @file ecs.hpp
 * @brief Declares the ECS registry and component handles.
 *
 * Exposes the public API and compile-time traits that bind component types to
 * storage without pulling in heavyweight implementation headers. Definitions
 * live in ecs.cpp so that registry internals remain private and templates are
 * instantiated in one translation unit, keeping compile times predictable.
 *
 * P.S. If you can think of a way to implement this ECS system without abusing
 * macros, please make a pull request.
 */

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "handle.hpp"

namespace silk {

// Central registry of ECS component types.
// To introduce a new component, add its type name and handle name to
// this list, then include the component's header in ecs.cpp so the traits and
// explicit instantiations can see the definition.
// These entries drive every ECS_X_MACRO expansion for declarations, handles,
// traits, and template instantiations.
#define ECS_X_MACRO(X)                        \
  X(ClothConfig, cloth_config)                \
  X(CollisionConfig, collision_config)        \
  X(TriMesh, tri_mesh)                        \
  X(Pin, pin)                                 \
  X(ClothTopology, cloth_topology)            \
  X(ClothSolverContext, cloth_solver_context) \
  X(ObjectState, object_state)                \
  X(ObstaclePosition, obstacle_position)      \
  X(ObjectCollider, object_collider)

// Forward declarations of component types.
#define ECS_FORWARD_DECLARATION(type, name) struct type;
ECS_X_MACRO(ECS_FORWARD_DECLARATION);
#undef ECS_FORWARD_DECLARATION

// Helper to trigger static_assert for unsupported component types.
template <typename T>
inline constexpr bool ALWAYS_FALSE_V = false;

/**
 * @brief Entity record containing handles to all possible components.
 *
 * Acts as an index into the Registry's component managers. Each handle either
 * points to a valid component or is empty. The actual component data is stored
 * in the Registry's Manager instances.
 */
#define ECS_ENTITY_DEFINITION(type, name) Handle name;
struct Entity {
  Handle self;
  ECS_X_MACRO(ECS_ENTITY_DEFINITION)
};
#undef ECS_ENTITY_DEFINITION

/**
 * @brief Central ECS registry managing all entities and components.
 */
class Registry {
  struct Impl;
  std::unique_ptr<Impl> impl_;

  /**
   * @brief Compile-time traits mapping component types to storage locations.
   *
   * Specialized in ecs.cpp for each supported component type. Primary template
   * intentionally fails on use for unsupported types.
   *
   * Specializations provide:
   * - HANDLE_PTR  : pointer to the Handle field in Entity for this component
   * - MANAGER_PTR : pointer to the Manager<T> field in Impl for this component
   */
  template <typename T>
  struct ComponentTraits {
    static_assert(ALWAYS_FALSE_V<T>, "This type is not an ECS component!!");
  };

 public:
  Registry();
  Registry(Registry&) = delete;
  Registry(Registry&& other) noexcept;

  ~Registry();

  Registry& operator=(Registry&) = delete;
  Registry& operator=(Registry&& other) noexcept;

  // -----------------------------------------------------
  // Component APIs
  // -----------------------------------------------------

  /**
   * @brief Retrieve component from entity.
   * @param entity Entity to query for component
   * @return Pointer to component or nullptr if not present
   */
  template <typename T>
  T* get(const Entity& entity);

  /**
   * @brief Retrieve const component from entity.
   * @param entity Entity to query for component
   * @return Const pointer to component or nullptr if not present
   */
  template <typename T>
  const T* get(const Entity& entity) const;

  /**
   * @brief Retrieve component from entity pointer with null safety.
   * @param entity Pointer to entity (may be nullptr)
   * @return Pointer to component or nullptr if entity is null or component
   * absent
   */
  template <typename T>
  T* get(const Entity* entity);

  /**
   * @brief Retrieve const component from entity pointer with null safety.
   * @param entity Pointer to entity (may be nullptr)
   * @return Const pointer to component or nullptr if entity is null or
   * component absent
   */
  template <typename T>
  const T* get(const Entity* entity) const;

  /**
   * @brief Access all components of specified type across all entities.
   * @return Reference to vector containing all components of type T
   */
  template <typename T>
  std::vector<T>& get_all();

  /**
   * @brief Access all components of specified type (const version).
   * @return Const reference to vector containing all components of type T
   */
  template <typename T>
  const std::vector<T>& get_all() const;

  /**
   * @brief Remove component from entity and invalidate handle. No-op if
   * component is absent.
   * @param entity Entity to remove component from
   */
  template <typename T>
  void remove(Entity& entity);

  /**
   * @brief Remove component from entity pointer with null safety. No-op if
   * component is absent.
   * @param entity Pointer to entity (may be nullptr)
   */
  template <typename T>
  void remove(Entity* entity);

  /**
   * @brief Set/replace component on entity.
   * @param entity Entity to attach component to
   * @param component Component data to move
   * @return Pointer to newly created component or nullptr if fails.
   */
  template <typename T>
  T* set(Entity& entity, T&& component);

  /**
   * @brief Set/replace component on entity pointer with null safety.
   * @param entity Pointer to entity (may be nullptr)
   * @param component Component data to move
   * @return Pointer to newly created component or nullptr if fails.
   */
  template <typename T>
  T* set(Entity* entity, T&& component);

  // -----------------------------------------------------
  // Entity APIs
  // -----------------------------------------------------

  /**
   * @brief Retrieve entity by handle.
   * @param entity_handle Handle to the entity
   * @return Pointer to entity or nullptr if handle is invalid
   */
  Entity* get_entity(Handle entity_handle);

  /**
   * @brief Retrieve entity by handle (const version).
   * @param entity_handle Handle to the entity
   * @return Const pointer to entity or nullptr if handle is invalid
   */
  const Entity* get_entity(Handle entity_handle) const;

  /** @brief Access all entities in the registry. */
  std::vector<Entity>& get_all_entities();

  /** @brief Access all entities in the registry (const version). */
  const std::vector<Entity>& get_all_entities() const;

  /**
   * @brief Create a new entity with no components.
   * @return Pair of (handle, entity pointer) or (empty handle, nullptr) on
   * failure
   */
  std::pair<Handle, Entity*> add_entity();

  /**
   * @brief Remove entity and all its associated components.
   * @param entity_handle Handle to the entity to remove
   */
  void remove_entity(Handle entity_handle);

  /**
   * @brief Clear entire registry, removing all entities and components.
   */
  void clear();
};

}  // namespace silk

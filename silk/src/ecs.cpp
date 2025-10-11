/**
 * @file ecs.cpp
 * @brief ECS registry implementation and template instantiations.
 */

#include "ecs.hpp"

#include <cassert>
#include <silk/silk.hpp>

#include "cloth_solver_data.hpp"
#include "manager.hpp"
#include "mesh.hpp"
#include "object_collider.hpp"
#include "object_state.hpp"
#include "obstacle_position.hpp"
#include "pin.hpp"

namespace silk {

struct Registry::Impl {
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
};

// ComponentTraits specializations
#define SILK_ECS_SPECIALIZE_COMPONENT_TRAIT(type, name)            \
  template <>                                                      \
  struct Registry::ComponentTraits<type> {                         \
    static constexpr Handle Entity::* HANDLE_PTR = &Entity::name;  \
    static constexpr Manager<type> Registry::Impl::* MANAGER_PTR = \
        &Registry::Impl::name;                                     \
  };

SILK_ECS_SPECIALIZE_COMPONENT_TRAIT(ClothConfig, cloth_config)
SILK_ECS_SPECIALIZE_COMPONENT_TRAIT(CollisionConfig, collision_config)
SILK_ECS_SPECIALIZE_COMPONENT_TRAIT(TriMesh, tri_mesh)
SILK_ECS_SPECIALIZE_COMPONENT_TRAIT(Pin, pin)
SILK_ECS_SPECIALIZE_COMPONENT_TRAIT(ClothTopology, cloth_topology)
SILK_ECS_SPECIALIZE_COMPONENT_TRAIT(ClothSolverContext, cloth_solver_context)
SILK_ECS_SPECIALIZE_COMPONENT_TRAIT(ObjectState, solver_state)
SILK_ECS_SPECIALIZE_COMPONENT_TRAIT(ObstaclePosition, obstacle_position)
SILK_ECS_SPECIALIZE_COMPONENT_TRAIT(ObjectCollider, object_collider)

#undef SILK_ECS_SPECIALIZE_COMPONENT_TRAIT

Registry::Registry() : impl_(std::make_unique<Impl>()) {}
Registry::Registry(Registry&& other) noexcept = default;
Registry::~Registry() = default;
Registry& Registry::operator=(Registry&& other) noexcept = default;

template <typename T>
T* Registry::get(const Entity& entity) {
  Impl* impl_ptr = impl_.get();
  Manager<T>& manager = impl_ptr->*ComponentTraits<T>::MANAGER_PTR;
  Handle handle = entity.*ComponentTraits<T>::HANDLE_PTR;
  return manager.get(handle);
}

template <typename T>
const T* Registry::get(const Entity& entity) const {
  const Impl* impl_ptr = impl_.get();
  const Manager<T>& manager = impl_ptr->*ComponentTraits<T>::MANAGER_PTR;
  Handle handle = entity.*ComponentTraits<T>::HANDLE_PTR;
  return manager.get(handle);
}

template <typename T>
T* Registry::get(const Entity* entity) {
  if (entity == nullptr) {
    return nullptr;
  }
  return get<T>(*entity);
}

template <typename T>
const T* Registry::get(const Entity* entity) const {
  if (entity == nullptr) {
    return nullptr;
  }
  return get<T>(*entity);
}

template <typename T>
std::vector<T>& Registry::get_all() {
  Impl* impl_ptr = impl_.get();
  Manager<T>& manager = impl_ptr->*ComponentTraits<T>::MANAGER_PTR;
  return manager.data();
}

template <typename T>
const std::vector<T>& Registry::get_all() const {
  const Impl* impl_ptr = impl_.get();
  const Manager<T>& manager = impl_ptr->*ComponentTraits<T>::MANAGER_PTR;
  return manager.data();
}

template <typename T>
void Registry::remove(Entity& entity) {
  Impl* impl_ptr = impl_.get();
  Manager<T>& manager = impl_ptr->*ComponentTraits<T>::MANAGER_PTR;
  manager.remove(entity.*ComponentTraits<T>::HANDLE_PTR);
  entity.*ComponentTraits<T>::HANDLE_PTR = {};
}

template <typename T>
void Registry::remove(Entity* entity) {
  if (entity == nullptr) {
    return;
  }
  remove<T>(*entity);
}

template <typename T>
T* Registry::set(Entity& entity, T&& component) {
  remove<T>(entity);

  Impl* impl_ptr = impl_.get();
  Manager<T>& manager = impl_ptr->*ComponentTraits<T>::MANAGER_PTR;
  Handle new_handle = manager.add(std::forward<T>(component));
  if (new_handle.is_empty()) {
    return nullptr;
  }
  entity.*ComponentTraits<T>::HANDLE_PTR = new_handle;

  // Newly added component always locates at the end of the data vector.
  return &manager.data().back();
}

template <typename T>
T* Registry::set(Entity* entity, T&& component) {
  if (entity == nullptr) {
    return nullptr;
  }
  return set<T>(*entity, std::forward<T>(component));
}

Entity* Registry::get_entity(Handle entity_handle) {
  return impl_->entity.get(entity_handle);
}

const Entity* Registry::get_entity(Handle entity_handle) const {
  return impl_->entity.get(entity_handle);
}

std::vector<Entity>& Registry::get_all_entities() {
  return impl_->entity.data();
}

const std::vector<Entity>& Registry::get_all_entities() const {
  return impl_->entity.data();
}

std::pair<Handle, Entity*> Registry::add_entity() {
  Handle handle = impl_->entity.add(Entity{});
  if (handle.is_empty()) {
    return {handle, nullptr};
  }
  Entity* entity = impl_->entity.get(handle);
  entity->self = handle;
  return {handle, entity};
}

void Registry::remove_entity(Handle entity_handle) {
  Entity* entity = impl_->entity.get(entity_handle);
  if (entity == nullptr) {
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

  impl_->entity.remove(entity_handle);
}

void Registry::clear() {
  impl_->entity.clear();
  impl_->cloth_config.clear();
  impl_->collision_config.clear();
  impl_->tri_mesh.clear();
  impl_->pin.clear();
  impl_->cloth_topology.clear();
  impl_->cloth_solver_context.clear();
  impl_->solver_state.clear();
  impl_->obstacle_position.clear();
  impl_->object_collider.clear();
}

// Template initialization.

#define SILK_ECS_COMPONENT_TYPES(X) \
  X(ClothConfig)                    \
  X(CollisionConfig)                \
  X(TriMesh)                        \
  X(Pin)                            \
  X(ClothTopology)                  \
  X(ClothSolverContext)             \
  X(ObjectState)                    \
  X(ObstaclePosition)               \
  X(ObjectCollider)

#define SILK_ECS_INSTANTIATE_FOR(T_)                                \
  template T_* Registry::get<T_>(const Entity& entity);             \
  template const T_* Registry::get<T_>(const Entity& entity) const; \
  template T_* Registry::get<T_>(const Entity* entity);             \
  template const T_* Registry::get<T_>(const Entity* entity) const; \
  template std::vector<T_>& Registry::get_all<T_>();                \
  template const std::vector<T_>& Registry::get_all<T_>() const;    \
  template void Registry::remove<T_>(Entity & entity);              \
  template void Registry::remove<T_>(Entity * entity);              \
  template T_* Registry::set<T_>(Entity & entity, T_ && component); \
  template T_* Registry::set<T_>(Entity * entity, T_ && component);

SILK_ECS_COMPONENT_TYPES(SILK_ECS_INSTANTIATE_FOR)

#undef SILK_ECS_INSTANTIATE_FOR
#undef SILK_ECS_COMPONENT_TYPES

}  // namespace silk

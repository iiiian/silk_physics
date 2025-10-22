#include "ecs.hpp"

// Make sure you've include all component definition here.

#include <cassert>
#include <silk/silk.hpp>

#include "cloth_topology.hpp"
#include "collision/cpu/object_collider.hpp"
#include "manager.hpp"
#include "mesh.hpp"
#include "object_state.hpp"
#include "obstacle_position.hpp"
#include "pin.hpp"
#include "solver/cpu/cloth_solver_context.hpp"

namespace silk {

#define ECS_REGISTRY_IMPL_DEFINITION(type, name) Manager<type> name;
struct Registry::Impl {
  Manager<Entity> entity;
  ECS_X_MACRO(ECS_REGISTRY_IMPL_DEFINITION)
};
#undef ECS_REGISTRY_IMPL_DEFINITION

// ComponentTraits specializations
#define ECS_SPECIALIZE_COMPONENT_TRAIT(type, name)                 \
  template <>                                                      \
  struct Registry::ComponentTraits<type> {                         \
    static constexpr Handle Entity::* HANDLE_PTR = &Entity::name;  \
    static constexpr Manager<type> Registry::Impl::* MANAGER_PTR = \
        &Registry::Impl::name;                                     \
  };
ECS_X_MACRO(ECS_SPECIALIZE_COMPONENT_TRAIT)
#undef ECS_SPECIALIZE_COMPONENT_TRAIT

#undef ECS_SPECIALIZE_COMPONENT_TRAIT

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

#define ECS_REMOVE_COMPONENT(type, name) remove<type>(entity);
  ECS_X_MACRO(ECS_REMOVE_COMPONENT)
#undef ECS_REMOVE_COMPONENT

  impl_->entity.remove(entity_handle);
}

void Registry::clear() {
  impl_->entity.clear();

#define ECS_CLEAR_MANAGER(type, name) impl_->name.clear();
  ECS_X_MACRO(ECS_CLEAR_MANAGER)
#undef ECS_CLEAR_MANAGER
}

// Template initialization.
#define ECS_INSTANTIATE_FOR(type, name)                                   \
  template type* Registry::get<type>(const Entity& entity);               \
  template const type* Registry::get<type>(const Entity& entity) const;   \
  template type* Registry::get<type>(const Entity* entity);               \
  template const type* Registry::get<type>(const Entity* entity) const;   \
  template std::vector<type>& Registry::get_all<type>();                  \
  template const std::vector<type>& Registry::get_all<type>() const;      \
  template void Registry::remove<type>(Entity & entity);                  \
  template void Registry::remove<type>(Entity * entity);                  \
  template type* Registry::set<type>(Entity & entity, type && component); \
  template type* Registry::set<type>(Entity * entity, type && component);
ECS_X_MACRO(ECS_INSTANTIATE_FOR)
#undef ECS_INSTANTIATE_FOR

}  // namespace silk

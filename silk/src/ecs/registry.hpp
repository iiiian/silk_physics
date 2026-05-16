#pragma once

#include <cassert>
#include <cstdint>
#include <limits>
#include <random>
#include <ranges>
#include <span>
#include <tuple>
#include <type_traits>
#include <unordered_set>

#include "ecs/component_storage.hpp"

namespace silk {

/// @brief Type-level list of component types.
template <typename... T>
struct ComponentList {};

/// @brief Optional customization point for dependency-driven component removal.
///
/// Specialize this trait for a registry/component pair to describe components
/// dependency.
///
/// This is invalidation only. The registry does not auto-create missing
/// components. Dependency graphs are expected to be acyclic.
///
/// Example:
///
/// Suppose DerivedComponentA and DerivedComponentB depends on SourceComponent.
///
/// @code
/// template <>
/// struct ComponentDependents<MyRegistry, SourceComponent> {
///   using Type = ComponentList<DerivedComponentA, DerivedComponentB>;
/// };
/// @endcode
///
/// @tparam RegistryType Registry type the rule applies to.
/// @tparam ComponentType Source component type being removed or replaced.
template <typename RegistryType, typename ComponentType>
struct ComponentDependents {
  /// @brief No dependents by default. Backends opt in by specializing.
  using Type = ComponentList<>;
};

template <typename... C>
class Registry {
 public:
  Registry() = default;
  ~Registry() = default;

  // Copying registry is super expensive. This should never happens.

  Registry(const Registry &) = delete;
  Registry(Registry &&other) noexcept = default;

  Registry &operator=(const Registry &) = delete;
  Registry &operator=(Registry &&other) noexcept = default;

  // -----------------------------------------------------
  // Entity APIs
  // -----------------------------------------------------

  size_t entity_num() const { return entities_.size(); };

  bool has_entity(uint32_t entity) const { return entities_.contains(entity); }

  uint32_t make_entity() {
    // I don't think registry will be near full.
    uint32_t entity;
    do {
      std::random_device rd;
      std::mt19937 engine(rd());
      // In legacy ECS zero implies invalid/empty entity.
      // Some demo code still depends on this behavior.
      std::uniform_int_distribution<uint32_t> dist(
          1, std::numeric_limits<uint32_t>::max());
      entity = dist(engine);
    } while (entities_.contains(entity));

    entities_.insert(entity);
    return entity;
  }

  void nuke_entity(uint32_t entity) {
    (remove_direct<C>(entity), ...);
    entities_.erase(entity);
  }

  auto get_all_entities() const { return entities_ | std::views::all; }

  /// @brief Gather all entities with compoents T...
  /// @tparam T All required components.
  /// @return Vector of entity id.
  template <typename... T>
  auto get_entity_with_components() const {
    return entities_ | std::views::filter([this](uint32_t e) {
             return (has_component<T>(e) && ...);
           });
  }

  // -----------------------------------------------------
  // Component APIs
  // -----------------------------------------------------

  /// @brief Return true if entity has component T.
  template <typename T>
  bool has_component(uint32_t entity) const {
    static_assert((std::is_same_v<T, C> || ...),
                  "T is not a component type, double check T appears as "
                  "template argument in registry declaration.");

    auto &c = std::get<ComponentStorage<T>>(components_);
    return (c.get(entity) != nullptr);
  }

  /// @brief Return ptr to entity component T. nullptr if not exists.
  template <typename T>
  T *get(uint32_t entity) {
    static_assert((std::is_same_v<T, C> || ...),
                  "T is not a component type, double check T appears as "
                  "template argument in registry declaration.");

    auto &c = std::get<ComponentStorage<T>>(components_);
    return c.get(entity);
  }

  /// @brief Return ptr to entity component T. nullptr if not exists.
  template <typename T>
  const T *get(uint32_t entity) const {
    static_assert((std::is_same_v<T, C> || ...),
                  "T is not a component type, double check T appears as "
                  "template argument in registry declaration.");

    auto &c = std::get<ComponentStorage<T>>(components_);
    return c.get(entity);
  }

  /// @brief Return all components of type T.
  template <typename T>
  std::span<T> get_all_components() {
    static_assert((std::is_same_v<T, C> || ...),
                  "T is not a component type, double check T appears as "
                  "template argument in registry declaration.");

    auto &c = std::get<ComponentStorage<T>>(components_);
    return c.data();
  }

  /// @brief Return all components of type T.
  template <typename T>
  std::span<const T> get_all_components() const {
    static_assert((std::is_same_v<T, C> || ...),
                  "T is not a component type, double check T appears as "
                  "template argument in registry declaration.");

    auto &c = std::get<ComponentStorage<T>>(components_);
    return c.data();
  }

  /// @brief Remove component T and its dependents from entity and invalidate
  /// handle. No-op if entity does not have such component.
  template <typename T>
  void remove(uint32_t entity) {
    static_assert((std::is_same_v<T, C> || ...),
                  "T is not a component type, double check T appears as "
                  "template argument in registry declaration.");

    remove_dependents<T>(entity);
    remove_direct<T>(entity);
  }

  /// @brief Remove all components of type T and their dependents.
  template <typename T>
  void remove_all_components() {
    static_assert((std::is_same_v<T, C> || ...),
                  "T is not a component type, double check T appears as "
                  "template argument in registry declaration.");

    for (uint32_t entity : entities_) {
      remove_dependents<T>(entity);
    }
    auto &c = std::get<ComponentStorage<T>>(components_);
    c.clear();
  }

  /// @brief Set/Replace component T of entity.
  /// @return Pointer to newly modified component. nullptr if entity is invalid.
  template <typename T>
  T *set(uint32_t entity, T component) {
    static_assert((std::is_same_v<T, C> || ...),
                  "T is not a component type, double check T appears as "
                  "template argument in registry declaration.");
    if (!has_entity(entity)) {
      return nullptr;
    }

    auto &c = std::get<ComponentStorage<T>>(components_);
    return c.set(entity, std::move(component));
  }

  /// @brief Remove dependents then set/replace component T of entity.
  /// @return Pointer to newly modified component. nullptr if entity is invalid.
  template <typename T>
  T *remove_deps_then_set(uint32_t entity, T component) {
    static_assert((std::is_same_v<T, C> || ...),
                  "T is not a component type, double check T appears as "
                  "template argument in registry declaration.");
    if (!has_entity(entity)) {
      return nullptr;
    }

    if (has_component<T>(entity)) {
      remove_dependents<T>(entity);
    }
    auto &c = std::get<ComponentStorage<T>>(components_);
    return c.set(entity, std::move(component));
  }

 private:
  template <typename T>
  void remove_direct(uint32_t entity) {
    auto &c = std::get<ComponentStorage<T>>(components_);
    c.remove(entity);
  }

  template <typename T>
  void remove_dependents(uint32_t entity) {
    using Dependents = typename ComponentDependents<Registry, T>::Type;
    remove_component_list(entity, Dependents{});
  }

  template <typename... T>
  void remove_component_list(uint32_t entity, ComponentList<T...>) {
    (remove<T>(entity), ...);
  }

  std::unordered_set<uint32_t> entities_;
  std::tuple<ComponentStorage<C>...> components_;
};

}  // namespace silk

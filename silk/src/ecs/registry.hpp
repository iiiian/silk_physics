#pragma once

#include <cassert>
#include <cstdint>
#include <limits>
#include <random>
#include <span>
#include <tuple>
#include <unordered_set>

#include "ecs/component_storage.hpp"

namespace silk::ecs {

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

  uint32_t make_entity() {
    // I don't think registry will be near full.
    uint32_t entity;
    do {
      std::random_device rd;
      std::mt19937 engine(rd());
      std::uniform_int_distribution<uint32_t> dist(
          0, std::numeric_limits<uint32_t>::max());
      entity = dist(engine);
    } while (!entities_.contains(entity));

    return entity;
  }

  void nuke_entity(uint32_t entity) {
    (remove<C>(entity), ...);
    entities_.erase(entity);
  }

  /// @brief Gather all entities with compoents T...
  /// @tparam T All required components.
  /// @return Vector of entity id.
  template <typename... T>
  std::vector<uint32_t> get_entity_with_components() const {
    std::vector<uint32_t> ret;
    for (uint32_t e : entities_) {
      if ((has_component<T>(e) && ...)) {
        ret.push_back(e);
      }
    }
    return ret;
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

  /// @brief Return ptr to component T of entity. nullptr if not exists.
  template <typename T>
  T *get(uint32_t entity) {
    static_assert((std::is_same_v<T, C> || ...),
                  "T is not a component type, double check T appears as "
                  "template argument in registry declaration.");

    auto &c = std::get<ComponentStorage<T>>(components_);
    return c.get(entity);
  }

  /// @brief Return ptr to component T of entity. nullptr if not exists.
  template <typename T>
  const T *get(int entity) const {
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

  /// @brief Remove component T from entity and invalidate handle.
  /// No-op if entity does not have such component.
  template <typename T>
  void remove(uint32_t entity) {
    static_assert((std::is_same_v<T, C> || ...),
                  "T is not a component type, double check T appears as "
                  "template argument in registry declaration.");

    auto &c = std::get<ComponentStorage<T>>(components_);
    c.remove(entity);
  }

  /// @brief Remove all components of type T.
  template <typename T>
  void remove_all_components() {
    static_assert((std::is_same_v<T, C> || ...),
                  "T is not a component type, double check T appears as "
                  "template argument in registry declaration.");

    auto &c = std::get<ComponentStorage<T>>(components_);
    c.clear();
  }

  /// @brief Set/Replace component T of entity.
  /// @return Pointer to newly modified component.
  template <typename T>
  T *set(uint32_t entity, T component) {
    static_assert((std::is_same_v<T, C> || ...),
                  "T is not a component type, double check T appears as "
                  "template argument in registry declaration.");

    auto &c = std::get<ComponentStorage<T>>(components_);
    return c.set(entity, std::move(component));
  }

 private:
  std::unordered_set<uint32_t> entities_;
  std::tuple<ComponentStorage<C>...> components_;
};

}  // namespace silk::ecs

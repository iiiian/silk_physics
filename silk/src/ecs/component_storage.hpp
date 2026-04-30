#pragma once

#include <cstdint>
#include <span>
#include <unordered_map>
#include <vector>

namespace silk::ecs {

template <typename T>
class ComponentStorage {
 private:
  std::unordered_map<uint32_t, size_t> map_;
  std::vector<uint32_t> inv_map_;
  std::vector<T> data_;

 public:
  /// @brief Invalidates all existing handles and clears all resources.
  void clear() {
    map_.clear();
    inv_map_.clear();
    data_.clear();
  }

  /// @brief Retrieves mutable resource pointer.
  /// @return Pointer to resource or nullptr if not exists.
  T* get(uint32_t entity) {
    if (map_.contains(entity)) {
      return data_.data() + map_.at(entity);
    }
    return nullptr;
  }

  /// @brief Retrieves immutable resource pointer.
  /// @return Pointer to resource or nullptr if not exists.
  const T* get(uint32_t entity) const {
    if (map_.contains(entity)) {
      return data_.data() + map_.at(entity);
    }
    return nullptr;
  }

  /// @brief Add/Set component.
  /// @return Pointer to newly modified component.
  T* set(uint32_t entity, T component) {
    if (map_.contains(entity)) {
      data_[map_.at(entity)] = std::move(component);
      return data_.data() + map_.at(entity);
    }
    map_.emplace(entity, data_.size());
    inv_map_.push_back(entity);
    data_.push_back(std::move(component));
    return data_.data() + map_.at(entity);
  }

  /// @brief Remove component.
  /// @return True if removed success, false if component does not exists.
  bool remove(uint32_t entity) {
    if (!map_.contains(entity)) {
      return false;
    }

    size_t idx = map_.at(entity);
    // component is at the back, remove directly.
    if (idx == data_.size() - 1) {
      data_.resize(idx);
      inv_map_.resize(idx);
      map_.erase(entity);
    }
    // swap then remove.
    else {
      std::swap(data_[idx], data_.back());
      std::swap(inv_map_[idx], inv_map_.back());
      map_.at(inv_map_[idx]) = idx;
      size_t new_size = data_.size() - 1;
      data_.resize(new_size);
      inv_map_.resize(new_size);
      map_.erase(entity);
    }

    return true;
  }

  /// @brief Returns mutable reference to dense component array.
  std::span<T>& data() { return data_; }

  /// @brief Returns immutable reference to dense component array.
  std::span<const T>& data() const { return data_; }
};

}  // namespace silk::ecs

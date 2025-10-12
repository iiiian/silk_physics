#pragma once

#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include "handle.hpp"

namespace silk {

/**
 * @brief A generational resource manager using stable handles.
 *
 * Provides safe resource management with automatic handle invalidation.
 * Resources are stored in a dense array for cache efficiency, while handles
 * provide stable references that become invalid when resources are deleted.
 * Uses a slot-based indirection system.
 *
 * @tparam T Resource type to manage
 */
template <typename T>
class Manager {
 private:
  static constexpr uint32_t INITIAL_CAPACITY = 1024;
  static_assert(INITIAL_CAPACITY <= Handle::MAX_INDEX,
                "Initial capacity larger max handle index.");

  // Indirection table mapping handle indices to data array indices.
  std::vector<Handle> slots_;
  // Queue of available slot indices for new allocations.
  std::deque<uint32_t> free_slots_;
  // Dense array storing actual resource data for cache efficiency.
  std::vector<T> data_;
  // Reverse mapping from data array index to slot index.
  std::vector<uint32_t> slot_of_data_;

 public:
  Manager() {
    slots_.resize(INITIAL_CAPACITY, Handle{});
    for (uint32_t i = 0; i < INITIAL_CAPACITY; ++i) {
      free_slots_.push_back(i);
    }
  }

  /**
   * @brief Invalidates all existing handles and clears all resources.
   */
  void clear() {
    for (Handle& s : slots_) {
      s.set_is_valid(false);
      s.increment_generation();
    }

    free_slots_.clear();
    for (uint32_t i = 0; i < slots_.size(); ++i) {
      free_slots_.push_back(i);
    }

    data_.clear();
    slot_of_data_.clear();
  }

  /**
   * @brief Retrieves mutable resource pointer from handle.
   *
   * @param handle Handle to the resource
   * @return Pointer to resource or nullptr if handle is invalid
   */
  T* get(Handle handle) {
    if (handle.is_empty()) {
      return nullptr;
    }

    Handle& slot = slots_[handle.get_index()];
    if (!slot.get_is_valid() ||
        slot.get_generation() != handle.get_generation()) {
      return nullptr;
    }

    return data_.data() + slot.get_index();
  }

  /**
   * @brief Retrieves immutable resource pointer from handle.
   *
   * @param handle Handle to the resource
   * @return Const pointer to resource or nullptr if handle is invalid
   */
  const T* get(Handle handle) const {
    if (handle.is_empty()) {
      return nullptr;
    }

    const Handle& slot = slots_[handle.get_index()];
    if (!slot.get_is_valid() ||
        slot.get_generation() != handle.get_generation()) {
      return nullptr;
    }

    return data_.data() + slot.get_index();
  }

  /**
   * @brief Adds a new resource and returns its handle.
   *
   * @param component Resource to add
   * @return Handle to new resource, or empty handle if out of capacity
   */
  Handle add(const T& component) {
    if (free_slots_.empty() && slots_.size() >= Handle::MAX_INDEX) {
      return Handle{};  // At maximum capacity
    }

    uint32_t slot_idx;
    if (free_slots_.empty()) {
      slot_idx = static_cast<uint32_t>(slots_.size());
      slots_.push_back(Handle{});
    } else {
      slot_idx = free_slots_.front();
      free_slots_.pop_front();
    }

    Handle& slot = slots_[slot_idx];
    slot.set_is_valid(true);
    slot.set_index(data_.size());
    data_.push_back(component);
    slot_of_data_.push_back(slot_idx);
    return Handle{true, slot.get_generation(), slot_idx};
  }

  Handle add(T&& component) {
    if (free_slots_.empty() && slots_.size() >= Handle::MAX_INDEX) {
      return Handle{};  // At maximum capacity
    }

    uint32_t slot_idx;
    if (free_slots_.empty()) {
      slot_idx = static_cast<uint32_t>(slots_.size());
      slots_.push_back(Handle{});
    } else {
      slot_idx = free_slots_.front();
      free_slots_.pop_front();
    }

    Handle& slot = slots_[slot_idx];
    slot.set_is_valid(true);
    slot.set_index(data_.size());
    data_.push_back(std::move(component));
    slot_of_data_.push_back(slot_idx);
    return Handle{true, slot.get_generation(), slot_idx};
  }

  /**
   * @brief Removes resource and invalidates its handle.
   *
   * @param handle Handle to resource to remove
   * @return true if resource was removed, false if handle was invalid
   */
  bool remove(const Handle& handle) {
    if (handle.is_empty()) {
      return false;
    }

    Handle& dead_slot = slots_[handle.get_index()];
    if (!dead_slot.get_is_valid() ||
        dead_slot.get_generation() != handle.get_generation()) {
      return false;
    }

    // Swap-and-pop: move last element to fill gap, maintaining dense layout
    data_[dead_slot.get_index()] = std::move_if_noexcept(data_.back());
    slots_[slot_of_data_.back()].set_index(dead_slot.get_index());
    slot_of_data_[dead_slot.get_index()] = slot_of_data_.back();
    data_.pop_back();
    slot_of_data_.pop_back();

    // Invalidate slot and increment generation to prevent stale handle reuse
    dead_slot.set_is_valid(false);
    dead_slot.increment_generation();
    free_slots_.push_front(handle.get_index());

    return true;
  }

  /**
   * @brief Returns mutable reference to dense resource array.
   *
   * @return Reference to dense data vector
   */
  std::vector<T>& data() { return data_; }

  /**
   * @brief Returns immutable reference to dense resource array.
   *
   * @return Const reference to dense data vector
   */
  const std::vector<T>& data() const { return data_; }
};

};  // namespace silk

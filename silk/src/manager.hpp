#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include "handle.hpp"

namespace silk {

template <typename T>
class Manager {
  std::vector<Handle> slots_;  // map handle to index of data_ array
  std::deque<uint32_t> free_slots_;
  std::vector<T> data_;
  std::vector<uint32_t> slot_of_data_;  // map data array to index of slots_

 public:
  Manager() {
    slots_.resize(Handle::MAX_INDEX);
    for (uint32_t i = 0; i < Handle::MAX_INDEX; ++i) {
      free_slots_.push_back(i);
    }
  }

  void clear() {
    for (Handle& s : slots_) {
      s.set_is_valid(true);
      s.increment_generation();
    }

    free_slots_.clear();
    for (uint32_t i = 0; i < Handle::MAX_INDEX; ++i) {
      free_slots_.push_back(i);
    }

    data_.clear();
    slot_of_data_.clear();
  }

  // return nullptr if handle is invalid
  T* get(const Handle& handle) {
    if (handle.is_empty()) {
      return nullptr;
    }

    Handle& slot = slots_[handle.get_index()];
    if (!slot.get_is_valid() ||
        slot.get_generation() != handle.get_generation()) {
      return nullptr;
    }

    return data_[slot.get_index()];
  }

  // return nullptr if handle is invalid
  const T* get(const Handle& handle) const {
    if (handle.is_empty()) {
      return nullptr;
    }

    const Handle& slot = slots_[handle.get_index()];
    if (!slot.get_is_valid() ||
        slot.get_generation() != handle.get_generation()) {
      return nullptr;
    }

    return data_[slot.get_index()];
  }

  // return nullopt if ready max resource
  Handle add(T resource) {
    if (free_slots_.empty()) {
      return Handle{};
    }

    uint32_t slot_idx = free_slots_.front();
    free_slots_.pop_front();

    Handle& slot = slots_[slot_idx];
    slot.set_is_valid(true);
    slot.set_index(data_.size());
    data_.emplace_back(std::move(resource));
    slot_of_data_.push_back(slot_idx);
    return Handle{true, slot.get_generation(), slot_idx};
  }

  bool remove(const Handle& handle) {
    Handle& dead_slot = slots_[handle.get_index()];
    if (!dead_slot.get_is_valid() ||
        dead_slot.get_generation() != handle.get_generation()) {
      return false;
    }

    // swap removed data element with the last element
    std::swap(data_[dead_slot.get_index()], data_.back());
    slots_[slot_of_data_.back()].set_index(dead_slot.get_index());
    slot_of_data_[dead_slot.get_index()] = slot_of_data_.back();
    data_.pop_back();
    slot_of_data_.pop_back();

    dead_slot.set_is_valid(false);
    dead_slot.increment_generation();
    free_slots_.push_front(handle.get_index());

    return true;
  }

  // return dense data vector
  std::vector<T>& data() { return data_; }

  // return dense data vector
  const std::vector<T>& data() const { return data_; }

  const std::vector<T>& get_dense_data() const { return data_; }
};

};  // namespace silk

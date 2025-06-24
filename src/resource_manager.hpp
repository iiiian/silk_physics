#pragma once

#include <cassert>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

// a non owning 32bit resource handle
struct ResourceHandle {
  bool padding : 1;  // memory padding to ensure the struct is 32 bits
  uint32_t generation : 11;
  uint32_t slot_idx : 20;
};

// an internal mapping data structure for resource manager,
// maps resource handle to internal dense data array
struct ResourceSlot {
  bool is_valid : 1;
  uint32_t generation : 11;
  uint32_t data_idx : 20;
};

enum class ResourceManagerResult : int {
  Success = 0,
  InvalidHandle = 1,
  ResourceFull = 2
};

template <typename T>
class ResourceManager {
  static constexpr uint32_t MAX_GENERATION = 2048;   // 2^11
  static constexpr uint32_t MAX_RESOURCE = 1048576;  // 2^20

  std::vector<ResourceSlot> slots_;
  std::deque<uint32_t> free_slots_;
  std::vector<T> data_;
  std::vector<uint32_t> slot_of_data_;

 public:
  ResourceManager() {
    slots_.resize(MAX_RESOURCE, {false, 0, 0});
    for (uint32_t i = 0; i < MAX_RESOURCE; ++i) {
      free_slots_.push_back(i);
    }
  }

  void clear() {
    for (auto& s : slots_) {
      s.is_valid = true;
      s.generation++;
    }

    free_slots_.clear();
    for (uint32_t i = 0; i < MAX_RESOURCE; ++i) {
      free_slots_.push_back(i);
    }

    data_.clear();
    slot_of_data_.clear();
  }

  std::optional<T*> get_resources(const ResourceHandle& handle) {
    auto& slot = slots_[handle.slot_idx];
    if (!slot.is_valid || slot.generation != handle.generation) {
      return std::nullopt;
    }

    return &data_[slot.data_idx];
  }

  std::optional<const T*> get_resources(const ResourceHandle& handle) const {
    auto& slot = slots_[handle.slot_idx];
    if (!slot.is_valid || slot.generation != handle.generation) {
      return std::nullopt;
    }

    return &data_[slot.data_idx];
  }

  std::optional<ResourceHandle> add_resource(T resource) {
    if (free_slots_.empty()) {
      return std::nullopt;
    }

    uint32_t slot_idx = free_slots_.front();
    free_slots_.pop_front();

    auto& slot = slots_[slot_idx];
    slot.is_valid = true;
    slot.data_idx = data_.size();
    data_.emplace_back(std::move(resource));
    slot_of_data_.push_back(slot_idx);
    return ResourceHandle{0, slot.generation, slot_idx};
  }

  bool remove_resource(const ResourceHandle& handle) {
    auto& dead_slot = slots_[handle.slot_idx];
    if (!dead_slot.is_valid || dead_slot.generation != handle.generation) {
      return false;
    }

    // swap removed data element with the last element
    std::swap(data_[dead_slot.data_idx], data_.back());
    slots_[slot_of_data_.back()].data_idx = dead_slot.data_idx;
    slot_of_data_[dead_slot.data_idx] = slot_of_data_.back();
    data_.pop_back();
    slot_of_data_.pop_back();

    dead_slot.is_valid = false;
    dead_slot.generation = (dead_slot.generation == MAX_GENERATION - 1)
                               ? 0
                               : dead_slot.generation + 1;
    free_slots_.push_front(handle.slot_idx);

    return true;
  }

  uint32_t size() const { return static_cast<uint32_t>(data_.size()); }

  std::vector<T>& get_dense_data() { return data_; }

  const std::vector<T>& get_dense_data() const { return data_; }
};

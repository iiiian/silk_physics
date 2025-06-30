#pragma once

#include <cassert>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace silk {

// a non owning 32bit resource handle
// bit layout: | padding (1) | generation (11) | slot_index (20) |
class ResourceHandle {
  static constexpr uint32_t GEN_BITS = 11;
  static constexpr uint32_t SLOT_BITS = 20;
  static constexpr uint32_t GEN_MAX = 1u << GEN_BITS;
  static constexpr uint32_t SLOT_MAX = 1u << SLOT_BITS;
  static constexpr uint32_t GEN_MASK = (GEN_MAX - 1u) << SLOT_BITS;
  static constexpr uint32_t SLOT_MASK = SLOT_MAX - 1u;

  uint32_t value = 0;

 public:
  ResourceHandle() = default;
  explicit ResourceHandle(uint32_t value);
  ResourceHandle(int generation, int slot_index);

  uint32_t get_value() const;
  int get_generation() const;
  int get_slot_index() const;
};

// an internal mapping data structure for resource manager,
// maps resource handle to internal dense data array
// bit layout: | is_valid (1) | generation (11) | data_index (20) |
class ResourceSlot {
  static constexpr uint32_t GEN_BITS = 11;
  static constexpr uint32_t DATA_BITS = 20;
  static constexpr uint32_t GEN_MAX = 1u << GEN_BITS;
  static constexpr uint32_t DATA_MAX = 1u << DATA_BITS;
  static constexpr uint32_t GEN_MASK = (GEN_MAX - 1u) << DATA_BITS;
  static constexpr uint32_t DATA_MASK = DATA_MAX - 1u;
  static constexpr uint32_t IS_VALID_MASK = 1u << (GEN_BITS + DATA_BITS);

  uint32_t value = 0;

 public:
  ResourceSlot() = default;
  ResourceSlot(bool is_valid, int generation, int data_index);

  bool get_is_valid() const;
  void set_is_valid(bool is_valid);
  int get_generation() const;
  void set_generation(int generation);
  void increment_generation();
  int get_data_index() const;
  void set_data_index(int data_index);
};

template <typename T>
class ResourceManager {
  static constexpr uint32_t MAX_GENERATION = 1u << 11;  // 2^11
  static constexpr uint32_t MAX_RESOURCE = 1u << 20;    // 2^20

  std::vector<ResourceSlot> slots_;
  std::deque<int> free_slots_;
  std::vector<T> data_;
  std::vector<int> slot_of_data_;

 public:
  ResourceManager() {
    slots_.resize(MAX_RESOURCE);
    for (int i = 0; i < MAX_RESOURCE; ++i) {
      free_slots_.push_back(i);
    }
  }

  void clear() {
    for (ResourceSlot& s : slots_) {
      s.set_is_valid(true);
      s.increment_generation();
    }

    free_slots_.clear();
    for (int i = 0; i < MAX_RESOURCE; ++i) {
      free_slots_.push_back(i);
    }

    data_.clear();
    slot_of_data_.clear();
  }

  // return nullptr if handle is invalid
  T* get_resources(const ResourceHandle& handle) {
    ResourceSlot& slot = slots_[handle.get_slot_index()];
    if (!slot.get_is_valid() ||
        slot.get_generation() != handle.get_generation()) {
      return nullptr;
    }

    return &data_[slot.get_data_index()];
  }

  // return nullptr if handle is invalid
  const T* get_resources(const ResourceHandle& handle) const {
    const ResourceSlot& slot = slots_[handle.get_slot_index()];
    if (!slot.get_is_valid() ||
        slot.get_generation() != handle.get_generation()) {
      return nullptr;
    }

    return &data_[slot.get_data_index()];
  }

  // return nullopt if ready max resource
  std::optional<ResourceHandle> add_resource(T resource) {
    if (free_slots_.empty()) {
      return std::nullopt;
    }

    int slot_idx = free_slots_.front();
    free_slots_.pop_front();

    ResourceSlot& slot = slots_[slot_idx];
    slot.set_is_valid(true);
    slot.set_data_index(data_.size());
    data_.emplace_back(std::move(resource));
    slot_of_data_.push_back(slot_idx);
    return ResourceHandle{slot.get_generation(), slot_idx};
  }

  bool remove_resource(const ResourceHandle& handle) {
    ResourceSlot& dead_slot = slots_[handle.get_slot_index()];
    if (!dead_slot.get_is_valid() ||
        dead_slot.get_generation() != handle.get_generation()) {
      return false;
    }

    // swap removed data element with the last element
    std::swap(data_[dead_slot.get_data_index()], data_.back());
    slots_[slot_of_data_.back()].set_data_index(dead_slot.get_data_index());
    slot_of_data_[dead_slot.get_data_index()] = slot_of_data_.back();
    data_.pop_back();
    slot_of_data_.pop_back();

    dead_slot.set_is_valid(false);
    dead_slot.increment_generation();
    free_slots_.push_front(handle.get_slot_index());

    return true;
  }

  int size() const { return int(data_.size()); }

  std::vector<T>& get_dense_data() { return data_; }

  const std::vector<T>& get_dense_data() const { return data_; }
};

};  // namespace silk

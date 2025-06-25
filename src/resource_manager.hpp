#pragma once

#include <cassert>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

// a non owning 32bit resource handle
// bit layout: | padding (1) | generation (11) | slot_index (20) |
struct ResourceHandle {
  static constexpr uint32_t GEN_BITS = 11;
  static constexpr uint32_t SLOT_BITS = 20;
  static constexpr uint32_t GEN_MAX = 1u << GEN_BITS;
  static constexpr uint32_t SLOT_MAX = 1u << SLOT_BITS;
  static constexpr uint32_t GEN_MASK = (GEN_MAX - 1u) << SLOT_BITS;
  static constexpr uint32_t SLOT_MASK = SLOT_MAX - 1u;

  uint32_t value = 0;

  ResourceHandle() = default;
  explicit ResourceHandle(uint32_t value);
  ResourceHandle(uint32_t generation, uint32_t slot_index);

  uint32_t generation() const;
  uint32_t slot_index() const;
};

// an internal mapping data structure for resource manager,
// maps resource handle to internal dense data array
// bit layout: | is_valid (1) | generation (11) | data_index (20) |
struct ResourceSlot {
  static constexpr uint32_t GEN_BITS = 11;
  static constexpr uint32_t DATA_BITS = 20;
  static constexpr uint32_t GEN_MAX = 1u << GEN_BITS;
  static constexpr uint32_t DATA_MAX = 1u << DATA_BITS;
  static constexpr uint32_t GEN_MASK = (GEN_MAX - 1u) << DATA_BITS;
  static constexpr uint32_t DATA_MASK = DATA_MAX - 1u;
  static constexpr uint32_t IS_VALID_MASK = 1u << (GEN_BITS + DATA_BITS);

  uint32_t value = 0;

  ResourceSlot() = default;
  ResourceSlot(bool is_valid, uint32_t generation, uint32_t data_index);

  bool is_valid() const;
  void set_is_valid(bool is_valid);
  uint32_t generation() const;
  void set_generation(uint32_t generation);
  void increment_generation();
  uint32_t data_index() const;
  void set_data_index(uint32_t data_index);
};

enum class ResourceManagerResult : int {
  Success = 0,
  InvalidHandle = 1,
  ResourceFull = 2
};

template <typename T>
class ResourceManager {
  static constexpr uint32_t MAX_GENERATION = 1u << 11;  // 2^11
  static constexpr uint32_t MAX_RESOURCE = 1u << 20;    // 2^20

  std::vector<ResourceSlot> slots_;
  std::deque<uint32_t> free_slots_;
  std::vector<T> data_;
  std::vector<uint32_t> slot_of_data_;

 public:
  ResourceManager() {
    slots_.resize(MAX_RESOURCE);
    for (uint32_t i = 0; i < MAX_RESOURCE; ++i) {
      free_slots_.push_back(i);
    }
  }

  void clear() {
    for (auto& s : slots_) {
      s.set_is_valid(true);
      s.increment_generation();
    }

    free_slots_.clear();
    for (uint32_t i = 0; i < MAX_RESOURCE; ++i) {
      free_slots_.push_back(i);
    }

    data_.clear();
    slot_of_data_.clear();
  }

  std::optional<T*> get_resources(const ResourceHandle& handle) {
    auto& slot = slots_[handle.slot_index()];
    if (!slot.is_valid() || slot.generation() != handle.generation()) {
      return std::nullopt;
    }

    return &data_[slot.data_index()];
  }

  std::optional<const T*> get_resources(const ResourceHandle& handle) const {
    auto& slot = slots_[handle.slot_index()];
    if (!slot.is_valid() || slot.generation() != handle.generation()) {
      return std::nullopt;
    }

    return &data_[slot.data_index()];
  }

  std::optional<ResourceHandle> add_resource(T resource) {
    if (free_slots_.empty()) {
      return std::nullopt;
    }

    uint32_t slot_idx = free_slots_.front();
    free_slots_.pop_front();

    auto& slot = slots_[slot_idx];
    slot.set_is_valid(true);
    slot.set_data_index(data_.size());
    data_.emplace_back(std::move(resource));
    slot_of_data_.push_back(slot_idx);
    return ResourceHandle{slot.generation(), slot_idx};
  }

  bool remove_resource(const ResourceHandle& handle) {
    auto& dead_slot = slots_[handle.slot_index()];
    if (!dead_slot.is_valid() ||
        dead_slot.generation() != handle.generation()) {
      return false;
    }

    // swap removed data element with the last element
    std::swap(data_[dead_slot.data_index()], data_.back());
    slots_[slot_of_data_.back()].set_data_index(dead_slot.data_index());
    slot_of_data_[dead_slot.data_index()] = slot_of_data_.back();
    data_.pop_back();
    slot_of_data_.pop_back();

    dead_slot.set_is_valid(false);
    dead_slot.increment_generation();
    free_slots_.push_front(handle.slot_index());

    return true;
  }

  uint32_t size() const { return static_cast<uint32_t>(data_.size()); }

  std::vector<T>& get_dense_data() { return data_; }

  const std::vector<T>& get_dense_data() const { return data_; }
};

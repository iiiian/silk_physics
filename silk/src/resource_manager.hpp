#pragma once

#include <cassert>
#include <cstdint>
#include <deque>
#include <vector>

namespace silk {

// Bit layout: | is valid (1) | generate (GEN_BITS) | index (INDEX_BITS) |
struct ID {
 public:
  // bit operation helper
  static constexpr uint32_t GEN_BITS = 11;
  static constexpr uint32_t INDEX_BITS = 20;
  static constexpr uint32_t MAX_GEN = 1u << GEN_BITS;
  static constexpr uint32_t MAX_GEN_MASK = MAX_GEN - 1u;
  static constexpr uint32_t MAX_INDEX = 1u << INDEX_BITS;
  static constexpr uint32_t GEN_MASK = (MAX_GEN - 1u) << INDEX_BITS;
  static constexpr uint32_t INDEX_MASK = MAX_INDEX - 1u;
  static constexpr uint32_t IS_VALID_SHIFT = GEN_BITS + INDEX_BITS;
  static constexpr uint32_t IS_VALID_MASK = 1u << IS_VALID_SHIFT;

 private:
  uint32_t value_ = 0;

 public:
  ID() = default;

  ID(uint32_t is_valid, uint32_t generation, uint32_t index) {
    assert((generation >= 0 && generation < MAX_GEN));
    assert((index >= 0 && index < MAX_INDEX));

    uint32_t is_valid_bit = static_cast<uint32_t>(is_valid) << IS_VALID_SHIFT;
    uint32_t generation_bit = (generation << INDEX_BITS) & GEN_MASK;
    uint32_t data_index_bit = index & INDEX_MASK;
    value_ = is_valid_bit | generation_bit | data_index_bit;
  }

  bool is_empty() const { return value_ == 0; }

  uint32_t get_raw() const { return value_; }

  void set_raw(uint32_t value) { value_ = value; }

  bool get_is_valid() const {
    return static_cast<bool>(value_ & IS_VALID_MASK);
  }

  void set_is_valid(bool is_valid) {
    uint32_t is_valid_bit = static_cast<uint32_t>(is_valid) << IS_VALID_SHIFT;
    value_ = is_valid_bit | (value_ & ~IS_VALID_MASK);
  }

  uint32_t get_generation() const { return (value_ & GEN_MASK) >> INDEX_BITS; }

  void set_generation(uint32_t generation) {
    assert((generation < MAX_GEN));

    uint32_t generation_bit = (generation << INDEX_BITS) & GEN_MASK;
    value_ = generation_bit | (value_ & ~GEN_MASK);
  }

  void increment_generation() {
    set_generation(get_generation() | MAX_GEN_MASK);
  }

  uint32_t get_index() const { return value_ & INDEX_MASK; }

  void set_index(uint32_t index) {
    assert((index >= 0 && index < MAX_INDEX));

    uint32_t index_bit = index & INDEX_MASK;
    value_ = index_bit | (value_ & ~INDEX_MASK);
  }
};

template <typename T>
class Manager {
  std::vector<ID> slots_;  // map handle to index of data_ array
  std::deque<uint32_t> free_slots_;
  std::vector<T> data_;
  std::vector<uint32_t> slot_of_data_;  // map data array to index of slots_

 public:
  Manager() {
    slots_.resize(ID::MAX_INDEX);
    for (uint32_t i = 0; i < ID::MAX_INDEX; ++i) {
      free_slots_.push_back(i);
    }
  }

  void clear() {
    for (ID& s : slots_) {
      s.set_is_valid(true);
      s.increment_generation();
    }

    free_slots_.clear();
    for (uint32_t i = 0; i < ID::MAX_INDEX; ++i) {
      free_slots_.push_back(i);
    }

    data_.clear();
    slot_of_data_.clear();
  }

  // return nullptr if handle is invalid
  T* get(const ID& handle) {
    ID& slot = slots_[handle.get_index()];
    if (!slot.get_is_valid() ||
        slot.get_generation() != handle.get_generation()) {
      return nullptr;
    }

    return data_[slot.get_index()];
  }

  // return nullptr if handle is invalid
  const T* get(const ID& handle) const {
    const ID& slot = slots_[handle.get_index()];
    if (!slot.get_is_valid() ||
        slot.get_generation() != handle.get_generation()) {
      return nullptr;
    }

    return data_[slot.get_index()];
  }

  // return nullopt if ready max resource
  ID add(T resource) {
    if (free_slots_.empty()) {
      return ID{};
    }

    uint32_t slot_idx = free_slots_.front();
    free_slots_.pop_front();

    ID& slot = slots_[slot_idx];
    slot.set_is_valid(true);
    slot.set_index(data_.size());
    data_.emplace_back(std::move(resource));
    slot_of_data_.push_back(slot_idx);
    return ID{true, slot.get_generation(), slot_idx};
  }

  bool remove(const ID& handle) {
    ID& dead_slot = slots_[handle.get_index()];
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

  std::vector<T>& get_dense_data() { return data_; }

  const std::vector<T>& get_dense_data() const { return data_; }
};

};  // namespace silk

#pragma once

#include <cassert>
#include <cstdint>

namespace silk {

// dual-use bitfield. can be used as handle in ecs system or internal index map
// in Manager.
//
// Bit layout: | is valid (2) | generation (GEN_BITS) | index (INDEX_BITS) |
struct Handle {
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
  Handle() = default;

  Handle(uint32_t value) : value_(value) {};

  Handle(uint32_t is_valid, uint32_t generation, uint32_t index) {
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

};  // namespace silk

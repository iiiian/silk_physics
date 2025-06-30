#include "resource_manager.hpp"

#include <cassert>

namespace silk {

ResourceHandle::ResourceHandle(uint32_t value) : value(value) {}

ResourceHandle::ResourceHandle(int generation, int slot_index) {
  assert((generation >= 0 && generation < GEN_MAX));
  assert((slot_index >= 0 && slot_index < SLOT_MAX));

  uint32_t generation_bit = (uint32_t(generation) << SLOT_BITS) & GEN_MASK;
  uint32_t slot_index_bit = uint32_t(slot_index) & SLOT_MASK;
  value = generation_bit | slot_index_bit;
}

uint32_t ResourceHandle::get_value() const { return value; }

int ResourceHandle::get_generation() const {
  return int((value & GEN_MASK) >> SLOT_BITS);
}

int ResourceHandle::get_slot_index() const { return int(value & SLOT_MASK); }

ResourceSlot::ResourceSlot(bool is_valid, int generation, int data_index) {
  assert((generation >= 0 && generation < GEN_MAX));
  assert((data_index >= 0 && data_index < DATA_MAX));

  uint32_t is_valid_bit = (is_valid) ? 1u << (GEN_BITS + DATA_BITS) : 0u;
  uint32_t generation_bit = (uint32_t(generation) << DATA_BITS) & GEN_MASK;
  uint32_t data_index_bit = uint32_t(data_index) & DATA_MASK;
  value = is_valid_bit | generation_bit | data_index_bit;
}

bool ResourceSlot::get_is_valid() const { return (value & IS_VALID_MASK) != 0; }

void ResourceSlot::set_is_valid(bool is_valid) {
  uint32_t is_valid_bit = (is_valid) ? 1u << (GEN_BITS + DATA_BITS) : 0u;
  value = is_valid_bit | (value & ~IS_VALID_MASK);
}

int ResourceSlot::get_generation() const {
  return int((value & GEN_MASK) >> DATA_BITS);
}

void ResourceSlot::set_generation(int generation) {
  assert((generation >= 0 && generation < GEN_MAX));

  uint32_t generation_bit = (uint32_t(generation) << DATA_BITS) & GEN_MASK;
  value = generation_bit | (value & ~GEN_MASK);
}

void ResourceSlot::increment_generation() {
  uint32_t generation = (value & GEN_MASK) >> DATA_BITS;
  generation = (generation == GEN_MAX - 1) ? 0u : generation + 1;
  uint32_t generation_bit = (generation << DATA_BITS) & GEN_MASK;
  value = generation_bit | (value & ~GEN_MASK);
}

int ResourceSlot::get_data_index() const { return int(value & DATA_MASK); }

void ResourceSlot::set_data_index(int data_index) {
  assert((data_index >= 0 && data_index < DATA_MAX));

  uint32_t data_index_bit = uint32_t(data_index) & DATA_MASK;
  value = data_index_bit | (value & ~DATA_MASK);
}

}  // namespace silk

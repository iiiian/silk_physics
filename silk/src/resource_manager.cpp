#include "resource_manager.hpp"

#include <cassert>

namespace silk {

ResourceHandle::ResourceHandle(uint32_t value) : value(value) {}

ResourceHandle::ResourceHandle(uint32_t generation, uint32_t slot_index) {
  assert((generation < GEN_MAX));
  assert((slot_index < SLOT_MAX));

  uint32_t generation_bit = (generation << SLOT_BITS) & GEN_MASK;
  uint32_t slot_index_bit = slot_index & SLOT_MASK;
  value = generation_bit | slot_index_bit;
}

uint32_t ResourceHandle::get_value() const { return value; }

uint32_t ResourceHandle::get_generation() const {
  return (value & GEN_MASK) >> SLOT_BITS;
}

uint32_t ResourceHandle::get_slot_index() const { return value & SLOT_MASK; }

ResourceSlot::ResourceSlot(bool is_valid, uint32_t generation,
                           uint32_t data_index) {
  assert((generation < GEN_MAX));
  assert((data_index < DATA_MAX));

  uint32_t is_valid_bit = (is_valid) ? 1u << (GEN_BITS + DATA_BITS) : 0u;
  uint32_t generation_bit = (generation << DATA_BITS) & GEN_MASK;
  uint32_t data_index_bit = data_index & DATA_MASK;
  value = is_valid_bit | generation_bit | data_index_bit;
}

bool ResourceSlot::get_is_valid() const { return (value & IS_VALID_MASK) != 0; }

void ResourceSlot::set_is_valid(bool is_valid) {
  uint32_t is_valid_bit = (is_valid) ? 1u << (GEN_BITS + DATA_BITS) : 0u;
  value = is_valid_bit | (value & ~IS_VALID_MASK);
}

uint32_t ResourceSlot::get_generation() const {
  return (value & GEN_MASK) >> DATA_BITS;
}

void ResourceSlot::set_generation(uint32_t generation) {
  assert((generation < GEN_MAX));

  uint32_t generation_bit = (generation << DATA_BITS) & GEN_MASK;
  value = generation_bit | (value & ~GEN_MASK);
}

void ResourceSlot::increment_generation() {
  uint32_t g = get_generation();
  g = (g == GEN_MAX - 1) ? 0u : g + 1;
  set_generation(g);
}

uint32_t ResourceSlot::get_data_index() const { return value & DATA_MASK; }

void ResourceSlot::set_data_index(uint32_t data_index) {
  assert((data_index < DATA_MAX));

  uint32_t data_index_bit = data_index & DATA_MASK;
  value = data_index_bit | (value & ~DATA_MASK);
}

}  // namespace silk

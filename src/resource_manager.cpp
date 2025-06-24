#include "resource_manager.hpp"

#include <cassert>

ResourceHandle::ResourceHandle(uint32_t generation, uint32_t slot_index) {
  assert((generation < GENERATION_MAX));
  assert((slot_index < SLOT_INDEX_MAX));

  uint32_t generation_bit = (generation << SLOT_INDEX_BITS) & GENERATION_MASK;
  uint32_t slot_index_bit = slot_index & SLOT_INDEX_MASK;
  value = generation_bit | slot_index_bit;
}

uint32_t ResourceHandle::generation() const {
  return (value & GENERATION_MASK) >> SLOT_INDEX_BITS;
}

uint32_t ResourceHandle::slot_index() const { return value & SLOT_INDEX_MASK; }

ResourceSlot::ResourceSlot(bool is_valid, uint32_t generation,
                           uint32_t data_index) {
  assert((generation < GENERATION_MAX));
  assert((data_index < DATA_INDEX_MAX));

  uint32_t is_valid_bit =
      (is_valid) ? 1u << (GENERATION_BITS + DATA_INDEX_BITS) : 0u;
  uint32_t generation_bit = (generation << DATA_INDEX_BITS) & GENERATION_MASK;
  uint32_t data_index_bit = data_index & DATA_INDEX_MASK;
  value = is_valid_bit | generation_bit | data_index_bit;
}

bool ResourceSlot::is_valid() const { return (value & IS_VALID_MASK) != 0; }

void ResourceSlot::set_is_valid(bool is_valid) {
  uint32_t is_valid_bit =
      (is_valid) ? 1u << (GENERATION_BITS + DATA_INDEX_BITS) : 0u;
  value = is_valid_bit | (value & ~IS_VALID_MASK);
}

uint32_t ResourceSlot::generation() const {
  return (value & GENERATION_MASK) >> DATA_INDEX_BITS;
}

void ResourceSlot::set_generation(uint32_t generation) {
  assert((generation < GENERATION_MAX));

  uint32_t generation_bit = (generation << DATA_INDEX_BITS) & GENERATION_MASK;
  value = generation_bit | (value & ~GENERATION_MASK);
}

void ResourceSlot::increment_generation() {
  uint32_t g = generation();
  g = (g == GENERATION_MAX - 1) ? 0u : g + 1;
  set_generation(g);
}

uint32_t ResourceSlot::data_index() const { return value & DATA_INDEX_MASK; }

void ResourceSlot::set_data_index(uint32_t data_index) {
  assert((data_index < DATA_INDEX_MAX));

  uint32_t data_index_bit = data_index & DATA_INDEX_MASK;
  value = data_index_bit | (value & ~DATA_INDEX_MASK);
}

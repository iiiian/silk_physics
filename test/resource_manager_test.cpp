#include "resource_manager.hpp"

#include <catch2/catch_test_macros.hpp>

// Test fixture for creating a resource manager with a simple type
struct ResourceManagerFixture {
  ResourceManager<int> rm;
};

// --- ResourceHandle Tests ---

TEST_CASE("Construction and Value Extraction", "[resource_handle]") {
  SECTION("Default constructor") {
    ResourceHandle handle;
    REQUIRE(handle.get_value() == 0);
    REQUIRE(handle.get_generation() == 0);
    REQUIRE(handle.get_slot_index() == 0);
  }

  SECTION("Value constructor") {
    uint32_t test_value = 5243003;
    ResourceHandle handle(test_value);
    REQUIRE(handle.get_value() == test_value);
  }

  SECTION("Generation and Slot Index constructor") {
    uint32_t generation = 15;
    uint32_t slot_index = 456;
    ResourceHandle handle(generation, slot_index);
    REQUIRE(handle.get_generation() == generation);
    REQUIRE(handle.get_slot_index() == slot_index);
  }

  SECTION("Max values") {
    uint32_t max_gen = (1u << 11) - 1u;
    uint32_t max_slot = (1u << 20) - 1u;
    ResourceHandle handle(max_gen, max_slot);
    REQUIRE(handle.get_generation() == max_gen);
    REQUIRE(handle.get_slot_index() == max_slot);
  }
}

// --- ResourceSlot Tests ---

TEST_CASE("Construction and Field Manipulation", "[resource_slot]") {
  SECTION("Default constructor") {
    ResourceSlot slot;
    REQUIRE_FALSE(slot.get_is_valid());
    REQUIRE(slot.get_generation() == 0);
    REQUIRE(slot.get_data_index() == 0);
  }

  SECTION("Parameterized constructor") {
    ResourceSlot slot(true, 10, 100);
    REQUIRE(slot.get_is_valid());
    REQUIRE(slot.get_generation() == 10);
    REQUIRE(slot.get_data_index() == 100);
  }

  SECTION("Setters and Getters") {
    ResourceSlot slot;

    slot.set_is_valid(true);
    REQUIRE(slot.get_is_valid());
    slot.set_is_valid(false);
    REQUIRE_FALSE(slot.get_is_valid());

    slot.set_generation(42);
    REQUIRE(slot.get_generation() == 42);

    slot.set_data_index(999);
    REQUIRE(slot.get_data_index() == 999);
  }

  SECTION("Increment Generation") {
    ResourceSlot slot(true, 5, 1);
    slot.increment_generation();
    REQUIRE(slot.get_generation() == 6);
    // Check wrap-around
    uint32_t max_gen = (1u << 11) - 1u;
    slot.set_generation(max_gen);
    slot.increment_generation();
    REQUIRE(slot.get_generation() == 0);
  }
}

// --- ResourceManager Tests ---

TEST_CASE_METHOD(ResourceManagerFixture, "Construction", "[resource_manager]") {
  SECTION("Initial state is correct") {
    REQUIRE(rm.size() == 0);
    REQUIRE(rm.get_dense_data().empty());
  }
}

TEST_CASE_METHOD(ResourceManagerFixture, "Add Resource", "[resource_manager]") {
  SECTION("Add a single resource") {
    auto handle_opt = rm.add_resource(100);
    REQUIRE(handle_opt.has_value());
    REQUIRE(rm.size() == 1);

    ResourceHandle handle = handle_opt.value();
    REQUIRE(handle.get_generation() == 0);
    REQUIRE(handle.get_slot_index() == 0);

    const auto* resource_ptr = rm.get_resources(handle);
    REQUIRE(resource_ptr != nullptr);
    REQUIRE(*resource_ptr == 100);
  }

  SECTION("Add multiple resources") {
    auto h1_opt = rm.add_resource(10);
    auto h2_opt = rm.add_resource(20);
    REQUIRE(h1_opt.has_value());
    REQUIRE(h2_opt.has_value());

    REQUIRE(rm.size() == 2);
    REQUIRE(rm.get_dense_data().size() == 2);
    REQUIRE(rm.get_dense_data()[0] == 10);
    REQUIRE(rm.get_dense_data()[1] == 20);

    ResourceHandle h1 = h1_opt.value();
    ResourceHandle h2 = h2_opt.value();

    REQUIRE(h1.get_slot_index() == 0);
    REQUIRE(h2.get_slot_index() == 1);

    const int* r1 = rm.get_resources(h1);
    const int* r2 = rm.get_resources(h2);

    REQUIRE(r1 != nullptr);
    REQUIRE(r2 != nullptr);
    REQUIRE(*r1 == 10);
    REQUIRE(*r2 == 20);
  }
}

TEST_CASE_METHOD(ResourceManagerFixture, "Remove Resource",
                 "[resource_manager]") {
  auto h1 = rm.add_resource(10).value();
  auto h2 = rm.add_resource(20).value();
  auto h3 = rm.add_resource(30).value();

  REQUIRE(rm.size() == 3);

  SECTION("Remove a resource from the middle") {
    bool removed = rm.remove_resource(h2);
    REQUIRE(removed);
    REQUIRE(rm.size() == 2);

    // h2 should now be invalid
    REQUIRE(rm.get_resources(h2) == nullptr);

    // The last element (30) should have been swapped into the place of the
    // removed element (20)
    const auto& data = rm.get_dense_data();
    REQUIRE(data.size() == 2);
    REQUIRE(data[0] == 10);
    REQUIRE(data[1] == 30);

    // Check that the handle for the swapped element (h3) still works
    const int* r3 = rm.get_resources(h3);
    REQUIRE(r3 != nullptr);
    REQUIRE(*r3 == 30);

    // h1 should be unaffected
    const int* r1 = rm.get_resources(h1);
    REQUIRE(r1 != nullptr);
    REQUIRE(*r1 == 10);
  }

  SECTION("Remove the last resource") {
    bool removed = rm.remove_resource(h3);
    REQUIRE(removed);
    REQUIRE(rm.size() == 2);
    REQUIRE(rm.get_resources(h3) == nullptr);

    const auto& data = rm.get_dense_data();
    REQUIRE(data[0] == 10);
    REQUIRE(data[1] == 20);
  }

  SECTION("Remove a resource and add a new one") {
    rm.remove_resource(h1);
    REQUIRE(rm.get_resources(h1) == nullptr);

    // The old slot (index 0) should be available again.
    auto h4_opt = rm.add_resource(40);
    REQUIRE(h4_opt.has_value());
    ResourceHandle h4 = h4_opt.value();

    // It should reuse the slot but with an incremented generation
    REQUIRE(h4.get_slot_index() == h1.get_slot_index());
    REQUIRE(h4.get_generation() == h1.get_generation() + 1);

    // The old handle should remain invalid
    REQUIRE(rm.get_resources(h1) == nullptr);
    // The new handle should be valid
    REQUIRE(*rm.get_resources(h4) == 40);
  }

  SECTION("Attempt to remove with an invalid handle") {
    // Create an invalid handle with the correct slot but wrong generation
    ResourceHandle invalid_handle(h1.get_generation() + 5, h1.get_slot_index());
    bool removed = rm.remove_resource(invalid_handle);
    REQUIRE_FALSE(removed);
    REQUIRE(rm.size() == 3);
  }
}

TEST_CASE_METHOD(ResourceManagerFixture, "Get Resource", "[resource_manager]") {
  auto h1 = rm.add_resource(10).value();

  SECTION("Get with valid handle") {
    int* resource = rm.get_resources(h1);
    REQUIRE(resource != nullptr);
    REQUIRE(*resource == 10);

    const auto& const_rm = rm;
    const int* const_resource = const_rm.get_resources(h1);
    REQUIRE(const_resource != nullptr);
    REQUIRE(*const_resource == 10);
  }

  SECTION("Get with invalid handle (wrong generation)") {
    ResourceHandle invalid_handle(h1.get_generation() + 1, h1.get_slot_index());
    REQUIRE(rm.get_resources(invalid_handle) == nullptr);
  }

  SECTION("Get with handle after removal") {
    rm.remove_resource(h1);
    REQUIRE(rm.get_resources(h1) == nullptr);
  }
}

TEST_CASE_METHOD(ResourceManagerFixture, "Clear", "[resource_manager]") {
  auto h1 = rm.add_resource(100).value();
  auto h2 = rm.add_resource(200).value();

  rm.clear();

  REQUIRE(rm.size() == 0);
  REQUIRE(rm.get_dense_data().empty());

  // Handles should be invalid because generation was incremented
  REQUIRE(rm.get_resources(h1) == nullptr);
  REQUIRE(rm.get_resources(h2) == nullptr);

  // Should be able to add new resources
  auto h3 = rm.add_resource(300).value();
  REQUIRE(rm.size() == 1);
  REQUIRE(*rm.get_resources(h3) == 300);
  REQUIRE(h3.get_slot_index() == 0);  // Reuses the first slot
  REQUIRE(h3.get_generation() == 1);  // Generation was incremented
}

TEST_CASE("Edge Cases", "[resource_manager]") {
  SECTION("Exhausting all resources") {
    ResourceManager<int> rm_full;
    // This is a slow test, but necessary for checking capacity.
    // It uses the public constants from ResourceManager if they exist,
    // otherwise it uses the local ones from the header.
    constexpr uint32_t max_res = 1u << 20;
    std::vector<ResourceHandle> handles;
    handles.reserve(max_res);

    for (uint32_t i = 0; i < max_res; ++i) {
      auto h_opt = rm_full.add_resource(i);
      REQUIRE(h_opt.has_value());
      handles.push_back(h_opt.value());
    }

    REQUIRE(rm_full.size() == max_res);

    // Try to add one more
    auto h_extra_opt = rm_full.add_resource(999);
    REQUIRE_FALSE(h_extra_opt.has_value());
  }
}

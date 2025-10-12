#include "manager.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <vector>

namespace silk {

struct Dummy {
  int value;
};

TEST_CASE("manager basic operations", "[manager]") {
  Manager<Dummy> manager;

  SECTION("add and get lifecycle") {
    Handle empty_handle;
    CHECK(manager.get(empty_handle) == nullptr);

    auto h1 = manager.add(Dummy{1});
    auto h2 = manager.add(Dummy{2});

    REQUIRE_FALSE(h1.is_empty());
    REQUIRE_FALSE(h2.is_empty());

    Dummy* v1 = manager.get(h1);
    Dummy* v2 = manager.get(h2);
    REQUIRE(v1 != nullptr);
    REQUIRE(v2 != nullptr);
    CHECK(v1->value == 1);
    CHECK(v2->value == 2);

    const auto& const_manager = manager;
    const Dummy* cv1 = const_manager.get(h1);
    const Dummy* cv2 = const_manager.get(h2);
    REQUIRE(cv1 != nullptr);
    REQUIRE(cv2 != nullptr);
    CHECK(cv1->value == 1);
    CHECK(cv2->value == 2);
  }

  SECTION("removal invalidation") {
    auto handle = manager.add(Dummy{7});
    REQUIRE_FALSE(handle.is_empty());

    CHECK(manager.remove(handle));
    CHECK(manager.get(handle) == nullptr);
    CHECK_FALSE(manager.remove(handle));
  }

  SECTION("neighbor stability after removal") {
    auto h1 = manager.add(Dummy{1});
    auto h2 = manager.add(Dummy{2});
    auto h3 = manager.add(Dummy{3});

    REQUIRE(manager.remove(h2));

    CHECK(manager.get(h2) == nullptr);
    Dummy* v1 = manager.get(h1);
    Dummy* v3 = manager.get(h3);
    REQUIRE(v1 != nullptr);
    REQUIRE(v3 != nullptr);
    CHECK(v1->value == 1);
    CHECK(v3->value == 3);
  }

  SECTION("clear resets state") {
    auto h1 = manager.add(Dummy{5});
    auto h2 = manager.add(Dummy{6});
    REQUIRE_FALSE(h1.is_empty());
    REQUIRE_FALSE(h2.is_empty());

    manager.clear();

    CHECK(manager.get(h1) == nullptr);
    CHECK(manager.get(h2) == nullptr);

    auto h3 = manager.add(Dummy{9});
    REQUIRE_FALSE(h3.is_empty());
    Dummy* v3 = manager.get(h3);
    REQUIRE(v3 != nullptr);
    CHECK(v3->value == 9);
  }
}

TEST_CASE("manager enforces capacity", "[manager]") {
  Manager<uint32_t> manager;

  std::vector<Handle> handles;
  handles.reserve(Handle::MAX_INDEX);

  for (uint32_t i = 0; i < Handle::MAX_INDEX; ++i) {
    auto handle = manager.add(i);
    if (handle.is_empty()) {
      FAIL("manager rejected allocation before reaching capacity");
    }
    handles.push_back(handle);
  }

  Handle overflow = manager.add(42);
  CHECK(overflow.is_empty());

  auto last_handle = handles.back();
  uint32_t* value = manager.get(last_handle);
  REQUIRE(value != nullptr);
  CHECK(*value == Handle::MAX_INDEX - 1);
}

}  // namespace silk

#include <array>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "backend/cuda/collision/dcd.cuh"
#include "backend/cuda/solver/contact_constraints.cuh"

namespace silk::cuda {

TEST_CASE("Edge closest points are scale independent") {
  Vec3f x0 = {-0.005f, 0.0f, 0.0f};
  Vec3f x1 = {0.005f, 0.0f, 0.0f};
  Vec3f x2 = {0.0f, -0.005f, 0.0005f};
  Vec3f x3 = {0.0f, 0.005f, 0.0005f};

  auto uv = exact_ee_uv(x0, x1, x2, x3, 1e-6f);

  REQUIRE(uv.has_value());
  REQUIRE(uv->first == Catch::Approx(0.5f));
  REQUIRE(uv->second == Catch::Approx(0.5f));
}

TEST_CASE("Coulomb contact leaves separating displacement unchanged") {
  Vec3f normal = {0.0f, 0.0f, 1.0f};
  Vec3f free_displacement = {1.0f, -2.0f, 0.5f};

  Vec3f solved = solve_coulomb_contact(free_displacement, normal, 0.7f);

  REQUIRE(solved(0) == free_displacement(0));
  REQUIRE(solved(1) == free_displacement(1));
  REQUIRE(solved(2) == free_displacement(2));
}

TEST_CASE("Coulomb contact sticks inside the friction cone") {
  Vec3f normal = {0.0f, 0.0f, 1.0f};
  Vec3f free_displacement = {0.2f, 0.0f, -1.0f};

  Vec3f solved = solve_coulomb_contact(free_displacement, normal, 0.5f);

  REQUIRE(solved(0) == 0.0f);
  REQUIRE(solved(1) == 0.0f);
  REQUIRE(solved(2) == 0.0f);
}

TEST_CASE("Coulomb contact slides on the cone boundary") {
  Vec3f normal = {0.0f, 0.0f, 1.0f};
  Vec3f free_displacement = {2.0f, 0.0f, -1.0f};

  Vec3f solved = solve_coulomb_contact(free_displacement, normal, 0.5f);

  REQUIRE(solved(0) == 1.5f);
  REQUIRE(solved(1) == 0.0f);
  REQUIRE(solved(2) == 0.0f);
}

TEST_CASE("Frictionless contact removes only penetration") {
  Vec3f normal = {0.0f, 0.0f, 1.0f};
  Vec3f free_displacement = {2.0f, -3.0f, -1.0f};

  Vec3f solved = solve_coulomb_contact(free_displacement, normal, 0.0f);

  REQUIRE(solved(0) == 2.0f);
  REQUIRE(solved(1) == -3.0f);
  REQUIRE(solved(2) == 0.0f);
}

TEST_CASE("Coulomb contact satisfies complementarity and the friction cone") {
  Vec3f normal = {0.0f, 0.0f, 1.0f};
  std::array<Vec3f, 6> free_displacements{
      Vec3f{0.0f, 0.0f, 1.0f},   Vec3f{2.0f, -1.0f, 0.0f},
      Vec3f{0.1f, 0.2f, -1.0f},  Vec3f{2.0f, 0.0f, -1.0f},
      Vec3f{-3.0f, 4.0f, -0.5f}, Vec3f{0.0f, 0.0f, -2.0f}};
  std::array<float, 4> frictions{0.0f, 0.1f, 0.5f, 2.0f};
  constexpr float TOLERANCE = 1e-5f;

  for (float friction : frictions) {
    for (const Vec3f& free_displacement : free_displacements) {
      Vec3f solved = solve_coulomb_contact(free_displacement, normal, friction);
      Vec3f reaction = vsub(solved, free_displacement);
      float solved_normal = dot(solved, normal);
      float reaction_normal = dot(reaction, normal);
      Vec3f reaction_tangent = vsub(reaction, ax(reaction_normal, normal));

      CAPTURE(friction, free_displacement(0), free_displacement(1),
              free_displacement(2));
      REQUIRE(solved_normal >= -TOLERANCE);
      REQUIRE(reaction_normal >= -TOLERANCE);
      REQUIRE(ctd::abs(solved_normal * reaction_normal) <= TOLERANCE);
      REQUIRE(norm(reaction_tangent) <= friction * reaction_normal + TOLERANCE);
    }
  }
}

}  // namespace silk::cuda

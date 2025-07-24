#include "ccd_poly.hpp"

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace silk;

TEST_CASE("CCDPoly", "[ccd_poly]") {
  SECTION("No Collision - Simple Translation") {
    Eigen::Vector3f x00(0, 0, 0), x10(1, 0, 0), x20(0, 1, 0), x30(0, 0, 1);
    Eigen::Vector3f x01(2, 0, 0), x11(3, 0, 0), x21(2, 1, 0), x31(2, 0, 1);

    Eigen::Matrix<float, 3, 4> x0, x1;
    x0.col(0) = x00;
    x0.col(1) = x10;
    x0.col(2) = x20;
    x0.col(3) = x30;
    x1.col(0) = x01;
    x1.col(1) = x11;
    x1.col(2) = x21;
    x1.col(3) = x31;

    auto poly = CCDPoly::try_make_ccd_poly(x0, x1, 1e-6f, 3, 1e-9f);
    REQUIRE(poly.has_value());
    auto result = poly->solve();
    REQUIRE_FALSE(result.has_value());
  }

  SECTION("Vertex-Face Collision - Linear Case") {
    // Static face
    Eigen::Vector3f x00(0, 0, 0), x10(2, 0, 0), x20(1, 2, 0);
    Eigen::Vector3f x01 = x00, x11 = x10, x21 = x20;
    // Moving vertex
    Eigen::Vector3f x30(1, 1, 1);
    Eigen::Vector3f x31(1, 1, -1);

    Eigen::Matrix<float, 3, 4> x0, x1;
    x0.col(0) = x00;
    x0.col(1) = x10;
    x0.col(2) = x20;
    x0.col(3) = x30;
    x1.col(0) = x01;
    x1.col(1) = x11;
    x1.col(2) = x21;
    x1.col(3) = x31;

    auto poly = CCDPoly::try_make_ccd_poly(x0, x1, 1e-6f, 3, 1e-9f);
    REQUIRE(poly.has_value());
    auto result = poly->solve();
    REQUIRE(result.has_value());
    CHECK_THAT(result.value(), Catch::Matchers::WithinAbs(0.5, 1e-6));
  }

  SECTION("Edge-Edge Collision") {
    // Edge 1 moving
    Eigen::Vector3f x00(-1, 0, 0), x10(1, 0, 0);
    Eigen::Vector3f x01(-1, 0, 1), x11(1, 0, 1);
    // Edge 2 static
    Eigen::Vector3f x20(0, -1, 1), x30(0, 1, 1);
    Eigen::Vector3f x21 = x20, x31 = x30;

    Eigen::Matrix<float, 3, 4> x0, x1;
    x0.col(0) = x00;
    x0.col(1) = x10;
    x0.col(2) = x20;
    x0.col(3) = x30;
    x1.col(0) = x01;
    x1.col(1) = x11;
    x1.col(2) = x21;
    x1.col(3) = x31;

    auto poly = CCDPoly::try_make_ccd_poly(x0, x1, 1e-6f, 3, 1e-9f);
    REQUIRE(poly.has_value());
    auto result = poly->solve();
    REQUIRE(result.has_value());
    CHECK_THAT(result.value(), Catch::Matchers::WithinAbs(1.0, 1e-6));
  }

  SECTION("General Cubic - No Collision") {
    Eigen::Vector3f x00(0, 0, 0), x10(1, 0, 0), x20(0, 1, 0), x30(0, 0, 1);
    Eigen::Vector3f x01(0, 0, 0);
    Eigen::Vector3f x11(1, 1, 0);
    Eigen::Vector3f x21(0, 1, 1);
    Eigen::Vector3f x31(1, 0, 1);

    Eigen::Matrix<float, 3, 4> x0, x1;
    x0.col(0) = x00;
    x0.col(1) = x10;
    x0.col(2) = x20;
    x0.col(3) = x30;
    x1.col(0) = x01;
    x1.col(1) = x11;
    x1.col(2) = x21;
    x1.col(3) = x31;

    auto poly = CCDPoly::try_make_ccd_poly(x0, x1, 1e-6f, 3, 1e-9f);
    REQUIRE(poly.has_value());
    auto result = poly->solve();
    REQUIRE_FALSE(result.has_value());
  }

  SECTION("Coplanar initial state") {
    Eigen::Vector3f x00(0, 0, 0), x10(1, 0, 0), x20(0, 1, 0), x30(1, 1, 0);
    Eigen::Vector3f x01(0, 0, 0), x11(1, 0, 0), x21(0, 1, 0), x31(1, 1, 1);

    Eigen::Matrix<float, 3, 4> x0, x1;
    x0.col(0) = x00;
    x0.col(1) = x10;
    x0.col(2) = x20;
    x0.col(3) = x30;
    x1.col(0) = x01;
    x1.col(1) = x11;
    x1.col(2) = x21;
    x1.col(3) = x31;

    auto poly = CCDPoly::try_make_ccd_poly(x0, x1, 1e-6f, 3, 1e-9f);
    REQUIRE_FALSE(poly.has_value());
  }
}

#include "ccd.hpp"

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <string>

#include "ccd_test_utils.hpp"

namespace fs = std::filesystem;

void test_query_category(const fs::path &root, const std::string &name) {
  QueryCategory category{root / name};

  for (const auto &q : category.edge_edge) {
    std::string fail_case = (q.result) ? "False negative" : "False positive";
    INFO("Category: " << name << "\n" << fail_case << "\n" << q.to_string());

    double dist2_a = (q.v10 - q.v00).squaredNorm();
    double dist2_b = (q.v30 - q.v20).squaredNorm();
    double dist2 = std::min(dist2_a, dist2_b);
    float h = 0.05f * std::sqrt(dist2);

    Eigen::Matrix<float, 3, 4> pos0, pos1;
    pos0.col(0) = q.v00.cast<float>();
    pos0.col(1) = q.v10.cast<float>();
    pos0.col(2) = q.v20.cast<float>();
    pos0.col(3) = q.v30.cast<float>();
    pos1.col(0) = q.v01.cast<float>();
    pos1.col(1) = q.v11.cast<float>();
    pos1.col(2) = q.v21.cast<float>();
    pos1.col(3) = q.v31.cast<float>();

    silk::CCDConfig config;
    config.dt = 1.0f;
    config.damping = 0.3f;
    config.friction = 0.3f;
    config.h = h;
    config.tol = 0.05f;
    config.bisect_it = 4;
    config.eps = 1e-6f;

    CHECK(silk::edge_edge_ccd(pos0, pos1, Eigen::Vector4f(0, 1, 2, 3), config)
              .has_value() == q.result);
  }

  for (const auto &q : category.point_triangle) {
    std::string fail_case = (q.result) ? "False negative" : "False positive";
    INFO("Category: " << name << "\n" << fail_case << "\n" << q.to_string());

    double dist2_a = (q.v10 - q.v00).squaredNorm();
    double dist2_b = (q.v20 - q.v10).squaredNorm();
    double dist2_c = (q.v00 - q.v20).squaredNorm();
    double dist2 = std::min(std::min(dist2_a, dist2_b), dist2_c);
    float h = 0.05f * std::sqrt(dist2);

    Eigen::Matrix<float, 3, 4> pos0, pos1;
    pos0.col(0) = q.v30.cast<float>();
    pos0.col(1) = q.v00.cast<float>();
    pos0.col(2) = q.v10.cast<float>();
    pos0.col(3) = q.v20.cast<float>();
    pos1.col(0) = q.v31.cast<float>();
    pos1.col(1) = q.v01.cast<float>();
    pos1.col(2) = q.v11.cast<float>();
    pos1.col(3) = q.v21.cast<float>();

    silk::CCDConfig config;
    config.dt = 1.0f;
    config.damping = 0.3f;
    config.friction = 0.3f;
    config.h = h;
    config.tol = 0.05f;
    config.bisect_it = 4;
    config.eps = 1e-6f;

    CHECK(silk::point_triangle_ccd(pos0, pos1, Eigen::Vector4f(0, 1, 2, 3),
                                   config)
              .has_value() == q.result);
  }
}

TEST_CASE("ccd-test", "[collision]") {
  fs::path root{SAMPLE_QUERY_ROOT};

  // SECTION("fail") { test_query_category(root, "fail"); }

  SECTION("chain") { test_query_category(root, "chain"); }
  SECTION("cow-heads") { test_query_category(root, "cow-heads"); }
  SECTION("golf-ball") { test_query_category(root, "golf-ball"); }
  SECTION("mat-twist") { test_query_category(root, "mat-twist"); }

  // SECTION("erleben-cube-cliff-edges") {
  //   test_query_category(root, "erleben-cube-cliff-edges");
  // }
  // SECTION("erleben-cube-internal-edges") {
  //   test_query_category(root, "erleben-cube-internal-edges");
  // }
  // SECTION("erleben-sliding-spike") {
  //   test_query_category(root, "erleben-sliding-spike");
  // }
  // SECTION("erleben-sliding-wedge") {
  //   test_query_category(root, "erleben-sliding-wedge");
  // }
  // SECTION("erleben-spike-crack") {
  //   test_query_category(root, "erleben-spike-crack");
  // }
  // SECTION("erleben-spike-hole") {
  //   test_query_category(root, "erleben-spike-hole");
  // }
  // SECTION("erleben-spikes") { test_query_category(root, "erleben-spikes"); }
  // SECTION("erleben-spike-wedge") {
  //   test_query_category(root, "erleben-spike-wedge");
  // }
  // SECTION("erleben-wedge-crack") {
  //   test_query_category(root, "erleben-wedge-crack");
  // }
  // SECTION("erleben-wedges") { test_query_category(root, "erleben-wedges"); }
  // SECTION("unit-tests") { test_query_category(root, "unit-tests"); }
}

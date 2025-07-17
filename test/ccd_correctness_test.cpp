#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <string>

#include "ccd.hpp"
#include "ccd_test_helper.hpp"

namespace fs = std::filesystem;

void test_query_category(const fs::path &root, const std::string &name) {
  QueryCategory category{root / name};
  float tol = 0.05f;
  float eps = 1e-6;
  // solver.h = 0.001;
  int refine_iter = 10;

  for (const auto &q : category.edge_edge) {
    std::string fail_case = (q.result) ? "False negative" : "False positive";
    INFO("Category: " << name << "\n" << fail_case << "\n" << q.to_string());

    double dist2_a = (q.v20 - q.v10).squaredNorm();
    double dist2_b = (q.v40 - q.v30).squaredNorm();
    double dist2 = std::min(dist2_a, dist2_b);
    float h = 0.01 * std::sqrt(dist2);

    CHECK(silk::edge_edge_ccd(q.v10.cast<float>(), q.v20.cast<float>(),
                              q.v30.cast<float>(), q.v40.cast<float>(),
                              q.v11.cast<float>(), q.v21.cast<float>(),
                              q.v31.cast<float>(), q.v41.cast<float>(), h, tol,
                              refine_iter, eps)
              .has_value() == q.result);
  }

  for (const auto &q : category.point_triangle) {
    std::string fail_case = (q.result) ? "False negative" : "False positive";
    INFO("Category: " << name << "\n" << fail_case << "\n" << q.to_string());

    double dist2_a = (q.v20 - q.v10).squaredNorm();
    double dist2_b = (q.v30 - q.v20).squaredNorm();
    double dist2_c = (q.v10 - q.v30).squaredNorm();
    double dist2 = std::min(std::min(dist2_a, dist2_b), dist2_c);
    float h = 0.01 * std::sqrt(dist2);
    CHECK(silk::point_triangle_ccd(q.v40.cast<float>(), q.v10.cast<float>(),
                                   q.v20.cast<float>(), q.v30.cast<float>(),
                                   q.v41.cast<float>(), q.v11.cast<float>(),
                                   q.v21.cast<float>(), q.v31.cast<float>(), h,
                                   tol, refine_iter, eps)
              .has_value() == q.result);
  }
}

TEST_CASE("ccd-tests", "[ccd]") {
  fs::path root{SAMPLE_QUERY_ROOT};

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
  //
  // SECTION("fail") { test_query_category(root, "fail"); }
}

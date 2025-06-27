#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "ccd.hpp"
#include "ccd_test_helper.hpp"

namespace fs = std::filesystem;

void benchmark_query_category(const fs::path &root, const std::string &name) {
  QueryCategory category{root / name};
  silk::CCDSolver solver;
  solver.tol = 0.01;
  solver.eps = 1e-6;
  // solver.h = 0.001;
  solver.max_iter = 10;

  namespace ch = std::chrono;
  ch::nanoseconds elapsed{0};

  for (const auto &q : category.edge_edge) {
    double dist2_a = (q.v20 - q.v10).squaredNorm();
    double dist2_b = (q.v40 - q.v30).squaredNorm();
    double dist2 = std::min(dist2_a, dist2_b);
    solver.h = 0.01 * std::sqrt(dist2);

    auto t0 = ch::steady_clock::now();
    solver.edge_edge_ccd(q.v10.cast<float>(), q.v20.cast<float>(),
                         q.v30.cast<float>(), q.v40.cast<float>(),
                         q.v11.cast<float>(), q.v21.cast<float>(),
                         q.v31.cast<float>(), q.v41.cast<float>(), 0.0, 1.0);
    elapsed += ch::steady_clock::now() - t0;
  }

  for (const auto &q : category.point_triangle) {
    double dist2_a = (q.v20 - q.v10).squaredNorm();
    double dist2_b = (q.v30 - q.v20).squaredNorm();
    double dist2_c = (q.v10 - q.v30).squaredNorm();
    double dist2 = std::min(std::min(dist2_a, dist2_b), dist2_c);
    solver.h = 0.01 * std::sqrt(dist2);

    auto t0 = ch::steady_clock::now();
    solver.point_triangle_ccd(
        q.v40.cast<float>(), q.v10.cast<float>(), q.v20.cast<float>(),
        q.v30.cast<float>(), q.v41.cast<float>(), q.v11.cast<float>(),
        q.v21.cast<float>(), q.v31.cast<float>(), 0.0, 1.0);
    elapsed += ch::steady_clock::now() - t0;
  }

  auto query_num = category.edge_edge.size() + category.point_triangle.size();
  std::cout << "Category: " << category.name << ", " << query_num
            << " queries, total time = "
            << ch::duration_cast<ch::microseconds>(elapsed).count() << " us \n";
}

TEST_CASE("ccd-benchmark", "[ccd]") {
  fs::path root{SAMPLE_QUERY_ROOT};

  SECTION("chain") { benchmark_query_category(root, "chain"); }
  SECTION("cow-heads") { benchmark_query_category(root, "cow-heads"); }
  SECTION("golf-ball") { benchmark_query_category(root, "golf-ball"); }
  SECTION("mat-twist") { benchmark_query_category(root, "mat-twist"); }
}

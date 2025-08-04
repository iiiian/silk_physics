#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "ccd.hpp"
#include "ccd_test_utils.hpp"

namespace fs = std::filesystem;

void benchmark_query_category(const fs::path &root, const std::string &name) {
  QueryCategory category{root / name};
  float tol = 0.01;
  float eps = 1e-6;
  // solver.h = 0.001;
  int max_iter = 10;

  namespace ch = std::chrono;
  ch::nanoseconds elapsed{0};

  for (const auto &q : category.edge_edge) {
    double dist2_a = (q.v10 - q.v00).squaredNorm();
    double dist2_b = (q.v30 - q.v20).squaredNorm();
    double dist2 = std::min(dist2_a, dist2_b);
    float h = 0.01 * std::sqrt(dist2);

    auto t0 = ch::steady_clock::now();
    Eigen::Matrix<float, 3, 4> pos0, pos1;
    pos0.col(0) = q.v00.cast<float>();
    pos0.col(1) = q.v10.cast<float>();
    pos0.col(2) = q.v20.cast<float>();
    pos0.col(3) = q.v30.cast<float>();
    pos1.col(0) = q.v01.cast<float>();
    pos1.col(1) = q.v11.cast<float>();
    pos1.col(2) = q.v21.cast<float>();
    pos1.col(3) = q.v31.cast<float>();

    silk::CCDConfig config{1.0f, 0.0f, 0.0f, h, tol, max_iter, eps};

    silk::edge_edge_ccd(pos0, pos1, Eigen::Vector4f(0, 1, 2, 3), config);
    elapsed += ch::steady_clock::now() - t0;
  }

  for (const auto &q : category.point_triangle) {
    double dist2_a = (q.v10 - q.v00).squaredNorm();
    double dist2_b = (q.v20 - q.v10).squaredNorm();
    double dist2_c = (q.v00 - q.v20).squaredNorm();
    double dist2 = std::min(std::min(dist2_a, dist2_b), dist2_c);
    float h = 0.01 * std::sqrt(dist2);

    auto t0 = ch::steady_clock::now();
    Eigen::Matrix<float, 3, 4> pos0, pos1;
    pos0.col(0) = q.v30.cast<float>();
    pos0.col(1) = q.v00.cast<float>();
    pos0.col(2) = q.v10.cast<float>();
    pos0.col(3) = q.v20.cast<float>();
    pos1.col(0) = q.v31.cast<float>();
    pos1.col(1) = q.v01.cast<float>();
    pos1.col(2) = q.v11.cast<float>();
    pos1.col(3) = q.v21.cast<float>();

    silk::CCDConfig config{1.0f, 0.0f, 0.0f, h, tol, max_iter, eps};

    silk::point_triangle_ccd(pos0, pos1, Eigen::Vector4f(0, 1, 2, 3), config);
    elapsed += ch::steady_clock::now() - t0;
  }

  auto query_num = category.edge_edge.size() + category.point_triangle.size();
  std::cout << "Category: " << category.name << ", " << query_num
            << " queries, total time = "
            << ch::duration_cast<ch::microseconds>(elapsed).count() << " us \n";
}

TEST_CASE("ccd-benchmark", "[collision]") {
  fs::path root{SAMPLE_QUERY_ROOT};

  SECTION("chain") { benchmark_query_category(root, "chain"); }
  SECTION("cow-heads") { benchmark_query_category(root, "cow-heads"); }
  SECTION("golf-ball") { benchmark_query_category(root, "golf-ball"); }
  SECTION("mat-twist") { benchmark_query_category(root, "mat-twist"); }
}

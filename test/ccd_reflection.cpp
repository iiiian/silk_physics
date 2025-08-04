#include <spdlog/fmt/fmt.h>

#include <Eigen/Core>
#include <filesystem>
#include <iostream>
#include <string>

#include "ccd.hpp"
#include "ccd_test_utils.hpp"

namespace fs = std::filesystem;

void print_header() {
  std::cout << "type;toi;x00;x10;x20;x30;x01;x11;x21;x31;x0r;x1r;x2r;x3r\n";
}

std::string vec2str(Eigen::Ref<const Eigen::Vector3f> vec) {
  return fmt::format("{:.10f},{:.10f},{:.10f}", vec(0), vec(1), vec(2));
}

void print_collision(const Eigen::Matrix<float, 3, 4> &pos0,
                     const Eigen::Matrix<float, 3, 4> &pos1,
                     const silk::Collision &c) {
  std::string type = (c.type == silk::CollisionType::PointTriangle)
                         ? "PointTriangle"
                         : "EdgeEdge";
  std::string x00 = vec2str(pos0.col(0));
  std::string x10 = vec2str(pos0.col(1));
  std::string x20 = vec2str(pos0.col(2));
  std::string x30 = vec2str(pos0.col(3));
  std::string x01 = vec2str(pos1.col(0));
  std::string x11 = vec2str(pos1.col(1));
  std::string x21 = vec2str(pos1.col(2));
  std::string x31 = vec2str(pos1.col(3));
  std::string x0r = vec2str(c.position.col(0));
  std::string x1r = vec2str(c.position.col(1));
  std::string x2r = vec2str(c.position.col(2));
  std::string x3r = vec2str(c.position.col(3));

  std::cout << fmt::format("{};{};{};{};{};{};{};{};{};{};{};{};{};{}\n", type,
                           c.toi, x00, x10, x20, x30, x01, x11, x21, x31, x0r,
                           x1r, x2r, x3r);
}

void test_query_category(const fs::path &root, const std::string &name) {
  QueryCategory category{root / name};

  for (const auto &q : category.edge_edge) {
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

    auto colli =
        silk::edge_edge_ccd(pos0, pos1, Eigen::Vector4f(0, 1, 2, 3), config);
    if (colli) {
      print_collision(pos0, pos1, *colli);
    }
  }

  for (const auto &q : category.point_triangle) {
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

    auto colli = silk::point_triangle_ccd(pos0, pos1,
                                          Eigen::Vector4f(0, 1, 2, 3), config);
    if (colli) {
      print_collision(pos0, pos1, *colli);
    }
  }
}

int main() {
  fs::path root{SAMPLE_QUERY_ROOT};

  print_header();
  test_query_category(root, "chain");
  test_query_category(root, "cow-heads");
  test_query_category(root, "golf-ball");
  test_query_category(root, "mat-twist");
}

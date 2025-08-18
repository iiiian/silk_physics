#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <filesystem>
#include <iostream>
#include <string>

#include "ccd_test_utils.hpp"
#include "collision_narrowphase.hpp"

namespace fs = std::filesystem;

void print_header() {
  std::cout << "type;toi;x00;x10;x20;x30;x01;x11;x21;x31;x0r;x1r;x2r;x3r\n";
}

template <typename Derived>
std::string vec2str(const Eigen::MatrixBase<Derived>& vec) {
  return fmt::format("{:.10f},{:.10f},{:.10f}", vec(0), vec(1), vec(2));
}

void print_collision(const Eigen::Vector3f& x00, const Eigen::Vector3f& x10,
                     const Eigen::Vector3f& x20, const Eigen::Vector3f& x30,
                     const Eigen::Vector3f& x01, const Eigen::Vector3f& x11,
                     const Eigen::Vector3f& x21, const Eigen::Vector3f& x31,
                     const Eigen::Matrix<float, 3, 4>& r,
                     silk::CollisionType type, float toi) {
  std::string type_str = (type == silk::CollisionType::PointTriangle)
                             ? "PointTriangle"
                             : "EdgeEdge";
  std::string x00_str = vec2str(x00);
  std::string x10_str = vec2str(x10);
  std::string x20_str = vec2str(x20);
  std::string x30_str = vec2str(x30);
  std::string x01_str = vec2str(x01);
  std::string x11_str = vec2str(x11);
  std::string x21_str = vec2str(x21);
  std::string x31_str = vec2str(x31);
  std::string x0r_str = vec2str(r.col(0));
  std::string x1r_str = vec2str(r.col(1));
  std::string x2r_str = vec2str(r.col(2));
  std::string x3r_str = vec2str(r.col(3));

  // std::cout << fmt::format("{};{};{};{};{};{};{};{};{};{};{};{};{};{}\n",
  //                          type_str, toi, x00_str, x10_str, x20_str, x30_str,
  //                          x01_str, x11_str, x21_str, x31_str, x0r_str,
  //                          x1r_str, x2r_str, x3r_str);
}

void test_query_category(const fs::path& root, const std::string& name) {
  QueryCategory category{root / name};

  for (const auto& q : category.edge_edge) {
    silk::MeshCollider ma, mb;

    // Edge A setup
    ma.type = silk::MeshColliderType::Edge;
    ma.index = {0, 1, -1};
    ma.inv_mass = {1.0f, 1.0f, 0.0f};
    ma.position_t0.col(0) = q.v00.cast<float>();
    ma.position_t0.col(1) = q.v10.cast<float>();
    ma.position_t0.col(2).setZero();
    ma.position_t1.col(0) = q.v01.cast<float>();
    ma.position_t1.col(1) = q.v11.cast<float>();
    ma.position_t1.col(2).setZero();

    // Edge B setup
    mb.type = silk::MeshColliderType::Edge;
    mb.index = {0, 1, -1};  // Local indices for this collider
    mb.inv_mass = {1.0f, 1.0f, 0.0f};
    mb.position_t0.col(0) = q.v20.cast<float>();
    mb.position_t0.col(1) = q.v30.cast<float>();
    mb.position_t0.col(2).setZero();
    mb.position_t1.col(0) = q.v21.cast<float>();
    mb.position_t1.col(1) = q.v31.cast<float>();
    mb.position_t1.col(2).setZero();

    // Bbox calculation
    ma.bbox.min =
        (ma.position_t0.leftCols(2).rowwise().minCoeff())
            .cwiseMin(ma.position_t1.leftCols(2).rowwise().minCoeff());
    ma.bbox.max =
        (ma.position_t0.leftCols(2).rowwise().maxCoeff())
            .cwiseMax(ma.position_t1.leftCols(2).rowwise().maxCoeff());
    mb.bbox.min =
        (mb.position_t0.leftCols(2).rowwise().minCoeff())
            .cwiseMin(mb.position_t1.leftCols(2).rowwise().minCoeff());
    mb.bbox.max =
        (mb.position_t0.leftCols(2).rowwise().maxCoeff())
            .cwiseMax(mb.position_t1.leftCols(2).rowwise().maxCoeff());

    silk::ObjectCollider oa, ob;

    oa.damping = 0.3f;
    ob.damping = 0.3f;
    oa.friction = 0.3f;
    ob.friction = 0.3f;
    // oa.bbox_padding = 0.001f * (ma.bbox.max - ma.bbox.min).norm();
    // ob.bbox_padding = 0.001f * (mb.bbox.max - mb.bbox.min).norm();
    oa.bbox_padding = 0.0f;
    ob.bbox_padding = 0.0f;

    auto colli =
        silk::narrow_phase(oa, ma, ob, mb, 1.0f, {-1, -1, -1}, {-1, -1, -1});

    if (colli) {
      print_collision(
          q.v00.cast<float>(), q.v10.cast<float>(), q.v20.cast<float>(),
          q.v30.cast<float>(), q.v01.cast<float>(), q.v11.cast<float>(),
          q.v21.cast<float>(), q.v31.cast<float>(), colli->reflection,
          silk::CollisionType::EdgeEdge, colli->toi);
    }
  }

  for (const auto& q : category.point_triangle) {
    silk::MeshCollider ma, mb;  // ma: point, mb: triangle

    // Point collider setup
    ma.type = silk::MeshColliderType::Point;
    ma.index = {0, -1, -1};  // Only first index is used
    ma.inv_mass = {1.0f, 0.0f, 0.0f};
    ma.position_t0.col(0) = q.v00.cast<float>();
    ma.position_t1.col(0) = q.v01.cast<float>();
    ma.position_t0.col(1).setZero();
    ma.position_t0.col(2).setZero();
    ma.position_t1.col(1).setZero();
    ma.position_t1.col(2).setZero();

    // Triangle collider setup
    mb.type = silk::MeshColliderType::Triangle;
    mb.index = {0, 1, 2};
    mb.inv_mass = {1.0f, 1.0f, 1.0f};
    mb.position_t0.col(0) = q.v10.cast<float>();
    mb.position_t0.col(1) = q.v20.cast<float>();
    mb.position_t0.col(2) = q.v30.cast<float>();
    mb.position_t1.col(0) = q.v11.cast<float>();
    mb.position_t1.col(1) = q.v21.cast<float>();
    mb.position_t1.col(2) = q.v31.cast<float>();

    // Bbox calculation
    ma.bbox.min = ma.position_t0.col(0).cwiseMin(ma.position_t1.col(0));
    ma.bbox.max = ma.position_t0.col(0).cwiseMax(ma.position_t1.col(0));

    mb.bbox.min = (mb.position_t0.rowwise().minCoeff())
                      .cwiseMin(mb.position_t1.rowwise().minCoeff());
    mb.bbox.max = (mb.position_t0.rowwise().maxCoeff())
                      .cwiseMax(mb.position_t1.rowwise().maxCoeff());

    // ObjectColliders
    silk::ObjectCollider oa, ob;

    oa.damping = 0.3f;
    ob.damping = 0.3f;
    oa.friction = 0.3f;
    ob.friction = 0.3f;
    // oa.bbox_padding = 0.001f * (ma.bbox.max - ma.bbox.min).norm();
    // ob.bbox_padding = 0.001f * (mb.bbox.max - mb.bbox.min).norm();
    oa.bbox_padding = 0.0f;
    ob.bbox_padding = 0.0f;

    auto colli =
        silk::narrow_phase(oa, ma, ob, mb, 1.0f, {-1, -1, -1}, {-1, -1, -1});

    if (colli) {
      print_collision(
          q.v00.cast<float>(), q.v10.cast<float>(), q.v20.cast<float>(),
          q.v30.cast<float>(), q.v01.cast<float>(), q.v11.cast<float>(),
          q.v21.cast<float>(), q.v31.cast<float>(), colli->reflection,
          silk::CollisionType::PointTriangle, colli->toi);
    }
  }
}

int main() {
  spdlog::set_level(spdlog::level::debug);

  fs::path root{SAMPLE_QUERY_ROOT};

  print_header();
  test_query_category(root, "chain");
  test_query_category(root, "cow-heads");
  test_query_category(root, "golf-ball");
  test_query_category(root, "mat-twist");
}

#include "vectorized_jacobian.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <optional>

namespace eg = Eigen;

std::optional<Matrix69f> vectorized_jacobian(eg::Ref<const eg::Vector3f> v0,
                                             eg::Ref<const eg::Vector3f> v1,
                                             eg::Ref<const eg::Vector3f> v2,
                                             float zero_threshold = 1e-8) {
  // convert triangle to 2D
  // use edge e1 = v0 -> v1 as x axis
  // use n x e1 as y axis
  eg::Vector3f e1 = v1 - v0;
  eg::Vector3f e2 = v2 - v0;

  // basis x
  eg::Vector3f bx = e1.normalized();
  // basis y
  eg::VectorXf e1xe2 = e1.cross(e2);
  if (e1xe2.norm() < zero_threshold) {
    return std::nullopt;
  }
  eg::Vector3f by = e1.cross(e2).cross(e1).normalized();

  eg::Matrix<float, 2, 2> dX;
  dX(0, 0) = bx.dot(e1);
  dX(1, 0) = 0;
  dX(0, 1) = bx.dot(e2);
  dX(1, 1) = by.dot(e2);

  // clang-format off
  const eg::Matrix<float, 3, 2> D =
        (eg::Matrix<float, 3, 2>() << -1, -1,
                                       1,  0,
                                       0,  1).finished();
  // clang-format on

  eg::Matrix<float, 2, 3> M = (D * dX.inverse()).transpose();

  // discrete vectorized deformation operator F
  // F = M ⊗ I3
  eg::Matrix<float, 6, 9> F = eg::Matrix<float, 6, 9>::Zero();
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      float weight = M(i, j);
      F(3 * i, 3 * j) = weight;
      F(3 * i + 1, 3 * j + 1) = weight;
      F(3 * i + 2, 3 * j + 2) = weight;
    }
  }

  return F;
}

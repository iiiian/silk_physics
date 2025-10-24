#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <glm/glm.hpp>
#include <vector>

#include "config.hpp"
#include "eigen_alias.hpp"

/**
 * @brief A utility class to handle 3D affine transformations using both GLM and
 * Eigen.
 */
class AffineTransformer {
 public:
  using EigenAffine =
      Eigen::Transform<float, 3, Eigen::TransformTraits::Affine>;

 private:
  glm::mat4 glm_affine_;
  EigenAffine eigen_affine_;

 public:
  /**
   * @brief Constructs an AffineTransformer from GLM vectors.
   * @param position The translation vector.
   * @param rotation The Euler angle rotation vector (XYZ order).
   * @param scale The scaling vector.
   */
  AffineTransformer(const glm::vec3& position, const glm::vec3& rotation,
                    const glm::vec3& scale);
  /**
   * @brief Constructs an AffineTransformer from GLM vectors with uniform
   * scaling.
   * @param position The translation vector.
   * @param rotation The Euler angle rotation vector (XYZ order).
   * @param scale The uniform scaling factor.
   */
  AffineTransformer(const glm::vec3& position, const glm::vec3& rotation,
                    float scale);
  /**
   * @brief Constructs an AffineTransformer from Eigen vectors.
   * @param position The translation vector.
   * @param rotation The Euler angle rotation vector (XYZ order).
   * @param scale The scaling vector.
   */
  AffineTransformer(const Eigen::Vector3f& position,
                    const Eigen::Vector3f& rotation,
                    const Eigen::Vector3f& scale);
  /**
   * @brief Constructs an AffineTransformer from Eigen vectors with uniform
   * scaling.
   * @param position The translation vector.
   * @param rotation The Euler angle rotation vector (XYZ order).
   * @param scale The uniform scaling factor.
   */
  AffineTransformer(const Eigen::Vector3f& position,
                    const Eigen::Vector3f& rotation, float scale);
  /**
   * @brief Constructs an AffineTransformer from std::array.
   * @param position The translation vector as a 3-element array.
   * @param rotation The Euler angle rotation vector (XYZ order) as a
   * 3-element array.
   * @param scale The scaling vector as a 3-element array.
   */
  AffineTransformer(const std::array<float, 3>& position,
                    const std::array<float, 3>& rotation,
                    const std::array<float, 3>& scale);
  /**
   * @brief Constructs an AffineTransformer from std::array with uniform
   * scaling.
   * @param position The translation vector as a 3-element array.
   * @param rotation The Euler angle rotation vector (XYZ order) as a
   * 3-element array.
   * @param scale The uniform scaling factor.
   */
  AffineTransformer(const std::array<float, 3>& position,
                    const std::array<float, 3>& rotation, float scale);
  /**
   * @brief Constructs an AffineTransformer from a configuration struct.
   * @param config The transform configuration.
   */
  AffineTransformer(const config::Transform& config);

  /**
   * @brief Gets the transformation as a GLM 4x4 matrix.
   * @return A const reference to the GLM affine transformation matrix.
   */
  const glm::mat4& get_glm_affine() const;
  /**
   * @brief Gets the transformation as an Eigen Affine transform.
   * @return A const reference to the Eigen affine transformation.
   */
  const EigenAffine& get_eigen_affine() const;

  /**
   * @brief Applies the transformation in-place to a vector of GLM vertices.
   * @param verts A vector of vertices to be transformed.
   */
  void apply(std::vector<glm::vec3>& verts);
  /**
   * @brief Applies the transformation in-place to an Eigen matrix of vertices.
   * @param verts A matrix where each row represents a vertex to be
   * transformed.
   */
  void apply(Vert& verts);
};

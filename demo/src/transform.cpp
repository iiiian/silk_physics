#include "transform.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>

// Helper function to build Eigen affine transform.
AffineTransformer::EigenAffine make_eigen_affine(

    const Eigen::Vector3f& position, const Eigen::Vector3f& rotation,
    const Eigen::Vector3f& scale) {
  AffineTransformer::EigenAffine t =
      Eigen::Translation3f(position) *
      Eigen::AngleAxisf(rotation.z(), Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(rotation.y(), Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(rotation.x(), Eigen::Vector3f::UnitX()) *
      Eigen::Scaling(scale);
  return t;
}

// Helper function to build glm::mat4.
glm::mat4 make_glm_affine(const glm::vec3& position, const glm::vec3& rotation,
                          const glm::vec3& scale) {
  glm::mat4 T = glm::translate(glm::mat4(1.0f), position);
  glm::mat4 R = glm::eulerAngleXYZ(rotation.x, rotation.y, rotation.z);
  glm::mat4 S = glm::scale(glm::mat4(1.0f), scale);
  return T * R * S;
}

AffineTransformer::AffineTransformer(const glm::vec3& position,
                                     const glm::vec3& rotation,
                                     const glm::vec3& scale) {
  glm_affine_ = make_glm_affine(position, rotation, scale);
  eigen_affine_ =
      make_eigen_affine(Eigen::Vector3f(position.x, position.y, position.z),
                        Eigen::Vector3f(rotation.x, rotation.y, rotation.z),
                        Eigen::Vector3f(scale.x, scale.y, scale.z));
}

AffineTransformer::AffineTransformer(const glm::vec3& position,
                                     const glm::vec3& rotation, float scale) {
  *this = AffineTransformer(position, rotation, glm::vec3(scale));
}

AffineTransformer::AffineTransformer(const Eigen::Vector3f& position,
                                     const Eigen::Vector3f& rotation,
                                     const Eigen::Vector3f& scale) {
  glm_affine_ =
      make_glm_affine(glm::vec3(position.x(), position.y(), position.z()),
                      glm::vec3(rotation.x(), rotation.y(), rotation.z()),
                      glm::vec3(scale.x(), scale.y(), scale.z()));
  eigen_affine_ = make_eigen_affine(position, rotation, scale);
}

AffineTransformer::AffineTransformer(const Eigen::Vector3f& position,
                                     const Eigen::Vector3f& rotation,
                                     float scale) {
  *this = AffineTransformer(position, rotation,
                            Eigen::Vector3f(scale, scale, scale));
}

AffineTransformer::AffineTransformer(const std::array<float, 3>& position,
                                     const std::array<float, 3>& rotation,
                                     const std::array<float, 3>& scale) {
  *this = AffineTransformer(Eigen::Vector3f(position.data()),
                            Eigen::Vector3f(rotation.data()),
                            Eigen::Vector3f(scale.data()));
}

AffineTransformer::AffineTransformer(const std::array<float, 3>& position,
                                     const std::array<float, 3>& rotation,
                                     float scale) {
  *this = AffineTransformer(Eigen::Vector3f(position.data()),
                            Eigen::Vector3f(rotation.data()), scale);
}

AffineTransformer::AffineTransformer(const config::Transform& config) {
  *this = AffineTransformer(config.translation, config.rotation_euler_deg,
                            config.scale);
}

const glm::mat4& AffineTransformer::get_glm_affine() const {
  return glm_affine_;
}

const AffineTransformer::EigenAffine& AffineTransformer::get_eigen_affine()
    const {
  return eigen_affine_;
}

void AffineTransformer::apply(std::vector<glm::vec3>& verts) {
  for (auto& v : verts) {
    v = glm::vec3(glm_affine_ * glm::vec4(v, 1.0f));
  }
}

void AffineTransformer::apply(Vert& verts) {
  for (int i = 0; i < verts.rows(); ++i) {
    verts.row(i) = (eigen_affine_ * verts.row(i).transpose()).eval();
  }
}

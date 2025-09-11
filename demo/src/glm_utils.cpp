#include "glm_utils.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>

glm::mat4 build_transformation(const glm::vec3& position,
                               const glm::vec3& rotation, float scale) {
  glm::mat4 T = glm::translate(glm::mat4(1.0f), position);
  glm::mat4 R = glm::eulerAngleXYZ(rotation.x, rotation.y, rotation.z);
  glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(scale));
  return T * R * S;
}

glm::vec3 transform_vertex(const glm::mat4& T, const glm::vec3& v) {
  return glm::vec3(T * glm::vec4(v, 1.0f));
}

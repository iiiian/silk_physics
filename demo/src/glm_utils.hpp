#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>

glm::mat4 build_transformation(const glm::vec3& position,
                               const glm::vec3& rotation, float scale);

glm::vec3 transform_vertex(const glm::mat4& T, const glm::vec3& v);

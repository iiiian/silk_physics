#pragma once

#include <Eigen/Core>

class MovingTriangle {
 public:
  // triange vertices
  Eigen::Vector3f v0;
  Eigen::Vector3f v1;
  Eigen::Vector3f v2;
  Eigen::Vector3f e1;  // v0 -> v1
  Eigen::Vector3f e2;  // v1 -> v2
  Eigen::Vector3f e3;  // v2 -> v0
  Eigen::Vector3f n;
  float dA = 0;  // double area
  // weight are inverse mass
  float w0 = 0;  // v0 weight
  float w1 = 0;  // v1 weight
  float w2 = 0;  // v2 weight

  void update_gemotry();
};

bool resolve_vertex_triangle_collision(Eigen::Vector3f& p, float w,
                                       MovingTriangle& t, float h);

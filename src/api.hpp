#pragma once

class ClothConfig {
 public:
  float elastic_stiffness = 1.0f;
  float bending_stiffness = 1.0f;
  float density = 1.0f;

  bool is_valid() const;
};

struct ClothHandle

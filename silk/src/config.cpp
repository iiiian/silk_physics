#include <unordered_set>

#include "silk/silk.hpp"

namespace silk {

Result MeshConfig::validate() const {
  if (vert_num <= 0) {
    return Result::InvalidConfig;
  }

  if (face_num <= 0) {
    return Result::InvalidConfig;
  }

  for (int i = 0; i < face_num; ++i) {
    for (int j = 0; j < 3; ++j) {
      int idx = faces[3 * i + j];
      if (idx < 0 || idx >= vert_num) {
        return Result::InvalidConfig;
      }
    }
  }

  return Result::Success;
}

Result PininingConfig::validate() const {
  if (pinned_num < 0) {
    return Result::InvalidConfig;
  }

  std::unordered_set<int> set;
  for (int i = 0; i < pinned_num; ++i) {
    if (pinned_vertices[i] < 0) {
      return Result::InvalidConfig;
    }
    if (set.find(pinned_vertices[i]) != set.end()) {
      return Result::InvalidConfig;
    }
    set.insert(pinned_vertices[i]);
  }

  return Result::Success;
}

Result CollisionConfig::validate() const { return Result::Success; }

Result ClothConfig::validate() const { return Result::Success; }

}  // namespace silk

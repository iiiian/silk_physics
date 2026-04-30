#pragma once

#include "ecs/registry.hpp"

namespace silk {

class ClothConfig;
class CollisionConfig;
class TriMesh;
class Pin;
class ClothTopology;

}  // namespace silk

namespace silk::cuda {

class ClothSolverContext;
class ObjectState;
class ObstaclePosition;
class ObjectCollider;

// clang-format off
using Registry = ::silk::ecs::Registry<ClothConfig,
                                       CollisionConfig,
                                       TriMesh,
                                       Pin,
                                       ClothTopology,
                                       ClothSolverContext,
                                       ObjectState,
                                       ObstaclePosition,
                                       ObjectCollider>;
// clang-format on

}  // namespace silk::cuda

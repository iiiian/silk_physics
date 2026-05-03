#pragma once

#include "ecs/registry.hpp"

namespace silk {

class ClothConfigPlus;
class CollisionConfigPlus;
class TriMesh;
class Pin;
class ClothTopology;

}  // namespace silk

namespace silk::cuda {

class ClothSolverContext;
class PhysicalState;
class ObstaclePosition;
class ObjectCollider;

// clang-format off
using Registry = ::silk::ecs::Registry<ClothConfigPlus,
                                       CollisionConfigPlus,
                                       TriMesh,
                                       Pin,
                                       ClothTopology,
                                       ClothSolverContext,
                                       PhysicalState,
                                       ObstaclePosition,
                                       ObjectCollider>;
// clang-format on

}  // namespace silk::cuda

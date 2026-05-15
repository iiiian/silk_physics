#pragma once

#include "ecs/registry.hpp"

namespace silk {
class ClothAssemblyL2Cache;
struct ClothConfig;
struct CollisionConfig;
struct Pin;
struct TriMesh;
}  // namespace silk

namespace silk::cpu {
class ClothSolverContext;
class ObjectCollider;
struct ObjectState;
struct ObstaclePosition;

using Registry = ::silk::ecs::Registry<ClothConfig, CollisionConfig, TriMesh,
                                       Pin, ClothAssemblyL2Cache,
                                       ClothSolverContext, ObjectState,
                                       ObstaclePosition, ObjectCollider>;

}  // namespace silk::cpu

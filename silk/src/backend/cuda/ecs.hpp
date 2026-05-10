#pragma once

#include "ecs/registry.hpp"

namespace silk {
class ClothConfig;
class CollisionConfigPlus;
class TriMesh;
class Pin;
class ClothAssemblyL2Cache;
}  // namespace silk

namespace silk::cuda {
class ClothAssemblyL1Cache;
class PhysicalState;
class ObstaclePosition;
class MeshPartition;
}  // namespace silk::cuda

namespace silk::cuda::collision {
class ObjectCollider;
}

namespace silk::cuda {

// clang-format off
using ObjRegistry = ::silk::ecs::Registry<ClothConfig,
                                          CollisionConfigPlus,
                                          TriMesh,
                                          MeshPartition,
                                          Pin,
                                          ClothAssemblyL2Cache,
                                          ClothAssemblyL1Cache,
                                          PhysicalState,
                                          ObstaclePosition,
                                          collision::ObjectCollider>;
// clang-format on

}  // namespace silk::cuda

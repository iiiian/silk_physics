#pragma once

#include "ecs/registry.hpp"

namespace silk {
class ClothConfig;
class CollisionConfigPlus;
class TriMesh;
class Pin;
class ClothAssemblyL2Cache;
class InitialState;
}  // namespace silk

namespace silk::cuda {
class ClothADMMHelper;
class PhysicalState;
class ObstaclePosition;
class MeshPartition;
}  // namespace silk::cuda

namespace silk::cuda::collision {
class ObjectCollider;
}

namespace silk::cuda::assembly {
class ClothAssemblyL1Cache;
}

namespace silk::cuda::solver {
class ClothADMMHelper;
}

namespace silk::cuda {

// clang-format off
using ObjRegistry = ::silk::ecs::Registry<ClothConfig,
                                          CollisionConfigPlus,
                                          TriMesh,
                                          MeshPartition,
                                          Pin,
                                          InitialState,
                                          ClothAssemblyL2Cache,
                                          PhysicalState,
                                          ObstaclePosition,
                                          collision::ObjectCollider,
                                          assembly::ClothAssemblyL1Cache,
                                          solver::ClothADMMHelper
                                          >;
// clang-format on

}  // namespace silk::cuda

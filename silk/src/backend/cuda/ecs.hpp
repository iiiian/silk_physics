#pragma once

#include "ecs/registry.hpp"

namespace silk {
class ClothConfig;
class CollisionConfigPlus;
class TriMesh;
class ClothAssemblyL2Cache;
class InitialState;
}  // namespace silk

namespace silk::cuda {
class ClothADMMHelper;
struct PinIndex;
struct PinPosition;
class PhysicalState;
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
                                          PinIndex,
                                          PinPosition,
                                          InitialState,
                                          ClothAssemblyL2Cache,
                                          PhysicalState,
                                          collision::ObjectCollider,
                                          assembly::ClothAssemblyL1Cache,
                                          solver::ClothADMMHelper
                                          >;
// clang-format on

}  // namespace silk::cuda

// ---------------------------------------
// Component Dependencies
// ---------------------------------------

namespace silk {

template <>
struct ecs::ComponentDependents<cuda::ObjRegistry, ClothConfig> {
  using Type =
      ecs::ComponentList<ClothAssemblyL2Cache, cuda::collision::ObjectCollider>;
};

template <>
struct ecs::ComponentDependents<cuda::ObjRegistry, CollisionConfigPlus> {
  using Type = ecs::ComponentList<cuda::collision::ObjectCollider>;
};

template <>
struct ecs::ComponentDependents<cuda::ObjRegistry, TriMesh> {
  using Type =
      ecs::ComponentList<cuda::MeshPartition, InitialState, cuda::PinIndex,
                         ClothAssemblyL2Cache, cuda::PhysicalState,
                         cuda::collision::ObjectCollider>;
};

template <>
struct ecs::ComponentDependents<cuda::ObjRegistry, InitialState> {
  using Type =
      ecs::ComponentList<cuda::PhysicalState, cuda::collision::ObjectCollider>;
};

template <>
struct ecs::ComponentDependents<cuda::ObjRegistry, cuda::MeshPartition> {
  using Type = ecs::ComponentList<ClothAssemblyL2Cache, cuda::PhysicalState,
                                  cuda::collision::ObjectCollider>;
};

template <>
struct ecs::ComponentDependents<cuda::ObjRegistry, cuda::PinIndex> {
  using Type =
      ecs::ComponentList<cuda::PinPosition, cuda::collision::ObjectCollider>;
};

template <>
struct ecs::ComponentDependents<cuda::ObjRegistry, ClothAssemblyL2Cache> {
  using Type = ecs::ComponentList<cuda::assembly::ClothAssemblyL1Cache,
                                  cuda::collision::ObjectCollider>;
};

template <>
struct ecs::ComponentDependents<cuda::ObjRegistry,
                                cuda::assembly::ClothAssemblyL1Cache> {
  using Type = ecs::ComponentList<cuda::solver::ClothADMMHelper>;
};

template <>
struct ecs::ComponentDependents<cuda::ObjRegistry, cuda::PhysicalState> {
  using Type = ecs::ComponentList<cuda::collision::ObjectCollider>;
};

}  // namespace silk

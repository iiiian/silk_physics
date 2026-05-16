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
class ClothAssemblyL1Cache;
class ObjectCollider;
struct PinIndex;
struct PinPosition;
class PhysicalState;
class MeshPartition;

// clang-format off
using ObjRegistry = ::silk::Registry<ClothConfig,
                                          CollisionConfigPlus,
                                          TriMesh,
                                          MeshPartition,
                                          PinIndex,
                                          PinPosition,
                                          InitialState,
                                          ClothAssemblyL2Cache,
                                          PhysicalState,
                                          ObjectCollider,
                                          ClothAssemblyL1Cache,
                                          ClothADMMHelper
                                          >;
// clang-format on

}  // namespace silk::cuda

// ---------------------------------------
// Component Dependencies
// ---------------------------------------

namespace silk {

template <>
struct ComponentDependents<cuda::ObjRegistry, ClothConfig> {
  using Type = ComponentList<ClothAssemblyL2Cache, cuda::ClothADMMHelper,
                             cuda::ClothAssemblyL1Cache, cuda::ObjectCollider>;
};

template <>
struct ComponentDependents<cuda::ObjRegistry, CollisionConfigPlus> {
  using Type = ComponentList<cuda::ObjectCollider>;
};

template <>
struct ComponentDependents<cuda::ObjRegistry, ClothAssemblyL2Cache> {
  using Type = ComponentList<cuda::ClothADMMHelper, cuda::ClothAssemblyL1Cache>;
};

template <>
struct ComponentDependents<cuda::ObjRegistry, cuda::ClothAssemblyL1Cache> {
  using Type = ComponentList<cuda::ClothADMMHelper>;
};

template <>
struct ComponentDependents<cuda::ObjRegistry, cuda::PinIndex> {
  using Type = ComponentList<cuda::PinPosition, cuda::ObjectCollider>;
};

template <>
struct ComponentDependents<cuda::ObjRegistry, cuda::MeshPartition> {
  using Type = ComponentList<ClothAssemblyL2Cache, cuda::ClothADMMHelper,
                             cuda::ClothAssemblyL1Cache, cuda::ObjectCollider,
                             cuda::PhysicalState>;
};

}  // namespace silk

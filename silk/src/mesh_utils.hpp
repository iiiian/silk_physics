#pragma once

#include <optional>

#include "mesh.hpp"
#include "silk/silk.hpp"

namespace silk {

/**
 * @brief Validates MeshConfig and constructs a TriMesh for cloth with
 * comprehensive sanity checks.
 *
 * Performs extensive validation including:
 * - Schema validation (non-null pointers, correct array sizes)
 * - Geometric validation (finite positions, valid face indices)
 * - Quality validation (triangle angle bounds, manifoldness)
 * - Topology validation (single connected component)
 *
 * @param mesh_config Input mesh configuration containing vertex/face data
 * @return Valid TriMesh on success, std::nullopt on validation failure
 */
std::optional<TriMesh> try_make_cloth_mesh(MeshConfig mesh_config);

/**
 * @brief Validates MeshConfig and constructs a TriMesh for obstacle with basic
 * checks.
 *
 * Performs basic validation including:
 * - Schema validation (non-null pointers, correct array sizes)
 * - Geometric validation (finite positions, valid face indices)
 *
 * @param mesh_config Input mesh configuration containing vertex/face data
 * @return Valid TriMesh on success, std::nullopt on validation failure
 */
std::optional<TriMesh> try_make_obstacle_mesh(MeshConfig mesh_config);

}  // namespace silk

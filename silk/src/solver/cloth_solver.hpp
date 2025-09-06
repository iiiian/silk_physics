#pragma once

#include <Eigen/Core>

#include "../barrier_constrain.hpp"
#include "../cloth_solver_data.hpp"
#include "../ecs.hpp"

namespace silk {

// ClothStaticSolverData make_cloth_static_solver_data(const ClothConfig&
// config,
//                                                     const TriMesh& mesh);
//
// std::optional<ClothDynamicSolverData> make_cloth_dynamic_solver_data(
//     const ClothConfig& config, const TriMesh& mesh,
//     const ClothStaticSolverData& static_data, const Pin& pin, float dt);

void reset_cloth_dynamic_solver_data(ClothDynamicSolverData& data);

bool init_all_cloth_solver_data(Registry& registry, float dt);

bool init_all_cloth_outer_loop(Registry& registry,
                               const Eigen::VectorXf& solver_state,
                               const Eigen::VectorXf& solver_state_velocity,
                               const Eigen::VectorXf& solver_state_acceleration,
                               const BarrierConstrain& barrier_constrain,
                               Eigen::VectorXf& rhs);

bool solve_all_cloth_inner_loop(Registry& registry,
                                const Eigen::VectorXf& solver_state,
                                const Eigen::VectorXf& init_rhs,
                                Eigen::VectorXf& out);
}  // namespace silk

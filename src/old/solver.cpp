#include "solver.hpp"

#include <igl/barycentric_coordinates.h>
#include <igl/cotmatrix.h>
#include <igl/doublearea.h>
#include <igl/edges.h>
#include <igl/massmatrix.h>
#include <omp.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <cassert>
#include <limits>
#include <unsupported/Eigen/ArpackSupport>
#include <unsupported/Eigen/KroneckerProduct>

// #include "exact_collision.hpp"
#include "vectorized_jacobian.hpp"

namespace eg = Eigen;

Eigen::SparseMatrix<float> ClothSolver::init_position_lhs() {
  assert(pV_);
  assert(pconstrain_set);
  assert((pV_->rows() != 0));

  std::vector<Eigen::Triplet<float>> triplets;
  for (int i : *pconstrain_set) {
    triplets.emplace_back(3 * i, 3 * i, position_stiffness);
    triplets.emplace_back(3 * i + 1, 3 * i + 1, position_stiffness);
    triplets.emplace_back(3 * i + 2, 3 * i + 2, position_stiffness);
  }

  int vnum = pV_->rows();
  Eigen::SparseMatrix<float> lhs(3 * vnum, 3 * vnum);
  lhs.setFromTriplets(triplets.begin(), triplets.end());
  return lhs;
}

Eigen::SparseMatrix<float> ClothSolver::init_bending_lhs() {
  assert(pV_);
  assert(pF_);
  assert((pV_->rows() != 0));
  assert((pF_->rows() != 0));
  assert((M_.rows() != 0));

  int vnum = pV_->rows();

  Eigen::SparseMatrix<float> cot;
  igl::cotmatrix(*pV_, *pF_, cot);
  Eigen::SparseMatrix<float> W(vnum, vnum);
  W.setIdentity();
  for (int i = 0; i < vnum; i++) {
    // M_ is the vectorized mass mastrix, while W is not vectorized
    W.coeffRef(i, i) = 1 / M_.coeff(3 * i, 3 * i);
  }
  Eigen::SparseMatrix<float> LWL =
      bending_stiffness_ * cot.transpose() * W * cot;
  // vectorize to 3vmum x 3vnum
  return Eigen::kroneckerProduct(LWL, Eigen::Matrix3f::Identity());
}

Eigen::SparseMatrix<float> ClothSolver::init_elastic_lhs() {
  assert(pV_);
  assert(pF_);
  assert((pV_->rows() != 0));
  assert((pF_->rows() != 0));

  // compute triangle area
  igl::doublearea(*pV_, *pF_, area_);
  area_ /= 2;

  // compute deformation matrix for each triangle
  jacobians_.resize(pF_->rows());
  std::vector<Eigen::Triplet<float>> triplets;
  for (int f = 0; f < pF_->rows(); ++f) {
    auto vidx = pF_->row(f);
    std::optional<Matrix69f> J = vectorized_jacobian(
        pV_->row(vidx(0)), pV_->row(vidx(1)), pV_->row(vidx(2)));
    if (!J) {
      spdlog::warn("degenerate triangle {}", f);
      continue;
    }
    jacobians_[f] = *J;
    Eigen::Matrix<float, 9, 9> local_AA = (*J).transpose() * (*J);

    // convert the local AA to global AA
    for (int vi = 0; vi < 3; ++vi) {
      for (int vj = 0; vj < 3; ++vj) {
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            // add stiffness and area as weight
            float val = elastic_stiffness_ * area_(f) *
                        local_AA(3 * vi + i, 3 * vj + j);
            if (abs(val) < zero_prune_threshold_) {
              continue;
            }
            triplets.emplace_back(3 * vidx(vi) + i, 3 * vidx(vj) + j, val);
          }
        }
      }
    }
  }

  int vnum = pV_->rows();
  Eigen::SparseMatrix<float> AA(3 * vnum, 3 * vnum);
  AA.setFromTriplets(triplets.begin(), triplets.end());
  return AA;
}

bool ClothSolver::is_neighboring_face(int f1, int f2) {
  auto f1_verts = pF_->row(f1);
  auto f2_verts = pF_->row(f2);

  for (int v1 : f1_verts) {
    for (int v2 : f2_verts) {
      if (v1 == v2) {
        return true;
      }
    }
  }
  return false;
}

// void ClothSolver::rtc_collision_callback(void* data, RTCCollision*
// collisions,
//                                          unsigned int num_collisions) {
//   ClothSolver* self = static_cast<ClothSolver*>(data);
//   self->velocity_ = (self->future_V_ - *self->pV_) / self->dt_;
//
//   for (unsigned int i = 0; i < num_collisions; ++i) {
//     RTCCollision* pc = (collisions + i);
//     int f1 = pc->primID0;
//     int f2 = pc->primID1;
//
//     if (f1 == f2) {
//       continue;
//     }
//
//     if (self->is_neighboring_face(f1, f2)) {
//       continue;
//     }
//
//     auto weight = [self](int idx) -> float {
//       if (self->pconstrain_set && self->pconstrain_set->contains(idx)) {
//         return 0;
//       }
//       return 1 / self->M_.coeffRef(3 * idx, 3 * idx);
//     };
//
//     auto f1_vidx = self->pF_->row(f1);
//     MovingTriangle t1;
//     t1.v0 = self->future_V_.row(f1_vidx(0));
//     t1.v1 = self->future_V_.row(f1_vidx(1));
//     t1.v2 = self->future_V_.row(f1_vidx(2));
//     t1.w0 = weight(f1_vidx(0));
//     t1.w1 = weight(f1_vidx(1));
//     t1.w2 = weight(f1_vidx(2));
//
//     auto f2_vidx = self->pF_->row(f2);
//     MovingTriangle t2;
//     t2.v0 = self->future_V_.row(f2_vidx(0));
//     t2.v1 = self->future_V_.row(f2_vidx(1));
//     t2.v2 = self->future_V_.row(f2_vidx(2));
//     t2.w0 = weight(f2_vidx(0));
//     t2.w1 = weight(f2_vidx(1));
//     t2.w2 = weight(f2_vidx(2));
//
//     float h = self->collision_thickness_;
//
//     auto update_position = [self, f1_vidx, f2_vidx, t1, t2]() {
//       self->future_V_.row(f1_vidx(0)) = t1.v0;
//       self->future_V_.row(f1_vidx(1)) = t1.v1;
//       self->future_V_.row(f1_vidx(2)) = t1.v2;
//       self->future_V_.row(f2_vidx(0)) = t2.v0;
//       self->future_V_.row(f2_vidx(1)) = t2.v1;
//       self->future_V_.row(f2_vidx(2)) = t2.v2;
//     };
//
//     // test vertex <-> face collision
//     // f1 v0 -- f2
//     if (resolve_vertex_triangle_collision(t1.v0, t1.w0, t2, h)) {
//       self->velocity_.row(f1_vidx(0)) = Eigen::RowVector3f::Zero();
//       update_position();
//       continue;
//     }
//     // f1 v1 -- f2
//     if (resolve_vertex_triangle_collision(t1.v1, t1.w1, t2, h)) {
//       self->velocity_.row(f1_vidx(1)) = Eigen::RowVector3f::Zero();
//       update_position();
//       continue;
//     }
//     // f1 v2 -- f2
//     if (resolve_vertex_triangle_collision(t1.v2, t1.w2, t2, h)) {
//       self->velocity_.row(f1_vidx(1)) = Eigen::RowVector3f::Zero();
//       update_position();
//       continue;
//     }
//     // f2 v0 -- f1
//     if (resolve_vertex_triangle_collision(t2.v0, t2.w0, t1, h)) {
//       self->velocity_.row(f2_vidx(0)) = Eigen::RowVector3f::Zero();
//       update_position();
//       continue;
//     }
//     // f2 v1 -- f1
//     if (resolve_vertex_triangle_collision(t2.v1, t2.w1, t1, h)) {
//       self->velocity_.row(f2_vidx(1)) = Eigen::RowVector3f::Zero();
//       update_position();
//       continue;
//     }
//     // f2 v2 -- f1
//     if (resolve_vertex_triangle_collision(t2.v2, t2.w2, t1, h)) {
//       self->velocity_.row(f2_vidx(2)) = Eigen::RowVector3f::Zero();
//       update_position();
//       continue;
//     }
//   }
// }

bool ClothSolver::init() {
  assert(pV_);
  assert(pF_);
  assert(pconstrain_set);
  assert((pV_->rows() != 0));
  assert((pF_->rows() != 0));

  // save rest position
  x0_ = *pV_;

  // initialize velocity to 0
  int vnum = pV_->rows();
  velocity_ = RMatrixX3f::Zero(vnum, 3);

  // voroni mass
  Eigen::SparseMatrix<float> mass;
  igl::massmatrix(*pV_, *pF_, igl::MASSMATRIX_TYPE_VORONOI, mass);
  mass *= density_;
  // vectorized to 3vnum x 3vnum
  M_ = Eigen::kroneckerProduct(mass, Eigen::Matrix3f::Identity());

  Eigen::SparseMatrix<float> position_lhs = init_position_lhs();
  Eigen::SparseMatrix<float> bending_lhs = init_bending_lhs();
  Eigen::SparseMatrix<float> elastic_lhs = init_elastic_lhs();

  // cholesky decomposition Ax = b
  lhs_ = M_ / dt_ / dt_ + position_lhs + bending_lhs + elastic_lhs;

  Eigen::ArpackGeneralizedSelfAdjointEigenSolver<
      Eigen::SparseMatrix<float>,
      Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>, true>
      eigen_solver;
  assert((low_freq_mode_num <= 3 * vnum));
  eigen_solver.compute(lhs_, low_freq_mode_num, "SM");
  if (eigen_solver.info() != Eigen::Success) {
    spdlog::error("eigen decomposition fail");
    return false;
  }

  lhs_x0_ = lhs_ * pV_->reshaped<Eigen::RowMajor>();
  lhs_eigen_val_ = eigen_solver.eigenvalues();
  lhs_eigen_vec_ = eigen_solver.eigenvectors();

  iterative_solver_.setMaxIterations(max_iterations);
  iterative_solver_.compute(lhs_);
  if (iterative_solver_.info() != Eigen::Success) {
    spdlog::error("iterative solver decomposition fail");
    return false;
  }

  return true;
}

void ClothSolver::reset() {
  velocity_ = {};
  area_ = {};
  M_ = {};
  jacobians_ = {};
  pV_ = nullptr;
  pF_ = nullptr;
  pconstrain_set = nullptr;
}

Eigen::VectorXf ClothSolver::project(const Eigen::VectorXf& V) const {
  int vnum = V.rows();
  // vectorize vertex matrix
  auto vec_V = V.reshaped<Eigen::RowMajor>();

  // thread local rhs
  assert((thread_num_ > 0));
  std::vector<Eigen::VectorXf> thread_proj(thread_num_,
                                           Eigen::VectorXf::Zero(3 * vnum));
// for each triangle, solve projection then modify the rhs
#pragma omp parallel for num_threads(thread_num_)
  for (int f = 0; f < pF_->rows(); ++f) {
    // assemble local vectorized vertex position
    auto vidx = pF_->row(f);
    Eigen::Matrix<float, 9, 1> buffer;
    buffer(Eigen::seqN(0, 3)) = vec_V(Eigen::seqN(3 * vidx(0), 3));
    buffer(Eigen::seqN(3, 3)) = vec_V(Eigen::seqN(3 * vidx(1), 3));
    buffer(Eigen::seqN(6, 3)) = vec_V(Eigen::seqN(3 * vidx(2), 3));

    // SVD decompose deformation.
    // replacing diagonal term with idenity gives us the projection
    Eigen::Matrix<float, 3, 2> J = (jacobians_[f] * buffer).reshaped(3, 2);
    // eigen can't compute thin U and V. so instead compute full UV
    Eigen::JacobiSVD<Eigen::Matrix<float, 3, 2>> svd(
        J, Eigen::ComputeFullV | Eigen::ComputeFullU);
    // this is the projection, deformation F cause by purely rotation +
    // translation
    Eigen::Matrix<float, 3, 2> T =
        svd.matrixU().block<3, 2>(0, 0) * svd.matrixV().transpose();

    // compute the elastic rhs, reuse buffer
    buffer = elastic_stiffness_ * area_[f] * jacobians_[f].transpose() *
             T.reshaped();
    // add it back to thread local rhs
    auto& local_rhs = thread_proj[omp_get_thread_num()];
    local_rhs(Eigen::seqN(3 * vidx(0), 3)) += buffer(Eigen::seqN(0, 3));
    local_rhs(Eigen::seqN(3 * vidx(1), 3)) += buffer(Eigen::seqN(3, 3));
    local_rhs(Eigen::seqN(3 * vidx(2), 3)) += buffer(Eigen::seqN(6, 3));
  }

  // merge thread local rhs back to global rhs
  Eigen::VectorXf proj = Eigen::VectorXf::Zero(3 * vnum);
  for (auto& r : thread_proj) {
    proj += r;
  }

  // position constrain
  for (int i : *pconstrain_set) {
    proj(Eigen::seqN(3 * i, 3)) = position_stiffness * pV_->row(i);
  }

  return proj;
}

std::optional<Eigen::MatrixX3f> ClothSolver::pd_solve() {
  int vnum = pV_->rows();
  // vectorize vertex matrix
  auto vec_V = pV_->reshaped<Eigen::RowMajor>();
  auto vec_veclocity = velocity_.reshaped<Eigen::RowMajor>();
  // basic linear velocity term
  Eigen::VectorXf rhs_base = (M_ / dt_ / dt_) * vec_V +
                             (M_ / dt_) * vec_veclocity +
                             M_ * constant_acce_field_.replicate(vnum, 1);

  Eigen::VectorXf V_next = vec_V + dt_ * vec_veclocity;

  Eigen::VectorXf rhs = rhs_base + project(*pV_);
  // sub space solve
  Eigen::VectorXf b = lhs_eigen_vec_.transpose() * (rhs - lhs_x0_);
  Eigen::VectorXf q = b.array() / lhs_eigen_val_.array();
  Eigen::VectorXf subspace_sol =
      x0_.reshaped<Eigen::RowMajor>() + lhs_eigen_vec_ * q;

  // iterative global solve
  assert((subspace_sol.rows() == 3 * vnum));
  assert((rhs.rows() == 3 * vnum));
  // Eigen::VectorXf sol = subspace_sol;
  Eigen::VectorXf sol = iterative_solver_.solveWithGuess(rhs, subspace_sol);
  if (iterative_solver_.info() != Eigen::Success &&
      iterative_solver_.info() != Eigen::NoConvergence) {
    spdlog::error("iterative solver solve fail");
    return std::nullopt;
  }
  spdlog::info("itertive solver iteration {}", iterative_solver_.iterations());
}

bool ClothSolver::solve() {
  int vnum = pV_->rows();
  // vectorize vertex matrix
  auto vec_V = pV_->reshaped<Eigen::RowMajor>();
  auto vec_veclocity = velocity_.reshaped<Eigen::RowMajor>();
  // basic linear velocity term
  Eigen::VectorXf rhs = (M_ / dt_ / dt_) * vec_V + (M_ / dt_) * vec_veclocity +
                        M_ * constant_acce_field_.replicate(vnum, 1);

  // position cosntrain
  for (int i : *pconstrain_set) {
    rhs(Eigen::seqN(3 * i, 3)) = position_stiffness * pV_->row(i);
  }

  // thread local rhs
  assert((thread_num_ > 0));
  std::vector<Eigen::VectorXf> thread_local_rhs(
      thread_num_, Eigen::VectorXf::Zero(3 * vnum));
// for each triangle, solve projection then modify the rhs
#pragma omp parallel for num_threads(thread_num_)
  for (int f = 0; f < pF_->rows(); ++f) {
    // assemble local vectorized vertex position
    auto vidx = pF_->row(f);
    Eigen::Matrix<float, 9, 1> buffer;
    buffer(Eigen::seqN(0, 3)) = vec_V(Eigen::seqN(3 * vidx(0), 3));
    buffer(Eigen::seqN(3, 3)) = vec_V(Eigen::seqN(3 * vidx(1), 3));
    buffer(Eigen::seqN(6, 3)) = vec_V(Eigen::seqN(3 * vidx(2), 3));

    // SVD decompose deformation.
    // replacing diagonal term with idenity gives us the projection
    Eigen::Matrix<float, 3, 2> J = (jacobians_[f] * buffer).reshaped(3, 2);
    // eigen can't compute thin U and V. so instead compute full UV
    Eigen::JacobiSVD<Eigen::Matrix<float, 3, 2>> svd(
        J, Eigen::ComputeFullV | Eigen::ComputeFullU);
    // this is the projection, deformation F cause by purely rotation +
    // translation
    Eigen::Matrix<float, 3, 2> T =
        svd.matrixU().block<3, 2>(0, 0) * svd.matrixV().transpose();

    // compute the elastic rhs, reuse buffer
    buffer = elastic_stiffness_ * area_[f] * jacobians_[f].transpose() *
             T.reshaped();
    // add it back to thread local rhs
    auto& local_rhs = thread_local_rhs[omp_get_thread_num()];
    local_rhs(Eigen::seqN(3 * vidx(0), 3)) += buffer(Eigen::seqN(0, 3));
    local_rhs(Eigen::seqN(3 * vidx(1), 3)) += buffer(Eigen::seqN(3, 3));
    local_rhs(Eigen::seqN(3 * vidx(2), 3)) += buffer(Eigen::seqN(6, 3));
  }
  // merge thread local rhs back to global rhs
  for (auto& r : thread_local_rhs) {
    rhs += r;
  }

  // sub space solve
  Eigen::VectorXf b = lhs_eigen_vec_.transpose() * (rhs - lhs_x0_);
  Eigen::VectorXf q = b.array() / lhs_eigen_val_.array();
  Eigen::VectorXf subspace_sol =
      x0_.reshaped<Eigen::RowMajor>() + lhs_eigen_vec_ * q;

  // iterative global solve
  assert((subspace_sol.rows() == 3 * vnum));
  assert((rhs.rows() == 3 * vnum));
  // Eigen::VectorXf sol = subspace_sol;
  Eigen::VectorXf sol = iterative_solver_.solveWithGuess(rhs, subspace_sol);
  if (iterative_solver_.info() != Eigen::Success &&
      iterative_solver_.info() != Eigen::NoConvergence) {
    spdlog::error("iterative solver solve fail");
    return false;
  }
  spdlog::info("itertive solver iteration {}", iterative_solver_.iterations());

  // detect and resolve collision and update velocity
  // the collision part is definitely half baked and its pretty janky
  // if (enable_collision) {
  //   collision_detector.detect(pV_, &future_V_, pF_,
  //                             ClothSolver::rtc_collision_callback, this);
  // }

  velocity_ = (sol.reshaped<Eigen::RowMajor>(vnum, 3) - *pV_) / dt_;
  *pV_ = sol.reshaped<Eigen::RowMajor>(vnum, 3);

  return true;
}

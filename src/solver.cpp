#include "solver.hpp"

#include <igl/cotmatrix.h>
#include <igl/doublearea.h>
#include <igl/edges.h>
#include <igl/massmatrix.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <cassert>
#include <iostream>
#include <unordered_set>

#include "vectorized_jacobian.hpp"

namespace eg = Eigen;

bool ClothSolver::init_elastic_constrain() {
  assert(pV_);
  assert(pF_);
  assert((pV_->rows() != 0));
  assert((pF_->rows() != 0));

  // compute triangle area
  igl::doublearea(*pV_, *pF_, area_);
  area_ /= 2;

  // compute deformation matrix for each triangle
  jacobians_.resize(pF_->rows());
  AA_triplets_.clear();
  for (int f = 0; f < pF_->rows(); ++f) {
    auto vidx = pF_->row(f);
    std::optional<Matrix69f> J = vectorized_jacobian(
        pV_->row(vidx(0)), pV_->row(vidx(1)), pV_->row(vidx(2)));
    if (!J) {
      spdlog::warn("degenerate triangle {}", f);
      return false;
    }
    jacobians_[f] = *J;
    eg::Matrix<float, 9, 9> local_AA = J->transpose() * (*J);

    // convert the local AA to global AA
    for (int vi = 0; vi < 3; ++vi) {
      for (int vj = 0; vj < 3; ++vj) {
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            // for global AA, add stiffness and area as weight
            float val = elastic_stiffness_ * area_(f) *
                        local_AA(3 * vi + i, 3 * vj + j);
            if (abs(val) < zero_prune_threshold_) {
              continue;
            }
            AA_triplets_.emplace_back(3 * vidx(vi) + i, 3 * vidx(vj) + j, val);
          }
        }
      }
    }
  }
  return true;
}

bool ClothSolver::init() {
  assert(pV_);
  assert(pF_);
  assert(pconstrain_set);
  assert((pV_->rows() != 0));
  assert((pF_->rows() != 0));
  // in most cases, there will be constrains
  assert(!pconstrain_set->empty());

  int vnum = pV_->rows();

  // initialize velocity to 0
  velocity_ = eg::VectorXf::Zero(3 * vnum);

  // compute voroni mass
  eg::SparseMatrix<float> mass;
  igl::massmatrix(*pV_, *pF_, igl::MASSMATRIX_TYPE_VORONOI, mass);
  std::vector<eg::Triplet<float>> M_triplets;
  for (int i = 0; i < vnum; ++i) {
    M_triplets.emplace_back(3 * i, 3 * i, density_ * mass.coeff(i, i));
    M_triplets.emplace_back(3 * i + 1, 3 * i + 1, density_ * mass.coeff(i, i));
    M_triplets.emplace_back(3 * i + 2, 3 * i + 2, density_ * mass.coeff(i, i));
  }
  M_.resize(3 * vnum, 3 * vnum);
  M_.setFromTriplets(M_triplets.begin(), M_triplets.end());
  // eg::SparseMatrix<float> W(vnum, vnum);
  // W.setIdentity();
  // for (int i = 0; i < vnum; i++) {
  //   W.coeffRef(i, i) = 1.0 / mass.coeff(i, i);
  // }

  // shuffle known and unknow
  // if idx_map(a) = b that means index a is mapped to index b
  // we are mapping 3 * nert vnum becuase vertex position needs to be
  // vectorized in the solving step
  eg::VectorXi idx_map(3 * vnum);
  knum_ = pconstrain_set->size();
  unum_ = vnum - knum_;
  int u_counter = 0;
  int k_counter = unum_;
  for (int i = 0; i < vnum; ++i) {
    if (pconstrain_set->contains(i)) {
      idx_map(3 * i) = 3 * k_counter;
      idx_map(3 * i + 1) = 3 * k_counter + 1;
      idx_map(3 * i + 2) = 3 * k_counter + 2;
      k_counter++;
    } else {
      idx_map(3 * i) = 3 * u_counter;
      idx_map(3 * i + 1) = 3 * u_counter + 1;
      idx_map(3 * i + 2) = 3 * u_counter + 2;
      u_counter++;
    }
  }
  perm_.indices() = idx_map;

  if (!init_elastic_constrain()) {
    return false;
  }
  eg::SparseMatrix<float> AA(3 * vnum, 3 * vnum);
  for (auto t : AA_triplets_) {
    spdlog::info("{} {} {}", t.row(), t.col(), t.value());
  }
  spdlog::info("{}", M_.coeff(0, 0) / dt_ / dt_);
  // exit(1);
  AA.setFromTriplets(AA_triplets_.begin(), AA_triplets_.end());
  AA_triplets_.clear();

  // cholesky decomposition Gx = b
  eg::SparseMatrix<float> G = (M_ / dt_ / dt_ + AA).eval();
  G = (perm_ * G * perm_.transpose()).eval();
  // to solve with constrain, G is decomposed into
  // block matrices for unknow and known
  //
  //     [ Guu Guk ]
  // G = [         ]
  //     [ Gku Gkk ]
  auto Guu = G.block(0, 0, 3 * unum_, 3 * unum_);
  Guk_ = G.block(0, 3 * unum_, 3 * unum_, 3 * knum_);
  cholesky_solver_.compute(Guu);
  if (cholesky_solver_.info() != eg::Success) {
    spdlog::error("cholesky decomposition fail");
    return false;
  }
  return true;
}

void ClothSolver::reset() {
  velocity_ = {};
  area_ = {};
  M_ = {};
  jacobians_ = {};
  AA_triplets_ = {};
  knum_ = 0;
  unum_ = 0;
  perm_ = {};
  Guk_ = {};
  pV_ = nullptr;
  pF_ = nullptr;
  pconstrain_set = nullptr;
}

bool ClothSolver::solve() {
  int vnum = pV_->rows();
  // vectorize vertex matrix
  auto vec_V = pV_->reshaped<eg::RowMajor>();
  // basic linear velocity term
  eg::VectorXf rhs = (M_ / dt_ / dt_) * vec_V + (M_ / dt_) * velocity_ +
                     M_ * constant_acce_field_.replicate(vnum, 1);

  // for each triangle, solve projection then modify the rhs
  for (int f = 0; f < pF_->rows(); ++f) {
    // assemble local vectorized vertex position
    auto vidx = pF_->row(f);
    eg::Matrix<float, 9, 1> buffer;
    buffer(eg::seqN(0, 3)) = vec_V(eg::seqN(3 * vidx(0), 3));
    buffer(eg::seqN(3, 3)) = vec_V(eg::seqN(3 * vidx(1), 3));
    buffer(eg::seqN(6, 3)) = vec_V(eg::seqN(3 * vidx(2), 3));

    // SVD decompose deformation.
    // replacing diagonal term with idenity gives us the projection
    eg::Matrix<float, 3, 2> J = (jacobians_[f] * buffer).reshaped(3, 2);
    // see https://gitlab.com/libeigen/eigen/-/merge_requests/658
    // eigen can't compute thin u v for static sized matrix,
    // above pull request addressed this but was reverted for API
    // compatibility reason. to work around this compute full U, V.
    eg::JacobiSVD<eg::Matrix<float, 3, 2>> svd(
        J, eg::ComputeFullV | eg::ComputeFullU);
    // this is the projection, deformation F cause by purely rotation +
    // translation
    eg::Matrix<float, 3, 2> T =
        svd.matrixU().block<3, 2>(0, 0) * svd.matrixV().transpose();

    // compute the elastic rhs, reuse buffer
    buffer = elastic_stiffness_ * area_[f] * jacobians_[f].transpose() *
             T.reshaped();
    // add it back to global V
    rhs(eg::seqN(3 * vidx(0), 3)) += buffer(eg::seqN(0, 3));
    rhs(eg::seqN(3 * vidx(1), 3)) += buffer(eg::seqN(3, 3));
    rhs(eg::seqN(3 * vidx(2), 3)) += buffer(eg::seqN(6, 3));
  }

  // permute the rhs then add the constrain
  eg::VectorXf perm_rhs = perm_ * rhs;
  eg::VectorXf perm_vec_V = perm_ * vec_V;
  eg::VectorXf b = perm_rhs(eg::seqN(0, 3 * unum_)) -
                   Guk_ * perm_vec_V(eg::seqN(3 * unum_, 3 * knum_));

  eg::VectorXf sol = cholesky_solver_.solve(b);
  if (cholesky_solver_.info() != eg::Success) {
    spdlog::error("cholesky solve fail");
    return false;
  }

  // combine our solution with constrain then permute it back
  // use perm vec V as buffer for new vertex position
  perm_vec_V(eg::seqN(0, 3 * unum_)) = sol;
  perm_vec_V = (perm_.transpose() * perm_vec_V).eval();

  // update veclocity and position
  velocity_ = (perm_vec_V - vec_V) / dt_;
  vec_V = perm_vec_V;
  return true;
}

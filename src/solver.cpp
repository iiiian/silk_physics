#include "solver.hpp"

#include <igl/barycentric_coordinates.h>
#include <igl/cotmatrix.h>
#include <igl/doublearea.h>
#include <igl/edges.h>
#include <igl/massmatrix.h>
#include <omp.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <cassert>
#include <unordered_set>
#include <unsupported/Eigen/KroneckerProduct>

// #include "exact_collision.hpp"
#include "vectorized_jacobian.hpp"

namespace eg = Eigen;

void ClothSolver::init_constrain_permutation() {
  assert(pV_);
  assert(pconstrain_set);

  int vnum = pV_->rows();

  if (pconstrain_set->empty()) {
    knum_ = 0;
    unum_ = vnum;
    return;
  }
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
}

eg::SparseMatrix<float> ClothSolver::init_elastic_constrain() {
  assert(pV_);
  assert(pF_);
  assert((pV_->rows() != 0));
  assert((pF_->rows() != 0));

  // compute triangle area
  igl::doublearea(*pV_, *pF_, area_);
  area_ /= 2;

  // compute deformation matrix for each triangle
  jacobians_.resize(pF_->rows());
  std::vector<eg::Triplet<float>> AA_triplets;
  for (int f = 0; f < pF_->rows(); ++f) {
    auto vidx = pF_->row(f);
    std::optional<Matrix69f> J = vectorized_jacobian(
        pV_->row(vidx(0)), pV_->row(vidx(1)), pV_->row(vidx(2)));
    if (!J) {
      spdlog::warn("degenerate triangle {}", f);
      continue;
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
            AA_triplets.emplace_back(3 * vidx(vi) + i, 3 * vidx(vj) + j, val);
          }
        }
      }
    }
  }

  int vnum = pV_->rows();
  eg::SparseMatrix<float> AA(3 * vnum, 3 * vnum);
  AA.setFromTriplets(AA_triplets.begin(), AA_triplets.end());
  return AA;
}

bool ClothSolver::cholesky_decomposition(const eg::SparseMatrix<float>& A) {
  if (knum_ == 0) {
    cholesky_solver_.compute(A);
    return (cholesky_solver_.info() == eg::Success);
  }

  eg::SparseMatrix<float> G = perm_ * A * perm_.transpose();
  // to solve with constrain, G is decomposed into
  // block matrices for unknow and known
  //
  //     [ Guu Guk ]
  // G = [         ]
  //     [ Gku Gkk ]
  auto Guu = G.block(0, 0, 3 * unum_, 3 * unum_);
  Guk_ = G.block(0, 3 * unum_, 3 * unum_, 3 * knum_);
  cholesky_solver_.compute(Guu);
  return (cholesky_solver_.info() == eg::Success);
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
//       self->velocity_.row(f1_vidx(0)) = eg::RowVector3f::Zero();
//       update_position();
//       continue;
//     }
//     // f1 v1 -- f2
//     if (resolve_vertex_triangle_collision(t1.v1, t1.w1, t2, h)) {
//       self->velocity_.row(f1_vidx(1)) = eg::RowVector3f::Zero();
//       update_position();
//       continue;
//     }
//     // f1 v2 -- f2
//     if (resolve_vertex_triangle_collision(t1.v2, t1.w2, t2, h)) {
//       self->velocity_.row(f1_vidx(1)) = eg::RowVector3f::Zero();
//       update_position();
//       continue;
//     }
//     // f2 v0 -- f1
//     if (resolve_vertex_triangle_collision(t2.v0, t2.w0, t1, h)) {
//       self->velocity_.row(f2_vidx(0)) = eg::RowVector3f::Zero();
//       update_position();
//       continue;
//     }
//     // f2 v1 -- f1
//     if (resolve_vertex_triangle_collision(t2.v1, t2.w1, t1, h)) {
//       self->velocity_.row(f2_vidx(1)) = eg::RowVector3f::Zero();
//       update_position();
//       continue;
//     }
//     // f2 v2 -- f1
//     if (resolve_vertex_triangle_collision(t2.v2, t2.w2, t1, h)) {
//       self->velocity_.row(f2_vidx(2)) = eg::RowVector3f::Zero();
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

  // initialize velocity to 0
  int vnum = pV_->rows();
  velocity_ = RMatrixX3f::Zero(vnum, 3);

  // init permutation
  init_constrain_permutation();

  // voroni mass
  eg::SparseMatrix<float> mass;
  igl::massmatrix(*pV_, *pF_, igl::MASSMATRIX_TYPE_VORONOI, mass);
  mass *= density_;
  // vectorized to 3vnum x 3vnum
  M_ = eg::kroneckerProduct(mass, eg::Matrix3f::Identity());

  // laplacian bending constrain
  eg::SparseMatrix<float> cot;
  igl::cotmatrix(*pV_, *pF_, cot);
  eg::SparseMatrix<float> W(vnum, vnum);
  W.setIdentity();
  for (int i = 0; i < vnum; i++) {
    W.coeffRef(i, i) = bending_stiffness_ / mass.coeffRef(i, i) / density_;
    // W.coeffRef(i, i) = bending_stiffness_ / mass.coeff(i, i);
  }
  eg::SparseMatrix<float> LWL = cot.transpose() * W * cot;
  // vectorize to 3vmum x 3vnum
  eg::SparseMatrix<float> vec_LWL =
      eg::kroneckerProduct(LWL, eg::Matrix3f::Identity());

  // elastic constrain
  eg::SparseMatrix<float> AA = init_elastic_constrain();

  // cholesky decomposition Gx = b
  eg::SparseMatrix<float> G = M_ / dt_ / dt_ + AA + vec_LWL;
  if (!cholesky_decomposition(G)) {
    spdlog::error("cholesky decomposition fail");
    return false;
  }

  future_V_.resize(vnum, 3);
  // collision_detector.init(pF_->rows());

  return true;
}

void ClothSolver::reset() {
  velocity_ = {};
  area_ = {};
  M_ = {};
  jacobians_ = {};
  knum_ = 0;
  unum_ = 0;
  perm_ = {};
  Guk_ = {};
  pV_ = nullptr;
  pF_ = nullptr;
  pconstrain_set = nullptr;
  future_V_ = {};
}

bool ClothSolver::solve() {
  int vnum = pV_->rows();
  // vectorize vertex matrix
  auto vec_V = pV_->reshaped<eg::RowMajor>();
  auto vec_veclocity = velocity_.reshaped<eg::RowMajor>();
  // basic linear velocity term
  eg::VectorXf rhs = (M_ / dt_ / dt_) * vec_V + (M_ / dt_) * vec_veclocity +
                     M_ * constant_acce_field_.replicate(vnum, 1);

  // thread local rhs
  assert((thread_num_ > 0));
  std::vector<eg::VectorXf> thread_local_rhs(thread_num_);
  for (auto& r : thread_local_rhs) {
    r = eg::VectorXf::Zero(3 * vnum);
  }
// for each triangle, solve projection then modify the rhs
#pragma omp parallel for num_threads(thread_num_)
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
    // eigen can't compute thin U and V. so instead compute full UV
    eg::JacobiSVD<eg::Matrix<float, 3, 2>> svd(
        J, eg::ComputeFullV | eg::ComputeFullU);
    // this is the projection, deformation F cause by purely rotation +
    // translation
    eg::Matrix<float, 3, 2> T =
        svd.matrixU().block<3, 2>(0, 0) * svd.matrixV().transpose();

    // compute the elastic rhs, reuse buffer
    buffer = elastic_stiffness_ * area_[f] * jacobians_[f].transpose() *
             T.reshaped();
    // add it back to thread local rhs
    auto& local_rhs = thread_local_rhs[omp_get_thread_num()];
    local_rhs(eg::seqN(3 * vidx(0), 3)) += buffer(eg::seqN(0, 3));
    local_rhs(eg::seqN(3 * vidx(1), 3)) += buffer(eg::seqN(3, 3));
    local_rhs(eg::seqN(3 * vidx(2), 3)) += buffer(eg::seqN(6, 3));
  }
  // merge thread local rhs back to global rhs
  for (auto& r : thread_local_rhs) {
    rhs += r;
  }

  eg::VectorXf b;
  eg::VectorXf perm_rhs;
  eg::VectorXf perm_vec_V;
  // permute and add constrain if necessary
  if (knum_ == 0) {
    b = rhs;
  } else {
    perm_rhs = perm_ * rhs;
    perm_vec_V = perm_ * vec_V;
    b = perm_rhs(eg::seqN(0, 3 * unum_)) -
        Guk_ * perm_vec_V(eg::seqN(3 * unum_, 3 * knum_));
  }

  eg::VectorXf sol = cholesky_solver_.solve(b);
  if (cholesky_solver_.info() != eg::Success) {
    spdlog::error("cholesky solve fail");
    return false;
  }

  if (knum_ == 0) {
    future_V_.reshaped<eg::RowMajor>() = sol;
  } else {
    // combine our solution with constrain then permute it back
    // use perm vec V as buffer for new vertex position
    perm_vec_V(eg::seqN(0, 3 * unum_)) = sol;
    future_V_.reshaped<eg::RowMajor>() = perm_.transpose() * perm_vec_V;
  }

  // detect and resolve collision and update velocity
  // the collision part is definitely half baked and its pretty janky
  // if (enable_collision) {
  //   collision_detector.detect(pV_, &future_V_, pF_,
  //                             ClothSolver::rtc_collision_callback, this);
  // }

  velocity_ = (future_V_ - *pV_) / dt_;
  *pV_ = future_V_;

  return true;
}

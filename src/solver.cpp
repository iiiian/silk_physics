#include <igl/cotmatrix.h>
#include <igl/doublearea.h>
#include <igl/edges.h>
#include <igl/massmatrix.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <cassert>
#include <unordered_set>

#include "vectorized_deformation_matrix.hpp"

namespace eg = Eigen;

class ClothSolver {
  eg::VectorXf velocity_;
  eg::VectorXf area_;
  // voroni mass
  eg::VectorXf M_;
  std::vector<eg::Matrix<float, 6, 9>> F_operators_;
  std::vector<eg::Triplet<float>> AA_triplets_;
  // constrain related
  int knum_ = 0;
  int unum_ = 0;
  eg::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm_;

  // bending related
  eg::SparseMatrix<float> Guk_;
  eg::SimplicialLDLT<eg::SparseMatrix<float>> cholesky_solver_;

  void init_elastic_constrain() {
    assert(pV_);
    assert(pF_);
    assert((pV_->rows() != 0));
    assert((pF_->rows() != 0));

    // compute triangle area
    igl::doublearea(*pV_, *pF_, area_);
    area_ /= 2;

    // compute deformation matrix for each triangle
    F_operators_.resize(pF_->rows());
    for (int f = 0; f < pF_->rows(); ++f) {
      auto vidx = pF_->row(f);
      eg::Matrix<float, 6, 9> A = vectorized_F_operator(
          pV_->row(vidx(0)), pV_->row(vidx(1)), pV_->row(vidx(2)));
      F_operators_[f] = A;
      eg::Matrix<float, 9, 9> local_AA = A.transpose() * A;

      // convert the local AA to global AA
      for (int vi = 0; vi < 3; ++vi) {
        for (int vj = 0; vj < 3; ++vj) {
          for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
              // for global AA, add stiffness and area as weight
              float val =
                  elastic_stiffness_ * area_(f) * A(3 * vi + i, 3 * vj + j);
              if (val < zero_prune_threshold_) {
                continue;
              }
              AA_triplets_.emplace_back(3 * vidx(vi) + i, 3 * vidx(vj) + j,
                                        val);
            }
          }
        }
      }
    }
  }

 public:
  // since libigl store vertices per row, we
  // require vert matrix be row major so vectorization is zero cost
  eg::Matrix<float, eg::Dynamic, 3, eg::RowMajorBit>* pV_ = nullptr;
  eg::MatrixX3i* pF_ = nullptr;
  // constrain set
  std::unordered_set<int>* pconstrain_set = nullptr;
  // constrain vertices, follow the original index order
  eg::Matrix<float, eg::Dynamic, 3, eg::RowMajorBit>* pCV_ = nullptr;

  float elastic_stiffness_ = 1;
  float bending_stiffness_ = 1;
  float density_ = 1;
  float dt_ = 1;
  int xpbd_substep_ = 1;
  float zero_prune_threshold_ = 1e-8;
  eg::Vector3f constant_force_field_ = {0, 0, -1};

  bool init() {
    assert(pV_);
    assert(pF_);
    assert(pconstrain_set);
    assert((pV_->rows() != 0));
    assert((pF_->rows() != 0));
    // in most cases, there will be constrains
    assert(!pconstrain_set->empty());

    int vnum = pV_->rows();

    // compute voroni mass
    eg::SparseMatrix<float> mass;
    igl::massmatrix(*pV_, *pF_, igl::MASSMATRIX_TYPE_VORONOI, mass);
    mass *= density_;
    M_ = mass.diagonal();
    // eg::SparseMatrix<float> W(vnum, vnum);
    // W.setIdentity();
    // for (int i = 0; i < vnum; i++) {
    //   W.coeffRef(i, i) = 1.0 / mass.coeff(i, i);
    // }

    // shuffle known and unknow
    // if idx_map(a) = b that means index a is mapped to index b
    eg::VectorXi idx_map(vnum);
    knum_ = pconstrain_set->size();
    unum_ = vnum - knum_;
    int u_counter = 0;
    int k_counter = unum_;
    for (int i = 0; i < vnum; ++i) {
      if (pconstrain_set->contains(i)) {
        idx_map(i) = k_counter;
        k_counter++;
      } else {
        idx_map(i) = u_counter;
        u_counter++;
      }
    }
    perm_.indices() = idx_map;

    init_elastic_constrain();
    eg::SparseMatrix<float> AA(3 * pV_->rows(), 3 * pV_->rows());
    AA.setFromTriplets(AA_triplets_.begin(), AA_triplets_.end());
    AA_triplets_.clear();

    // cholesky decomposition Gx = b
    eg::SparseMatrix<float> G =
        perm_ * (M_ / dt_ / dt_ + AA) * perm_.transpose();
    // to solve with constrain, G is decomposed into
    // block matrices for unknow and known (constrains)
    //
    //     [ Guu Guk ]
    // G = [         ]
    //     [ Gku Gkk ]
    auto Guu = G.block(0, 0, unum_, unum_);
    Guk_ = G.block(0, unum_, unum_, knum_);
    cholesky_solver_.compute(Guu);
    if (cholesky_solver_.info() != eg::Success) {
      spdlog::error("cholesky decomposition fail");
      return false;
    }
    return true;
  }

  bool solve() {
    int vnum = pV_->rows();
    // vectorize vertices matrix
    auto vec_V = pV_->reshaped<eg::RowMajor>();

    // basic linear speed term
    eg::VectorXf rhs = (M_ / dt_ / dt_).asDiagonal() * vec_V +
                       (M_ / dt_).asDiagonal() * velocity_ +
                       constant_force_field_.replicate(vnum, 1);

    // for each triangle, solve projection then modify the rhs
    for (int f = 0; f < pF_->rows(); ++f) {
      // assemble local vectorized vertex position
      auto vidx = pF_->row(f);
      eg::Matrix<float, 9, 1> local_V;
      local_V(eg::seqN(0, 3)) = vec_V(eg::seqN(3 * vidx(0), 3));
      local_V(eg::seqN(3, 3)) = vec_V(eg::seqN(3 * vidx(1), 3));
      local_V(eg::seqN(6, 3)) = vec_V(eg::seqN(3 * vidx(2), 3));

      // SVD decompose deformation.
      // replacing diagonal term with idenity gives us the projection
      eg::Matrix<float, 3, 2> D = (F_operators_[f] * local_V).reshaped(3, 2);
      eg::JacobiSVD<eg::Matrix<float, 3, 2>,
                    eg::ComputeThinU | eg::ComputeThinV>
          svd(D);
      eg::Matrix<float, 3, 2> T = svd.matrixU() * svd.matrixV().transpose();

      // compute the elastic rhs
      local_V = elastic_stiffness_ * area_[f] * F_operators_[f].transpose() *
                T.reshaped();
      // add it back to global V
      rhs(eg::seqN(3 * vidx(0), 3)) += local_V(eg::seqN(0, 3));
      rhs(eg::seqN(3 * vidx(1), 3)) += local_V(eg::seqN(3, 3));
      rhs(eg::seqN(3 * vidx(2), 3)) += local_V(eg::seqN(6, 3));
    }

    // permute the rhs then add the constrain
    eg::VectorXf perm_rhs = perm_ * rhs;
    eg::VectorXf b = perm_rhs(eg::seqN(0, 3 * unum_)) -
                     Guk_ * pCV_->reshaped<eg::RowMajor>();

    eg::VectorXf sol = cholesky_solver_.solve(b);
    if (cholesky_solver_.info() != eg::Success) {
      spdlog::error("cholesky solve fail");
      return false;
    }

    // combine out solution with constrain then permute it back
    eg::VectorXf temp(3 * vnum);
    temp(eg::seqN(0, 3 * unum_)) = sol;
    temp(eg::seqN(3 * unum_, 3 * knum_)) = pCV_->reshaped<eg::RowMajor>();
    vec_V = perm_.transpose() * temp;
    return true;
  }
};

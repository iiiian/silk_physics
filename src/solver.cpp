#include <igl/cotmatrix.h>
#include <igl/edges.h>
#include <igl/massmatrix.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <cassert>
#include <unordered_set>

namespace eg = Eigen;

class ClothSolver {
  eg::VectorXf Velocity_;
  // mass
  eg::VectorXf M_;
  // edge list
  eg::MatrixX2i EV_;
  // rest length
  eg::VectorXf L0_;
  // constrain related
  int knum_ = 0;
  int unum_ = 0;
  eg::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm_;

  // bending related
  eg::SparseMatrix<float> Auk_;
  eg::SimplicialLDLT<eg::SparseMatrix<float>> cholesky_solver_;

  void solve_edge_constrain() {
    assert((M_.rows() != 0));
    assert((EV_.rows() != 0));
    assert((L0_.rows() != 0));
    assert(pV_);

    for (int i = 0; i < EV_.rows(); ++i) {
      int v1_idx = EV_(i, 0);
      int v2_idx = EV_(i, 1);
      auto v1 = pV_->row(v1_idx);
      auto v2 = pV_->row(v2_idx);
      float m1 = M_(v1_idx);
      float m2 = M_(v2_idx);

      eg::RowVector3f delta = v1 - v2;
      float length_delta = L0_(i) - delta.norm();

      v1 += m2 / (m1 + m2) * length_delta * (-delta);
      v2 += m1 / (m1 + m2) * length_delta * delta;
    }
  }

  bool solve_bending_constrain(eg::MatrixX3f& inertia_V) {
    eg::MatrixX3f Vperm = perm_ * (*pV_);
    auto xk = Vperm.block(unum_, 0, knum_, 3);
    eg::MatrixX3d sol = cholesky_solver_.solve(bu_ - Auk_ * xk);
    if (solver_.info() != eg::Success) {
      spdlog::error("simplicial LDLT solving fail");
      throw std::runtime_error("simplicial LDLT solving fail");
    }

    V_p.block(0, 0, unum_, 3) = sol;
    return perm_.transpose() * V_p;
  }

 public:
  eg::MatrixX3f* pV_ = nullptr;
  eg::MatrixX3i* pF_ = nullptr;
  // constrain
  std::unordered_set<int>* pC_ = nullptr;

  // we assumes stretching stiffness is infinite
  // aka. the cloth barely stretch
  // bending stiffness
  float stiffness_ = 1;
  float density_ = 1;
  float dt_ = 1;
  int xpbd_substep_ = 1;

  bool init() {
    assert(pV_);
    assert(pF_);
    assert(pC_);
    assert((pV_->rows() != 0));
    assert((pF_->rows() != 0));
    // in most cases, there will be constrains
    assert(!pC_->empty());

    int vnum = pV_->rows();

    // compute voroni mass
    eg::SparseMatrix<float> mass;
    igl::massmatrix(*pV_, *pF_, igl::MASSMATRIX_TYPE_VORONOI, mass);
    mass *= density_;
    M_.resize(vnum);
    eg::SparseMatrix<float> W(vnum, vnum);
    W.setIdentity();
    for (int i = 0; i < vnum; i++) {
      M_(i) = mass.coeff(i, i);
      W.coeffRef(i, i) = 1.0 / mass.coeff(i, i);
    }

    eg::SparseMatrix<float> Cot;
    igl::cotmatrix(*pV_, *pF_, Cot);
    eg::SparseMatrix<float> LWL = Cot.transpose() * W * Cot;

    // shuffle known and unknow
    // if idx_map(a) = b that means index a is mapped to index b
    eg::VectorXi idx_map(vnum);
    knum_ = pC_->size();
    unum_ = vnum - knum_;
    int u_counter = 0;
    int k_counter = unum_;
    for (int i = 0; i < vnum; ++i) {
      if (pC_->contains(i)) {
        idx_map(i) = k_counter;
        k_counter++;
      } else {
        idx_map(i) = u_counter;
        u_counter++;
      }
    }
    perm_.indices() = idx_map;

    // cholesky decomposition
    eg::SparseMatrix<float> A =
        perm_ * M_ +
        (dt_ * dt_) * stiffness_ * (perm_.transpose() * LWL * perm_);
    auto Auu = LWL.block(0, 0, unum_, unum_);
    Auk_ = LWL.block(0, unum_, unum_, knum_);
    cholesky_solver_.compute(Auu);
    if (cholesky_solver_.info() != eg::Success) {
      spdlog::error("cholesky decomposition fail");
      return false;
    }
    return true;
  }

  bool solve() {}
};

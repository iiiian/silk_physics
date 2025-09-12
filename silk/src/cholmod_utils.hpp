#pragma once

#include <cholmod.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cassert>

#include "symmetry.hpp"

namespace silk::cholmod_raii {

class CholmodCommon {
 private:
  cholmod_common common_;

 public:
  CholmodCommon();
  CholmodCommon(const CholmodCommon&) = delete;
  CholmodCommon(CholmodCommon&&) = delete;

  ~CholmodCommon();

  CholmodCommon& operator=(const CholmodCommon&) = delete;
  CholmodCommon& operator=(CholmodCommon&&) = delete;

  operator cholmod_common*();

  cholmod_common* raw();
};

// all cholmod function call in this codebase should use this common
extern CholmodCommon common;

template <typename T>
using CholmodFreeFunc = int (*)(T**, cholmod_common*);

template <typename T>
using CholmodCopyFunc = T* (*)(T*, cholmod_common*);

template <typename T, CholmodCopyFunc<T> cholmod_copy,
          CholmodFreeFunc<T> cholmod_free>
class RAIIWrapper {
 private:
  // ptr to cholmod type like cholmod_sparse*
  T* handle_ = nullptr;

 public:
  RAIIWrapper() = default;

  RAIIWrapper(T* handle) : handle_(handle) {}

  RAIIWrapper(const RAIIWrapper& other) {
    if (other.handle_) {
      this->handle_ = cholmod_copy(const_cast<T*>(other.handle_), common);
    } else {
      this->handle_ = nullptr;
    }
  }

  RAIIWrapper(RAIIWrapper&& other) noexcept {
    this->handle_ = other.handle_;
    other.handle_ = nullptr;
  }

  ~RAIIWrapper() { clear(); }

  RAIIWrapper& operator=(const RAIIWrapper& other) {
    if (this == &other) {
      return *this;
    }

    clear();
    if (other.handle_) {
      handle_ = cholmod_copy(const_cast<T*>(other.handle_), common);
    }

    return *this;
  }

  RAIIWrapper& operator=(RAIIWrapper&& other) noexcept {
    if (this == &other) {
      return *this;
    }

    clear();
    handle_ = other.handle_;
    other.handle_ = nullptr;

    return *this;
  }

  void clear() {
    if (!handle_) {
      return;
    }

    cholmod_free(&handle_, common);
  }

  operator T*() const { return handle_; }

  T* raw() const { return handle_; }

  bool is_empty() const { return (handle_ == nullptr); }
};

using CholmodDense =
    RAIIWrapper<cholmod_dense, &cholmod_copy_dense, &cholmod_free_dense>;
using CholmodSparse =
    RAIIWrapper<cholmod_sparse, &cholmod_copy_sparse, &cholmod_free_sparse>;
using CholmodFactor =
    RAIIWrapper<cholmod_factor, &cholmod_copy_factor, &cholmod_free_factor>;

template <typename Scalar>
cholmod_sparse make_cholmod_sparse_view(
    Eigen::SparseMatrix<Scalar, Eigen::ColMajor, int>& m,
    Symmetry sym_stat = Symmetry::NotSymmetric) {
  static_assert(
      std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>,
      "cholmod sparse view only accepts float/double (real) matrices");

  if (!m.isCompressed()) {
    m.makeCompressed();
  }

  cholmod_sparse s;
  s.nrow = m.rows();
  s.ncol = m.cols();
  s.nzmax = m.nonZeros();
  s.p = static_cast<void*>(m.outerIndexPtr());
  s.i = static_cast<void*>(m.innerIndexPtr());
  s.nz = nullptr;
  s.x = static_cast<void*>(m.valuePtr());
  s.z = nullptr;
  s.stype = static_cast<int>(sym_stat);
  s.itype = CHOLMOD_INT;
  s.xtype = CHOLMOD_REAL;

  if constexpr (std::is_same_v<Scalar, double>) {
    s.dtype = CHOLMOD_DOUBLE;
  } else {
    s.dtype = CHOLMOD_SINGLE;
  }

  s.sorted = 1;
  s.packed = 1;

  return s;
}

template <typename Derived>
cholmod_dense make_cholmod_dense_view(Eigen::DenseBase<Derived>& M) {
  static_assert(!Derived::IsRowMajor, "cholmod expect col major matrices");

  using Scalar = typename Derived::Scalar;
  static_assert(std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>,
                "cholmod dense view only accepts float/double (real) matrices");

  int dtype;
  if constexpr (std::is_same_v<Scalar, float>) {
    dtype = CHOLMOD_SINGLE;
  } else
    dtype = CHOLMOD_DOUBLE;

  cholmod_dense d;
  d.nrow = M.rows();
  d.ncol = M.cols();
  d.d = M.outerStride();
  d.nzmax = d.d * d.ncol;
  d.x = static_cast<void*>(M.data());
  d.z = nullptr;
  d.xtype = CHOLMOD_REAL;
  d.dtype = dtype;

  return d;
}

Eigen::Map<Eigen::VectorXf> make_eigen_dense_vector_view(cholmod_dense* v);

}  // namespace silk::cholmod_raii

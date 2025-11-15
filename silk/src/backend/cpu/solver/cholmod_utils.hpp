/** @file
 * RAII wrappers and utilities for interfacing between Eigen and CHOLMOD sparse
 * linear algebra library.
 *
 * Provides safe memory management for CHOLMOD objects and zero-copy view
 * conversions between Eigen and CHOLMOD data structures.
 */

#pragma once

#include <cholmod.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cassert>

#include "common/symmetry.hpp"

namespace silk::cpu::cholmod_raii {

/** RAII wrapper for cholmod_common context object.
 *
 * Manages CHOLMOD library initialization and cleanup. Non-copyable and
 * non-movable to ensure single ownership of the CHOLMOD context.
 */
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
  /** Implicit conversion to cholmod_common* for CHOLMOD function calls. */
  operator cholmod_common*();
  /** Explicit access to underlying cholmod_common pointer. */
  cholmod_common* raw();
};

/** Global CHOLMOD context - all CHOLMOD function calls in this codebase should
 * use this. */
extern CholmodCommon common;

/** Function pointer type for CHOLMOD free functions (e.g.,
 * cholmod_free_sparse). */
template <typename T>
using CholmodFreeFunc = int (*)(T**, cholmod_common*);
/** Function pointer type for CHOLMOD copy functions (e.g.,
 * cholmod_copy_sparse). */
template <typename T>
using CholmodCopyFunc = T* (*)(T*, cholmod_common*);

/** Generic RAII wrapper for CHOLMOD objects with automatic memory management.
 *
 * Provides safe ownership, copying, and moving of CHOLMOD data structures.
 *
 * @tparam T CHOLMOD object type (e.g., cholmod_sparse, cholmod_dense)
 * @tparam cholmod_copy Function to deep-copy the CHOLMOD object
 * @tparam cholmod_free Function to free the CHOLMOD object
 */
template <typename T, CholmodCopyFunc<T> cholmod_copy,
          CholmodFreeFunc<T> cholmod_free>
class RAIIWrapper {
 private:
  /** Pointer to the managed CHOLMOD object (e.g., cholmod_sparse*). */
  T* handle_ = nullptr;

 public:
  RAIIWrapper() = default;
  /** Constructor takes ownership of existing CHOLMOD object.
   * @param handle CHOLMOD object to manage (takes ownership)
   */
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
  /** Explicitly free the managed CHOLMOD object. */
  void clear() {
    if (!handle_) {
      return;
    }
    cholmod_free(&handle_, common);
  }
  /** Implicit conversion to raw CHOLMOD pointer for function calls. */
  operator T*() const { return handle_; }
  /** Explicit access to raw CHOLMOD pointer. */
  T* raw() const { return handle_; }
  /** Check if wrapper is empty (managing no object). */
  bool is_empty() const { return (handle_ == nullptr); }
};

/** RAII wrapper for cholmod_dense objects (dense matrices/vectors). */
using CholmodDense =
    RAIIWrapper<cholmod_dense, &cholmod_copy_dense, &cholmod_free_dense>;
/** RAII wrapper for cholmod_sparse objects (sparse matrices). */
using CholmodSparse =
    RAIIWrapper<cholmod_sparse, &cholmod_copy_sparse, &cholmod_free_sparse>;
/** RAII wrapper for cholmod_factor objects (Cholesky factorizations). */
using CholmodFactor =
    RAIIWrapper<cholmod_factor, &cholmod_copy_factor, &cholmod_free_factor>;

/** Create a zero-copy CHOLMOD view of an Eigen sparse matrix.
 *
 * The returned cholmod_sparse shares memory with the input matrix - no data
 * is copied. The input matrix must remain valid while the view is used.
 *
 * @tparam Scalar Matrix element type (must be float or double)
 * @param m Input Eigen sparse matrix (will be compressed if needed)
 * @param sym_stat Matrix symmetry property for CHOLMOD optimization
 * @return cholmod_sparse view sharing memory with input matrix
 */
template <typename Scalar>
cholmod_sparse make_cholmod_sparse_view(
    Eigen::SparseMatrix<Scalar, Eigen::ColMajor, int>& m,
    Symmetry sym_stat = Symmetry::NotSymmetric) {
  static_assert(
      std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>,
      "cholmod sparse view only accepts float/double (real) matrices");
  // CHOLMOD requires compressed format for efficient operations
  if (!m.isCompressed()) {
    m.makeCompressed();
  }
  cholmod_sparse s;
  s.nrow = m.rows();
  s.ncol = m.cols();
  s.nzmax = m.nonZeros();
  s.p = static_cast<void*>(m.outerIndexPtr());  // Column pointers
  s.i = static_cast<void*>(m.innerIndexPtr());  // Row indices
  s.nz = nullptr;                          // Not used for compressed format
  s.x = static_cast<void*>(m.valuePtr());  // Non-zero values
  s.z = nullptr;                           // No imaginary part (real matrix)
  s.stype = static_cast<int>(sym_stat);    // Symmetry property
  s.itype = CHOLMOD_INT;                   // Integer type for indices
  s.xtype = CHOLMOD_REAL;                  // Real (not complex) values
  if constexpr (std::is_same_v<Scalar, double>) {
    s.dtype = CHOLMOD_DOUBLE;
  } else {
    s.dtype = CHOLMOD_SINGLE;
  }
  s.sorted = 1;  // Eigen matrices are always sorted within columns
  s.packed = 1;  // Compressed format is always packed
  return s;
}

/** Create a zero-copy CHOLMOD view of an Eigen dense matrix.
 *
 * The returned cholmod_dense shares memory with the input matrix - no data
 * is copied. The input matrix must remain valid while the view is used.
 *
 * @tparam Derived Eigen dense matrix type (must be column-major)
 * @param M Input Eigen dense matrix
 * @return cholmod_dense view sharing memory with input matrix
 */
template <typename Derived>
cholmod_dense make_cholmod_dense_view(Eigen::DenseBase<Derived>& M) {
  static_assert(!Derived::IsRowMajor, "cholmod expect col major matrices");
  using Scalar = typename Derived::Scalar;
  static_assert(std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>,
                "cholmod dense view only accepts float/double (real) matrices");
  // Determine precision type for CHOLMOD
  int dtype;
  if constexpr (std::is_same_v<Scalar, float>) {
    dtype = CHOLMOD_SINGLE;
  } else
    dtype = CHOLMOD_DOUBLE;
  cholmod_dense d;
  d.nrow = M.rows();
  d.ncol = M.cols();
  d.d = M.outerStride();   // Leading dimension (stride between columns)
  d.nzmax = d.d * d.ncol;  // Total allocated size
  d.x = static_cast<void*>(M.derived().data());  // Data pointer
  d.z = nullptr;           // No imaginary part (real matrix)
  d.xtype = CHOLMOD_REAL;  // Real (not complex) values
  d.dtype = dtype;         // Precision type
  return d;
}

/** Create an Eigen vector view of a CHOLMOD dense vector.
 *
 * Creates a zero-copy Eigen::Map that allows accessing CHOLMOD dense data
 * as an Eigen vector. Currently only supports single-precision float vectors.
 *
 * @param v CHOLMOD dense vector (must be single-precision, real, single-column)
 * @return Eigen::Map view of the CHOLMOD vector data
 */
Eigen::Map<Eigen::VectorXf> make_eigen_dense_vector_view(cholmod_dense* v);

}  // namespace silk::cpu::cholmod_raii

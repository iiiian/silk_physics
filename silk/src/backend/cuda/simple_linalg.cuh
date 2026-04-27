#pragma once

#include <cassert>
#include <cstdint>
#include <cuda/std/array>
#include <cuda/std/limits>
#include <cuda/std/mdspan>
#include <cuda/std/type_traits>
#include <cuda/std/utility>

#include "backend/cuda/cuda_utils.cuh"

namespace silk::cuda {

template <typename T, int m, int n>
class MatView {
 public:
  static_assert(m > 0 && n > 0, "Invalid row size or col size.");

  static constexpr int M = m;
  static constexpr int N = n;

  using Scalar = ctd::remove_const_t<T>;
  using ConstScalar = ctd::add_const_t<T>;
  using Extent = ctd::extents<int, m, n>;
  using Mapping = ctd::layout_stride::mapping<Extent>;
  using Mdspan = ctd::mdspan<T, Extent, ctd::layout_stride>;

  Mdspan mdspan;

  __both__ constexpr MatView() = delete;

  __both__ constexpr MatView(Mdspan mdspan) : mdspan(mdspan){};

  __both__ constexpr MatView(T* ptr, int row_stride, int col_stride)
      : mdspan(ptr,
               Mapping{Extent{}, ctd::array<int, 2>{row_stride, col_stride}}) {
    assert(ptr);
  };

  __both__ static constexpr MatView row_major(T* ptr) {
    assert(ptr);
    return MatView{ptr, n, 1};
  }

  __both__ static constexpr MatView col_major(T* ptr) {
    assert(ptr);
    return MatView{ptr, 1, m};
  }

  __both__ constexpr MatView<ConstScalar, m, n> const_view() const {
    if constexpr (ctd::is_same_v<T, ConstScalar>) {
      return *this;
    } else {
      return MatView<ConstScalar, m, n>(mdspan.data_handle(), mdspan.stride(0),
                                        mdspan.stride(1));
    }
  }

  __both__ constexpr operator MatView<ConstScalar, m, n>() const {
    return const_view();
  }

  __both__ constexpr T& operator()(int i, int j) const {
    assert(i >= 0 && i < m && "Invalid row index");
    assert(j >= 0 && j < n && "Invalid col index");
    return mdspan(i, j);
  }

  __both__ constexpr T& operator()(int i) const {
    static_assert(n == 1 || m == 1, "Can not index 2d matrix using one index.");
    assert(i >= 0 && i < m * n && "Invalid index");

    if constexpr (m == 1) {
      return mdspan(0, i);
    } else {
      return mdspan(i, 0);
    }
  }

  template <int row_num, int col_num>
  __both__ constexpr MatView<T, row_num, col_num> block(int row_start,
                                                        int col_start) const {
    static_assert(row_num > 0 && col_num > 0, "Invalid row_num or col_num.");
    assert(row_start >= 0 && col_start >= 0 &&
           "Invalid row_start or col_start");
    assert((row_start + row_num) <= m && "Row index out of range.");
    assert((col_start + col_num) <= n && "Col index out of range.");

    int row_stride = mdspan.stride(0);
    int col_stride = mdspan.stride(1);
    T* data =
        mdspan.data_handle() + row_start * row_stride + col_start * col_stride;
    return MatView<T, row_num, col_num>{data, row_stride, col_stride};
  }

  __both__ constexpr MatView<T, 1, n> row(int row) const {
    assert(row >= 0 && row < m && "Row index out of range.");
    return block<1, n>(row, 0);
  }

  __both__ constexpr MatView<T, m, 1> col(int col) const {
    assert(col >= 0 && col < n && "Col index out of range.");
    return block<m, 1>(0, col);
  }

  __both__ constexpr MatView<T, n, m> transpose() const {
    // Swap row and col stride.
    return MatView<T, n, m>{mdspan.data_handle(), mdspan.stride(1),
                            mdspan.stride(0)};
  }
};

/// @brief Row major dense matrix.
template <typename T, int m, int n>
class Mat {
 public:
  static_assert(m > 0 && n > 0, "Invalid rows and cols.");

  static constexpr int M = m;
  static constexpr int N = n;

  using Scalar = ctd::remove_const_t<T>;

  ctd::array<T, m * n> data;

  __both__ static constexpr Mat zeros() {
    // Should default init to zero.
    return Mat{};
  }

  __both__ static constexpr Mat ones() {
    Mat result;

#pragma unroll
    for (int i = 0; i < m; ++i) {
#pragma unroll
      for (int j = 0; j < n; ++j) {
        result(i, j) = T{1};
      }
    }
    return result;
  }

  __both__ static constexpr Mat max() {
    Mat result;

#pragma unroll
    for (int i = 0; i < m; ++i) {
#pragma unroll
      for (int j = 0; j < n; ++j) {
        result(i, j) = ctd::numeric_limits<T>::max();
      }
    }
    return result;
  }

  __both__ static constexpr Mat min() {
    Mat result;

#pragma unroll
    for (int i = 0; i < m; ++i) {
#pragma unroll
      for (int j = 0; j < n; ++j) {
        result(i, j) = ctd::numeric_limits<T>::min();
      }
    }
    return result;
  }

  __both__ static constexpr Mat identity() {
    static_assert(m == n, "Identity ctor requires square matrix.");
    Mat result = zeros();

#pragma unroll
    for (int i = 0; i < n; ++i) {
      result(i, i) = T{1};
    }

    return result;
  }

  template <typename V>
  static Mat vec_like(const V& vec) {
    Mat result;
    for (int i = 0; i < m * n; ++i) {
      result(i) = vec(i);
    }
    return m;
  }

  template <typename M>
  static Mat mat_like(const M& mat) {
    Mat result;
    for (int i = 0; i < m; ++i) {
      for (int j = 0; j < n; ++j) {
        result(i, j) = mat(i, j);
      }
    }
    return m;
  }

  __both__ constexpr MatView<T, m, n> view() {
    return MatView<T, m, n>::row_major(data.data());
  }

  __both__ constexpr operator MatView<T, m, n>() { return view(); }

  __both__ constexpr MatView<const T, m, n> const_view() const {
    return MatView<const T, m, n>::row_major(data.data());
  }

  __both__ constexpr operator MatView<const T, m, n>() const {
    return const_view();
  }

  __both__ constexpr T& operator()(int i, int j) { return view()(i, j); }

  __both__ constexpr const T& operator()(int i, int j) const {
    return const_view()(i, j);
  }

  __both__ constexpr T& operator()(int i) { return view()(i); }

  __both__ constexpr const T& operator()(int i) const {
    return const_view()(i);
  }
};

using Vec2f = Mat<float, 2, 1>;
using Vec3f = Mat<float, 3, 1>;
using Vec4f = Mat<float, 4, 1>;
using Mat22f = Mat<float, 2, 2>;
using Mat23f = Mat<float, 2, 3>;
using Mat32f = Mat<float, 3, 2>;
using Mat33f = Mat<float, 3, 3>;
using Mat24f = Mat<float, 2, 4>;
using Mat34f = Mat<float, 3, 4>;
using Mat42f = Mat<float, 4, 2>;
using Mat43f = Mat<float, 4, 3>;
using Mat44f = Mat<float, 4, 4>;

using Vec2fV = MatView<const float, 2, 1>;
using Vec3fV = MatView<const float, 3, 1>;
using Vec4fV = MatView<const float, 4, 1>;
using Mat22fV = MatView<const float, 2, 2>;
using Mat23fV = MatView<const float, 2, 3>;
using Mat32fV = MatView<const float, 3, 2>;
using Mat33fV = MatView<const float, 3, 3>;
using Mat24fV = MatView<const float, 2, 4>;
using Mat34fV = MatView<const float, 3, 4>;
using Mat42fV = MatView<const float, 4, 2>;
using Mat43fV = MatView<const float, 4, 3>;
using Mat44fV = MatView<const float, 4, 4>;

using Vec2i = Mat<int, 2, 1>;
using Vec3i = Mat<int, 3, 1>;
using Vec4i = Mat<int, 4, 1>;
using Mat22i = Mat<int, 2, 2>;
using Mat23i = Mat<int, 2, 3>;
using Mat32i = Mat<int, 3, 2>;
using Mat33i = Mat<int, 3, 3>;
using Mat24i = Mat<int, 2, 4>;
using Mat34i = Mat<int, 3, 4>;
using Mat42i = Mat<int, 4, 2>;
using Mat43i = Mat<int, 4, 3>;
using Mat44i = Mat<int, 4, 4>;

using Vec2iV = MatView<const int, 2, 1>;
using Vec3iV = MatView<const int, 3, 1>;
using Vec4iV = MatView<const int, 4, 1>;
using Mat22iV = MatView<const int, 2, 2>;
using Mat23iV = MatView<const int, 2, 3>;
using Mat32iV = MatView<const int, 3, 2>;
using Mat33iV = MatView<const int, 3, 3>;
using Mat24iV = MatView<const int, 2, 4>;
using Mat34iV = MatView<const int, 3, 4>;
using Mat42iV = MatView<const int, 4, 2>;
using Mat43iV = MatView<const int, 4, 3>;
using Mat44iV = MatView<const int, 4, 4>;

using Vec2u = Mat<uint32_t, 2, 1>;
using Vec3u = Mat<uint32_t, 3, 1>;
using Vec4u = Mat<uint32_t, 4, 1>;
using Mat22u = Mat<uint32_t, 2, 2>;
using Mat23u = Mat<uint32_t, 2, 3>;
using Mat32u = Mat<uint32_t, 3, 2>;
using Mat33u = Mat<uint32_t, 3, 3>;
using Mat24u = Mat<uint32_t, 2, 4>;
using Mat34u = Mat<uint32_t, 3, 4>;
using Mat42u = Mat<uint32_t, 4, 2>;
using Mat43u = Mat<uint32_t, 4, 3>;
using Mat44u = Mat<uint32_t, 4, 4>;

using Vec2uV = MatView<const uint32_t, 2, 1>;
using Vec3uV = MatView<const uint32_t, 3, 1>;
using Vec4uV = MatView<const uint32_t, 4, 1>;
using Mat22uV = MatView<const uint32_t, 2, 2>;
using Mat23uV = MatView<const uint32_t, 2, 3>;
using Mat32uV = MatView<const uint32_t, 3, 2>;
using Mat33uV = MatView<const uint32_t, 3, 3>;
using Mat24uV = MatView<const uint32_t, 2, 4>;
using Mat34uV = MatView<const uint32_t, 3, 4>;
using Mat42uV = MatView<const uint32_t, 4, 2>;
using Mat43uV = MatView<const uint32_t, 4, 3>;
using Mat44uV = MatView<const uint32_t, 4, 4>;

template <typename Dst, typename Src>
__both__ constexpr void assign(Dst& dst, const Src& src) {
  static_assert(ctd::is_same_v<typename Dst::Scalar, typename Src::Scalar>,
                "Matric type mismatch.");
  static_assert(Dst::M == Src::M && Dst::N == Src::N, "Matrix dim mismatch.");

#pragma unroll
  for (int i = 0; i < Dst::M; ++i) {
#pragma unroll
    for (int j = 0; j < Dst::N; ++j) {
      dst(i, j) = src(i, j);
    }
  }
}

template <typename X, typename Y>
__both__ constexpr Mat<typename X::Scalar, X::M, X::N> axpby(
    typename X::Scalar a, const X& x, typename X::Scalar b, const Y& y) {
  static_assert(ctd::is_same_v<typename X::Scalar, typename Y::Scalar>,
                "Matric type mismatch.");
  static_assert(X::M == Y::M && X::N == Y::N, "Matrix dim mismatch.");

  Mat<typename X::Scalar, X::M, X::N> result;
#pragma unroll
  for (int i = 0; i < X::M; ++i) {
#pragma unroll
    for (int j = 0; j < X::N; ++j) {
      result(i, j) = a * x(i, j) + b * y(i, j);
    }
  }
  return result;
}

template <typename X>
__both__ constexpr Mat<typename X::Scalar, X::M, X::N> axpb(
    typename X::Scalar a, const X& x, typename X::Scalar b) {
  Mat<typename X::Scalar, X::M, X::N> result;
#pragma unroll
  for (int i = 0; i < X::M; ++i) {
#pragma unroll
    for (int j = 0; j < X::N; ++j) {
      result(i, j) = a * x(i, j) + b;
    }
  }
  return result;
}

template <typename X>
__both__ constexpr Mat<typename X::Scalar, X::M, X::N> ax(typename X::Scalar a,
                                                          const X& x) {
  Mat<typename X::Scalar, X::M, X::N> result;
#pragma unroll
  for (int i = 0; i < X::M; ++i) {
#pragma unroll
    for (int j = 0; j < X::N; ++j) {
      result(i, j) = a * x(i, j);
    }
  }
  return result;
}

template <typename X, typename Y>
__both__ constexpr X::Scalar dot(const X& x, const Y& y) {
  static_assert(ctd::is_same_v<typename X::Scalar, typename Y::Scalar>,
                "Matric type mismatch.");
  static_assert(X::M == Y::M && X::N == Y::N, "Matrix dim mismatch.");

  float result = 0.0f;
#pragma unroll
  for (int i = 0; i < X::M; ++i) {
#pragma unroll
    for (int j = 0; j < X::N; ++j) {
      result += x(i, j) * y(i, j);
    }
  }
  return result;
}

template <typename X>
__both__ constexpr X::Scalar squared_norm(const X& x) {
  float result = 0.0f;
#pragma unroll
  for (int i = 0; i < X::M; ++i) {
#pragma unroll
    for (int j = 0; j < X::N; ++j) {
      result += x(i, j) * x(i, j);
    }
  }
  return result;
}

template <typename X>
__both__ constexpr X::Scalar norm(const X& x) {
  float result = 0.0f;
#pragma unroll
  for (int i = 0; i < X::M; ++i) {
#pragma unroll
    for (int j = 0; j < X::N; ++j) {
      result += x(i, j) * x(i, j);
    }
  }
  return sqrt(result);
}

template <typename X, typename Y>
__both__ constexpr auto mat_mul(const X& x, const Y& y)
    -> Mat<typename X::Scalar, X::M, Y::N> {
  static_assert(ctd::is_same_v<typename X::Scalar, typename Y::Scalar>,
                "Matric type mismatch.");
  static_assert(X::N == Y::M, "Matrix dim mismatch.");

  auto result = Mat<typename X::Scalar, X::M, Y::N>::zeros();
#pragma unroll
  for (int i = 0; i < X::M; ++i) {
#pragma unroll
    for (int k = 0; k < Y::N; ++k) {
#pragma unroll
      for (int j = 0; j < X::N; ++j) {
        result(i, k) += x(i, j) * y(j, k);
      }
    }
  }

  return result;
}

template <typename X, typename Y, typename BinaryOp>
__both__ constexpr auto binary(const X& x, const Y& y, BinaryOp op)
    -> Mat<typename X::Scalar, X::M, X::N> {
  static_assert(ctd::is_same_v<typename X::Scalar, typename Y::Scalar>,
                "Matric type mismatch.");
  static_assert(X::M == Y::M && X::N == Y::N, "Matrix dim mismatch.");

  Mat<typename X::Scalar, X::M, X::N> result;
#pragma unroll
  for (int i = 0; i < X::M; ++i) {
#pragma unroll
    for (int j = 0; j < X::N; ++j) {
      result(i, j) = op(x(i, j), y(i, j));
    }
  }

  return result;
}

template <typename X, typename Y>
__both__ constexpr auto vmin(const X& x, const Y& y)
    -> Mat<typename X::Scalar, X::M, X::N> {
  return binary(x, y, [] __both__(float a, float b) { return a > b ? b : a; });
}

template <typename X, typename Y>
__both__ constexpr auto vmax(const X& x, const Y& y)
    -> Mat<typename X::Scalar, X::M, X::N> {
  return binary(x, y, [] __both__(float a, float b) { return a > b ? a : b; });
}

template <typename X, typename Y>
__both__ constexpr auto vadd(const X& x, const Y& y)
    -> Mat<typename X::Scalar, X::M, X::N> {
  return binary(x, y, [] __both__(float a, float b) { return a + b; });
}

template <typename X, typename Y>
__both__ constexpr auto vsub(const X& x, const Y& y)
    -> Mat<typename X::Scalar, X::M, X::N> {
  return binary(x, y, [] __both__(float a, float b) { return a - b; });
}

template <typename X, typename Y>
__both__ constexpr auto vmul(const X& x, const Y& y)
    -> Mat<typename X::Scalar, X::M, X::N> {
  return binary(x, y, [] __both__(float a, float b) { return a * b; });
}

template <typename X, typename Y>
__both__ constexpr auto vdiv(const X& x, const Y& y)
    -> Mat<typename X::Scalar, X::M, X::N> {
  return binary(x, y, [] __both__(float a, float b) { return a / b; });
}

template <typename X, typename Y, typename Pred>
__both__ constexpr bool any(const X& x, const Y& y, Pred pred) {
  static_assert(ctd::is_same_v<typename X::Scalar, typename Y::Scalar>,
                "Matric type mismatch.");
  static_assert(X::M == Y::M && X::N == Y::N, "Matrix dim mismatch.");

  bool result = false;
#pragma unroll
  for (int i = 0; i < X::M; ++i) {
#pragma unroll
    for (int j = 0; j < X::N; ++j) {
      result |= pred(x(i, j), y(i, j));
    }
  }
  return result;
}

template <typename X, typename Y>
__both__ constexpr bool any_gt(const X& x, const Y& y) {
  return any(x, y, [] __both__(float a, float b) { return a > b; });
}

template <typename X, typename Y>
__both__ constexpr bool any_geq(const X& x, const Y& y) {
  return any(x, y, [] __both__(float a, float b) { return a >= b; });
}

template <typename X, typename Y>
__both__ constexpr bool any_lt(const X& x, const Y& y) {
  return any(x, y, [] __both__(float a, float b) { return a < b; });
}

template <typename X, typename Y>
__both__ constexpr bool any_leq(const X& x, const Y& y) {
  return any(x, y, [] __both__(float a, float b) { return a <= b; });
}

template <typename X, typename Y, typename Pred>
__both__ constexpr bool all(const X& x, const Y& y, Pred pred) {
  static_assert(ctd::is_same_v<typename X::Scalar, typename Y::Scalar>,
                "Matric type mismatch.");
  static_assert(X::M == Y::M && X::N == Y::N, "Matrix dim mismatch.");

  bool result = true;
#pragma unroll
  for (int i = 0; i < X::M; ++i) {
#pragma unroll
    for (int j = 0; j < X::N; ++j) {
      result &= pred(x(i, j), y(i, j));
    }
  }
  return result;
}

template <typename X, typename Y>
__both__ constexpr bool all_gt(const X& x, const Y& y) {
  return all(x, y, [] __both__(float a, float b) { return a > b; });
}

template <typename X, typename Y>
__both__ constexpr bool all_geq(const X& x, const Y& y) {
  return all(x, y, [] __both__(float a, float b) { return a >= b; });
}

template <typename X, typename Y>
__both__ constexpr bool all_lt(const X& x, const Y& y) {
  return all(x, y, [] __both__(float a, float b) { return a < b; });
}

template <typename X, typename Y>
__both__ constexpr bool all_leq(const X& x, const Y& y) {
  return all(x, y, [] __both__(float a, float b) { return a <= b; });
}

__both__ constexpr Vec3f cross(Vec3fV x, Vec3fV y) {
  Vec3f res;
  res(0) = x(1) * y(2) - x(2) * y(1);
  res(1) = x(2) * y(0) - x(0) * y(2);
  res(2) = x(0) * y(1) - x(1) * y(0);
  return res;
}

}  // namespace silk::cuda

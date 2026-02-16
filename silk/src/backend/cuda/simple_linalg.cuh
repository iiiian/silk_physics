#pragma once

#include <cassert>
#include <cuda/std/array>
#include <cuda/std/mdspan>
#include <cuda/std/type_traits>
#include <cuda/std/utility>

#include "backend/cuda/cuda_utils.hpp"

namespace silk::cuda {

template <typename T, int m, int n>
class MatViewImpl {
 public:
  static_assert(m > 0 && n > 0, "Invalid row size or col size.");

  static constexpr int M = m;
  static constexpr int N = n;

  using ConstT = ::cuda::std::add_const_t<T>;
  using Extent = ::cuda::std::extents<int, m, n>;
  using Mapping = ::cuda::std::layout_stride::mapping<Extent>;
  using Mdspan = ::cuda::std::mdspan<T, Extent, ::cuda::std::layout_stride>;

  Mdspan mdspan;

  __both__ constexpr MatViewImpl() = delete;

  __both__ constexpr MatViewImpl(Mdspan mdspan) : mdspan(mdspan){};

  __both__ constexpr MatViewImpl(T* ptr, int row_stride, int col_stride)
      : mdspan(ptr, Mapping{Extent{}, ::cuda::std::array<int, 2>{row_stride,
                                                                 col_stride}}) {
    assert(ptr);
  };

  __both__ static constexpr MatViewImpl row_major(T* ptr) {
    assert(ptr);
    return MatViewImpl{ptr, n, 1};
  }

  __both__ static constexpr MatViewImpl col_major(T* ptr) {
    assert(ptr);
    return MatViewImpl{ptr, 1, m};
  }

  __both__ constexpr MatViewImpl<ConstT, m, n> const_view() const {
    if constexpr (::cuda::std::is_same_v<T, ConstT>) {
      return *this;
    } else {
      return MatViewImpl<ConstT, m, n>(mdspan.data_handle(), mdspan.stride(0),
                                       mdspan.stride(1));
    }
  }

  __both__ constexpr operator MatViewImpl<ConstT, m, n>() const {
    return const_view();
  }

  __both__ constexpr T& operator()(int i, int j) const {
    assert(i >= 0 && i < m && "Invalid row index");
    assert(j >= 0 && j < n && "Invalid col index");
    return mdspan(i, j);
  }

  template <int row_num, int col_num>
  __both__ constexpr MatViewImpl<T, row_num, col_num> block(
      int row_start, int col_start) const {
    static_assert(row_num > 0 && col_num > 0, "Invalid row_num or col_num.");
    assert(row_start >= 0 && col_start >= 0 &&
           "Invalid row_start or col_start");
    assert((row_start + row_num) <= m && "Row index out of range.");
    assert((col_start + col_num) <= n && "Col index out of range.");

    auto row_range = ::cuda::std::make_pair(row_start, row_start + row_num);
    auto col_range = ::cuda::std::make_pair(col_start, col_start + col_num);
    auto new_mdspan = ::cuda::std::submdspan(mdspan, row_range, col_range);

    return MatViewImpl<T, row_num, col_num>{new_mdspan};
  }

  __both__ constexpr MatViewImpl<T, 1, n> row(int row) const {
    assert(row >= 0 && row < m && "Row index out of range.");
    return block<1, n>(row, 0);
  }

  __both__ constexpr MatViewImpl<T, m, 1> col(int col) const {
    assert(col >= 0 && col < n && "Col index out of range.");
    return block<m, 1>(0, col);
  }

  __both__ constexpr MatViewImpl<T, n, m> transpose() const {
    // Swap row and col stride.
    return MatViewImpl<T, n, m>{mdspan.data_handle(), mdspan.stride(1),
                                mdspan.stride(0)};
  }
};

template <int m, int n>
using MatView = MatViewImpl<float, m, n>;

template <int m, int n>
using ConstMatView = MatViewImpl<const float, m, n>;

/// @brief Row major dense matrix.
template <int m, int n>
class Mat {
 public:
  static_assert(m > 0 && n > 0, "Invalid rows and cols.");

  static constexpr int M = m;
  static constexpr int N = n;

  ::cuda::std::array<float, m * n> data;

  __both__ static constexpr Mat zeros() { return Mat{}; }

  __both__ static constexpr Mat ones() {
    Mat<m, n> result;

#pragma unroll
    for (int i = 0; i < m; ++i) {
#pragma unroll
      for (int j = 0; j < n; ++j) {
        result(i, j) = 1.0f;
      }
    }
    return result;
  }

  __both__ static constexpr Mat identity() {
    static_assert(m == n, "Identity ctor requires square matrix.");
    Mat result = zeros();

#pragma unroll
    for (int i = 0; i < n; ++i) {
      result(i, i) = 1.0f;
    }

    return result;
  }

  __both__ constexpr MatView<m, n> view() {
    return MatView<m, n>::row_major(data.data());
  }

  __both__ constexpr operator MatView<m, n>() { return view(); }

  __both__ constexpr ConstMatView<m, n> const_view() const {
    return ConstMatView<m, n>::row_major(data.data());
  }

  __both__ constexpr operator ConstMatView<m, n>() const {
    return const_view();
  }

  __both__ constexpr float& operator()(int i, int j) { return view()(i, j); }

  __both__ constexpr const float& operator()(int i, int j) const {
    return const_view()(i, j);
  }
};

using Vec2 = Mat<2, 1>;
using Vec3 = Mat<3, 1>;
using Vec4 = Mat<4, 1>;
using Mat22 = Mat<2, 2>;
using Mat23 = Mat<2, 3>;
using Mat32 = Mat<3, 2>;
using Mat33 = Mat<3, 3>;
using Mat24 = Mat<2, 4>;
using Mat34 = Mat<3, 4>;
using Mat42 = Mat<4, 2>;
using Mat43 = Mat<4, 3>;
using Mat44 = Mat<4, 4>;

template <typename Dst, typename Src>
__both__ constexpr void assign(Dst& dst, const Src& src) {
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
__both__ constexpr auto axpby(float a, const X& x, float b, const Y& y)
    -> Mat<X::M, X::N> {
  static_assert(X::M == Y::M && X::N == Y::N, "Matrix dim mismatch.");

  Mat<X::M, X::N> result;
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
__both__ constexpr auto axpb(float a, const X& x, float b) -> Mat<X::M, X::N> {
  Mat<X::M, X::N> result;
#pragma unroll
  for (int i = 0; i < X::M; ++i) {
#pragma unroll
    for (int j = 0; j < X::N; ++j) {
      result(i, j) = a * x(i, j) + b;
    }
  }
  return result;
}

template <typename X, typename Y>
__both__ constexpr float dot(const X& x, const Y& y) {
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

template <typename X, typename Y>
__both__ constexpr auto mat_mul(const X& x, const Y& y) -> Mat<X::M, Y::N> {
  static_assert(X::N == Y::M, "Matrix dim mismatch.");

  auto result = Mat<X::M, Y::N>::zeros();
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
    -> Mat<X::M, X::N> {
  static_assert(X::M == Y::M && X::N == Y::N, "Matrix dim mismatch.");

  Mat<X::M, X::N> result;
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
__both__ constexpr auto vmin(const X& x, const Y& y) -> Mat<X::M, X::N> {
  return binary(x, y, [] __both__(float a, float b) { return a > b ? b : a; });
}

template <typename X, typename Y>
__both__ constexpr auto vmax(const X& x, const Y& y) -> Mat<X::M, X::N> {
  return binary(x, y, [] __both__(float a, float b) { return a > b ? a : b; });
}

template <typename X, typename Y, typename Pred>
__both__ constexpr bool any(const X& x, const Y& y, Pred pred) {
  static_assert(X::M == Y::M && X::N == Y::N, "Matrix dim mismatch.");

#pragma unroll
  for (int i = 0; i < X::M; ++i) {
#pragma unroll
    for (int j = 0; j < X::N; ++j) {
      if (pred(x(i, j), y(i, j))) {
        return true;
      }
    }
  }
  return false;
}

template <typename X, typename Y>
__both__ constexpr bool any_gt(const X& x, const Y& y) {
  return any(x, y, [] __both__(float a, float b) { return a > b; });
}

template <typename X, typename Y>
__both__ constexpr bool any_lt(const X& x, const Y& y) {
  return any(x, y, [] __both__(float a, float b) { return a < b; });
}

template <typename X, typename Y, typename Pred>
__both__ constexpr bool all(const X& x, const Y& y, Pred pred) {
  static_assert(X::M == Y::M && X::N == Y::N, "Matrix dim mismatch.");

#pragma unroll
  for (int i = 0; i < X::M; ++i) {
#pragma unroll
    for (int j = 0; j < X::N; ++j) {
      if (!pred(x(i, j), y(i, j))) {
        return false;
      }
    }
  }
  return true;
}

template <typename X, typename Y>
__both__ constexpr bool all_gt(const X& x, const Y& y) {
  return all(x, y, [] __both__(float a, float b) { return a > b; });
}

template <typename X, typename Y>
__both__ constexpr bool all_lt(const X& x, const Y& y) {
  return all(x, y, [] __both__(float a, float b) { return a < b; });
}

}  // namespace silk::cuda

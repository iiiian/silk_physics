#pragma once

#include <random>

#include "collision_helper.hpp"

namespace silk {

constexpr int SAP_APPROX_AXIS_SAMPLE_NUM = 16;
constexpr int SAP_APPROX_AXIS_SAMPLE_THRESHOLD = 32;

template <typename T>
int sap_optimal_axis(BboxCollider<T>** proxies, int proxy_num) {
  assert((proxy_num > 0));

  auto [mean, variance] = find_proxy_mean_variance(proxies, proxy_num);
  int axis;
  variance.maxCoeff(&axis);
  return axis;
}

template <typename T>
int sap_optimal_axis(BboxCollider<T>** proxies_a, int proxy_num_a,
                     BboxCollider<T>** proxies_b, int proxy_num_b) {
  assert((proxy_num_a > 0));
  assert((proxy_num_b > 0));

  auto [ma, va] = find_proxy_mean_variance(proxies_a, proxy_num_a);
  auto [mb, vb] = find_proxy_mean_variance(proxies_b, proxy_num_b);
  Eigen::Vector3f mean =
      (proxy_num_a * ma + proxy_num_b * mb) / (proxy_num_a + proxy_num_b);
  Eigen::Vector3f variance = proxy_num_a * (va + (ma - mean).square()) +
                             proxy_num_b * (vb + (mb - mean).square());
  int axis;
  variance.maxCoeff(&axis);
  return axis;
}

template <typename T>
int sap_approx_axis(BboxCollider<T>** proxies, int proxy_num) {
  assert((proxy_num > 0));

  static std::random_device rand_device;
  static std::mt19937 rand_generator(rand_device());

  if (proxy_num <= SAP_APPROX_AXIS_SAMPLE_THRESHOLD) {
    return sap_optimal_axis(proxies, proxy_num);
  }

  for (size_t i = 0; i < SAP_APPROX_AXIS_SAMPLE_NUM; ++i) {
    std::uniform_int_distribution<size_t> rand_dist(i, proxy_num - 1);
    size_t rand_idx = rand_dist(rand_generator);
    std::swap(proxies[i], proxies[rand_idx]);
  }

  return sap_optimal_axis(proxies, SAP_APPROX_AXIS_SAMPLE_NUM);
}

template <typename T>
int sap_approx_axis(BboxCollider<T>** proxies_a, int proxy_num_a,
                    BboxCollider<T>** proxies_b, int proxy_num_b) {
  assert((proxy_num_a > 0));
  assert((proxy_num_b > 0));

  static std::random_device rand_device;
  static std::mt19937 rand_generator(rand_device());

  int num = proxy_num_a + proxy_num_b;
  if (num <= SAP_APPROX_AXIS_SAMPLE_THRESHOLD) {
    return sap_optimal_axis(proxies_a, proxy_num_a, proxies_b, proxy_num_b);
  }

  int sample_num_a = proxy_num_a * SAP_APPROX_AXIS_SAMPLE_NUM / num + 1;
  int sample_num_b = proxy_num_b * SAP_APPROX_AXIS_SAMPLE_NUM / num + 1;
  for (int i = 0; i < sample_num_a; ++i) {
    std::uniform_int_distribution<int> rand_dist(i, proxy_num_a - 1);
    int rand_idx = rand_dist(rand_generator);
    std::swap(proxies_a[i], proxies_a[rand_idx]);
  }
  for (int i = 0; i < sample_num_b; ++i) {
    std::uniform_int_distribution<int> rand_dist(i, proxy_num_b - 1);
    int rand_idx = rand_dist(rand_generator);
    std::swap(proxies_b[i], proxies_b[rand_idx]);
  }

  return sap_optimal_axis(proxies_a, sample_num_a, proxies_b, sample_num_b);
}

template <typename T>
void sap_sort_proxies(BboxCollider<T>** proxies, int proxy_num, int axis) {
  assert((proxy_num != 0));

  auto comp = [axis](BboxCollider<T>* a, BboxCollider<T>* b) -> bool {
    return (a->bbox.min(axis) < b->bbox.min(axis));
  };
  std::sort(proxies, proxies + proxy_num, comp);
}

template <typename T>
void sap_sorted_collision(BboxCollider<T>* p1, BboxCollider<T>** proxies,
                          int proxy_num, int axis,
                          CollisionFilterCallback<T> filter_callback,
                          CollisionCache<T>& cache) {
  assert((proxy_num != 0));

  if (p1->bbox.max(axis) < proxies[0]->bbox.min(axis)) {
    return;
  }

  // optimize for avx SIMD.
  // each col is the min / max of a KDObject.
  // test 8 objects in a row.
  using Matrix38f = Eigen::Matrix<float, 3, 8>;
  Matrix38f simd_p1_min = p1->bbox.min.replicate(1, 8);
  Matrix38f simd_p1_max = p1->bbox.max.replicate(1, 8);
  Matrix38f simd_p2_min;
  Matrix38f simd_p2_max;
  Eigen::Matrix<BboxCollider<T>*, 8, 1> simd_proxies;

  int simd_count = 0;
  for (int i = 0; i < proxy_num; ++i) {
    BboxCollider<T>* p2 = proxies[i];
    // axis test
    if (p1->bbox.max(axis) < p2->bbox.min(axis)) {
      break;
    }
    // exclude staic-static pair
    if (p1->is_static && p2->is_static) {
      continue;
    }
    // user provided collision filter
    if (!filter_callback_(p1->data, p2->data)) {
      continue;
    }

    // potential collision candidate
    simd_p2_min.col(simd_count) = p2->bbox.min;
    simd_p2_max.col(simd_count) = p2->bbox.max;
    simd_proxies(simd_count) = p2;
    ++simd_count;

    // simd bbox intersection test
    if (simd_count == 8) {
      Matrix38f max_min = simd_p1_min.cwiseMax(simd_p2_min);
      Matrix38f min_max = simd_p1_max.cwiseMin(simd_p2_max);
      Eigen::Matrix<bool, 8, 1> is_bbox_colliding =
          (max_min.array() < min_max.array()).colwise().all();
      for (int j = 0; j < 8; ++j) {
        if (is_bbox_colliding[j]) {
          cache->emplace_back({p1->data, simd_proxies[j]->data});
        }
      }
      simd_count = 0;
    }
  }

  // test the last chunk
  if (simd_count != 0) {
    Matrix38f max_min = simd_p1_min.cwiseMin(simd_p2_min);
    Matrix38f min_max = simd_p1_max.cwiseMin(simd_p2_max);
    Eigen::Matrix<float, 8, 1> is_bbox_colliding =
        (max_min.array() < min_max.array()).colwise().all();
    for (int j = 0; j < simd_count; ++j) {
      if (is_bbox_colliding[j]) {
        cache->emplace_back({p1->data, simd_proxies[j]->data});
      }
    }
  }
}

template <typename T>
void sap_sorted_group_self_collision(BboxCollider<T>** proxies, int proxy_num,
                                     int axis,
                                     CollisionFilterCallback<T> filter_callback,
                                     CollisionCache<T>& cache) {
  assert((proxy_num > 0));

  for (int i = 0; i < proxy_num - 1; ++i) {
    sap_test_collision(proxies[i], proxies + i + 1, proxy_num - i, axis,
                       filter_callback, cache);
  }
}

template <typename T>
void sap_sorted_group_group_collision(
    BboxCollider<T>** proxies_a, int proxy_num_a, BboxCollider<T>** proxies_b,
    int proxy_num_b, int axis, CollisionFilterCallback<T> filter_callback,
    CollisionCache<T>& cache) {
  assert((proxy_num_a > 0));
  assert((proxy_num_b > 0));

  int a = 0;
  int b = 0;
  while (a < proxy_num_a && b < proxy_num_b) {
    BboxCollider<T>* pa = proxies_a[a];
    BboxCollider<T>* pb = proxies_b[b];

    if (pa->bbox.min(axis) < pb->bbox.min(axis)) {
      sap_test_collision(pa, proxies_b + b, proxy_num_b - b, axis,
                         filter_callback, cache);
    } else {
      sap_test_collision(pb, proxies_a + a, proxy_num_a - a, axis,
                         filter_callback, cache);
    }
  }
}

}  // namespace silk

#pragma once

#include <pdqsort.h>

#include <cassert>
#include <random>

#include "collision.hpp"
#include "collision_internal.hpp"

namespace silk {

constexpr int SAP_APPROX_AXIS_SAMPLE_NUM = 16;
constexpr int SAP_APPROX_AXIS_SAMPLE_THRESHOLD = 32;

template <typename T>
int sap_optimal_axis(const Collider<T>** proxies, int proxy_num) {
  assert((proxy_num > 0));

  auto [mean, var] = proxy_mean_variance(proxies, proxy_num);
  int axis;
  var.maxCoeff(&axis);
  return axis;
}

template <typename T>
int sap_optimal_axis(const Collider<T>** proxies_a, int proxy_num_a,
                     const Collider<T>** proxies_b, int proxy_num_b) {
  assert((proxy_num_a > 0));
  assert((proxy_num_b > 0));

  auto [mean_a, var_a] = proxy_mean_variance(proxies_a, proxy_num_a);
  auto [mean_b, var_b] = proxy_mean_variance(proxies_b, proxy_num_b);
  Eigen::Vector3f mean = (proxy_num_a * mean_a + proxy_num_b * mean_b) /
                         (proxy_num_a + proxy_num_b);
  Eigen::Vector3f tmp_a = (mean_a - mean).array().square();
  Eigen::Vector3f tmp_b = (mean_b - mean).array().square();
  Eigen::Vector3f var =
      proxy_num_a * (var_a + tmp_a) + proxy_num_b * (var_b + tmp_b);
  int axis;
  var.maxCoeff(&axis);
  return axis;
}

template <typename T>
int sap_approx_axis(Collider<T>** proxies, int proxy_num) {
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
int sap_approx_axis(Collider<T>** proxies_a, int proxy_num_a,
                    Collider<T>** proxies_b, int proxy_num_b) {
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
void sap_sort_proxies(ColliderProxy<T>* proxies, int proxy_num, int axis) {
  assert((proxy_num != 0));

  auto comp = [axis](ColliderProxy<T> a, ColliderProxy<T> b) -> bool {
    return (a->bbox.min(axis) < b->bbox.min(axis));
  };
  pdqsort_branchless(proxies, proxies + proxy_num, comp);
}

template <typename T>
void sap_sorted_collision(ColliderProxy<T> p1, const ColliderProxy<T>* proxies,
                          int proxy_num, int axis, CollisionFilter<T> filter,
                          CollisionCache<T>& cache) {
  assert((proxy_num != 0));

  if (p1->bbox.max(axis) < proxies[0]->bbox.min(axis)) {
    return;
  }

  for (int i = 0; i < proxy_num; ++i) {
    ColliderProxy<T> p2 = proxies[i];
    // axis test
    if (p1->bbox.max(axis) < p2->bbox.min(axis)) {
      break;
    }
    // user provided collision filter
    if (!filter(p1->data, p2->data)) {
      continue;
    }

    if (Bbox::is_colliding(p1->bbox, p2->bbox)) {
      cache.emplace_back(p1->data, p2->data);
    }
  }
}

template <typename T>
void sap_sorted_group_self_collision(const ColliderProxy<T>* proxies,
                                     int proxy_num, int axis,
                                     CollisionFilter<T> filter,
                                     CollisionCache<T>& cache) {
  assert((proxy_num > 0));

  for (int i = 0; i < proxy_num - 1; ++i) {
    sap_sorted_collision(proxies[i], proxies + i + 1, proxy_num - i - 1, axis,
                         filter, cache);
  }
}

template <typename T>
void sap_sorted_group_group_collision(const ColliderProxy<T>* proxies_a,
                                      int proxy_num_a,
                                      const ColliderProxy<T>* proxies_b,
                                      int proxy_num_b, int axis,
                                      CollisionFilter<T> filter,
                                      CollisionCache<T>& cache) {
  assert((proxy_num_a > 0));
  assert((proxy_num_b > 0));

  int a = 0;
  int b = 0;
  while (a < proxy_num_a && b < proxy_num_b) {
    ColliderProxy<T> pa = proxies_a[a];
    ColliderProxy<T> pb = proxies_b[b];

    if (pa->bbox.min(axis) < pb->bbox.min(axis)) {
      sap_sorted_collision(pa, proxies_b + b, proxy_num_b - b, axis, filter,
                           cache);
      a++;
    } else {
      sap_sorted_collision(pb, proxies_a + a, proxy_num_a - a, axis, filter,
                           cache);
      b++;
    }
  }
}

}  // namespace silk

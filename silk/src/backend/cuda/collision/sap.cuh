#pragma once

#include <algorithm>
#include <cassert>
#include <span>
#include <utility>
#include <vector>

#include "backend/cuda/collision/bbox.cuh"

namespace silk::cuda {

template <typename C>
using SapCollisionCache = std::vector<std::pair<C*, C*>>;

template <typename C>
std::pair<Vec3f, Vec3f> sap_proxy_mean_variance(std::span<const C> colliders,
                                                const int* proxies,
                                                int proxy_num) {
  assert(proxy_num > 0);

  Vec3f mean = Vec3f::zeros();
  Vec3f variance = Vec3f::zeros();
  for (int i = 0; i < proxy_num; ++i) {
    int p = proxies[i];
    Vec3f center = colliders[p].bbox.center();
    mean = vadd(mean, center);
    variance = vadd(variance, vmul(center, center));
  }
  mean = ax(1.0f / proxy_num, mean);
  variance = vsub(ax(1.0f / proxy_num, variance), vmul(mean, mean));

  return std::make_pair(mean, variance);
}

template <typename C>
int sap_optimal_axis(std::span<const C> colliders, const int* proxies,
                     int proxy_num) {
  auto [mean, var] = sap_proxy_mean_variance(colliders, proxies, proxy_num);
  (void)mean;
  if (var(0) > var(1) && var(0) > var(2)) {
    return 0;
  }
  if (var(1) > var(2)) {
    return 1;
  }
  return 2;
}

template <typename C>
void sap_sort_proxies(std::span<const C> colliders, int* proxies, int proxy_num,
                      int axis) {
  assert(proxy_num > 0);

  auto comp = [axis, colliders](int a, int b) {
    return colliders[a].bbox.min(axis) < colliders[b].bbox.min(axis);
  };
  std::sort(proxies, proxies + proxy_num, comp);
}

template <typename C, bool flip = false, typename Filter>
void sap_sorted_collision(C& ca, std::span<C> colliders_b, const int* proxies_b,
                          int proxy_num_b, int axis, const Filter& filter,
                          SapCollisionCache<C>& cache) {
  assert(proxy_num_b > 0);

  for (int i = 0; i < proxy_num_b; ++i) {
    int p2 = proxies_b[i];
    C& cb = colliders_b[p2];

    if (ca.bbox.max(axis) < cb.bbox.min(axis)) {
      break;
    }
    if (!filter(ca, cb)) {
      continue;
    }
    if (Bbox::is_colliding(ca.bbox, cb.bbox)) {
      if constexpr (flip) {
        cache.emplace_back(&cb, &ca);
      } else {
        cache.emplace_back(&ca, &cb);
      }
    }
  }
}

template <typename C, typename FilterT>
void sap_sorted_group_self_collision(std::span<C> colliders, const int* proxies,
                                     int proxy_num, int axis,
                                     const FilterT& filter,
                                     SapCollisionCache<C>& cache) {
  assert(proxy_num > 0);

  for (int i = 0; i < proxy_num - 1; ++i) {
    int p = proxies[i];
    sap_sorted_collision(colliders[p], colliders, proxies + i + 1,
                         proxy_num - i - 1, axis, filter, cache);
  }
}

}  // namespace silk::cuda

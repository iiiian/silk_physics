#pragma once

#include "collision_helper.hpp"

namespace silk {

template <typename T>
class SAP {
 public:
  using Collider = BboxCollider<T>;

  void group_self_collision(Collider** proxies, int proxy_num,
                            CollisionFilterCallback<T> filter_callback,
                            CollisionCache<T>* cache) {
    assert((proxy_num != 0));
    assert(cache);

    filter_callback_ = filter_callback;
    cache_ = cache;

    find_optimal_axis(proxies, proxy_num);
    sort_collider(proxies, proxy_num);
    for (int i = 0; i < proxy_num - 1; ++i) {
      test_collision(proxies[i], proxies + i + 1, proxy_num - i);
    }
  }

  void group_group_collision(Collider** proxies_a, int proxy_num_a,
                             Collider** proxies_b, int proxy_num_b,
                             bool use_axis_a,
                             CollisionFilterCallback<T> filter_callback,
                             CollisionCache<T>* cache) {
    assert((proxy_num_a != 0));
    assert((proxy_num_b != 0));
    assert(cache);

    filter_callback_ = filter_callback;
    cache_ = cache;

    if (use_axis_a) {
      find_optimal_axis(proxies_a, proxy_num_a);
    } else {
      find_optimal_axis(proxies_b, proxy_num_b);
    }

    sort_collider(proxies_a, proxy_num_a);
    sort_collider(proxies_b, proxy_num_b);

    int a = 0;
    int b = 0;
    while (a < proxy_num_a && b < proxy_num_b) {
      Collider* pa = proxies_a[a];
      Collider* pb = proxies_b[b];

      if (pa->bbox.min(axis_) < pb->bbox.min(axis_)) {
        test_collision(pa, proxies_b + b, proxy_num_b - b);
      } else {
        test_collision(pb, proxies_a + a, proxy_num_a - a);
      }
    }
  }

 private:
  int axis_;
  CollisionFilterCallback<T> filter_callback_;
  CollisionCache<T>* cache_;

  void find_optimal_axis(Collider** proxies, int proxy_num) {
    assert((proxy_num != 0));

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    Eigen::Vector3f variance = Eigen::Vector3f::Zero();
    for (int i = 0; i < proxy_num; ++i) {
      Eigen::Vector3f center = proxies[i]->bbox.center();
      mean += center;
      variance += center.cwiseAbs2();
    }
    mean /= proxy_num;
    variance = variance / proxy_num - mean.cwiseAbs2();
    variance.maxCoeff(&this->axis_);
  }

  void sort_collider(Collider** proxies, int proxy_num) const {
    assert((proxy_num != 0));

    auto comp = [axis = this->axis_](Collider* a, Collider* b) -> bool {
      return (a->bbox.min(axis) < b->bbox.min(axis));
    };
    std::sort(proxies, proxies + proxy_num, comp);
  }

  void test_collision(Collider* p1, Collider** proxies, int proxy_num) {
    if (p1->bbox.max(axis_) < proxies[0]->bbox.min(axis_)) {
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
    Eigen::Matrix<Collider*, 8, 1> simd_colliders;

    int simd_count = 0;
    for (int i = 0; i < proxy_num; ++i) {
      Collider* p2 = proxies[i];
      // axis test
      if (p1->bbox.max(axis_) > p2->bbox.min(axis_)) {
        continue;
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
      simd_colliders(simd_count) = p2;
      simd_count++;

      // simd bbox intersection test
      if (simd_count == 8) {
        Matrix38f max_min = simd_p1_min.cwiseMin(simd_p2_min);
        Matrix38f min_max = simd_p1_max.cwiseMin(simd_p2_max);
        Eigen::Matrix<bool, 8, 1> is_bbox_colliding =
            (max_min.array() < min_max.array()).colwise().all();
        for (int j = 0; j < 8; ++j) {
          if (is_bbox_colliding[j]) {
            cache_->emplace_back({p1->data, simd_colliders[j]->data});
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
          cache_->emplace_back({&p1->data, &simd_colliders[j]->data});
        }
      }
    }
  }
};

}  // namespace silk

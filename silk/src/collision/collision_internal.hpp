#pragma once

#include <functional>

#include "collision.hpp"

namespace silk {

template <typename T>
using CollisionCache = std::vector<std::pair<T*, T*>>;

template <typename T>
using CollisionFilter = std::function<bool(const T&, const T&)>;

struct MeanVariance {
  Eigen::Vector3f mean;
  Eigen::Vector3f variance;
};

template <typename T>
MeanVariance proxy_mean_variance(const Collider<T>** proxies, int proxy_num) {
  assert((proxy_num > 0));

  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  Eigen::Vector3f variance = Eigen::Vector3f::Zero();
  for (int i = 0; i < proxy_num; ++i) {
    Eigen::Vector3f center = proxies[i]->bbox.center();
    mean += center;
    variance += center.cwiseAbs2();
  }
  mean /= proxy_num;
  variance = variance / proxy_num - mean.cwiseAbs2();

  return {std::move(mean), std::move(variance)};
}

}  // namespace silk

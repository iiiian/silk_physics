#pragma once

#include <Eigen/Core>
#include <functional>

namespace silk {

template <typename C>
using CollisionCache = std::vector<std::pair<C*, C*>>;

template <typename C>
using CollisionFilter = std::function<bool(const C&, const C&)>;

struct MeanVariance {
  Eigen::Vector3f mean;
  Eigen::Vector3f variance;
};

template <typename C>
MeanVariance proxy_mean_variance(C* const* proxies, int proxy_num) {
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

#include "cholmod_utils.hpp"

namespace silk::cholmod_raii {

CholmodCommon::CholmodCommon() { cholmod_start(&common_); }

CholmodCommon::~CholmodCommon() { cholmod_finish(&common_); }

CholmodCommon::operator cholmod_common*() { return &common_; }

cholmod_common* CholmodCommon::raw() { return &common_; }

CholmodCommon common;

Eigen::Map<Eigen::VectorXf> make_eigen_dense_vector_view(cholmod_dense* v) {
  assert((v->xtype == CHOLMOD_REAL));
  assert((v->dtype == CHOLMOD_SINGLE));
  assert((v->ncol == 1));
  assert((v->x != nullptr));

  float* x = static_cast<float*>(v->x);
  return Eigen::Map<Eigen::VectorXf>(x, static_cast<Eigen::Index>(v->nrow));
}

}  // namespace silk::cholmod_raii

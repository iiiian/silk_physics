#include "backend/cuda/collision/cubql_bvh.cuh"

namespace silk::cuda::detail {

void build_cubql_bvh(cuBQL::bvh3f& bvh, const cuBQL::box3f* boxes, int box_num,
                     cudaStream_t stream) {
  cuBQL::BuildConfig config;
  cuBQL::cuda::radixBuilder(bvh, boxes, box_num, config, stream);
  assert(bvh.numPrims == box_num);
}

void free_cubql_bvh(cuBQL::bvh3f& bvh, cudaStream_t stream) {
  cuBQL::cuda::free(bvh, stream);
  bvh = {};
}

}  // namespace silk::cuda::detail

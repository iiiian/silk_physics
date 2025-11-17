#pragma once

#include <cuda_runtime.h>

namespace silk::cuda {

__device__ void svd33(
    // input A
    float a11, float a21, float a31, float a12, float a22, float a32, float a13,
    float a23, float a33,
    // output U
    float &u11, float &u21, float &u31, float &u12, float &u22, float &u32,
    float &u13, float &u23, float &u33,
    // output S
    float &s11, float &s22, float &s33,
    // output V
    float &v11, float &v21, float &v31, float &v12, float &v22, float &v32,
    float &v13, float &v23, float &v33);

__device__ void svd32(
    // input A
    float a11, float a21, float a12, float a22, float a13, float a23,
    // output U
    float &u11, float &u21, float &u12, float &u22, float &u13, float &u23,
    // output S
    float &s11, float &s22,
    // output V
    float &v11, float &v21, float &v12, float &v22);

}  // namespace silk::cuda

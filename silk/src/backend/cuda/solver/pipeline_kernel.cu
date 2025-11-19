#include <cuda_runtime.h>

#include <cassert>
#include <cub/cub.cuh>

#include "backend/cuda/cuda_utils.hpp"
#include "backend/cuda/solver/barrier_constrain.hpp"
#include "backend/cuda/solver/pipeline_kernel.hpp"

namespace silk::cuda {

__global__ void predict_kernel(int state_num, float dt, float acc_x,
                               float acc_y, float acc_z,
                               const float* d_curr_state,
                               const float* d_state_velocity,
                               float* d_next_state) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (3 * tid < state_num) {
    int o1 = 3 * tid;
    int o2 = 3 * tid + 1;
    int o3 = 3 * tid + 2;
    d_next_state[o1] = d_curr_state[o1] + dt * d_state_velocity[o1];
    d_next_state[o2] = d_curr_state[o2] + dt * d_state_velocity[o2];
    d_next_state[o3] = d_curr_state[o3] + dt * d_state_velocity[o3];

    // d_next_state[o1] =
    //     d_curr_state[o1] + dt * d_state_velocity[o1] + dt * dt * acc_x;
    // d_next_state[o2] =
    //     d_curr_state[o2] + dt * d_state_velocity[o2] + dt * dt * acc_y;
    // d_next_state[o3] =
    //     d_curr_state[o3] + dt * d_state_velocity[o3] + dt * dt * acc_z;
  }
}

void predict(int state_num, float dt, float acc_x, float acc_y, float acc_z,
             const float* d_curr_state, const float* d_state_velocity,
             float* d_next_state) {
  assert(state_num != 0);
  assert(d_curr_state && d_next_state && d_state_velocity);

  int block_size;
  int min_grid_size;
  cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                     predict_kernel, 0, 0);
  int grid_size = (state_num / 3 + block_size - 1) / block_size;

  predict_kernel<<<grid_size, block_size>>>(state_num, dt, acc_x, acc_y, acc_z,
                                            d_curr_state, d_state_velocity,
                                            d_next_state);
}

__global__ void diff_squared_kernel(int num, const float* d_a, const float* d_b,
                                    float* d_out) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num) {
    float temp = d_a[tid] - d_b[tid];
    d_out[tid] = temp * temp;
  }
}

float compute_L2_distance(int num, const float* d_a, const float* d_b,
                          float* d_buffer) {
  assert(num != 0);
  assert(d_a && d_b && d_buffer);

  int block_size;
  int min_grid_size;
  cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                     diff_squared_kernel, 0, 0);
  int grid_size = (num + block_size - 1) / block_size;
  diff_squared_kernel<<<grid_size, block_size>>>(num, d_a, d_b, d_buffer);
  cudaDeviceSynchronize();

  float* d_sum = nullptr;
  CHECK_CUDA(cudaMalloc((void**)&d_sum, sizeof(float)));
  // CUB DeviceReduce requires a two-phase API with temp storage in this CUDA
  // version.
  void* d_temp = nullptr;
  size_t temp_bytes = 0;
  cub::DeviceReduce::Sum(nullptr, temp_bytes, d_buffer, d_sum, num);
  CHECK_CUDA(cudaMalloc(&d_temp, temp_bytes));
  cub::DeviceReduce::Sum(d_temp, temp_bytes, d_buffer, d_sum, num);
  cudaDeviceSynchronize();
  CHECK_CUDA(cudaGetLastError());

  float h_sum = 0.0f;
  CHECK_CUDA(cudaMemcpy(&h_sum, d_sum, sizeof(float), cudaMemcpyDeviceToHost));
  CHECK_CUDA(cudaFree(d_temp));
  CHECK_CUDA(cudaFree(d_sum));

  return sqrt(h_sum);
}

__global__ void enforce_barrier_constrain_kernel(int constrain_num,
                                                 const int* d_index,
                                                 const float* d_lhs,
                                                 const float* d_rhs,
                                                 float* state) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < constrain_num) {
    int idx = d_index[tid];
    assert(d_lhs[idx] != 0.0f);
    state[idx] = d_rhs[idx] / d_lhs[idx];
  }
}

void enforce_barrier_constrain(const BarrierConstrain& barrer, float* state) {
  if (barrer.constrain_num == 0) {
    return;
  }

  assert(state != 0);
  assert(state && barrer.d_index && barrer.d_lhs && barrer.d_rhs);

  int block_size;
  int min_grid_size;
  cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                     enforce_barrier_constrain_kernel, 0, 0);
  int grid_size = (barrer.state_num + block_size - 1) / block_size;
  // Launch by number of active constraints, not full state size
  grid_size = (barrer.constrain_num + block_size - 1) / block_size;

  enforce_barrier_constrain_kernel<<<grid_size, block_size>>>(
      barrer.constrain_num, barrer.d_index, barrer.d_lhs, barrer.d_rhs, state);
}

__global__ void vec_mix_kernel(int num, float nr, const float* d_a,
                               const float* d_b, float* d_out) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num) {
    d_out[tid] = nr * d_a[tid] + (1.0f - nr) * d_b[tid];
  }
}

void vec_mix(int num, float nr, const float* d_a, const float* d_b,
             float* d_out) {
  assert(nr >= 0.0f && nr <= 1.0f);
  assert(d_a && d_b && d_out);

  int block_size;
  int min_grid_size;
  cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                     vec_mix_kernel, 0, 0);
  int grid_size = (num + block_size - 1) / block_size;

  vec_mix_kernel<<<grid_size, block_size>>>(num, nr, d_a, d_b, d_out);
}

__global__ void update_velocity_kernel(int state_num, float dt,
                                       const float* d_curr_state,
                                       const float* d_next_state,
                                       float* d_state_velocity) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < state_num) {
    d_state_velocity[tid] = (d_next_state[tid] - d_curr_state[tid]) / dt;
  }
}

void update_velocity(int state_num, float dt, const float* d_curr_state,
                     const float* d_next_state, float* d_state_velocity) {
  int block_size;
  int min_grid_size;
  cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                     update_velocity_kernel, 0, 0);
  int grid_size = (state_num + block_size - 1) / block_size;

  update_velocity_kernel<<<grid_size, block_size>>>(
      state_num, dt, d_curr_state, d_next_state, d_state_velocity);
}

__global__ void gather_and_damp_velocity_kernel(float damp_factor,
                                                int state_num, const float* src,
                                                float* dst) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < state_num) {
    dst[tid] = damp_factor * src[tid];
  }
}

void gather_and_damp_velocity(float damp_factor, int state_num,
                              const float* src, float* dst) {
  int block_size;
  int min_grid_size;
  cudaOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                     gather_and_damp_velocity_kernel, 0, 0);
  int grid_size = (state_num + block_size - 1) / block_size;
  gather_and_damp_velocity_kernel<<<grid_size, block_size>>>(
      damp_factor, state_num, src, dst);
}

}  // namespace silk::cuda

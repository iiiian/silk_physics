#pragma once

namespace silk::cuda {

class BarrierConstrain {
 public:
  int constrain_num = 0;
  int state_num = 0;
  int* d_index = nullptr;
  float* d_lhs = nullptr;
  float* d_rhs = nullptr;

 public:
  BarrierConstrain() = default;
  BarrierConstrain(int state_num);
  BarrierConstrain(const BarrierConstrain& other) = delete;
  BarrierConstrain(BarrierConstrain&& other) noexcept;
  BarrierConstrain& operator=(const BarrierConstrain& other) = delete;
  BarrierConstrain& operator=(BarrierConstrain&& other) noexcept;
  ~BarrierConstrain();

 private:
  void swap(BarrierConstrain& other) noexcept;
};

}  // namespace silk::cuda

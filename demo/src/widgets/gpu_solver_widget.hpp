#pragma once

#include "../gui_utils.hpp"

//**************************************************************/
//**             Toggle for GPU Solver                         */
//**************************************************************/

enum class SolverBackend { Auto, CPU, GPU };

class GpuSolverWidget : public IWidget {
 private:
  Context& ctx_;
  SolverBackend backend_ = SolverBackend::CPU;

 public:
  explicit GpuSolverWidget(Context& ctx);

  void draw() override;
};

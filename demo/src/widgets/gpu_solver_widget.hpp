#pragma once

#include "../gui_utils.hpp"
#include <silk/silk.hpp>

//**************************************************************/
//**             Toggle for GPU Solver                         */
//**************************************************************/

class GpuSolverWidget : public IWidget {
 private:
  Context& ctx_;
  silk::SolverBackend backend_ = silk::SolverBackend::CPU;

 public:
  explicit GpuSolverWidget(Context& ctx);

  void draw() override;
};

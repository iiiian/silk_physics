#pragma once
#include <polyscope/point_cloud.h>
#include <Eigen/Core>
#include <glm/glm.hpp>
#include "../gui_utils.hpp"
#include "ui_console.hpp"

//**************************************************************/
//**             Toggle for GPU Solver                         */
//**************************************************************/

enum class SolverBackend { Auto, CPU, GPU };

class GpuSolverWidget : public IWidget {
 private:
  Context& ctx_;
  SolverBackend backend_ = SolverBackend::CPU;  // Default: CPU
  //std::function<void(SolverBackend,SolverBackend)> on_change_;
  std::function<void(SolverBackend)> n_change_;
  std::string title_ = "Solver";  // Title

 public:
   // onChange
  explicit GpuSolverWidget(Context& ctx,
                           std::function<void(SolverBackend)> nChange = nullptr);
  // draw 
  void draw() override;            

  // extern setter
  void set_backend(SolverBackend b);
  SolverBackend backend() const { return backend_; }

  void set_title(std::string t) { title_ = std::move(t); }

};


#pragma once

#include "gui_helper.hpp"
#include "widgets/cloth_simulator_widget.hpp"
#include "widgets/model_loader_widget.hpp"
#include "widgets/selector_widget.hpp"

class Demo {
  UIContext ui_ctx_ = {};
  EngineContext engine_ctx_ = {};

  ModelLoaderWidget model_loader_widget_{ui_ctx_, engine_ctx_};
  SelectorWidget selector_widget_{ui_ctx_};
  ClothSimulatorWidget cloth_sim_widget_{ui_ctx_, engine_ctx_};
  HelpBarWidget help_bar_widget_{ui_ctx_};

  void draw();

 public:
  Demo();
  void run();
  bool load_model_from_path(const std::string& path);
};

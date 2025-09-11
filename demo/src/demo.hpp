#pragma once

#include "gui_utils.hpp"
#include "widgets/help_bar_widget.hpp"
#include "widgets/object_setting_widget.hpp"
#include "widgets/scene_widget.hpp"
#include "widgets/sim_setting_widget.hpp"
#include "widgets/simulator_widget.hpp"
#include "widgets/statistic_widget.hpp"

class Demo {
 private:
  Context ctx_ = {};

  SceneWidget scene_widget_{ctx_};
  ObjectSettingWidget object_setting_widget_{ctx_};
  SimSettingWidget sim_setting_widget_{ctx_};
  SimulatorWidget simulator_widget_{ctx_};
  HelpBarWidget help_bar_widget_{ctx_};
  StatisticWidget statistic_widget_{ctx_};

 public:
  Demo();
  void run();
  bool load_model_from_path(const std::string& path);

 private:
  void draw();
};

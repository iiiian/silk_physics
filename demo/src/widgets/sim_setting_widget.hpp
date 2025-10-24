#pragma once

#include "../gui_utils.hpp"

class SimSettingWidget : public IWidget {
 private:
  Context& ctx_;

  int target_fps_ = 360;

 public:
  explicit SimSettingWidget(Context& context);
  void draw() override;
};

#pragma once

#include "../gui_helper.hpp"

class SimSettingWidget : public IWidget {
 private:
  Context& ctx_;

  int target_fps_ = 60;
  float accleration_[3] = {0.0f, 0.0f, 0.0f};

 public:
  explicit SimSettingWidget(Context& context);
  void draw() override;
};

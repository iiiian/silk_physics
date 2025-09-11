#pragma once

#include "../gui_utils.hpp"

class StatisticWidget : public IWidget {
  Context& ctx_;

 public:
  explicit StatisticWidget(Context& context);
  void draw() override;
};

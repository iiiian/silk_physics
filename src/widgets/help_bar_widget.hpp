#pragma once

#include "../gui_helper.hpp"

class HelpBarWidget : public IWidget {
  UIContext& ui_ctx_;

 public:
  explicit HelpBarWidget(UIContext& context);

  EventFlag draw() override;
  void on_event(EventFlag) override;
};

#pragma once

#include "../gui_utils.hpp"

class HelpBarWidget : public IWidget {
 private:
  Context& ctx_;

 public:
  explicit HelpBarWidget(Context& context);

  void draw() override;
};

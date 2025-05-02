#pragma once

#include "../gui_helper.hpp"

class ModelLoaderWidget : public IWidget {
  UIContext& ctx_;
  EngineContext& p_engine_ctx_;

 public:
  explicit ModelLoaderWidget(UIContext& ui_context,
                             EngineContext& engine_context);
  EventFlag draw() override;
  void on_event(EventFlag events) override;
};

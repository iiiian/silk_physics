#pragma once

#include <string>

#include "../gui_helper.hpp"

class ModelLoaderWidget : public IWidget {
  UIContext& ui_ctx_;
  EngineContext& engine_ctx_;

 public:
  ModelLoaderWidget(UIContext& ui_context, EngineContext& engine_context);
  bool load_model_from_path(const std::string& path);
  EventFlag draw() override;
  void on_event(EventFlag events) override;
};

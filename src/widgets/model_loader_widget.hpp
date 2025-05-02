#pragma once

#include "../gui_helper.hpp"

class ModelLoaderWidget : public IWidget {
  UIContext& ui_ctx_;
  EngineContext& engine_ctx_;

  bool load_model_from_path(const std::string& path);

 public:
  ModelLoaderWidget(UIContext& ui_context, EngineContext& engine_context);
  EventFlag draw() override;
  void on_event(EventFlag events) override;
};

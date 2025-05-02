#pragma once

#include <polyscope/point_cloud.h>

#include <Eigen/Core>
#include <glm/glm.hpp>
#include <nanoflann.hpp>

#include "../gui_helper.hpp"

class ModelLoaderWidget : public IWidget {
  UIContext& ctx_;
  EngineContext* p_engine_ctx_ = nullptr;

 public:
  explicit ModelLoaderWidget(UIContext& context);
  void draw() override;
  void on_event(EventFlag events) override;
};

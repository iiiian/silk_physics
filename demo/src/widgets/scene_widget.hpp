#pragma once

#include <string>

#include "../gui_helper.hpp"

class SceneWidget : public IWidget {
  Context& ctx_;

 public:
  SceneWidget(Context& context);

  bool load_object_from_path(const std::string& path, SilkObjectType type);
  bool load_object(SilkObjectType type);
  void draw() override;
};

#pragma once

#include <string>

#include "../gui_utils.hpp"

class SceneWidget : public IWidget {
 private:
  Context& ctx_;

 public:
  SceneWidget(Context& context);

  bool load_object_from_path(const std::string& path, SilkObjectType type);
  bool load_object(SilkObjectType type);
  void draw() override;

 private:
  std::string generate_unique_name(const std::string& base_name);
};

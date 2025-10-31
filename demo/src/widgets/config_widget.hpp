#pragma once

#include "../config.hpp"
#include "../gui_utils.hpp"

//**************************************************************/
//**             Configuration Selection                       */
//**************************************************************/

class ConfigWidget : public IWidget {
 private:
  Context& ctx_;

 public:
  explicit ConfigWidget(Context& ctx);
  void draw() override;
  void apply_config_to_gui(const SimConfig& config);
};

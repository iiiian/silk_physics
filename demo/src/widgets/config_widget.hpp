#pragma once

#include <string>
#include <string_view>

#include "../gui_utils.hpp"

//**************************************************************/
//**             Configuration Selection                       */
//**************************************************************/

class ConfigWidget : public IWidget {
 private:
  static constexpr std::string_view DEFAULT_JSON = "/DTest/default.json";

  Context& ctx_;
  std::string title_ = "Config Selection";
  bool use_default_json_ = true;

 public:
  explicit ConfigWidget(Context& ctx);

  void draw() override;
};

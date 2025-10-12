#pragma once
#include <functional>
#include <string>

#include <polyscope/point_cloud.h>

#include <Eigen/Core>
#include <glm/glm.hpp>
#include "ui_console.hpp"
#include "../gui_utils.hpp"

//**************************************************************/
//**             Configuration Selection                       */
//**************************************************************/

//Work in Progress...
enum class jsonPath { DEFAULT, CUSTOM };
class ConfigWidget: public IWidget{
    private:
        Context& ctx_;
        jsonPath path= jsonPath::DEFAULT;
        std::string title_ = "Config Selection";  // Title
        std::function<void(jsonPath)> status_;
        // default path
         std::string picked_path_ = "/DTest/default.json";

 
    
    public:
         // onChange
  explicit ConfigWidget(Context& ctx,
                           std::function<void(jsonPath)> status = nullptr);
  // draw 
  void draw() override;  

  void set_path(jsonPath p);
  jsonPath p() const { return path; }          

};
#pragma once
#include <functional>
#include <string>

#include <polyscope/point_cloud.h>

#include <Eigen/Core>
#include <glm/glm.hpp>

#include "../gui_utils.hpp"

//**************************************************************/
//**             Configuration Selection                       */
//**************************************************************/

//Work in Progress...
class configWidget: public IWidget{
    private:
        Context& ctx_;
        std::string title_ = "Config Selection";  // Title
    
    public:
      explicit configWidget(Context& ctx,
                           std::function<void()> onChange = nullptr);
        // draw 
         void draw();

};
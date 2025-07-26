#include "statistic_widget.hpp"

#include <igl/edges.h>

StatisticWidget::StatisticWidget(Context& context) : ctx_(context) {}

void StatisticWidget::draw() {
  if (ImGui::CollapsingHeader("Statistics", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("FPS %f", ImGui::GetIO().Framerate);
    ImGui::Separator();
    if (ctx_.selection != -1) {
      Object& obj = ctx_.objects[ctx_.selection];

      ImGui::Text("Object Vertex Num: %d", int(obj.V.rows()));
      ImGui::Text("Object Face Num: %d", int(obj.F.rows()));
    }
  }
}

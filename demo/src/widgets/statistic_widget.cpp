#include "statistic_widget.hpp"

#include <igl/edges.h>

StatisticWidget::StatisticWidget(Context& context) : ctx_(context) {}

void StatisticWidget::draw() {
  if (ImGui::CollapsingHeader("Statistics", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("FPS %f", ImGui::GetIO().Framerate);
    ImGui::Separator();
    if (ctx_.selection != -1) {
      auto& obj = ctx_.objects[ctx_.selection];
      ObjectStat stat = obj->get_stat();

      ImGui::Text("Object Vertex Num: %d", stat.vert_num);
      ImGui::Text("Object Face Num: %d", stat.face_num);
    }
  }
}

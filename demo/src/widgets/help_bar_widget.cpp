#include "help_bar_widget.hpp"

#include <string>

HelpBarWidget::HelpBarWidget(Context& context) : ctx_(context) {};

void HelpBarWidget::draw() {
  constexpr float PAD = 10.0f;

  ImGuiIO& io = ImGui::GetIO();
  ImVec2 window_pos = ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y - PAD);
  ImVec2 window_pos_pivot = ImVec2(0.5f, 1.0f);

  ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
  // ImGui::SetNextWindowBgAlpha(0.35f);

  std::string help_text;
  switch (ctx_.ui_mode) {
    case UIMode::Normal:

      help_text = "NORMAL MODE | Left Click Rotate / Right Click Move";
      break;
    case UIMode::Paint:
      help_text = "PAINT MODE | Left Click Paint / Right Click Erase";
      break;
    case UIMode::Sim:
      help_text =
          "SIM MODE | Left Click Rotate / Right Click Move/ Ctrl + Left Click "
          "Drag";
      break;
  }

  if (ImGui::Begin("Help Bar", nullptr,
                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDecoration |
                       ImGuiWindowFlags_AlwaysAutoResize |
                       ImGuiWindowFlags_NoSavedSettings |
                       ImGuiWindowFlags_NoFocusOnAppearing |
                       ImGuiWindowFlags_NoNav)) {
    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 0.9f), "%s", help_text.c_str());
  }
  ImGui::End();
}

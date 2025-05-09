#pragma once
#include "gui_helper.hpp"

class HelpBarWidget : public IWidget {
  UIContext& ui_ctx_;
  
public:
  explicit HelpBarWidget(UIContext& context) : ui_ctx_(context) {}
  
  EventFlag draw() override {
    const float PAD = 10.0f;
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 window_pos = ImVec2(io.DisplaySize.x - PAD, io.DisplaySize.y - PAD);
    ImVec2 window_pos_pivot = ImVec2(1.0f, 1.0f);
    
    ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
    ImGui::SetNextWindowBgAlpha(0.35f);
    
    if (ImGui::Begin("Help Bar", nullptr, 
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav)) {
        
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 0.9f), "%s", 
                        ui_ctx_.help_text.c_str());
    }
    ImGui::End();
    
    return EventFlag::NoEvent;
  }

  void on_event(EventFlag) override {}
};

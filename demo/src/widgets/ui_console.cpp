#include "ui_console.hpp"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <deque>
#include <string>

std::deque<std::string> g_lines;
constexpr size_t G_CAP = 2000;
bool g_auto_scroll = true;
char g_filter[128] = {0};

void ui_console_push(std::string line) {
  if (g_lines.size() >= G_CAP) {
    g_lines.pop_front();
  }
  g_lines.push_back(std::move(line));
}

void ui_console_clear() { g_lines.clear(); }

static void ui_console_render_list() {
  const bool use_filter = (g_filter[0] != '\0');
  for (const auto& s : g_lines) {
    if (use_filter) {
      std::string hay = s, nee = g_filter;
      std::transform(hay.begin(), hay.end(), hay.begin(), ::tolower);
      std::transform(nee.begin(), nee.end(), nee.begin(), ::tolower);
      if (hay.find(nee) == std::string::npos) {
        continue;
      }
    }
    ImGui::TextUnformatted(s.c_str());
  }

  if (g_auto_scroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
    ImGui::SetScrollHereY(1.0f);
  }
}

// console panel
void ui_console_draw_window(const char* title) {
  if (!ImGui::Begin(title)) {
    ImGui::End();
    return;
  }

  if (ImGui::Button("Clear")) {
    ui_console_clear();
  }
  ImGui::SameLine();
  ImGui::Checkbox("Auto-scroll", &g_auto_scroll);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(220.f);
  ImGui::InputTextWithHint("##filter", "filter (substring)", g_filter,
                           IM_ARRAYSIZE(g_filter));
  ImGui::Separator();

  ImGui::BeginChild("console_region_window", ImVec2(0, 0), false,
                    ImGuiWindowFlags_HorizontalScrollbar);
  ui_console_render_list();
  ImGui::EndChild();

  ImGui::End();
}

void ui_console_draw_inline(float height) {
  // color
  ImVec4 orange = ImVec4(1.00f, 0.647f, 0.00f, 1.0f);  // #FFA500
  ImVec4 orange_act = ImVec4(0.80f, 0.52f, 0.10f, 1.0f);

  ImGui::PushStyleColor(ImGuiCol_Header, orange_act);
  ImGui::PushStyleColor(ImGuiCol_HeaderHovered, orange);
  ImGui::PushStyleColor(ImGuiCol_HeaderActive, orange_act);

  // retracktable
  const ImGuiTreeNodeFlags flags =
      ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_SpanFullWidth;

  bool open = ImGui::CollapsingHeader("Console", flags);
  ImGui::PopStyleColor(3);

  if (!open) {
    return;
  }

  if (ImGui::Button("Clear")) {
    ui_console_clear();
  }
  ImGui::SameLine();
  ImGui::Checkbox("Auto-scroll", &g_auto_scroll);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(220.f);
  ImGui::InputTextWithHint("##filter_inline", "filter (substring)", g_filter,
                           IM_ARRAYSIZE(g_filter));
  ImGui::Separator();

  if (height < 0.0f) {
    height = ImGui::GetTextLineHeight() * 12.0f;
  }
  ImGui::BeginChild("console_region_inline", ImVec2(0, height), false,
                    ImGuiWindowFlags_HorizontalScrollbar);
  ui_console_render_list();
  ImGui::EndChild();
}

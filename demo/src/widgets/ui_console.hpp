#pragma once

#include <spdlog/fmt/bundled/format.h>
#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

#include <string>
#include <string_view>

void ui_console_draw_window(const char* title = "Console");
void ui_console_draw_inline(float height = -1.0f);

void ui_console_push(std::string line);
void ui_console_clear();

template <typename... T>
void ui_info(std::string_view format_string, T... args) {
  std::string s = fmt::format(format_string, args...);
  spdlog::info("{}", s);
  ui_console_push(std::move(s));
}

template <typename... T>
void ui_warning(std::string_view format_string, T... args) {
  std::string s = fmt::format(format_string, args...);
  spdlog::warn("{}", s);
  ui_console_push(std::move(s));
}

template <typename... T>
void ui_error(std::string_view format_string, T... args) {
  std::string s = fmt::format(format_string, args...);
  spdlog::warn("{}", s);
  ui_console_push(std::move(s));
}

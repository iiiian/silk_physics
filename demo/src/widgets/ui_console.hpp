#pragma once

#include <spdlog/spdlog.h>

#include <format>
#include <string>
#include <string_view>

void ui_console_draw_window(const char* title = "Console");
void ui_console_draw_inline(float height = -1.0f);

void ui_console_push(std::string line);
void ui_console_clear();

template <typename... T>
void ui_info(std::format_string<T...> format_string, T&&... args) {
  std::string s = std::format(format_string, std::forward<T>(args)...);
  spdlog::info("{}", s);
  ui_console_push(std::move(s));
}

template <typename... T>
void ui_warning(std::format_string<T...> format_string, T&&... args) {
  std::string s = std::format(format_string, std::forward<T>(args)...);
  spdlog::warn("{}", s);
  ui_console_push(std::move(s));
}

template <typename... T>
void ui_error(std::format_string<T...> format_string, T&&... args) {
  std::string s = std::format(format_string, std::forward<T>(args)...);
  spdlog::error("{}", s);
  ui_console_push(std::move(s));
}

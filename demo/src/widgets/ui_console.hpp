#pragma once
#include <string>

void ui_console_draw_window(const char* title = "Console");
void ui_console_draw_inline(float height = -1.0f);

void ui_console_push(std::string line);
void ui_console_clear();

// Console Panel
#include <spdlog/fmt/bundled/format.h>
#include <spdlog/spdlog.h>

#define UI_LOGI(fmt_, ...)                      \
  do {                                          \
    auto _s = fmt::format(fmt_, ##__VA_ARGS__); \
    spdlog::info("{}", _s);                     \
    ui_console_push(std::move(_s));             \
  } while (0)

#define UI_LOGW(fmt_, ...)                      \
  do {                                          \
    auto _s = fmt::format(fmt_, ##__VA_ARGS__); \
    spdlog::warn("{}", _s);                     \
    ui_console_push(std::move(_s));             \
  } while (0)

#define UI_LOGE(fmt_, ...)                      \
  do {                                          \
    auto _s = fmt::format(fmt_, ##__VA_ARGS__); \
    spdlog::error("{}", _s);                    \
    ui_console_push(std::move(_s));             \
  } while (0)

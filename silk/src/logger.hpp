#pragma once

// uncomment when debugging
// #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <sstream>
#include <type_traits>

// eigen formatter support
template <typename T>
struct fmt::formatter<
    T, std::enable_if_t<std::is_base_of_v<Eigen::EigenBase<T>, T>, char>> {
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const T& m, FormatContext& ctx) const -> decltype(ctx.out()) {
    std::stringstream ss;
    ss << m;
    return fmt::format_to(ctx.out(), "{}", ss.str());
  }
};

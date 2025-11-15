#pragma once

// Portable unreachable statement

#if defined(__clang__) || defined(__GNUC__)
#define SILK_UNREACHABLE() __builtin_unreachable()
#elif defined(_MSC_VER)
#define SILK_UNREACHABLE() __assume(0)
#else
#include <cstdlib>
#define SILK_UNREACHABLE() std::abort()
#endif

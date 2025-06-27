#pragma once

#include <cstdint>
#include <type_traits>

// bitwise operator for bit flag enum

// By default, bitmask operations are disabled for enums.
// To enable, specialize template:
// template <>
// inline constexpr bool is_bitflag_v<MyEnum> = true;
template <typename T>
inline constexpr bool is_bitflag_v = false;

// Helper trait to check for BitFlagEnum requirements
template <typename T, typename Enable = void>
struct is_bitflag_enum : std::false_type {};

// bit flag should be a enum of uint32_t and specialize the is_bitflag_v
// template
template <typename T>
struct is_bitflag_enum<
    T, std::enable_if_t<std::is_enum_v<T> && is_bitflag_v<T> &&
                        std::is_same_v<std::underlying_type_t<T>, uint32_t>>>
    : std::true_type {};

template <typename T>
inline constexpr bool is_bitflag_enum_v = is_bitflag_enum<T>::value;

// Bitwise NOT
template <typename T, std::enable_if_t<is_bitflag_enum_v<T>, int> = 0>
inline T operator~(T a) {
  return static_cast<T>(~static_cast<uint32_t>(a));
}

// Bitwise OR
template <typename T, std::enable_if_t<is_bitflag_enum_v<T>, int> = 0>
inline T operator|(T a, T b) {
  return static_cast<T>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

// Bitwise AND
template <typename T, std::enable_if_t<is_bitflag_enum_v<T>, int> = 0>
inline T operator&(T a, T b) {
  return static_cast<T>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

// Bitwise XOR
template <typename T, std::enable_if_t<is_bitflag_enum_v<T>, int> = 0>
inline T operator^(T a, T b) {
  return static_cast<T>(static_cast<uint32_t>(a) ^ static_cast<uint32_t>(b));
}

// Bitwise OR assignment
template <typename T, std::enable_if_t<is_bitflag_enum_v<T>, int> = 0>
inline T& operator|=(T& a, T b) {
  a = a | b;
  return a;
}

// Bitwise AND assignment
template <typename T, std::enable_if_t<is_bitflag_enum_v<T>, int> = 0>
inline T& operator&=(T& a, T b) {
  a = a & b;
  return a;
}

// Bitwise XOR assignment
template <typename T, std::enable_if_t<is_bitflag_enum_v<T>, int> = 0>
inline T& operator^=(T& a, T b) {
  a = a ^ b;
  return a;
}

// get underlying uint32
template <typename T, std::enable_if_t<is_bitflag_enum_v<T>, int> = 0>
inline uint32_t raw(T a) {
  return static_cast<uint32_t>(a);
}

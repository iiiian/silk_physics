#pragma once

#include <cstdint>
#include <type_traits>

// bitwise operator for bit flag enum

// By default, bitmask operations are disabled for enums.
// To enable, specialize template:
// template <>
// inline constexpr bool is_bitflag_v<MyEnum> = true;
template <typename E>
inline constexpr bool is_bitflag_v = false;

template <typename E>
concept BitFlagEnum = std::is_enum_v<E> && is_bitflag_v<E> &&
                      std::is_same_v<std::underlying_type_t<E>, uint32_t>;

// Bitwise NOT
template <BitFlagEnum T>
inline T operator~(T a) {
  return static_cast<T>(~static_cast<uint32_t>(a));
}

// Bitwise OR
template <BitFlagEnum T>
inline T operator|(T a, T b) {
  return static_cast<T>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

// Bitwise AND
template <BitFlagEnum T>
inline T operator&(T a, T b) {
  return static_cast<T>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

// Bitwise XOR
template <BitFlagEnum T>
inline T operator^(T a, T b) {
  return static_cast<T>(static_cast<uint32_t>(a) ^ static_cast<uint32_t>(b));
}

// Bitwise OR assignment
template <BitFlagEnum T>
inline T& operator|=(T& a, T b) {
  a = a | b;
  return a;
}

// Bitwise AND assignment
template <BitFlagEnum T>
inline T& operator&=(T& a, T b) {
  a = a & b;
  return a;
}

// Bitwise XOR assignment
template <BitFlagEnum T>
inline T& operator^=(T& a, T b) {
  a = a ^ b;
  return a;
}

// get underlying uint
template <BitFlagEnum T>
inline uint32_t raw(T a) {
  return static_cast<uint32_t>(a);
}

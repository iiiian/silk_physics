#pragma once

#include <type_traits> // For std::is_enum_v, std::underlying_type_t, std::is_same_v
#include <cstdint>     // For uint32_t

// --- Mechanism to enable bitmask operations for specific enums ---

// By default, bitmask operations are disabled for enums.
// To enable them for a specific enum MyEnum, specialize this variable:
// template <>
// inline constexpr bool enable_bitmask_operators_v<MyEnum> = true;
template <typename E>
inline constexpr bool enable_bitmask_operators_v = false;

// Concept to identify enum types that have opted-in for bitmask operations
// and have uint32_t as their underlying type.
template <typename E>
concept BitmaskEnum = std::is_enum_v<E> &&
                      enable_bitmask_operators_v<E> &&
                      std::is_same_v<std::underlying_type_t<E>, uint32_t>;

// --- Type-safe bitwise operator overloads for BitmaskEnums ---

// Bitwise NOT
template <BitmaskEnum T>
inline T operator~(T a) {
  // The concept already ensures underlying_type_t<T> is uint32_t
  return static_cast<T>(~static_cast<uint32_t>(a));
}

// Bitwise OR
template <BitmaskEnum T>
inline T operator|(T a, T b) {
  return static_cast<T>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

// Bitwise AND
template <BitmaskEnum T>
inline T operator&(T a, T b) {
  return static_cast<T>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

// Bitwise XOR
template <BitmaskEnum T>
inline T operator^(T a, T b) {
  return static_cast<T>(static_cast<uint32_t>(a) ^ static_cast<uint32_t>(b));
}

// Bitwise OR assignment
template <BitmaskEnum T>
inline T& operator|=(T& a, T b) {
  a = a | b; // Uses the operator| defined above
  return a;
}

// Bitwise AND assignment
template <BitmaskEnum T>
inline T& operator&=(T& a, T b) {
  a = a & b; // Uses the operator& defined above
  return a;
}

// Bitwise XOR assignment
template <BitmaskEnum T>
inline T& operator^=(T& a, T b) {
  a = a ^ b; // Uses the operator^ defined above
  return a;
}

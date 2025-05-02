#pragma once

#include <type_traits>
#include <concepts>

template<typename E>
concept Enum = std::is_enum_v<E>;

template<Enum E>
class Flags {
    using U = std::underlying_type_t<E>;
    U bits_ = 0;

public:
    constexpr Flags() noexcept = default;
    constexpr Flags(E e) noexcept : bits_(static_cast<U>(e)) {}
  
    //–– Compound assignment
    constexpr Flags& operator|=(Flags o) noexcept { bits_ |= o.bits_; return *this; }
    constexpr Flags& operator&=(Flags o) noexcept { bits_ &= o.bits_; return *this; }
    constexpr Flags& operator^=(Flags o) noexcept { bits_ ^= o.bits_; return *this; }

    //–– Boolean queries
    constexpr bool any()   const noexcept { return bits_ != 0; }
    constexpr bool none()  const noexcept { return bits_ == 0; }
    constexpr bool test(E e) const noexcept { return (bits_ & static_cast<U>(e)) != 0; }

    //–– Single‐flag modifiers
    constexpr void set(E e)   noexcept { bits_ |= static_cast<U>(e); }
    constexpr void reset(E e) noexcept { bits_ &= ~static_cast<U>(e); }
    constexpr void flip(E e)  noexcept { bits_ ^= static_cast<U>(e); }

    //–– Bitwise operators between Flags
    friend constexpr Flags operator|(Flags a, Flags b) noexcept {
        return Flags(a.bits_ | b.bits_);
    }
    friend constexpr Flags operator&(Flags a, Flags b) noexcept {
        return Flags(a.bits_ & b.bits_);
    }
    friend constexpr Flags operator^(Flags a, Flags b) noexcept {
        return Flags(a.bits_ ^ b.bits_);
    }
};


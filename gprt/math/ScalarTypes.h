#pragma once

#include "Float16.h"

// Boost inspired library for string formatting.
// #include <fmt/core.h>

#include <string>
#include <cstdint>

namespace math
{

enum class Handedness
{
    RightHanded,
    LeftHanded,
};

using uint = uint32_t;

// clang-format off
template<typename T> [[nodiscard]] std::string to_string(T v);
template<> [[nodiscard]] inline std::string to_string(bool v) { return v ? "1" : "0"; }
template<> [[nodiscard]] inline std::string to_string(int v) { return std::to_string(v); }
template<> [[nodiscard]] inline std::string to_string(uint v) { return std::to_string(v); }
template<> [[nodiscard]] inline std::string to_string(float v) { return std::to_string(v); }
template<> [[nodiscard]] inline std::string to_string(double v) { return std::to_string(v); }
template<> [[nodiscard]] inline std::string to_string(float16_t v) { return std::to_string(float(v)); }
// clang-format on

// clang-format off
template<typename T> struct is_bool : ::std::is_same<T, bool> {};
template<typename T> struct is_int : ::std::is_same<T, int32_t> {};
template<typename T> struct is_uint : ::std::is_same<T, uint32_t> {};
template<typename T> struct is_float : ::std::is_same<T, float> {};
template<typename T> struct is_double : ::std::is_same<T, double> {};
template<typename T> struct is_float16_t : ::std::is_same<T, float16_t> {};

template<typename T> constexpr bool is_bool_v = is_bool<T>::value;
template<typename T> constexpr bool is_int_v = is_int<T>::value;
template<typename T> constexpr bool is_uint_v = is_uint<T>::value;
template<typename T> constexpr bool is_float_v = is_float<T>::value;
template<typename T> constexpr bool is_double_v = is_double<T>::value;
template<typename T> constexpr bool is_float16_t_v = is_float16_t<T>::value;

template<typename T> constexpr bool is_arithmetic_v = std::is_arithmetic_v<T> || is_float16_t_v<T>;
template<typename T> constexpr bool is_floating_point_v = is_float_v<T> || is_double_v<T> || is_float16_t_v<T>;
using std::is_integral_v;
using std::is_signed_v;
using std::is_unsigned_v;
// clang-format on

template<typename T>
struct ScalarTraits
{};

template<>
struct ScalarTraits<bool>
{
    static constexpr const char* name{"bool"};
};

template<>
struct ScalarTraits<int>
{
    static constexpr const char* name{"int"};
};

template<>
struct ScalarTraits<uint>
{
    static constexpr const char* name{"uint"};
};

template<>
struct ScalarTraits<float>
{
    static constexpr const char* name{"float"};
};

template<>
struct ScalarTraits<double>
{
    static constexpr const char* name{"double"};
};

template<>
struct ScalarTraits<float16_t>
{
    static constexpr const char* name{"float16_t"};
};

} // namespace math

using uint = math::uint;
using float16_t = math::float16_t;

#if GPRT_MSVC
#pragma warning(push)
#pragma warning(disable : 4455) // disable warning about literal suffixes not starting with an underscore
#endif

using math::operator""h;

#if GPRT_MSVC
#pragma warning(pop)
#endif


// Commented out, but we can add this back in if we want to allow for printing out float16_t values.
// // Formatter for the float16_t.
// template<>
// struct fmt::formatter<Falcor::math::float16_t> : formatter<float>
// {
//     template<typename FormatContext>
//     auto format(Falcor::math::float16_t value, FormatContext& ctx) const
//     {
//         return formatter<float>::format(float(value), ctx);
//     }
// };

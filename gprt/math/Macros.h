#pragma once

/**
 * Compilers.
 */
#define GPRT_COMPILER_MSVC 1
#define GPRT_COMPILER_CLANG 2
#define GPRT_COMPILER_GCC 3

/**
 * Determine the compiler in use.
 * http://sourceforge.net/p/predef/wiki/Compilers/
 */
#ifndef GPRT_COMPILER
#if defined(_MSC_VER)
#define GPRT_COMPILER GPRT_COMPILER_MSVC
#elif defined(__clang__)
#define GPRT_COMPILER GPRT_COMPILER_CLANG
#elif defined(__GNUC__)
#define GPRT_COMPILER GPRT_COMPILER_GCC
#else
#error "Unsupported compiler"
#endif
#endif // GPRT_COMPILER

#define GPRT_MSVC (GPRT_COMPILER == GPRT_COMPILER_MSVC)
#define GPRT_CLANG (GPRT_COMPILER == GPRT_COMPILER_CLANG)
#define GPRT_GCC (GPRT_COMPILER == GPRT_COMPILER_GCC)

/**
 * Platforms.
 */
#define GPRT_PLATFORM_WINDOWS 1
#define GPRT_PLATFORM_LINUX 2

/**
 * Determine the target platform in use.
 * http://sourceforge.net/p/predef/wiki/OperatingSystems/
 */
#ifndef GPRT_PLATFORM
#if defined(_WIN64)
#define GPRT_PLATFORM GPRT_PLATFORM_WINDOWS
#elif defined(__linux__)
#define GPRT_PLATFORM GPRT_PLATFORM_LINUX
#else
#error "Unsupported target platform"
#endif
#endif // GPRT_PLATFORM

#define GPRT_WINDOWS (GPRT_PLATFORM == GPRT_PLATFORM_WINDOWS)
#define GPRT_LINUX (GPRT_PLATFORM == GPRT_PLATFORM_LINUX)

/**
 * D3D12 Agility SDK.
 */
#if GPRT_HAS_D3D12_AGILITY_SDK
#define GPRT_D3D12_AGILITY_SDK_VERSION 4
#define GPRT_D3D12_AGILITY_SDK_PATH ".\\D3D12\\"
// To enable the D3D12 Agility SDK, this macro needs to be added to the main source file of the executable.
#define GPRT_EXPORT_D3D12_AGILITY_SDK                                                                 \
    extern "C"                                                                                          \
    {                                                                                                   \
        GPRT_API_EXPORT extern const unsigned int D3D12SDKVersion = GPRT_D3D12_AGILITY_SDK_VERSION; \
    }                                                                                                   \
    extern "C"                                                                                          \
    {                                                                                                   \
        GPRT_API_EXPORT extern const char* D3D12SDKPath = GPRT_D3D12_AGILITY_SDK_PATH;              \
    }
#else
#define GPRT_EXPORT_D3D12_AGILITY_SDK
#endif

/**
 * Define for checking if NVAPI is available.
 */
#define GPRT_NVAPI_AVAILABLE (GPRT_HAS_D3D12 && GPRT_HAS_NVAPI)

/**
 * Shared library (DLL) export and import.
 */
#if GPRT_WINDOWS
#define GPRT_API_EXPORT __declspec(dllexport)
#define GPRT_API_IMPORT __declspec(dllimport)
#elif GPRT_LINUX
#define GPRT_API_EXPORT __attribute__((visibility("default")))
#define GPRT_API_IMPORT
#endif

#ifdef GPRT_DLL
#define GPRT_API GPRT_API_EXPORT
#else // GPRT_DLL
#define GPRT_API GPRT_API_IMPORT
#endif // GPRT_DLL

/**
 * Force inline.
 */
#if GPRT_MSVC
#define GPRT_FORCEINLINE __forceinline
#elif GPRT_CLANG | GPRT_GCC
#define GPRT_FORCEINLINE __attribute__((always_inline))
#endif

/**
 * Preprocessor stringification.
 */
#define GPRT_STRINGIZE(a) #a
#define GPRT_CONCAT_STRINGS_(a, b) a##b
#define GPRT_CONCAT_STRINGS(a, b) GPRT_CONCAT_STRINGS_(a, b)

/**
 * Implement logical operators on a class enum for making it usable as a flags enum.
 */
// clang-format off
#define GPRT_ENUM_CLASS_OPERATORS(e_) \
    inline e_ operator& (e_ a, e_ b) { return static_cast<e_>(static_cast<int>(a)& static_cast<int>(b)); } \
    inline e_ operator| (e_ a, e_ b) { return static_cast<e_>(static_cast<int>(a)| static_cast<int>(b)); } \
    inline e_& operator|= (e_& a, e_ b) { a = a | b; return a; }; \
    inline e_& operator&= (e_& a, e_ b) { a = a & b; return a; }; \
    inline e_  operator~ (e_ a) { return static_cast<e_>(~static_cast<int>(a)); } \
    inline bool is_set(e_ val, e_ flag) { return (val & flag) != static_cast<e_>(0); } \
    inline void flip_bit(e_& val, e_ flag) { val = is_set(val, flag) ? (val & (~flag)) : (val | flag); }
// clang-format on

#pragma once

#include "ScalarTypes.h"

namespace math
{

// ----------------------------------------------------------------------------
// Vector types
// ----------------------------------------------------------------------------

/**
 * Vector type.
 *
 * The semantics are aligned with Slang:
 * - Math operators are element-wise (e.g. +, -, *, /)
 * - Free standing functions for vector operations (e.g. dot(), cross(), etc.)
 *
 * @tparam T Scalar type
 * @tparam N Number of elements (1-4)
 */
template<typename T, int N>
struct vector;

template<typename T>
struct vector<T, 1>
{
    static constexpr int dimension = 1;
    using value_type = T;

    union
    {
        T x;
        T r;
        T s;
    };
    // clang-format off
    /// Default constructor.
    constexpr vector() noexcept = default;
    /// Copy constructor.
    constexpr vector(const vector<T, 1>& other) noexcept = default;
    /// Explicit basic constructor.
    explicit constexpr vector(T x) noexcept : x{x} {}
    /// Explicit basic constructor (scalar).
    template<typename U>
    explicit constexpr vector(U x) noexcept : x{T(x)} {}
    // clang-format on

    template<typename U>
    constexpr vector(const vector<U, 1>& other) noexcept : x{T(other.x)} {};

    [[nodiscard]] constexpr T& operator[](int index) noexcept { return (&(this->x))[index]; }
    [[nodiscard]] constexpr const T& operator[](int index) const noexcept { return (&(this->x))[index]; }

    [[nodiscard]] static constexpr int length() noexcept { return dimension; }
};

template<typename T>
struct vector<T, 2>
{
    static constexpr int dimension = 2;
    using value_type = T;

    union
    {
        struct
        {
            T x, y;
        };
        struct
        {
            T r, g;
        };
        struct
        {
            T s, t;
        };
    };
    // clang-format off
    /// Default constructor.
    constexpr vector() noexcept = default;
    /// Copy constructor.
    constexpr vector(const vector<T, 2>& other) noexcept = default;
    /// Explicit basic constructor.
    constexpr vector(T x, T y) noexcept : x{x}, y{y} {}
    /// Explicit basic constructor (scalar).
    explicit constexpr vector(T scalar) noexcept : x{scalar}, y{scalar} {}
    /// Explicit basic constructor (scalar).
    template<typename U>
    explicit constexpr vector(U scalar) noexcept : x{T(scalar)}, y{T(scalar)} {}

    template<typename X, typename Y>
    constexpr vector(X x, Y y) noexcept : x{T(x)}, y{T(y)} {}
    // clang-format on

    template<typename U>
    constexpr vector(const vector<U, 2>& other) noexcept : x{T(other.x)}, y{T(other.y)} {};

    [[nodiscard]] constexpr T& operator[](int index) noexcept { return (&(this->x))[index]; }
    [[nodiscard]] constexpr const T& operator[](int index) const noexcept { return (&(this->x))[index]; }

    [[nodiscard]] static constexpr int length() noexcept { return dimension; }

    /* <<<PYMACRO
    from itertools import product
    for c in product(["x", "y"], repeat=2):
        name = "".join(c)
        components = ", ".join(c)
        print(f"[[nodiscard]] constexpr auto {name}() const noexcept {{ return vector<T, 2>({components}); }}")
    >>> */
    [[nodiscard]] constexpr auto xx() const noexcept { return vector<T, 2>(x, x); }
    [[nodiscard]] constexpr auto xy() const noexcept { return vector<T, 2>(x, y); }
    [[nodiscard]] constexpr auto yx() const noexcept { return vector<T, 2>(y, x); }
    [[nodiscard]] constexpr auto yy() const noexcept { return vector<T, 2>(y, y); }
    /* <<<PYMACROEND>>> */
};

template<typename T>
struct vector<T, 3>
{
    static constexpr int dimension = 3;
    using value_type = T;

    union
    {
        struct
        {
            T x, y, z;
        };
        struct
        {
            T r, g, b;
        };
        struct
        {
            T s, t, p;
        };
    };
    // clang-format off
    /// Default constructor.
    constexpr vector() noexcept = default;
    /// Copy constructor.
    constexpr vector(const vector<T, 3>& other) noexcept = default;
    /// Explicit basic constructor.
    constexpr vector(T x, T y, T z) noexcept : x{x}, y{y}, z{z} {}
    /// Explicit basic constructor (scalar).
    explicit constexpr vector(T scalar) noexcept : x{scalar}, y{scalar}, z{scalar} {}
    /// Explicit basic constructor (scalar).
    template<typename U>
    explicit constexpr vector(U scalar) noexcept : x{T(scalar)}, y{T(scalar)}, z{T(scalar)} {}

    template<typename X, typename Y, typename Z>
    constexpr vector(X x, Y y, Z z) noexcept : x{T(x)}, y{T(y)}, z{T(z)} {}
    template<typename XY, typename Z>
    constexpr vector(vector<XY, 2> xy, Z z) noexcept : x{T(xy.x)}, y{T(xy.y)}, z{T(z)} {}
    template<typename X, typename YZ>
    constexpr vector(X x, vector<YZ, 2> yz) noexcept : x{T(x)}, y{T(yz.x)}, z{T(yz.y)} {}
    // clang-format on

    template<typename U>
    constexpr vector(const vector<U, 3>& other) noexcept : x{T(other.x)}, y{T(other.y)}, z{T(other.z)} {};

    [[nodiscard]] constexpr T& operator[](int index) noexcept { return (&(this->x))[index]; }
    [[nodiscard]] constexpr const T& operator[](int index) const noexcept { return (&(this->x))[index]; }

    [[nodiscard]] static constexpr int length() noexcept { return dimension; }

    // clang-format off
    /* <<<PYMACRO
    from itertools import product
    for dim in [2, 3]:
        for c in product(["x", "y", "z"], repeat=dim):
            name = "".join(c)
            components = ", ".join(c)
            print(f"[[nodiscard]] constexpr auto {name}() const noexcept {{ return vector<T, {dim}>({components}); }}")
    >>> */
    [[nodiscard]] constexpr auto xx() const noexcept { return vector<T, 2>(x, x); }
    [[nodiscard]] constexpr auto xy() const noexcept { return vector<T, 2>(x, y); }
    [[nodiscard]] constexpr auto xz() const noexcept { return vector<T, 2>(x, z); }
    [[nodiscard]] constexpr auto yx() const noexcept { return vector<T, 2>(y, x); }
    [[nodiscard]] constexpr auto yy() const noexcept { return vector<T, 2>(y, y); }
    [[nodiscard]] constexpr auto yz() const noexcept { return vector<T, 2>(y, z); }
    [[nodiscard]] constexpr auto zx() const noexcept { return vector<T, 2>(z, x); }
    [[nodiscard]] constexpr auto zy() const noexcept { return vector<T, 2>(z, y); }
    [[nodiscard]] constexpr auto zz() const noexcept { return vector<T, 2>(z, z); }
    [[nodiscard]] constexpr auto xxx() const noexcept { return vector<T, 3>(x, x, x); }
    [[nodiscard]] constexpr auto xxy() const noexcept { return vector<T, 3>(x, x, y); }
    [[nodiscard]] constexpr auto xxz() const noexcept { return vector<T, 3>(x, x, z); }
    [[nodiscard]] constexpr auto xyx() const noexcept { return vector<T, 3>(x, y, x); }
    [[nodiscard]] constexpr auto xyy() const noexcept { return vector<T, 3>(x, y, y); }
    [[nodiscard]] constexpr auto xyz() const noexcept { return vector<T, 3>(x, y, z); }
    [[nodiscard]] constexpr auto xzx() const noexcept { return vector<T, 3>(x, z, x); }
    [[nodiscard]] constexpr auto xzy() const noexcept { return vector<T, 3>(x, z, y); }
    [[nodiscard]] constexpr auto xzz() const noexcept { return vector<T, 3>(x, z, z); }
    [[nodiscard]] constexpr auto yxx() const noexcept { return vector<T, 3>(y, x, x); }
    [[nodiscard]] constexpr auto yxy() const noexcept { return vector<T, 3>(y, x, y); }
    [[nodiscard]] constexpr auto yxz() const noexcept { return vector<T, 3>(y, x, z); }
    [[nodiscard]] constexpr auto yyx() const noexcept { return vector<T, 3>(y, y, x); }
    [[nodiscard]] constexpr auto yyy() const noexcept { return vector<T, 3>(y, y, y); }
    [[nodiscard]] constexpr auto yyz() const noexcept { return vector<T, 3>(y, y, z); }
    [[nodiscard]] constexpr auto yzx() const noexcept { return vector<T, 3>(y, z, x); }
    [[nodiscard]] constexpr auto yzy() const noexcept { return vector<T, 3>(y, z, y); }
    [[nodiscard]] constexpr auto yzz() const noexcept { return vector<T, 3>(y, z, z); }
    [[nodiscard]] constexpr auto zxx() const noexcept { return vector<T, 3>(z, x, x); }
    [[nodiscard]] constexpr auto zxy() const noexcept { return vector<T, 3>(z, x, y); }
    [[nodiscard]] constexpr auto zxz() const noexcept { return vector<T, 3>(z, x, z); }
    [[nodiscard]] constexpr auto zyx() const noexcept { return vector<T, 3>(z, y, x); }
    [[nodiscard]] constexpr auto zyy() const noexcept { return vector<T, 3>(z, y, y); }
    [[nodiscard]] constexpr auto zyz() const noexcept { return vector<T, 3>(z, y, z); }
    [[nodiscard]] constexpr auto zzx() const noexcept { return vector<T, 3>(z, z, x); }
    [[nodiscard]] constexpr auto zzy() const noexcept { return vector<T, 3>(z, z, y); }
    [[nodiscard]] constexpr auto zzz() const noexcept { return vector<T, 3>(z, z, z); }
    /* <<<PYMACROEND>>> */
    // clang-format on
};

template<typename T>
struct vector<T, 4>
{
    static constexpr int dimension = 4;
    using value_type = T;

    union
    {
        struct
        {
            T x, y, z, w;
        };
        struct
        {
            T r, g, b, a;
        };
        struct
        {
            T s, t, p, q;
        };
    };
    // clang-format off
    /// Default constructor.
    constexpr vector() noexcept = default;
    /// Copy constructor.
    constexpr vector(const vector<T, 4>& other) noexcept = default;
    /// Explicit basic constructor.
    constexpr vector(T x, T y, T z, T w) noexcept : x{x}, y{y}, z{z}, w{w} {}
    /// Explicit basic constructor (scalar).
    explicit constexpr vector(T scalar) noexcept : x{scalar}, y{scalar}, z{scalar}, w{scalar} {}
    /// Explicit basic constructor (scalar).
    template<typename U>
    explicit constexpr vector(U scalar) noexcept : x{T(scalar)}, y{T(scalar)}, z{T(scalar)}, w{T(scalar)} {}

    template<typename X, typename Y, typename Z, typename W>
    constexpr vector(X x, Y y, Z z, W w) noexcept : x{T(x)}, y{T(y)}, z{T(z)}, w{T(w)} {}
    template<typename XY, typename Z, typename W>
    constexpr vector(vector<XY, 2> xy, Z z, W w) noexcept : x{T(xy.x)}, y{T(xy.y)}, z{T(z)}, w{T(w)} {}
    template<typename X, typename YZ, typename W>
    constexpr vector(X x, vector<YZ, 2> yz, W w) noexcept : x{T(x)}, y{T(yz.x)}, z{T(yz.y)}, w{T(w)} {}
    template<typename X, typename Y, typename ZW>
    constexpr vector(X x, Y y, vector<ZW, 2> zw) noexcept : x{T(x)}, y{T(y)}, z{T(zw.x)}, w{T(zw.y)} {}
    template<typename XY, typename ZW>
    constexpr vector(vector<XY, 2> xy, vector<ZW, 2> zw) noexcept : x{T(xy.x)}, y{T(xy.y)}, z{T(zw.x)}, w{T(zw.y)} {}
    template<typename XYZ, typename W>
    constexpr vector(vector<XYZ, 3> xyz, W w) noexcept : x{T(xyz.x)}, y{T(xyz.y)}, z{T(xyz.z)}, w{T(w)} {}
    template<typename X, typename YZW>
    constexpr vector(X x, vector<YZW, 3> yzw) noexcept : x{T(x)}, y{T(yzw.x)}, z{T(yzw.y)}, w{T(yzw.z)} {}
    // clang-format on

    template<typename U>
    constexpr vector(const vector<U, 4>& other) noexcept : x{T(other.x)}, y{T(other.y)}, z{T(other.z)}, w{T(other.w)} {};

    [[nodiscard]] constexpr T& operator[](int index) noexcept { return (&(this->x))[index]; }
    [[nodiscard]] constexpr const T& operator[](int index) const noexcept { return (&(this->x))[index]; }

    [[nodiscard]] static constexpr int length() noexcept { return dimension; }

    /* <<<PYMACRO
    from itertools import product
    for dim in [2, 3, 4]:
        for c in product(["x", "y", "z", "w"], repeat=dim):
            name = "".join(c)
            components = ", ".join(c)
            print(f"[[nodiscard]] constexpr auto {name}() const noexcept {{ return vector<T, {dim}>({components}); }}")
    >>> */
    [[nodiscard]] constexpr auto xx() const noexcept { return vector<T, 2>(x, x); }
    [[nodiscard]] constexpr auto xy() const noexcept { return vector<T, 2>(x, y); }
    [[nodiscard]] constexpr auto xz() const noexcept { return vector<T, 2>(x, z); }
    [[nodiscard]] constexpr auto xw() const noexcept { return vector<T, 2>(x, w); }
    [[nodiscard]] constexpr auto yx() const noexcept { return vector<T, 2>(y, x); }
    [[nodiscard]] constexpr auto yy() const noexcept { return vector<T, 2>(y, y); }
    [[nodiscard]] constexpr auto yz() const noexcept { return vector<T, 2>(y, z); }
    [[nodiscard]] constexpr auto yw() const noexcept { return vector<T, 2>(y, w); }
    [[nodiscard]] constexpr auto zx() const noexcept { return vector<T, 2>(z, x); }
    [[nodiscard]] constexpr auto zy() const noexcept { return vector<T, 2>(z, y); }
    [[nodiscard]] constexpr auto zz() const noexcept { return vector<T, 2>(z, z); }
    [[nodiscard]] constexpr auto zw() const noexcept { return vector<T, 2>(z, w); }
    [[nodiscard]] constexpr auto wx() const noexcept { return vector<T, 2>(w, x); }
    [[nodiscard]] constexpr auto wy() const noexcept { return vector<T, 2>(w, y); }
    [[nodiscard]] constexpr auto wz() const noexcept { return vector<T, 2>(w, z); }
    [[nodiscard]] constexpr auto ww() const noexcept { return vector<T, 2>(w, w); }
    [[nodiscard]] constexpr auto xxx() const noexcept { return vector<T, 3>(x, x, x); }
    [[nodiscard]] constexpr auto xxy() const noexcept { return vector<T, 3>(x, x, y); }
    [[nodiscard]] constexpr auto xxz() const noexcept { return vector<T, 3>(x, x, z); }
    [[nodiscard]] constexpr auto xxw() const noexcept { return vector<T, 3>(x, x, w); }
    [[nodiscard]] constexpr auto xyx() const noexcept { return vector<T, 3>(x, y, x); }
    [[nodiscard]] constexpr auto xyy() const noexcept { return vector<T, 3>(x, y, y); }
    [[nodiscard]] constexpr auto xyz() const noexcept { return vector<T, 3>(x, y, z); }
    [[nodiscard]] constexpr auto xyw() const noexcept { return vector<T, 3>(x, y, w); }
    [[nodiscard]] constexpr auto xzx() const noexcept { return vector<T, 3>(x, z, x); }
    [[nodiscard]] constexpr auto xzy() const noexcept { return vector<T, 3>(x, z, y); }
    [[nodiscard]] constexpr auto xzz() const noexcept { return vector<T, 3>(x, z, z); }
    [[nodiscard]] constexpr auto xzw() const noexcept { return vector<T, 3>(x, z, w); }
    [[nodiscard]] constexpr auto xwx() const noexcept { return vector<T, 3>(x, w, x); }
    [[nodiscard]] constexpr auto xwy() const noexcept { return vector<T, 3>(x, w, y); }
    [[nodiscard]] constexpr auto xwz() const noexcept { return vector<T, 3>(x, w, z); }
    [[nodiscard]] constexpr auto xww() const noexcept { return vector<T, 3>(x, w, w); }
    [[nodiscard]] constexpr auto yxx() const noexcept { return vector<T, 3>(y, x, x); }
    [[nodiscard]] constexpr auto yxy() const noexcept { return vector<T, 3>(y, x, y); }
    [[nodiscard]] constexpr auto yxz() const noexcept { return vector<T, 3>(y, x, z); }
    [[nodiscard]] constexpr auto yxw() const noexcept { return vector<T, 3>(y, x, w); }
    [[nodiscard]] constexpr auto yyx() const noexcept { return vector<T, 3>(y, y, x); }
    [[nodiscard]] constexpr auto yyy() const noexcept { return vector<T, 3>(y, y, y); }
    [[nodiscard]] constexpr auto yyz() const noexcept { return vector<T, 3>(y, y, z); }
    [[nodiscard]] constexpr auto yyw() const noexcept { return vector<T, 3>(y, y, w); }
    [[nodiscard]] constexpr auto yzx() const noexcept { return vector<T, 3>(y, z, x); }
    [[nodiscard]] constexpr auto yzy() const noexcept { return vector<T, 3>(y, z, y); }
    [[nodiscard]] constexpr auto yzz() const noexcept { return vector<T, 3>(y, z, z); }
    [[nodiscard]] constexpr auto yzw() const noexcept { return vector<T, 3>(y, z, w); }
    [[nodiscard]] constexpr auto ywx() const noexcept { return vector<T, 3>(y, w, x); }
    [[nodiscard]] constexpr auto ywy() const noexcept { return vector<T, 3>(y, w, y); }
    [[nodiscard]] constexpr auto ywz() const noexcept { return vector<T, 3>(y, w, z); }
    [[nodiscard]] constexpr auto yww() const noexcept { return vector<T, 3>(y, w, w); }
    [[nodiscard]] constexpr auto zxx() const noexcept { return vector<T, 3>(z, x, x); }
    [[nodiscard]] constexpr auto zxy() const noexcept { return vector<T, 3>(z, x, y); }
    [[nodiscard]] constexpr auto zxz() const noexcept { return vector<T, 3>(z, x, z); }
    [[nodiscard]] constexpr auto zxw() const noexcept { return vector<T, 3>(z, x, w); }
    [[nodiscard]] constexpr auto zyx() const noexcept { return vector<T, 3>(z, y, x); }
    [[nodiscard]] constexpr auto zyy() const noexcept { return vector<T, 3>(z, y, y); }
    [[nodiscard]] constexpr auto zyz() const noexcept { return vector<T, 3>(z, y, z); }
    [[nodiscard]] constexpr auto zyw() const noexcept { return vector<T, 3>(z, y, w); }
    [[nodiscard]] constexpr auto zzx() const noexcept { return vector<T, 3>(z, z, x); }
    [[nodiscard]] constexpr auto zzy() const noexcept { return vector<T, 3>(z, z, y); }
    [[nodiscard]] constexpr auto zzz() const noexcept { return vector<T, 3>(z, z, z); }
    [[nodiscard]] constexpr auto zzw() const noexcept { return vector<T, 3>(z, z, w); }
    [[nodiscard]] constexpr auto zwx() const noexcept { return vector<T, 3>(z, w, x); }
    [[nodiscard]] constexpr auto zwy() const noexcept { return vector<T, 3>(z, w, y); }
    [[nodiscard]] constexpr auto zwz() const noexcept { return vector<T, 3>(z, w, z); }
    [[nodiscard]] constexpr auto zww() const noexcept { return vector<T, 3>(z, w, w); }
    [[nodiscard]] constexpr auto wxx() const noexcept { return vector<T, 3>(w, x, x); }
    [[nodiscard]] constexpr auto wxy() const noexcept { return vector<T, 3>(w, x, y); }
    [[nodiscard]] constexpr auto wxz() const noexcept { return vector<T, 3>(w, x, z); }
    [[nodiscard]] constexpr auto wxw() const noexcept { return vector<T, 3>(w, x, w); }
    [[nodiscard]] constexpr auto wyx() const noexcept { return vector<T, 3>(w, y, x); }
    [[nodiscard]] constexpr auto wyy() const noexcept { return vector<T, 3>(w, y, y); }
    [[nodiscard]] constexpr auto wyz() const noexcept { return vector<T, 3>(w, y, z); }
    [[nodiscard]] constexpr auto wyw() const noexcept { return vector<T, 3>(w, y, w); }
    [[nodiscard]] constexpr auto wzx() const noexcept { return vector<T, 3>(w, z, x); }
    [[nodiscard]] constexpr auto wzy() const noexcept { return vector<T, 3>(w, z, y); }
    [[nodiscard]] constexpr auto wzz() const noexcept { return vector<T, 3>(w, z, z); }
    [[nodiscard]] constexpr auto wzw() const noexcept { return vector<T, 3>(w, z, w); }
    [[nodiscard]] constexpr auto wwx() const noexcept { return vector<T, 3>(w, w, x); }
    [[nodiscard]] constexpr auto wwy() const noexcept { return vector<T, 3>(w, w, y); }
    [[nodiscard]] constexpr auto wwz() const noexcept { return vector<T, 3>(w, w, z); }
    [[nodiscard]] constexpr auto www() const noexcept { return vector<T, 3>(w, w, w); }
    [[nodiscard]] constexpr auto xxxx() const noexcept { return vector<T, 4>(x, x, x, x); }
    [[nodiscard]] constexpr auto xxxy() const noexcept { return vector<T, 4>(x, x, x, y); }
    [[nodiscard]] constexpr auto xxxz() const noexcept { return vector<T, 4>(x, x, x, z); }
    [[nodiscard]] constexpr auto xxxw() const noexcept { return vector<T, 4>(x, x, x, w); }
    [[nodiscard]] constexpr auto xxyx() const noexcept { return vector<T, 4>(x, x, y, x); }
    [[nodiscard]] constexpr auto xxyy() const noexcept { return vector<T, 4>(x, x, y, y); }
    [[nodiscard]] constexpr auto xxyz() const noexcept { return vector<T, 4>(x, x, y, z); }
    [[nodiscard]] constexpr auto xxyw() const noexcept { return vector<T, 4>(x, x, y, w); }
    [[nodiscard]] constexpr auto xxzx() const noexcept { return vector<T, 4>(x, x, z, x); }
    [[nodiscard]] constexpr auto xxzy() const noexcept { return vector<T, 4>(x, x, z, y); }
    [[nodiscard]] constexpr auto xxzz() const noexcept { return vector<T, 4>(x, x, z, z); }
    [[nodiscard]] constexpr auto xxzw() const noexcept { return vector<T, 4>(x, x, z, w); }
    [[nodiscard]] constexpr auto xxwx() const noexcept { return vector<T, 4>(x, x, w, x); }
    [[nodiscard]] constexpr auto xxwy() const noexcept { return vector<T, 4>(x, x, w, y); }
    [[nodiscard]] constexpr auto xxwz() const noexcept { return vector<T, 4>(x, x, w, z); }
    [[nodiscard]] constexpr auto xxww() const noexcept { return vector<T, 4>(x, x, w, w); }
    [[nodiscard]] constexpr auto xyxx() const noexcept { return vector<T, 4>(x, y, x, x); }
    [[nodiscard]] constexpr auto xyxy() const noexcept { return vector<T, 4>(x, y, x, y); }
    [[nodiscard]] constexpr auto xyxz() const noexcept { return vector<T, 4>(x, y, x, z); }
    [[nodiscard]] constexpr auto xyxw() const noexcept { return vector<T, 4>(x, y, x, w); }
    [[nodiscard]] constexpr auto xyyx() const noexcept { return vector<T, 4>(x, y, y, x); }
    [[nodiscard]] constexpr auto xyyy() const noexcept { return vector<T, 4>(x, y, y, y); }
    [[nodiscard]] constexpr auto xyyz() const noexcept { return vector<T, 4>(x, y, y, z); }
    [[nodiscard]] constexpr auto xyyw() const noexcept { return vector<T, 4>(x, y, y, w); }
    [[nodiscard]] constexpr auto xyzx() const noexcept { return vector<T, 4>(x, y, z, x); }
    [[nodiscard]] constexpr auto xyzy() const noexcept { return vector<T, 4>(x, y, z, y); }
    [[nodiscard]] constexpr auto xyzz() const noexcept { return vector<T, 4>(x, y, z, z); }
    [[nodiscard]] constexpr auto xyzw() const noexcept { return vector<T, 4>(x, y, z, w); }
    [[nodiscard]] constexpr auto xywx() const noexcept { return vector<T, 4>(x, y, w, x); }
    [[nodiscard]] constexpr auto xywy() const noexcept { return vector<T, 4>(x, y, w, y); }
    [[nodiscard]] constexpr auto xywz() const noexcept { return vector<T, 4>(x, y, w, z); }
    [[nodiscard]] constexpr auto xyww() const noexcept { return vector<T, 4>(x, y, w, w); }
    [[nodiscard]] constexpr auto xzxx() const noexcept { return vector<T, 4>(x, z, x, x); }
    [[nodiscard]] constexpr auto xzxy() const noexcept { return vector<T, 4>(x, z, x, y); }
    [[nodiscard]] constexpr auto xzxz() const noexcept { return vector<T, 4>(x, z, x, z); }
    [[nodiscard]] constexpr auto xzxw() const noexcept { return vector<T, 4>(x, z, x, w); }
    [[nodiscard]] constexpr auto xzyx() const noexcept { return vector<T, 4>(x, z, y, x); }
    [[nodiscard]] constexpr auto xzyy() const noexcept { return vector<T, 4>(x, z, y, y); }
    [[nodiscard]] constexpr auto xzyz() const noexcept { return vector<T, 4>(x, z, y, z); }
    [[nodiscard]] constexpr auto xzyw() const noexcept { return vector<T, 4>(x, z, y, w); }
    [[nodiscard]] constexpr auto xzzx() const noexcept { return vector<T, 4>(x, z, z, x); }
    [[nodiscard]] constexpr auto xzzy() const noexcept { return vector<T, 4>(x, z, z, y); }
    [[nodiscard]] constexpr auto xzzz() const noexcept { return vector<T, 4>(x, z, z, z); }
    [[nodiscard]] constexpr auto xzzw() const noexcept { return vector<T, 4>(x, z, z, w); }
    [[nodiscard]] constexpr auto xzwx() const noexcept { return vector<T, 4>(x, z, w, x); }
    [[nodiscard]] constexpr auto xzwy() const noexcept { return vector<T, 4>(x, z, w, y); }
    [[nodiscard]] constexpr auto xzwz() const noexcept { return vector<T, 4>(x, z, w, z); }
    [[nodiscard]] constexpr auto xzww() const noexcept { return vector<T, 4>(x, z, w, w); }
    [[nodiscard]] constexpr auto xwxx() const noexcept { return vector<T, 4>(x, w, x, x); }
    [[nodiscard]] constexpr auto xwxy() const noexcept { return vector<T, 4>(x, w, x, y); }
    [[nodiscard]] constexpr auto xwxz() const noexcept { return vector<T, 4>(x, w, x, z); }
    [[nodiscard]] constexpr auto xwxw() const noexcept { return vector<T, 4>(x, w, x, w); }
    [[nodiscard]] constexpr auto xwyx() const noexcept { return vector<T, 4>(x, w, y, x); }
    [[nodiscard]] constexpr auto xwyy() const noexcept { return vector<T, 4>(x, w, y, y); }
    [[nodiscard]] constexpr auto xwyz() const noexcept { return vector<T, 4>(x, w, y, z); }
    [[nodiscard]] constexpr auto xwyw() const noexcept { return vector<T, 4>(x, w, y, w); }
    [[nodiscard]] constexpr auto xwzx() const noexcept { return vector<T, 4>(x, w, z, x); }
    [[nodiscard]] constexpr auto xwzy() const noexcept { return vector<T, 4>(x, w, z, y); }
    [[nodiscard]] constexpr auto xwzz() const noexcept { return vector<T, 4>(x, w, z, z); }
    [[nodiscard]] constexpr auto xwzw() const noexcept { return vector<T, 4>(x, w, z, w); }
    [[nodiscard]] constexpr auto xwwx() const noexcept { return vector<T, 4>(x, w, w, x); }
    [[nodiscard]] constexpr auto xwwy() const noexcept { return vector<T, 4>(x, w, w, y); }
    [[nodiscard]] constexpr auto xwwz() const noexcept { return vector<T, 4>(x, w, w, z); }
    [[nodiscard]] constexpr auto xwww() const noexcept { return vector<T, 4>(x, w, w, w); }
    [[nodiscard]] constexpr auto yxxx() const noexcept { return vector<T, 4>(y, x, x, x); }
    [[nodiscard]] constexpr auto yxxy() const noexcept { return vector<T, 4>(y, x, x, y); }
    [[nodiscard]] constexpr auto yxxz() const noexcept { return vector<T, 4>(y, x, x, z); }
    [[nodiscard]] constexpr auto yxxw() const noexcept { return vector<T, 4>(y, x, x, w); }
    [[nodiscard]] constexpr auto yxyx() const noexcept { return vector<T, 4>(y, x, y, x); }
    [[nodiscard]] constexpr auto yxyy() const noexcept { return vector<T, 4>(y, x, y, y); }
    [[nodiscard]] constexpr auto yxyz() const noexcept { return vector<T, 4>(y, x, y, z); }
    [[nodiscard]] constexpr auto yxyw() const noexcept { return vector<T, 4>(y, x, y, w); }
    [[nodiscard]] constexpr auto yxzx() const noexcept { return vector<T, 4>(y, x, z, x); }
    [[nodiscard]] constexpr auto yxzy() const noexcept { return vector<T, 4>(y, x, z, y); }
    [[nodiscard]] constexpr auto yxzz() const noexcept { return vector<T, 4>(y, x, z, z); }
    [[nodiscard]] constexpr auto yxzw() const noexcept { return vector<T, 4>(y, x, z, w); }
    [[nodiscard]] constexpr auto yxwx() const noexcept { return vector<T, 4>(y, x, w, x); }
    [[nodiscard]] constexpr auto yxwy() const noexcept { return vector<T, 4>(y, x, w, y); }
    [[nodiscard]] constexpr auto yxwz() const noexcept { return vector<T, 4>(y, x, w, z); }
    [[nodiscard]] constexpr auto yxww() const noexcept { return vector<T, 4>(y, x, w, w); }
    [[nodiscard]] constexpr auto yyxx() const noexcept { return vector<T, 4>(y, y, x, x); }
    [[nodiscard]] constexpr auto yyxy() const noexcept { return vector<T, 4>(y, y, x, y); }
    [[nodiscard]] constexpr auto yyxz() const noexcept { return vector<T, 4>(y, y, x, z); }
    [[nodiscard]] constexpr auto yyxw() const noexcept { return vector<T, 4>(y, y, x, w); }
    [[nodiscard]] constexpr auto yyyx() const noexcept { return vector<T, 4>(y, y, y, x); }
    [[nodiscard]] constexpr auto yyyy() const noexcept { return vector<T, 4>(y, y, y, y); }
    [[nodiscard]] constexpr auto yyyz() const noexcept { return vector<T, 4>(y, y, y, z); }
    [[nodiscard]] constexpr auto yyyw() const noexcept { return vector<T, 4>(y, y, y, w); }
    [[nodiscard]] constexpr auto yyzx() const noexcept { return vector<T, 4>(y, y, z, x); }
    [[nodiscard]] constexpr auto yyzy() const noexcept { return vector<T, 4>(y, y, z, y); }
    [[nodiscard]] constexpr auto yyzz() const noexcept { return vector<T, 4>(y, y, z, z); }
    [[nodiscard]] constexpr auto yyzw() const noexcept { return vector<T, 4>(y, y, z, w); }
    [[nodiscard]] constexpr auto yywx() const noexcept { return vector<T, 4>(y, y, w, x); }
    [[nodiscard]] constexpr auto yywy() const noexcept { return vector<T, 4>(y, y, w, y); }
    [[nodiscard]] constexpr auto yywz() const noexcept { return vector<T, 4>(y, y, w, z); }
    [[nodiscard]] constexpr auto yyww() const noexcept { return vector<T, 4>(y, y, w, w); }
    [[nodiscard]] constexpr auto yzxx() const noexcept { return vector<T, 4>(y, z, x, x); }
    [[nodiscard]] constexpr auto yzxy() const noexcept { return vector<T, 4>(y, z, x, y); }
    [[nodiscard]] constexpr auto yzxz() const noexcept { return vector<T, 4>(y, z, x, z); }
    [[nodiscard]] constexpr auto yzxw() const noexcept { return vector<T, 4>(y, z, x, w); }
    [[nodiscard]] constexpr auto yzyx() const noexcept { return vector<T, 4>(y, z, y, x); }
    [[nodiscard]] constexpr auto yzyy() const noexcept { return vector<T, 4>(y, z, y, y); }
    [[nodiscard]] constexpr auto yzyz() const noexcept { return vector<T, 4>(y, z, y, z); }
    [[nodiscard]] constexpr auto yzyw() const noexcept { return vector<T, 4>(y, z, y, w); }
    [[nodiscard]] constexpr auto yzzx() const noexcept { return vector<T, 4>(y, z, z, x); }
    [[nodiscard]] constexpr auto yzzy() const noexcept { return vector<T, 4>(y, z, z, y); }
    [[nodiscard]] constexpr auto yzzz() const noexcept { return vector<T, 4>(y, z, z, z); }
    [[nodiscard]] constexpr auto yzzw() const noexcept { return vector<T, 4>(y, z, z, w); }
    [[nodiscard]] constexpr auto yzwx() const noexcept { return vector<T, 4>(y, z, w, x); }
    [[nodiscard]] constexpr auto yzwy() const noexcept { return vector<T, 4>(y, z, w, y); }
    [[nodiscard]] constexpr auto yzwz() const noexcept { return vector<T, 4>(y, z, w, z); }
    [[nodiscard]] constexpr auto yzww() const noexcept { return vector<T, 4>(y, z, w, w); }
    [[nodiscard]] constexpr auto ywxx() const noexcept { return vector<T, 4>(y, w, x, x); }
    [[nodiscard]] constexpr auto ywxy() const noexcept { return vector<T, 4>(y, w, x, y); }
    [[nodiscard]] constexpr auto ywxz() const noexcept { return vector<T, 4>(y, w, x, z); }
    [[nodiscard]] constexpr auto ywxw() const noexcept { return vector<T, 4>(y, w, x, w); }
    [[nodiscard]] constexpr auto ywyx() const noexcept { return vector<T, 4>(y, w, y, x); }
    [[nodiscard]] constexpr auto ywyy() const noexcept { return vector<T, 4>(y, w, y, y); }
    [[nodiscard]] constexpr auto ywyz() const noexcept { return vector<T, 4>(y, w, y, z); }
    [[nodiscard]] constexpr auto ywyw() const noexcept { return vector<T, 4>(y, w, y, w); }
    [[nodiscard]] constexpr auto ywzx() const noexcept { return vector<T, 4>(y, w, z, x); }
    [[nodiscard]] constexpr auto ywzy() const noexcept { return vector<T, 4>(y, w, z, y); }
    [[nodiscard]] constexpr auto ywzz() const noexcept { return vector<T, 4>(y, w, z, z); }
    [[nodiscard]] constexpr auto ywzw() const noexcept { return vector<T, 4>(y, w, z, w); }
    [[nodiscard]] constexpr auto ywwx() const noexcept { return vector<T, 4>(y, w, w, x); }
    [[nodiscard]] constexpr auto ywwy() const noexcept { return vector<T, 4>(y, w, w, y); }
    [[nodiscard]] constexpr auto ywwz() const noexcept { return vector<T, 4>(y, w, w, z); }
    [[nodiscard]] constexpr auto ywww() const noexcept { return vector<T, 4>(y, w, w, w); }
    [[nodiscard]] constexpr auto zxxx() const noexcept { return vector<T, 4>(z, x, x, x); }
    [[nodiscard]] constexpr auto zxxy() const noexcept { return vector<T, 4>(z, x, x, y); }
    [[nodiscard]] constexpr auto zxxz() const noexcept { return vector<T, 4>(z, x, x, z); }
    [[nodiscard]] constexpr auto zxxw() const noexcept { return vector<T, 4>(z, x, x, w); }
    [[nodiscard]] constexpr auto zxyx() const noexcept { return vector<T, 4>(z, x, y, x); }
    [[nodiscard]] constexpr auto zxyy() const noexcept { return vector<T, 4>(z, x, y, y); }
    [[nodiscard]] constexpr auto zxyz() const noexcept { return vector<T, 4>(z, x, y, z); }
    [[nodiscard]] constexpr auto zxyw() const noexcept { return vector<T, 4>(z, x, y, w); }
    [[nodiscard]] constexpr auto zxzx() const noexcept { return vector<T, 4>(z, x, z, x); }
    [[nodiscard]] constexpr auto zxzy() const noexcept { return vector<T, 4>(z, x, z, y); }
    [[nodiscard]] constexpr auto zxzz() const noexcept { return vector<T, 4>(z, x, z, z); }
    [[nodiscard]] constexpr auto zxzw() const noexcept { return vector<T, 4>(z, x, z, w); }
    [[nodiscard]] constexpr auto zxwx() const noexcept { return vector<T, 4>(z, x, w, x); }
    [[nodiscard]] constexpr auto zxwy() const noexcept { return vector<T, 4>(z, x, w, y); }
    [[nodiscard]] constexpr auto zxwz() const noexcept { return vector<T, 4>(z, x, w, z); }
    [[nodiscard]] constexpr auto zxww() const noexcept { return vector<T, 4>(z, x, w, w); }
    [[nodiscard]] constexpr auto zyxx() const noexcept { return vector<T, 4>(z, y, x, x); }
    [[nodiscard]] constexpr auto zyxy() const noexcept { return vector<T, 4>(z, y, x, y); }
    [[nodiscard]] constexpr auto zyxz() const noexcept { return vector<T, 4>(z, y, x, z); }
    [[nodiscard]] constexpr auto zyxw() const noexcept { return vector<T, 4>(z, y, x, w); }
    [[nodiscard]] constexpr auto zyyx() const noexcept { return vector<T, 4>(z, y, y, x); }
    [[nodiscard]] constexpr auto zyyy() const noexcept { return vector<T, 4>(z, y, y, y); }
    [[nodiscard]] constexpr auto zyyz() const noexcept { return vector<T, 4>(z, y, y, z); }
    [[nodiscard]] constexpr auto zyyw() const noexcept { return vector<T, 4>(z, y, y, w); }
    [[nodiscard]] constexpr auto zyzx() const noexcept { return vector<T, 4>(z, y, z, x); }
    [[nodiscard]] constexpr auto zyzy() const noexcept { return vector<T, 4>(z, y, z, y); }
    [[nodiscard]] constexpr auto zyzz() const noexcept { return vector<T, 4>(z, y, z, z); }
    [[nodiscard]] constexpr auto zyzw() const noexcept { return vector<T, 4>(z, y, z, w); }
    [[nodiscard]] constexpr auto zywx() const noexcept { return vector<T, 4>(z, y, w, x); }
    [[nodiscard]] constexpr auto zywy() const noexcept { return vector<T, 4>(z, y, w, y); }
    [[nodiscard]] constexpr auto zywz() const noexcept { return vector<T, 4>(z, y, w, z); }
    [[nodiscard]] constexpr auto zyww() const noexcept { return vector<T, 4>(z, y, w, w); }
    [[nodiscard]] constexpr auto zzxx() const noexcept { return vector<T, 4>(z, z, x, x); }
    [[nodiscard]] constexpr auto zzxy() const noexcept { return vector<T, 4>(z, z, x, y); }
    [[nodiscard]] constexpr auto zzxz() const noexcept { return vector<T, 4>(z, z, x, z); }
    [[nodiscard]] constexpr auto zzxw() const noexcept { return vector<T, 4>(z, z, x, w); }
    [[nodiscard]] constexpr auto zzyx() const noexcept { return vector<T, 4>(z, z, y, x); }
    [[nodiscard]] constexpr auto zzyy() const noexcept { return vector<T, 4>(z, z, y, y); }
    [[nodiscard]] constexpr auto zzyz() const noexcept { return vector<T, 4>(z, z, y, z); }
    [[nodiscard]] constexpr auto zzyw() const noexcept { return vector<T, 4>(z, z, y, w); }
    [[nodiscard]] constexpr auto zzzx() const noexcept { return vector<T, 4>(z, z, z, x); }
    [[nodiscard]] constexpr auto zzzy() const noexcept { return vector<T, 4>(z, z, z, y); }
    [[nodiscard]] constexpr auto zzzz() const noexcept { return vector<T, 4>(z, z, z, z); }
    [[nodiscard]] constexpr auto zzzw() const noexcept { return vector<T, 4>(z, z, z, w); }
    [[nodiscard]] constexpr auto zzwx() const noexcept { return vector<T, 4>(z, z, w, x); }
    [[nodiscard]] constexpr auto zzwy() const noexcept { return vector<T, 4>(z, z, w, y); }
    [[nodiscard]] constexpr auto zzwz() const noexcept { return vector<T, 4>(z, z, w, z); }
    [[nodiscard]] constexpr auto zzww() const noexcept { return vector<T, 4>(z, z, w, w); }
    [[nodiscard]] constexpr auto zwxx() const noexcept { return vector<T, 4>(z, w, x, x); }
    [[nodiscard]] constexpr auto zwxy() const noexcept { return vector<T, 4>(z, w, x, y); }
    [[nodiscard]] constexpr auto zwxz() const noexcept { return vector<T, 4>(z, w, x, z); }
    [[nodiscard]] constexpr auto zwxw() const noexcept { return vector<T, 4>(z, w, x, w); }
    [[nodiscard]] constexpr auto zwyx() const noexcept { return vector<T, 4>(z, w, y, x); }
    [[nodiscard]] constexpr auto zwyy() const noexcept { return vector<T, 4>(z, w, y, y); }
    [[nodiscard]] constexpr auto zwyz() const noexcept { return vector<T, 4>(z, w, y, z); }
    [[nodiscard]] constexpr auto zwyw() const noexcept { return vector<T, 4>(z, w, y, w); }
    [[nodiscard]] constexpr auto zwzx() const noexcept { return vector<T, 4>(z, w, z, x); }
    [[nodiscard]] constexpr auto zwzy() const noexcept { return vector<T, 4>(z, w, z, y); }
    [[nodiscard]] constexpr auto zwzz() const noexcept { return vector<T, 4>(z, w, z, z); }
    [[nodiscard]] constexpr auto zwzw() const noexcept { return vector<T, 4>(z, w, z, w); }
    [[nodiscard]] constexpr auto zwwx() const noexcept { return vector<T, 4>(z, w, w, x); }
    [[nodiscard]] constexpr auto zwwy() const noexcept { return vector<T, 4>(z, w, w, y); }
    [[nodiscard]] constexpr auto zwwz() const noexcept { return vector<T, 4>(z, w, w, z); }
    [[nodiscard]] constexpr auto zwww() const noexcept { return vector<T, 4>(z, w, w, w); }
    [[nodiscard]] constexpr auto wxxx() const noexcept { return vector<T, 4>(w, x, x, x); }
    [[nodiscard]] constexpr auto wxxy() const noexcept { return vector<T, 4>(w, x, x, y); }
    [[nodiscard]] constexpr auto wxxz() const noexcept { return vector<T, 4>(w, x, x, z); }
    [[nodiscard]] constexpr auto wxxw() const noexcept { return vector<T, 4>(w, x, x, w); }
    [[nodiscard]] constexpr auto wxyx() const noexcept { return vector<T, 4>(w, x, y, x); }
    [[nodiscard]] constexpr auto wxyy() const noexcept { return vector<T, 4>(w, x, y, y); }
    [[nodiscard]] constexpr auto wxyz() const noexcept { return vector<T, 4>(w, x, y, z); }
    [[nodiscard]] constexpr auto wxyw() const noexcept { return vector<T, 4>(w, x, y, w); }
    [[nodiscard]] constexpr auto wxzx() const noexcept { return vector<T, 4>(w, x, z, x); }
    [[nodiscard]] constexpr auto wxzy() const noexcept { return vector<T, 4>(w, x, z, y); }
    [[nodiscard]] constexpr auto wxzz() const noexcept { return vector<T, 4>(w, x, z, z); }
    [[nodiscard]] constexpr auto wxzw() const noexcept { return vector<T, 4>(w, x, z, w); }
    [[nodiscard]] constexpr auto wxwx() const noexcept { return vector<T, 4>(w, x, w, x); }
    [[nodiscard]] constexpr auto wxwy() const noexcept { return vector<T, 4>(w, x, w, y); }
    [[nodiscard]] constexpr auto wxwz() const noexcept { return vector<T, 4>(w, x, w, z); }
    [[nodiscard]] constexpr auto wxww() const noexcept { return vector<T, 4>(w, x, w, w); }
    [[nodiscard]] constexpr auto wyxx() const noexcept { return vector<T, 4>(w, y, x, x); }
    [[nodiscard]] constexpr auto wyxy() const noexcept { return vector<T, 4>(w, y, x, y); }
    [[nodiscard]] constexpr auto wyxz() const noexcept { return vector<T, 4>(w, y, x, z); }
    [[nodiscard]] constexpr auto wyxw() const noexcept { return vector<T, 4>(w, y, x, w); }
    [[nodiscard]] constexpr auto wyyx() const noexcept { return vector<T, 4>(w, y, y, x); }
    [[nodiscard]] constexpr auto wyyy() const noexcept { return vector<T, 4>(w, y, y, y); }
    [[nodiscard]] constexpr auto wyyz() const noexcept { return vector<T, 4>(w, y, y, z); }
    [[nodiscard]] constexpr auto wyyw() const noexcept { return vector<T, 4>(w, y, y, w); }
    [[nodiscard]] constexpr auto wyzx() const noexcept { return vector<T, 4>(w, y, z, x); }
    [[nodiscard]] constexpr auto wyzy() const noexcept { return vector<T, 4>(w, y, z, y); }
    [[nodiscard]] constexpr auto wyzz() const noexcept { return vector<T, 4>(w, y, z, z); }
    [[nodiscard]] constexpr auto wyzw() const noexcept { return vector<T, 4>(w, y, z, w); }
    [[nodiscard]] constexpr auto wywx() const noexcept { return vector<T, 4>(w, y, w, x); }
    [[nodiscard]] constexpr auto wywy() const noexcept { return vector<T, 4>(w, y, w, y); }
    [[nodiscard]] constexpr auto wywz() const noexcept { return vector<T, 4>(w, y, w, z); }
    [[nodiscard]] constexpr auto wyww() const noexcept { return vector<T, 4>(w, y, w, w); }
    [[nodiscard]] constexpr auto wzxx() const noexcept { return vector<T, 4>(w, z, x, x); }
    [[nodiscard]] constexpr auto wzxy() const noexcept { return vector<T, 4>(w, z, x, y); }
    [[nodiscard]] constexpr auto wzxz() const noexcept { return vector<T, 4>(w, z, x, z); }
    [[nodiscard]] constexpr auto wzxw() const noexcept { return vector<T, 4>(w, z, x, w); }
    [[nodiscard]] constexpr auto wzyx() const noexcept { return vector<T, 4>(w, z, y, x); }
    [[nodiscard]] constexpr auto wzyy() const noexcept { return vector<T, 4>(w, z, y, y); }
    [[nodiscard]] constexpr auto wzyz() const noexcept { return vector<T, 4>(w, z, y, z); }
    [[nodiscard]] constexpr auto wzyw() const noexcept { return vector<T, 4>(w, z, y, w); }
    [[nodiscard]] constexpr auto wzzx() const noexcept { return vector<T, 4>(w, z, z, x); }
    [[nodiscard]] constexpr auto wzzy() const noexcept { return vector<T, 4>(w, z, z, y); }
    [[nodiscard]] constexpr auto wzzz() const noexcept { return vector<T, 4>(w, z, z, z); }
    [[nodiscard]] constexpr auto wzzw() const noexcept { return vector<T, 4>(w, z, z, w); }
    [[nodiscard]] constexpr auto wzwx() const noexcept { return vector<T, 4>(w, z, w, x); }
    [[nodiscard]] constexpr auto wzwy() const noexcept { return vector<T, 4>(w, z, w, y); }
    [[nodiscard]] constexpr auto wzwz() const noexcept { return vector<T, 4>(w, z, w, z); }
    [[nodiscard]] constexpr auto wzww() const noexcept { return vector<T, 4>(w, z, w, w); }
    [[nodiscard]] constexpr auto wwxx() const noexcept { return vector<T, 4>(w, w, x, x); }
    [[nodiscard]] constexpr auto wwxy() const noexcept { return vector<T, 4>(w, w, x, y); }
    [[nodiscard]] constexpr auto wwxz() const noexcept { return vector<T, 4>(w, w, x, z); }
    [[nodiscard]] constexpr auto wwxw() const noexcept { return vector<T, 4>(w, w, x, w); }
    [[nodiscard]] constexpr auto wwyx() const noexcept { return vector<T, 4>(w, w, y, x); }
    [[nodiscard]] constexpr auto wwyy() const noexcept { return vector<T, 4>(w, w, y, y); }
    [[nodiscard]] constexpr auto wwyz() const noexcept { return vector<T, 4>(w, w, y, z); }
    [[nodiscard]] constexpr auto wwyw() const noexcept { return vector<T, 4>(w, w, y, w); }
    [[nodiscard]] constexpr auto wwzx() const noexcept { return vector<T, 4>(w, w, z, x); }
    [[nodiscard]] constexpr auto wwzy() const noexcept { return vector<T, 4>(w, w, z, y); }
    [[nodiscard]] constexpr auto wwzz() const noexcept { return vector<T, 4>(w, w, z, z); }
    [[nodiscard]] constexpr auto wwzw() const noexcept { return vector<T, 4>(w, w, z, w); }
    [[nodiscard]] constexpr auto wwwx() const noexcept { return vector<T, 4>(w, w, w, x); }
    [[nodiscard]] constexpr auto wwwy() const noexcept { return vector<T, 4>(w, w, w, y); }
    [[nodiscard]] constexpr auto wwwz() const noexcept { return vector<T, 4>(w, w, w, z); }
    [[nodiscard]] constexpr auto wwww() const noexcept { return vector<T, 4>(w, w, w, w); }
    /* <<<PYMACROEND>>> */
    // clang-format on

};

using bool1 = vector<bool, 1>;
using bool2 = vector<bool, 2>;
using bool3 = vector<bool, 3>;
using bool4 = vector<bool, 4>;
using int1 = vector<int, 1>;
using int2 = vector<int, 2>;
using int3 = vector<int, 3>;
using int4 = vector<int, 4>;
using uint1 = vector<uint, 1>;
using uint2 = vector<uint, 2>;
using uint3 = vector<uint, 3>;
using uint4 = vector<uint, 4>;
using float1 = vector<float, 1>;
using float2 = vector<float, 2>;
using float3 = vector<float, 3>;
using float4 = vector<float, 4>;
using float16_t1 = vector<float16_t, 1>;
using float16_t2 = vector<float16_t, 2>;
using float16_t3 = vector<float16_t, 3>;
using float16_t4 = vector<float16_t, 4>;
using double1 = vector<double, 1>;
using double2 = vector<double, 2>;
using double3 = vector<double, 3>;
using double4 = vector<double, 4>;

} // namespace math

using bool1 = math::bool1;
using bool2 = math::bool2;
using bool3 = math::bool3;
using bool4 = math::bool4;
using int1 = math::int1;
using int2 = math::int2;
using int3 = math::int3;
using int4 = math::int4;
using uint1 = math::uint1;
using uint2 = math::uint2;
using uint3 = math::uint3;
using uint4 = math::uint4;
using float1 = math::float1;
using float2 = math::float2;
using float3 = math::float3;
using float4 = math::float4;
using float16_t1 = math::float16_t1;
using float16_t2 = math::float16_t2;
using float16_t3 = math::float16_t3;
using float16_t4 = math::float16_t4;
using double1 = math::double1;
using double2 = math::double2;
using double3 = math::double3; 
using double4 = math::double4;

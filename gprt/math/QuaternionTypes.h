#pragma once

#include "ScalarTypes.h"
#include "VectorTypes.h"

#include <type_traits>

namespace math
{

/**
 * Quaternion type.
 *
 * A quaternion is an expression of the form:
 *
 * q = w + xi + yj + zk
 *
 * where w, x, y, and z are real numbers and i, j, and k are the imaginary units.
 *
 * The quaternion is normalized if:
 * w^2 + x^2 + y^2 + z^2 = 1
 *
 * Quaternions are stored as (x, y, z, w) to make them better for interop with the GPU.
 */
template<typename T>
struct quat
{
    using value_type = T;
    static_assert(std::disjunction_v<std::is_same<T, float>, std::is_same<T, double>>, "Invalid quaternion type");

    T x, y, z, w;

    quat() : x{T(0)}, y{T(0)}, z{T(0)}, w{T(1)} {}

    explicit quat(const vector<T, 3>& xyz, const T& w) : x{xyz.x}, y{xyz.y}, z{xyz.z}, w{w} {}
    explicit quat(const T& x, const T& y, const T& z, const T& w) : x{x}, y{y}, z{z}, w{w} {}

    /// Identity quaternion.
    [[nodiscard]] static quat identity() { return quat(T(0), T(0), T(0), T(1)); }

    // Accesses
    value_type& operator[](size_t i) { return (&x)[i]; }
    const value_type& operator[](size_t i) const { return (&x)[i]; }
};

using quatf = quat<float>;

} // namespace math

using quatf = math::quatf;


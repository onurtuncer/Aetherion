// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Math.h
//
// Common CppAD-friendly math helpers for Aerodynamics headers.
//
// Notes:
// - Uses CppAD math functions explicitly (CppAD::sqrt/sin/cos/atan2) to avoid
//   MSVC overload ambiguity that can happen when importing both std and CppAD.
// - Works with double/float and CppAD::AD<double> (and similar scalar types).
//

#pragma once

#include <array>
#include <cppad/cppad.hpp>

namespace Aetherion::Aerodynamics {

/// @brief 3-element column vector backed by std::array; compatible with CppAD scalar types.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
template <class Scalar>
using Vec3 = std::array<Scalar, 3>;

/// @brief 3x3 matrix backed by a row-major std::array of arrays; compatible with CppAD scalar types.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
template <class Scalar>
using Mat3 = std::array<std::array<Scalar, 3>, 3>;

// --- Scalar ops -----------------------------------------------------------

/// @brief Smooth (differentiable) approximation of |x| via hypot: sqrt(x^2 + eps^2).
/// @tparam Scalar Numeric scalar type.
/// @param x Input value.
/// @param eps Smoothing parameter; must be > 0.
/// @return Smooth approximation of |x|, always >= eps.
template <class Scalar>
inline Scalar SmoothAbs(const Scalar& x, const Scalar& eps)
{
    return CppAD::sqrt(x * x + eps * eps);
}

/// @brief Smooth (differentiable) approximation of max(x, 0).
/// @tparam Scalar Numeric scalar type.
/// @param x Input value.
/// @param eps Smoothing parameter; must be > 0.
/// @return Smooth approximation of max(x, 0), always >= 0.
template <class Scalar>
inline Scalar SmoothMax0(const Scalar& x, const Scalar& eps)
{
    return Scalar(0.5) * (x + SmoothAbs(x, eps));
}

/// @brief Numerically safe division: num / SmoothAbs(den, eps).
/// @tparam Scalar Numeric scalar type.
/// @param num Numerator.
/// @param den Denominator.
/// @param eps Smoothing parameter preventing division by zero; must be > 0.
/// @return Quotient with smoothed denominator.
template <class Scalar>
inline Scalar SafeDivide(const Scalar& num, const Scalar& den, const Scalar& eps)
{
    return num / SmoothAbs(den, eps);
}

/// @brief Smooth sqrt that clamps x to a non-negative value before taking the square root.
/// @tparam Scalar Numeric scalar type.
/// @param x Input value.
/// @param eps Smoothing parameter; must be > 0.
/// @return sqrt(max(x, 0)) approximated smoothly.
template <class Scalar>
inline Scalar SqrtSafe(const Scalar& x, const Scalar& eps)
{
    // Smooth clamp to non-negative then sqrt.
    return CppAD::sqrt(SmoothMax0(x, eps));
}

/// @brief CppAD-friendly sine.
/// @tparam Scalar Numeric scalar type.
/// @param x Angle in radians.
/// @return sin(x).
template <class Scalar>
inline Scalar Sin(const Scalar& x) { return CppAD::sin(x); }

/// @brief CppAD-friendly cosine.
/// @tparam Scalar Numeric scalar type.
/// @param x Angle in radians.
/// @return cos(x).
template <class Scalar>
inline Scalar Cos(const Scalar& x) { return CppAD::cos(x); }

/// @brief CppAD-friendly two-argument arctangent.
/// @tparam Scalar Numeric scalar type.
/// @param y Y component.
/// @param x X component.
/// @return atan2(y, x) in radians, in range (-pi, pi].
template <class Scalar>
inline Scalar Atan2(const Scalar& y, const Scalar& x) { return CppAD::atan2(y, x); }

// --- Vector / matrix ops --------------------------------------------------

/// @brief Dot product of two 3-vectors.
/// @tparam Scalar Numeric scalar type.
/// @param a First vector.
/// @param b Second vector.
/// @return Scalar dot product a · b.
template <class Scalar>
inline Scalar Dot(const Vec3<Scalar>& a, const Vec3<Scalar>& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/// @brief Cross product of two 3-vectors.
/// @tparam Scalar Numeric scalar type.
/// @param a First vector.
/// @param b Second vector.
/// @return 3-vector a × b.
template <class Scalar>
inline Vec3<Scalar> Cross(const Vec3<Scalar>& a, const Vec3<Scalar>& b)
{
    return Vec3<Scalar>{
        a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
    };
}

/// @brief Euclidean speed (2-norm) of a 3D velocity vector, with smooth regularisation.
/// @tparam Scalar Numeric scalar type.
/// @param v Velocity vector [m/s].
/// @param eps Smoothing parameter to avoid sqrt(0) singularity; default 1e-12.
/// @return ||v||_2 regularised by eps.
template <class Scalar>
inline Scalar SpeedFromVelocity(const Vec3<Scalar>& v, const Scalar& eps = Scalar(1e-12))
{
    return CppAD::sqrt(Dot(v, v) + eps * eps);
}

/// @brief Matrix-vector product A * x for a 3x3 matrix and a 3-vector.
/// @tparam Scalar Numeric scalar type.
/// @param A Row-major 3x3 matrix.
/// @param x Column vector (length 3).
/// @return 3-vector result of A * x.
template <class Scalar>
inline Vec3<Scalar> MatVecMul(const Mat3<Scalar>& A, const Vec3<Scalar>& x)
{
    return Vec3<Scalar>{
        A[0][0] * x[0] + A[0][1] * x[1] + A[0][2] * x[2],
            A[1][0] * x[0] + A[1][1] * x[1] + A[1][2] * x[2],
            A[2][0] * x[0] + A[2][1] * x[1] + A[2][2] * x[2],
    };
}

// --- Aero scalars ---------------------------------------------------------

/// @brief Dynamic pressure: q = 0.5 * rho * V^2.
/// @tparam Scalar Numeric scalar type.
/// @param density_kg_m3 Air density [kg/m^3].
/// @param speed_m_s Airspeed magnitude [m/s].
/// @return Dynamic pressure [Pa].
template <class Scalar>
inline Scalar DynamicPressure(const Scalar& density_kg_m3, const Scalar& speed_m_s)
{
    return Scalar(0.5) * density_kg_m3 * speed_m_s * speed_m_s;
}

} // namespace Aetherion::Aerodynamics

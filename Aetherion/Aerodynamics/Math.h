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

    template <class Scalar>
    using Vec3 = std::array<Scalar, 3>;

    template <class Scalar>
    using Mat3 = std::array<std::array<Scalar, 3>, 3>;

    // --- Scalar ops -----------------------------------------------------------

    template <class Scalar>
    inline Scalar SmoothAbs(const Scalar& x, const Scalar& eps)
    {
        return CppAD::sqrt(x * x + eps * eps);
    }

    template <class Scalar>
    inline Scalar SmoothMax0(const Scalar& x, const Scalar& eps)
    {
        return Scalar(0.5) * (x + SmoothAbs(x, eps));
    }

    template <class Scalar>
    inline Scalar SafeDivide(const Scalar& num, const Scalar& den, const Scalar& eps)
    {
        return num / SmoothAbs(den, eps);
    }

    template <class Scalar>
    inline Scalar SqrtSafe(const Scalar& x, const Scalar& eps)
    {
        // Smooth clamp to non-negative then sqrt.
        return CppAD::sqrt(SmoothMax0(x, eps));
    }

    template <class Scalar>
    inline Scalar Sin(const Scalar& x) { return CppAD::sin(x); }

    template <class Scalar>
    inline Scalar Cos(const Scalar& x) { return CppAD::cos(x); }

    template <class Scalar>
    inline Scalar Atan2(const Scalar& y, const Scalar& x) { return CppAD::atan2(y, x); }

    // --- Vector / matrix ops --------------------------------------------------

    template <class Scalar>
    inline Scalar Dot(const Vec3<Scalar>& a, const Vec3<Scalar>& b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    template <class Scalar>
    inline Vec3<Scalar> Cross(const Vec3<Scalar>& a, const Vec3<Scalar>& b)
    {
        return Vec3<Scalar>{
            a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
        };
    }

    template <class Scalar>
    inline Scalar SpeedFromVelocity(const Vec3<Scalar>& v, const Scalar& eps = Scalar(1e-12))
    {
        return CppAD::sqrt(Dot(v, v) + eps * eps);
    }

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

    template <class Scalar>
    inline Scalar DynamicPressure(const Scalar& density_kg_m3, const Scalar& speed_m_s)
    {
        return Scalar(0.5) * density_kg_m3 * speed_m_s * speed_m_s;
    }

} // namespace Aetherion::Aerodynamics

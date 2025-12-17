// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

// CppAD-friendly Mach number utilities.
//
// Notes:
// - All functions are templated on Scalar so they work with double, float,
//   CppAD::AD<double>, etc.
// - No `if` branches; “safety” is handled with smooth approximations.
// - Units: velocity [m/s], temperature [K], pressure [Pa], density [kg/m^3].
// 
#pragma once

#include <array>
#include <cmath>
#include <cppad/cppad.hpp>

namespace Aetherion::Aerodynamics {

    template <class Scalar>
    using Vec3 = std::array<Scalar, 3>;

    // Bring both std and CppAD overloads into scope (works for plain and AD scalars).
    namespace detail {
        using std::sqrt;
        using CppAD::sqrt;
    }

    // Smooth |x| ≈ sqrt(x^2 + eps^2). Differentiable everywhere.
    template <class Scalar>
    inline Scalar SmoothAbs(const Scalar& x, const Scalar& eps)
    {
        return detail::sqrt(x * x + eps * eps);
    }

    // Smooth max(x, 0) ≈ 0.5 * (x + |x|_smooth). Differentiable everywhere.
    template <class Scalar>
    inline Scalar SmoothMax0(const Scalar& x, const Scalar& eps)
    {
        return Scalar(0.5) * (x + SmoothAbs(x, eps));
    }

    // Safe divide: num / (|den|_smooth). Differentiable and avoids den=0.
    template <class Scalar>
    inline Scalar SafeDivide(const Scalar& num, const Scalar& den, const Scalar& eps)
    {
        return num / SmoothAbs(den, eps);
    }

    // Euclidean speed from a 3D velocity vector.
    template <class Scalar>
    inline Scalar SpeedFromVelocity(const Vec3<Scalar>& v, const Scalar& eps = Scalar(1e-12))
    {
        const Scalar v2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
        // sqrt(v^2 + eps^2) to keep derivatives finite at v=0.
        return detail::sqrt(v2 + eps * eps);
    }

    // Speed of sound from temperature: a = sqrt(gamma * R * T).
    // Uses SmoothMax0(T) to prevent sqrt of negative temps during optimization.
    template <class Scalar>
    inline Scalar SpeedOfSoundFromTemperature(
        const Scalar& gamma,
        const Scalar& R,
        const Scalar& temperature_K,
        const Scalar& eps = Scalar(1e-12))
    {
        const Scalar Tpos = SmoothMax0(temperature_K, eps);
        return detail::sqrt(gamma * R * Tpos);
    }

    // Speed of sound from pressure and density: a = sqrt(gamma * p / rho).
    // Uses SafeDivide to avoid rho=0.
    template <class Scalar>
    inline Scalar SpeedOfSoundFromPressureDensity(
        const Scalar& gamma,
        const Scalar& pressure_Pa,
        const Scalar& density_kg_m3,
        const Scalar& eps = Scalar(1e-12))
    {
        const Scalar p_over_rho = SafeDivide(pressure_Pa, density_kg_m3, eps);
        return detail::sqrt(gamma * p_over_rho);
    }

    // Mach number from speed and speed of sound: M = V / a.
    template <class Scalar>
    inline Scalar MachFromSpeedSoundSpeed(
        const Scalar& speed_m_s,
        const Scalar& sound_speed_m_s,
        const Scalar& eps = Scalar(1e-12))
    {
        return SafeDivide(speed_m_s, sound_speed_m_s, eps);
    }

} // namespace Aetherion::Aerodynamics

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// MachNumber.h
//
// CppAD-friendly Mach number utilities.
//
// Units: velocity [m/s], temperature [K], pressure [Pa], density [kg/m^3].
//

#pragma once

#include <Aetherion/Aerodynamics/Math.h>

namespace Aetherion::Aerodynamics {

/// @brief Speed of sound from static temperature using the ideal-gas relation: a = sqrt(gamma * R * T).
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param gamma Ratio of specific heats (dimensionless); ~1.4 for air.
/// @param R Specific gas constant [J/(kg·K)]; ~287.05 for dry air.
/// @param temperature_K Static temperature [K].
/// @param eps Smoothing parameter to prevent sqrt of negative temperature; default 1e-12.
/// @return Speed of sound [m/s].
template <class Scalar>
inline Scalar SpeedOfSoundFromTemperature(
    const Scalar& gamma,
    const Scalar& R,
    const Scalar& temperature_K,
    const Scalar& eps = Scalar(1e-12))
{
    const Scalar Tpos = SmoothMax0(temperature_K, eps);
    return CppAD::sqrt(gamma * R * Tpos);
}

/// @brief Speed of sound from static pressure and density: a = sqrt(gamma * p / rho).
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param gamma Ratio of specific heats (dimensionless); ~1.4 for air.
/// @param pressure_Pa Static pressure [Pa].
/// @param density_kg_m3 Air density [kg/m^3].
/// @param eps Smoothing parameter to avoid division by near-zero density; default 1e-12.
/// @return Speed of sound [m/s].
template <class Scalar>
inline Scalar SpeedOfSoundFromPressureDensity(
    const Scalar& gamma,
    const Scalar& pressure_Pa,
    const Scalar& density_kg_m3,
    const Scalar& eps = Scalar(1e-12))
{
    const Scalar p_over_rho = SafeDivide(pressure_Pa, density_kg_m3, eps);
    return CppAD::sqrt(gamma * p_over_rho);
}

/// @brief Mach number from airspeed and speed of sound: M = V / a.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param speed_m_s Airspeed magnitude [m/s].
/// @param sound_speed_m_s Local speed of sound [m/s].
/// @param eps Smoothing parameter to avoid division by near-zero sound speed; default 1e-12.
/// @return Mach number (dimensionless).
template <class Scalar>
inline Scalar MachFromSpeedSoundSpeed(
    const Scalar& speed_m_s,
    const Scalar& sound_speed_m_s,
    const Scalar& eps = Scalar(1e-12))
{
    return SafeDivide(speed_m_s, sound_speed_m_s, eps);
}

/// @brief Mach number from a 3D velocity vector and speed of sound.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param v_m_s Velocity vector in any consistent frame [m/s].
/// @param sound_speed_m_s Local speed of sound [m/s].
/// @param eps Smoothing parameter for both speed norm and division; default 1e-12.
/// @return Mach number (dimensionless).
template <class Scalar>
inline Scalar MachFromVelocityAndSoundSpeed(
    const Vec3<Scalar>& v_m_s,
    const Scalar& sound_speed_m_s,
    const Scalar& eps = Scalar(1e-12))
{
    const Scalar V = SpeedFromVelocity(v_m_s, eps);
    return MachFromSpeedSoundSpeed(V, sound_speed_m_s, eps);
}

} // namespace Aetherion::Aerodynamics

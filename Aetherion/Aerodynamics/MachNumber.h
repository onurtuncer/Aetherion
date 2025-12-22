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

    template <class Scalar>
    inline Scalar MachFromSpeedSoundSpeed(
        const Scalar& speed_m_s,
        const Scalar& sound_speed_m_s,
        const Scalar& eps = Scalar(1e-12))
    {
        return SafeDivide(speed_m_s, sound_speed_m_s, eps);
    }

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


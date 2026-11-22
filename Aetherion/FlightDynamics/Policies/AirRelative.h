// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// AirRelative.h
//
// The one place where a vehicle policy turns the ECI state into what the air
// sees.  Every policy that needs an airspeed, an angle of attack or a rate
// relative to the air mass (F16AeroPolicy, F16PropPolicy, RocketAeroPolicy)
// calls these two functions, so wind, gust and the Earth's rotation enter the
// aerodynamics and the propulsion identically.  Before 0.16.0 each policy
// carried its own copy of the Earth-surface term; the propulsion copy did
// not know about wind and formed a different Mach number from the trim's.
//
//     v_rel     = v_B - R^T (omega_E x r) - R^T R_ECEF->ECI(t) v_wind_ECEF - v_gust_B
//     omega_air = omega_B - R^T omega_E - omega_gust_B
//
// Calm air is bit-identical to a policy without the wind and gust terms: the
// subtractions are skipped when the vectors are zero, so nothing changes
// numerically and nothing extra lands on an AD tape.
//
// The six gust components are read as tape parameters
// (ODE/RKMK/Core/TapeParameters.h), and the "is there a gust" test is made on
// the scalar type. On an ordinary tape that is the constant and the plain
// comparison it always was. On a tape kept from one step to the next the gust
// is a dynamic parameter and the test is recorded, so a gust that appears or
// vanishes re-records the tape instead of going unnoticed.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Environment/DrydenTurbulence.h> // GustState
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Environment/detail/MathWrappers.h>
#include <Aetherion/ODE/RKMK/Core/TapeParameters.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Dense>

namespace Aetherion::FlightDynamics {

    /// @brief Air-relative velocity in body axes.
    /// @param g          Pose (body → ECI rotation R, ECI position p).
    /// @param nu_B       Body twist [ω_B; v_B], ECI-relative, body axes.
    /// @param t          Time [s]; the Earth rotation angle is ω_E t.
    /// @param windECEF   Steady wind as an ECEF vector [m/s] (zero = calm).
    /// @param gust       Gust in body axes; only its linear part is used here.
    template <class S>
    [[nodiscard]] Eigen::Matrix<S, 3, 1>
    AirRelativeVelocity_B(const ODE::RKMK::Lie::SE3<S>& g,
        const Eigen::Matrix<S, 6, 1>& nu_B,
        const S& t,
        const Eigen::Vector3d& windECEF,
        const Environment::GustState& gust)
    {
        using Environment::detail::Sine;
        using Environment::detail::Cosine;

        constexpr double kOmegaE = Environment::WGS84::kRotationRate_rad_s;
        const Eigen::Matrix<S, 3, 1> omega_E(S(0), S(0), S(kOmegaE));
        const Eigen::Matrix<S, 3, 1> v_surface = g.R.transpose() * omega_E.cross(g.p);
        Eigen::Matrix<S, 3, 1> v_rel = nu_B.template tail<3>() - v_surface;

        if (windECEF.squaredNorm() > 0.0) {
            const S theta = S(kOmegaE) * t;
            const S ct = Cosine(theta), st = Sine(theta);
            const Eigen::Matrix<S, 3, 1> v_wind_eci(
                ct * S(windECEF.x()) - st * S(windECEF.y()),
                st * S(windECEF.x()) + ct * S(windECEF.y()),
                S(windECEF.z()));
            v_rel -= g.R.transpose() * v_wind_eci;
        }
        const S gust_u = ODE::RKMK::Core::TapeParameter<S>(gust.u_mps);
        const S gust_v = ODE::RKMK::Core::TapeParameter<S>(gust.v_mps);
        const S gust_w = ODE::RKMK::Core::TapeParameter<S>(gust.w_mps);
        if (gust_u != S(0.0) || gust_v != S(0.0) || gust_w != S(0.0)) {
            v_rel(0) -= gust_u;
            v_rel(1) -= gust_v;
            v_rel(2) -= gust_w;
        }
        return v_rel;
    }

    /// @brief Body angular rate relative to the air mass: the ECI-relative rate
    /// minus the Earth rate (the atmosphere rotates with the Earth) minus the
    /// angular velocity of the gust field.
    template <class S>
    [[nodiscard]] Eigen::Matrix<S, 3, 1>
    AirRelativeRates_B(const ODE::RKMK::Lie::SE3<S>& g,
        const Eigen::Matrix<S, 6, 1>& nu_B,
        const Environment::GustState& gust)
    {
        constexpr double kOmegaE = Environment::WGS84::kRotationRate_rad_s;
        const Eigen::Matrix<S, 3, 1> omega_E(S(0), S(0), S(kOmegaE));
        Eigen::Matrix<S, 3, 1> omega_B = nu_B.template head<3>() - g.R.transpose() * omega_E;
        const S gust_p = ODE::RKMK::Core::TapeParameter<S>(gust.p_rad_s);
        const S gust_q = ODE::RKMK::Core::TapeParameter<S>(gust.q_rad_s);
        const S gust_r = ODE::RKMK::Core::TapeParameter<S>(gust.r_rad_s);
        if (gust_p != S(0.0) || gust_q != S(0.0) || gust_r != S(0.0)) {
            omega_B(0) -= gust_p;
            omega_B(1) -= gust_q;
            omega_B(2) -= gust_r;
        }
        return omega_B;
    }

} // namespace Aetherion::FlightDynamics

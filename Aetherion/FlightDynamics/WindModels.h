// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// WindModels.h
//
// Policy concept and concrete wind models for use with WindAwareDragPolicy.
//
// A WindModel must expose:
//
//   template<class S>
//   Eigen::Matrix<S,3,1> velocity_ecef(S h_m, S t_s) const;
//
// returning the wind velocity in the ECEF frame [m/s] at geocentric altitude
// h_m [m] and simulation time t_s [s].  The method must be callable for both
// S = double and S = CppAD::AD<double>.
//
// Provided models
// ───────────────
//   ZeroWind             — calm atmosphere, always returns (0,0,0)
//   ConstantECEFWind     — uniform wind, fixed ECEF vector
//   PowerLawWindShear    — magnitude ∝ (h/h_ref)^n, fixed ECEF direction
//
// To add a custom profile, create a struct satisfying the WindModel concept
// and pass it as the WindModel template parameter of WindAwareDragPolicy.
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Environment/detail/MathWrappers.h>
#include <concepts>

namespace Aetherion::FlightDynamics {

// ─────────────────────────────────────────────────────────────────────────────
// WindModel concept
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Concept satisfied by any type that provides a wind velocity in ECEF.
///
/// The method must be templated on scalar type S (works for double and
/// CppAD::AD<double>).
template<class W>
concept WindModel = requires(const W& w) {
    { w.template velocity_ecef<double>(double{}, double{}) }
        -> std::convertible_to<Eigen::Matrix<double, 3, 1>>;
};

// ─────────────────────────────────────────────────────────────────────────────
// ZeroWind
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Calm atmosphere — always returns zero wind.
struct ZeroWind {
    template<class S>
    Eigen::Matrix<S, 3, 1> velocity_ecef(S /*h_m*/, S /*t_s*/) const {
        return Eigen::Matrix<S, 3, 1>::Zero();
    }
};

static_assert(WindModel<ZeroWind>);

// ─────────────────────────────────────────────────────────────────────────────
// ConstantECEFWind
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Uniform, time-invariant wind — fixed vector in ECEF frame [m/s].
///
/// The ECEF wind vector is obtained from a NED wind specification at a given
/// (lat, lon) via the constructor helper @c from_ned().
struct ConstantECEFWind {
    double vx{ 0.0 }; ///< ECEF X component [m/s].
    double vy{ 0.0 }; ///< ECEF Y component [m/s].
    double vz{ 0.0 }; ///< ECEF Z component [m/s].

    ConstantECEFWind() = default;
    ConstantECEFWind(double x, double y, double z) : vx(x), vy(y), vz(z) {}

    /// @brief Construct from NED wind components at a known geodetic position.
    static ConstantECEFWind from_ned(
        double north_mps, double east_mps, double down_mps,
        double lat_rad, double lon_rad)
    {
        const double sLat = std::sin(lat_rad), cLat = std::cos(lat_rad);
        const double sLon = std::sin(lon_rad), cLon = std::cos(lon_rad);
        return {
            north_mps * (-sLat*cLon) + east_mps * (-sLon) + down_mps * (-cLat*cLon),
            north_mps * (-sLat*sLon) + east_mps * ( cLon) + down_mps * (-cLat*sLon),
            north_mps * ( cLat)                            + down_mps * (-sLat)
        };
    }

    template<class S>
    Eigen::Matrix<S, 3, 1> velocity_ecef(S /*h_m*/, S /*t_s*/) const {
        return Eigen::Matrix<S, 3, 1>{ S(vx), S(vy), S(vz) };
    }
};

static_assert(WindModel<ConstantECEFWind>);

// ─────────────────────────────────────────────────────────────────────────────
// PowerLawWindShear
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Altitude-dependent wind following a power-law shear profile.
///
/// The wind at geocentric altitude @p h is:
/// @f[ \mathbf{v}(h) = \mathbf{v}_\text{ref,ECEF} \cdot
///     \left(\frac{h}{h_\text{ref}}\right)^n @f]
///
/// The reference wind @p v_ref_ecef is the ECEF wind vector at @p h_ref_m.
/// Use @c from_ned() to construct from NED components at the launch lat/lon.
///
/// @note Setting @c shear_exp = 1.0 gives linear (constant-gradient) shear.
///       @c shear_exp ≈ 4/3 fits the NASA TM-2015-218675 Scenario 8 data.
struct PowerLawWindShear {
    double vx_ref   { 0.0 };    ///< ECEF X wind at h_ref [m/s].
    double vy_ref   { 0.0 };    ///< ECEF Y wind at h_ref [m/s].
    double vz_ref   { 0.0 };    ///< ECEF Z wind at h_ref [m/s].
    double h_ref_m  { 9144.0 }; ///< Reference altitude [m].
    double shear_exp{ 1.3333 }; ///< Power-law exponent n (4/3 ≈ 1.333).

    PowerLawWindShear() = default;
    PowerLawWindShear(double vx, double vy, double vz,
                      double h_ref, double n)
        : vx_ref(vx), vy_ref(vy), vz_ref(vz), h_ref_m(h_ref), shear_exp(n) {}

    /// @brief Construct from NED wind at reference altitude and launch lat/lon.
    static PowerLawWindShear from_ned(
        double north_mps, double east_mps, double down_mps,
        double lat_rad, double lon_rad,
        double h_ref_m, double shear_exp = 4.0/3.0)
    {
        auto c = ConstantECEFWind::from_ned(north_mps, east_mps, down_mps,
                                            lat_rad, lon_rad);
        return { c.vx, c.vy, c.vz, h_ref_m, shear_exp };
    }

    template<class S>
    Eigen::Matrix<S, 3, 1> velocity_ecef(S h_m, S /*t_s*/) const {
        using Environment::detail::Power;
        const S scale = Power(h_m / S(h_ref_m), S(shear_exp));
        return Eigen::Matrix<S, 3, 1>{ S(vx_ref)*scale, S(vy_ref)*scale, S(vz_ref)*scale };
    }
};

static_assert(WindModel<PowerLawWindShear>);

} // namespace Aetherion::FlightDynamics

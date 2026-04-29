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
// WindModel concept and built-in implementations for WindAwareDragPolicy.
//
// INTERFACE CONTRACT
// ──────────────────
// A WindModel must provide a single templated method:
//
//   template<class S>
//   Eigen::Matrix<S,3,1> velocity_ecef(
//       const Eigen::Matrix<S,3,1>& r_eci,   // ECI position [m]
//       S t_s) const;                          // simulation time [s]
//
// The method returns the ambient wind velocity in the ECI frame [m/s].
// It must be callable for both S = double and S = CppAD::AD<double>.
//
// BUILT-IN MODELS
// ───────────────
//   ZeroWind             — calm atmosphere, always (0,0,0)
//   ConstantECEFWind     — uniform, time-invariant ECEF vector; from_ned() helper
//   PowerLawWindShear    — v ∝ (h/h_ref)^n in fixed ECEF direction; from_ned() helper
//   GeodesicCallbackWind — wraps any callable f(lat,lon,alt,t)→NED for weather
//                          APIs; AD path uses a frozen ECEF cache updated during
//                          each double evaluation.
//
// EXTENDING
// ─────────
// Implement the templated velocity_ecef(r_eci, t) method.  For AD-safe operation
// without external data (e.g. a lookup table), return an expression involving
// r_eci and t using only AD-compatible operations (arithmetic, SquareRoot, etc.).
// For external-data sources (weather APIs), see GeodesicCallbackWind.
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Environment/detail/MathWrappers.h>
#include <type_traits>

namespace Aetherion::FlightDynamics {

// ─────────────────────────────────────────────────────────────────────────────
// WindModel concept
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Concept satisfied by any type providing ECI wind velocity.
///
/// The implementor receives the full ECI position (allowing altitude- and
/// position-dependent models) and the current simulation time.
/// @brief Tag struct: specialise to std::true_type for each WindModel type.
///
/// All built-in models (ZeroWind, ConstantECEFWind, PowerLawWindShear,
/// GeodesicCallbackWind) are registered below.  User-defined wind models
/// must also specialise this to enable use with WindAwareDragPolicy:
///
///   namespace Aetherion::FlightDynamics {
///     template<> struct is_wind_model<MyWind> : std::true_type {};
///   }
template<class W> struct is_wind_model : std::false_type {};
template<class W> inline constexpr bool is_wind_model_v = is_wind_model<W>::value;

// ─────────────────────────────────────────────────────────────────────────────
// ZeroWind
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Calm atmosphere — always returns zero wind.
struct ZeroWind {
    template<class S>
    Eigen::Matrix<S, 3, 1>
    velocity_ecef(const Eigen::Matrix<S,3,1>& /*r_eci*/, S /*t*/) const {
        return Eigen::Matrix<S, 3, 1>::Zero();
    }
};
template<> struct is_wind_model<ZeroWind> : std::true_type {};

// ─────────────────────────────────────────────────────────────────────────────
// ConstantECEFWind
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Uniform, time-invariant wind — fixed ECEF vector [m/s].
struct ConstantECEFWind {
    double vx{ 0.0 }, vy{ 0.0 }, vz{ 0.0 };

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
            north_mps*(-sLat*cLon) + east_mps*(-sLon) + down_mps*(-cLat*cLon),
            north_mps*(-sLat*sLon) + east_mps*( cLon) + down_mps*(-cLat*sLon),
            north_mps*( cLat)                          + down_mps*(-sLat)
        };
    }

    template<class S>
    Eigen::Matrix<S, 3, 1>
    velocity_ecef(const Eigen::Matrix<S,3,1>& /*r_eci*/, S /*t*/) const {
        return Eigen::Matrix<S, 3, 1>{ S(vx), S(vy), S(vz) };
    }
};
template<> struct is_wind_model<ConstantECEFWind> : std::true_type {};

// ─────────────────────────────────────────────────────────────────────────────
// PowerLawWindShear
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Altitude-dependent wind: v(h) = v_ref × (h/h_ref)^n.
///
/// The geocentric altitude is derived from |r_eci| − Rₑ.  The direction
/// is fixed (stored as an ECEF unit vector scaled by v_ref magnitude).
/// Use @c from_ned() to specify the reference wind in the NED frame at the
/// launch lat/lon.
///
/// @note shear_exp ≈ 4/3 = 1.333 fits the NASA TM-2015-218675 Scenario 8 data.
struct PowerLawWindShear {
    double vx_ref   { 0.0 };
    double vy_ref   { 0.0 };
    double vz_ref   { 0.0 };
    double h_ref_m  { 9144.0 };
    double shear_exp{ 1.3333 };

    PowerLawWindShear() = default;
    PowerLawWindShear(double vx, double vy, double vz,
                      double h_ref, double n)
        : vx_ref(vx), vy_ref(vy), vz_ref(vz), h_ref_m(h_ref), shear_exp(n) {}

    static PowerLawWindShear from_ned(
        double north_mps, double east_mps, double down_mps,
        double lat_rad, double lon_rad,
        double h_ref_m, double shear_exp = 4.0/3.0)
    {
        auto c = ConstantECEFWind::from_ned(
            north_mps, east_mps, down_mps, lat_rad, lon_rad);
        return { c.vx, c.vy, c.vz, h_ref_m, shear_exp };
    }

    template<class S>
    Eigen::Matrix<S, 3, 1>
    velocity_ecef(const Eigen::Matrix<S,3,1>& r_eci, S /*t*/) const {
        using Environment::detail::Power;
        const S h     = r_eci.norm() - S(Environment::WGS84::kSemiMajorAxis_m);
        const S scale = Power(h / S(h_ref_m), S(shear_exp));
        return Eigen::Matrix<S, 3, 1>{ S(vx_ref)*scale,
                                       S(vy_ref)*scale,
                                       S(vz_ref)*scale };
    }
};
template<> struct is_wind_model<PowerLawWindShear> : std::true_type {};

// GeodesicCallbackWind (wraps a position-aware callback for weather APIs) is
// defined in a separate header to avoid pulling in heavy geodetic dependencies:
//   #include <Aetherion/FlightDynamics/GeodesicCallbackWind.h>

} // namespace Aetherion::FlightDynamics

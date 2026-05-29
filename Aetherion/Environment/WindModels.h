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
// Wind configuration structs and built-in functional wind models.
//
// CONFIGURATION STRUCTS (NED, serialisable)
// ──────────────────────────────────────────
//   ConstantWind    — ambient wind as NED components [m/s]; used in RigidBody::Config
//   WindShear       — linear altitude wind-shear coefficients in NED; used in RigidBody::Config
//
// WIND MODEL INTERFACE CONTRACT
// ──────────────────────────────
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
// BUILT-IN FUNCTIONAL MODELS
// ───────────────────────────
//   ZeroWind             — calm atmosphere, always (0,0,0)
//   ConstantECEFWind     — uniform, time-invariant ECEF vector; from_ned() helper
//   LinearWindShear      — v(h) = gradient × h + intercept in NED; from_ned() helper
//
// GeodesicCallbackWind (wraps any callable f(lat,lon,alt,t)→NED for weather APIs;
//   AD path uses a frozen ECEF cache updated during each double evaluation) is
//   defined in a separate header to avoid pulling in heavy geodetic dependencies:
//     #include <Aetherion/FlightDynamics/GeodesicCallbackWind.h>
//
// EXTENDING
// ─────────
// Implement the templated velocity_ecef(r_eci, t) method.  Register with:
//   namespace Aetherion::Environment {
//     template<> struct is_wind_model<MyWind> : std::true_type {};
//   }
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Environment/detail/MathWrappers.h>
#include <type_traits>
#include <cmath>

namespace Aetherion::Environment {

// ─────────────────────────────────────────────────────────────────────────────
// NED configuration structs (serialisable, used in RigidBody::Config)
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Constant ambient wind velocity in the local NED frame.
///
/// A positive East component means the wind blows eastward (meteorological
/// "westerly").  All components default to zero (calm atmosphere).
struct ConstantWind
{
    double north_mps{ 0.0 }; ///< Northward wind component [m/s].
    double east_mps { 0.0 }; ///< Eastward  wind component [m/s].
    double down_mps { 0.0 }; ///< Downward  wind component [m/s] (usually zero).
};

/// @brief Linear altitude wind-shear configuration (NED frame).
///
/// Wind at geocentric altitude h [m]:
///   v_N(h) = gradient_N_mps_m * h + intercept_N_mps
///   v_E(h) = gradient_E_mps_m * h + intercept_E_mps
///
/// NASA TM-2015-218675 Scenario 8:
///   v_E(h) = 0.003 * h_m - 6.096  m/s   (= (0.003*h_ft - 20) ft/s from west)
///   gradient_E_mps_m = 0.003
///   intercept_E_mps  = -6.096
///
/// Use with WindAwareDragPolicy\<LinearWindShear\>.
struct WindShear
{
    double gradient_N_mps_m { 0.0 }; ///< North wind altitude gradient [m/s per m].
    double gradient_E_mps_m { 0.0 }; ///< East  wind altitude gradient [m/s per m].
    double intercept_N_mps  { 0.0 }; ///< North wind at h = 0 m (sea level) [m/s].
    double intercept_E_mps  { 0.0 }; ///< East  wind at h = 0 m (sea level) [m/s].
};

// ─────────────────────────────────────────────────────────────────────────────
// WindModel concept tag
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Tag struct: specialise to std::true_type for each WindModel type.
///
/// All built-in models (ZeroWind, ConstantECEFWind, LinearWindShear,
/// GeodesicCallbackWind) are registered below.  User-defined wind models
/// must also specialise this to enable use with WindAwareDragPolicy:
///
///   namespace Aetherion::Environment {
///     template\<\> struct is_wind_model\<MyWind\> : std::true_type {};
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
// LinearWindShear
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Linear altitude wind-shear: v(h) = gradient × h + intercept.
///
/// Each NED component varies independently and linearly with geocentric
/// altitude h = |r_ECI| − Rₑ.  The NED wind is converted to ECEF once at
/// construction using the launch lat/lon; the same NED-to-ECEF matrix is
/// applied at every evaluation (valid as long as the sphere's lat/lon barely
/// changes, which is the case for the short Aetherion validation trajectories).
///
/// **NASA TM-2015-218675 Scenario 8** (2D wind shear, sphere):
///   @f[ v_E(h) = 0.003\,h_\text{m} - 6.096 \text{ m/s} @f]
///   (i.e. @f$(0.003\,h_\text{ft} - 20)\text{ ft/s}@f$ from the west)
///
/// Construct via:
/// @code
///   auto ws = LinearWindShear::from_ned(
///       0.0, 0.003,     // gradient (N, E) [m/s / m]
///       0.0, -6.096,    // intercept (N, E) [m/s]
///       lat0, lon0);
/// @endcode
struct LinearWindShear {
    // NED gradients [m/s per m of geocentric altitude]
    double grad_N{ 0.0 }, grad_E{ 0.0 };
    // NED intercepts at h = 0 [m/s]
    double int_N { 0.0 }, int_E { 0.0 };
    // NED-to-ECEF rotation rows for N and E (D component ignored, usually zero)
    // R_NE = [ [Nx, Ex], [Ny, Ey], [Nz, Ez] ] stored as 6 scalars
    double Nx{ 0.0 }, Ex{ 0.0 };
    double Ny{ 0.0 }, Ey{ 1.0 };
    double Nz{ 1.0 }, Ez{ 0.0 };

    LinearWindShear() = default;

    /// @brief Construct from NED gradient+intercept pairs and launch geodetic coords.
    ///
    /// @param gradient_N   North-wind altitude gradient [m/s per m].
    /// @param gradient_E   East-wind  altitude gradient [m/s per m].
    /// @param intercept_N  North wind at h = 0 m [m/s].
    /// @param intercept_E  East  wind at h = 0 m [m/s].
    /// @param lat0_rad     Launch geodetic latitude [rad].
    /// @param lon0_rad     Launch geodetic longitude [rad].
    static LinearWindShear from_ned(
        double gradient_N, double gradient_E,
        double intercept_N, double intercept_E,
        double lat0_rad, double lon0_rad)
    {
        const double sLat = std::sin(lat0_rad), cLat = std::cos(lat0_rad);
        const double sLon = std::sin(lon0_rad), cLon = std::cos(lon0_rad);
        LinearWindShear w;
        w.grad_N = gradient_N; w.grad_E = gradient_E;
        w.int_N  = intercept_N; w.int_E  = intercept_E;
        // North direction in ECEF: (-sin(lat)cos(lon), -sin(lat)sin(lon),  cos(lat))
        w.Nx = -sLat*cLon;  w.Ny = -sLat*sLon;  w.Nz = cLat;
        // East  direction in ECEF: (-sin(lon),          cos(lon),           0       )
        w.Ex = -sLon;       w.Ey =  cLon;        w.Ez = 0.0;
        return w;
    }

    template<class S>
    Eigen::Matrix<S, 3, 1>
    velocity_ecef(const Eigen::Matrix<S,3,1>& r_eci, S /*t*/) const {
        // Geocentric altitude
        const S h  = r_eci.norm() - S(WGS84::kSemiMajorAxis_m);
        // Linear NED wind at altitude h
        const S vN = S(grad_N) * h + S(int_N);
        const S vE = S(grad_E) * h + S(int_E);
        // Rotate NED → ECEF
        return Eigen::Matrix<S, 3, 1>{
            S(Nx)*vN + S(Ex)*vE,
            S(Ny)*vN + S(Ey)*vE,
            S(Nz)*vN + S(Ez)*vE
        };
    }
};
template<> struct is_wind_model<LinearWindShear> : std::true_type {};

} // namespace Aetherion::Environment

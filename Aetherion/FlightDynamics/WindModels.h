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
        const S h  = r_eci.norm() - S(Environment::WGS84::kSemiMajorAxis_m);
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

// ─────────────────────────────────────────────────────────────────────────────
// PowerLawWindShear  (kept for reference; not used by any current scenario)
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Altitude-dependent wind: v(h) = v_ref × (h/h_ref)^n.
/// @deprecated Use LinearWindShear for NASA Scenario 8.
struct PowerLawWindShear {
    double vx_ref   { 0.0 };
    double vy_ref   { 0.0 };
    double vz_ref   { 0.0 };
    double h_ref_m  { 9144.0 };
    double shear_exp{ 1.3333 };

    PowerLawWindShear() = default;
    PowerLawWindShear(double vx, double vy, double vz, double h_ref, double n)
        : vx_ref(vx), vy_ref(vy), vz_ref(vz), h_ref_m(h_ref), shear_exp(n) {}

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

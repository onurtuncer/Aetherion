// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// GeodesicCallbackWind.h
//
// Wraps an arbitrary user-provided wind function
//   f(lat_rad, lon_rad, alt_m, t_s) → Eigen::Vector3d (NED wind [m/s])
// as a WindModel for use with WindAwareDragPolicy.
//
// Kept separate from WindModels.h to avoid pulling the geodetic-conversion
// headers (Coordinate/InertialToLocal.h) into every translation unit that
// uses the basic wind models.
//
// Usage:
//   #include <Aetherion/FlightDynamics/GeodesicCallbackWind.h>
//
//   auto wind = GeodesicCallbackWind([](double lat, double lon,
//                                       double alt, double t) -> Eigen::Vector3d {
//       // query weather API, return NED wind [m/s]
//       return { 0.0, 6.0, 0.0 };
//   });
//   auto policy = WindAwareDragPolicy<GeodesicCallbackWind>{ CD, S, std::move(wind) };
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/FlightDynamics/WindModels.h>
#include <Aetherion/Coordinate/InertialToLocal.h>
#include <Eigen/Dense>
#include <cmath>
#include <functional>

namespace Aetherion::FlightDynamics {

/// @brief WindModel that calls a user callback with geodetic (lat, lon, alt, t).
///
/// **AD compatibility**: when called with S = CppAD::AD<double>, the method
/// returns the ECEF wind cached from the most recent *double* evaluation
/// (frozen-Jacobian approximation, standard for external data sources).
struct GeodesicCallbackWind {
    using Callback = std::function<
        Eigen::Vector3d(double lat_rad, double lon_rad, double alt_m, double t_s)>;

    Callback callback;

    GeodesicCallbackWind() = default;
    explicit GeodesicCallbackWind(Callback fn) : callback(std::move(fn)) {}

    template<class S>
    Eigen::Matrix<S, 3, 1>
    velocity_ecef(const Eigen::Matrix<S,3,1>& r_eci, S t_s) const
    {
        if constexpr (std::is_same_v<S, double>) {
            if (!callback) return Eigen::Matrix<S,3,1>::Zero();

            // ECI → ECEF (rotate by −θ around z)
            const double theta = Environment::WGS84::kRotationRate_rad_s * t_s;
            const double ct = std::cos(theta), st = std::sin(theta);
            const Eigen::Vector3d r_ecef{
                 ct * r_eci(0) + st * r_eci(1),
                -st * r_eci(0) + ct * r_eci(1),
                       r_eci(2)
            };

            // ECEF → geodetic WGS-84
            std::array<double,3> ra{ r_ecef(0), r_ecef(1), r_ecef(2) };
            double lat{}, lon{}, alt{};
            Coordinate::ECEFToGeodeticWGS84(ra, lat, lon, alt);

            // User callback → NED wind
            const Eigen::Vector3d v_ned = callback(lat, lon, alt, t_s);

            // NED → ECEF
            const double sLat = std::sin(lat), cLat = std::cos(lat);
            const double sLon = std::sin(lon), cLon = std::cos(lon);
            m_cached_ecef = {
                v_ned(0)*(-sLat*cLon) + v_ned(1)*(-sLon) + v_ned(2)*(-cLat*cLon),
                v_ned(0)*(-sLat*sLon) + v_ned(1)*( cLon) + v_ned(2)*(-cLat*sLon),
                v_ned(0)*( cLat)                          + v_ned(2)*(-sLat)
            };
            return m_cached_ecef;
        } else {
            // AD path: frozen ECEF cache cast to S
            return m_cached_ecef.template cast<S>();
        }
    }

private:
    mutable Eigen::Vector3d m_cached_ecef{ 0.0, 0.0, 0.0 };
};

template<> struct is_wind_model<GeodesicCallbackWind> : std::true_type {};

} // namespace Aetherion::FlightDynamics

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// ECEFtoGeodetic.cpp
//
// Double-only iterative ECEF → WGS-84 geodetic conversion (Bowring, 5 iterations).
//
// WGS-84 constants are now sourced exclusively from WGS84.h instead of being
// hardcoded as raw literals here and in the template version in InertialToLocal.cpp.
// ------------------------------------------------------------------------------

#include <Aetherion/Coordinate/InertialToLocal.h>
#include <Aetherion/Environment/WGS84.h>
#include <cmath>

namespace Aetherion::Coordinate {

    template <>
    void ECEFToGeodeticWGS84<double>(
        const Vec3<double>& r_ecef,
        double& lat_rad,
        double& lon_rad,
        double& h_m)
    {
        const double x = r_ecef[0];
        const double y = r_ecef[1];
        const double z = r_ecef[2];

        const double a = Environment::WGS84::kSemiMajorAxis_m;
        const double f = Environment::WGS84::kFlattening;
        const double e2 = Environment::WGS84::kEccentricitySq;

        lon_rad = std::atan2(y, x);

        const double p = std::sqrt(x * x + y * y);

        // Iterative Bowring — 5 iterations is sufficient for double precision
        double lat = std::atan2(z, p * (1.0 - e2));
        double h = 0.0;

        for (int i = 0; i < 5; ++i) {
            const double sLat = std::sin(lat);
            const double cLat = std::cos(lat);
            const double N = a / std::sqrt(1.0 - e2 * sLat * sLat);
            h = p / cLat - N;
            lat = std::atan2(z, p * (1.0 - e2 * N / (N + h)));
        }

        lat_rad = lat;
        h_m = h;
    }

} // namespace Aetherion::Coordinate

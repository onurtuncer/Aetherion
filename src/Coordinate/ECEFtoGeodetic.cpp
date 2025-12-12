// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

// double only implementation of ECEF to Geodetic WGS-84

#include <Aetherion/Coordinate/InertialToLocal.h>
#include <cmath>

namespace Aetherion::Coordinate {

    void ECEFToGeodeticWGS84(
        const Vec3<double>& r_ecef,
        double& lat_rad,
        double& lon_rad,
        double& h_m)
    {
        const double x = r_ecef[0];
        const double y = r_ecef[1];
        const double z = r_ecef[2];

        // WGS-84 constants
        const double a = 6378137.0;
        const double f = 1.0 / 298.257223563;
        const double e2 = f * (2.0 - f);

        lon_rad = std::atan2(y, x);

        const double p = std::sqrt(x * x + y * y);

        // Initial guess
        double lat = std::atan2(z, p * (1.0 - e2));
        double h = 0.0;

        // Fixed iterations
        for (int i = 0; i < 5; ++i) {
            const double sLat = std::sin(lat);
            const double cLat = std::cos(lat);

            const double N = a / std::sqrt(1.0 - e2 * sLat * sLat);
            h = p / cLat - N;

            const double denom = p * (1.0 - e2 * (N / (N + h)));
            lat = std::atan2(z, denom);
        }

        lat_rad = lat;
        h_m = h;
    }

} // namespace Aetherion::Coordinate


/*#include <Aetherion/Coordinate/InertialToLocal.h>

namespace Aetherion::Coordinate {

    template <class Scalar>
    void ECEFToGeodeticWGS84(
        const Vec3<Scalar>& r_ecef,
        Scalar& lat_rad,
        Scalar& lon_rad,
        Scalar& h_m)
    {
        using detail::Sine;
        using detail::Cosine;
        using detail::SquareRoot;
        using detail::ArcTangent2;

        const Scalar x = r_ecef[0];
        const Scalar y = r_ecef[1];
        const Scalar z = r_ecef[2];

        // WGS-84 constants
        const Scalar a = Scalar(6378137.0);            // semi-major axis [m]
        const Scalar f = Scalar(1.0 / 298.257223563);  // flattening
        const Scalar e2 = f * (Scalar(2) - f);          // eccentricity^2

        const Scalar one = Scalar(1);

        // Longitude
        lon_rad = ArcTangent2(y, x);

        const Scalar p = SquareRoot(x * x + y * y);

        // Initial guess for latitude
        Scalar lat = ArcTangent2(z, p * (one - e2));
        Scalar h = Scalar(0);

        // Fixed iterations (AD-friendly)
        for (int i = 0; i < 5; ++i) {
            const Scalar sLat = Sine(lat);
            const Scalar cLat = Cosine(lat);

            const Scalar N = a / SquareRoot(one - e2 * sLat * sLat);
            h = p / cLat - N;

            // lat = atan2(z, p * (1 - e2*N/(N+h)))
            const Scalar denom = p * (one - e2 * N / (N + h));
            lat = ArcTangent2(z, denom);
        }

        lat_rad = lat;
        h_m = h;
    }

    // ---- Explicit instantiations (add more if you need them) ----
    template void ECEFToGeodeticWGS84<double>(
        const Vec3<double>&, double&, double&, double&);

} // namespace Aetherion::Coordinate */

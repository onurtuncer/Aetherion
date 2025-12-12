#include <Aetherion/Coordinate/InertialToLocal.h>

namespace Aetherion::Coordinate {

    template <class Scalar>
    void ECEFToGeodeticWGS84(const Vec3<Scalar>& r_ecef,
        Scalar& lat_rad,
        Scalar& lon_rad,
        Scalar& h_m)
    {
        using detail::ArcTangent2;
        using detail::Sine;
        using detail::Cosine;
        using detail::SquareRoot;

        const Scalar x = r_ecef[0];
        const Scalar y = r_ecef[1];
        const Scalar z = r_ecef[2];

        const Scalar a = Scalar(6378137.0);
        const Scalar f = Scalar(1.0 / 298.257223563);
        const Scalar e2 = f * (Scalar(2) - f);

        const Scalar one = Scalar(1);

        lon_rad = ArcTangent2(y, x);

        const Scalar p = SquareRoot(x * x + y * y);

        Scalar lat = ArcTangent2(z, p * (one - e2));
        Scalar h = Scalar(0);

        for (int i = 0; i < 5; ++i) {
            const Scalar sLat = Sine(lat);
            const Scalar cLat = Cosine(lat);

            const Scalar N = a / SquareRoot(one - e2 * sLat * sLat);
            h = p / cLat - N;

            lat = ArcTangent2(z, p * (one - e2 * N / (N + h)));
        }

        lat_rad = lat;
        h_m = h;
    }

    // explicit instantiation for double (add more if needed)
    template void ECEFToGeodeticWGS84<double>(const Vec3<double>&, double&, double&, double&);

} // namespace Aetherion::Coordinate

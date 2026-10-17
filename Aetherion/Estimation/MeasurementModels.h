// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// MeasurementModels.h
//
// Measurement models of the first configuration (sensor fusion step 5;
// doc/sensor_fusion.rst "Measurement models"), written against what
// Hemerion's drivers decode rather than against idealised quantities:
//
//   GNSS position   UBX-NAV-PVT latitude, longitude, height. The height is
//                   ellipsoidal (NAV-PVT `height`, not `hMSL`). Mapped to LCI
//                   through the Earth rotation angle at the fix's time,
//                     y_p = R_LW ( R_WE(t) r_E(lat, lon, h) - r_W0 ),
//                   and modelled as y_p = p + noise.
//   GNSS velocity   UBX-NAV-PVT velN, velE, velD: Earth-relative. The
//                   transport term w_e x r_E (up to 465 m/s) makes it
//                   inertial before the rotation to LCI,
//                     y_v = R_LW R_WE(t) ( R_EN v_N + w_e x r_E ),
//                   modelled as y_v = v + noise.
//   Barometer       compensated static pressure, converted to pressure
//                   altitude by inverting the standard atmosphere
//                   (pressureAltitude_m), modelled as
//                     y_b = h(p) + b_p + noise
//                   with h the height above the WGS-84 ellipsoid of the
//                   estimated position (Environment::GeometricAltitude_m,
//                   0.1 m below 15 km) and b_p the barometer bias block.
//                   Geoid height and non-standard sea-level pressure are
//                   constant offsets absorbed by b_p; the temperature
//                   offset makes the error grow with height, which is why
//                   b_p is a random walk, not a constant.
//
// Assumptions of the first configuration, stated here because the models
// bake them in: the GNSS antenna is at the IMU (no lever arm), the fix is
// time-stamped on the simulation time base (the replay harness, step 6,
// maps host time onto it before calling these), and no magnetometer.
//
// Linearisation. GNSS position and velocity are left-invariant observations
// in the sense of Barrau & Bonnabel (Y = X b with b = (0,0,1) or (0,1,0)),
// so their Jacobian is constant for the left-invariant error and
// estimate-dependent for the right-invariant one; the barometer is a
// nonlinear function of position and depends on the estimate either way.
// With the innovation z and H on (phi, nu, rho), rows/columns by the
// configuration's block offsets, and grad h the LCI gradient of h at p:
//
//                      right-invariant                 left-invariant
//   GNSS position      z = y - p,                      z = R^T (y - p),
//                      H = [-[p]x  0  I]               H = [0  0  I]
//                      R_meas = Sigma_L                R_meas = R^T Sigma_L R
//   GNSS velocity      z = y - v,                      z = R^T (y - v),
//                      H = [-[v]x  I  0]               H = [0  I  0]
//   Barometer          z = y - h(p) - b_p,             same z,
//                      H = [-grad^T [p]x  0  grad^T]   H = [0  0  grad^T R]
//                      H[b_p] = 1                      H[b_p] = 1
//
// Sigma_L is the measurement covariance in LCI; nedToLci() maps the NED
// accuracies the receiver reports into it. Every H here is checked against
// central differences of the nonlinear maps, and through CppAD, in
// tests/Estimation/test_MeasurementModels.cpp.
// ------------------------------------------------------------------------------

#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include <Eigen/Dense>

#include <Aetherion/Coordinate/InertialToLocal.h>
#include <Aetherion/Coordinate/LocalToInertial.h>
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Environment/GeometricAltitude.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Estimation/Blocks.h>
#include <Aetherion/Estimation/Configuration.h>
#include <Aetherion/Estimation/EstimationState.h>
#include <Aetherion/Estimation/LciFrame.h>
#include <Aetherion/ODE/RKMK/Core/Scalar.h>
#include <Aetherion/Spatial/Skew.h>

namespace Aetherion::Estimation {

    /// Pressure altitude [m] of a static pressure on a standard day: the exact
    /// inverse of Environment::US1976Atmosphere (zero offsets), layer by layer,
    /// including the geopotential-to-geometric conversion. Pressures above
    /// sea-level pressure extrapolate the tropospheric layer downwards (a
    /// negative pressure altitude, as a barometer reports on a high-QNH day);
    /// pressures below the model's 84.852 km (geometric) ceiling invert its
    /// isothermal decay. Sensor preprocessing in double: this is not part of h(x).
    [[nodiscard]] inline double pressureAltitude_m(double p_Pa)
    {
        using L = Environment::detail::Us1976Layers;
        constexpr double Re = 6356766.0;  // geopotential reference radius of US1976Atmosphere
        constexpr double h_max = 84852.0; // geometric; only 83.7 km geopotential, i.e. still inside layer 6

        // Above the ceiling the model decays p exponentially in geometric height
        // from p(h_max), with the scale height of the ceiling temperature.
        static const double p_ceiling = Environment::US1976Atmosphere(h_max).p;
        if (p_Pa < p_ceiling)
        {
            const double H_scale = L::R * L::Tb_K[7] / L::g0;
            return h_max + H_scale * std::log(p_ceiling / p_Pa);
        }

        std::size_t b = 0;
        while (b + 1 < 7 && p_Pa <= L::Pb_Pa[b + 1])
            ++b;

        const double Hb = L::Hb_km[b] * 1000.0;
        const double Tb = L::Tb_K[b];
        const double Pb = L::Pb_Pa[b];
        const double Lb = L::Lb_K_per_km[b] * 1.0e-3;

        double H = 0.0;
        if (Lb == 0.0)
            H = Hb - L::R * Tb / L::g0 * std::log(p_Pa / Pb);
        else
            H = Hb + (Tb * std::pow(p_Pa / Pb, -Lb * L::R / L::g0) - Tb) / Lb;

        return Re * H / (Re - H);
    }

    /// One UBX-NAV-PVT fix as Hemerion decodes it; the height is ellipsoidal.
    template <class S>
    struct GnssFix
    {
        S lat_rad;
        S lon_rad;
        S height_m;
    };

    template <class S>
    class MeasurementModels
    {
    public:
        using Mat3 = Eigen::Matrix<S, 3, 3>;
        using Vec3 = Eigen::Matrix<S, 3, 1>;
        using VecX = Eigen::Matrix<S, Eigen::Dynamic, 1>;
        using MatX = Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>;
        using State = EstimationState<S>;

        explicit MeasurementModels(LciFrame<S> frame)
            : frame_(std::move(frame))
        {}

        [[nodiscard]] const LciFrame<S>& frame() const { return frame_; }

        // ---- raw measurement -> LCI ----------------------------------------------
        // era_rad is the Earth rotation angle at the fix's time,
        // frame().earthRotationAngle(t - t0).

        [[nodiscard]] Vec3 gnssPositionLci(const GnssFix<S>& fix, const S& era_rad) const
        {
            const auto r_E = Coordinate::GeodeticToECEF(fix.lat_rad, fix.lon_rad, fix.height_m);
            const auto r_W = Coordinate::ECEFToECI(r_E, era_rad);
            return frame_.toLci(Vec3(r_W[0], r_W[1], r_W[2]));
        }

        [[nodiscard]] Vec3 gnssVelocityLci(const GnssFix<S>& fix, const Vec3& v_ned, const S& era_rad) const
        {
            const auto r_E = Coordinate::GeodeticToECEF(fix.lat_rad, fix.lon_rad, fix.height_m);
            const auto v_E = Coordinate::NEDToECEF(std::array<S, 3>{ v_ned[0], v_ned[1], v_ned[2] }, fix.lat_rad, fix.lon_rad);
            const S w = S(Environment::WGS84::kRotationRate_rad_s);
            const std::array<S, 3> v_inertial_E{ v_E[0] - w * r_E[1], v_E[1] + w * r_E[0], v_E[2] }; // + w_e x r_E
            const auto v_W = Coordinate::ECEFToECI(v_inertial_E, era_rad);
            return frame_.rotateToLci(Vec3(v_W[0], v_W[1], v_W[2]));
        }

        /// Rotation NED(fix) -> LCI, for mapping the receiver's NED accuracies:
        /// Sigma_L = C Sigma_NED C^T.
        [[nodiscard]] Mat3 nedToLci(const GnssFix<S>& fix, const S& era_rad) const
        {
            const auto R = Coordinate::NEDToInertialRotationMatrix(fix.lat_rad, fix.lon_rad, era_rad); // row-major, NED -> ECI
            Mat3 R_WN;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    R_WN(r, c) = R[3 * r + c];
            return frame_.R_WL.transpose() * R_WN;
        }

        // ---- nonlinear h(x) --------------------------------------------------------

        [[nodiscard]] static Vec3 predictGnssPosition(const State& x) { return x.X.p; }
        [[nodiscard]] static Vec3 predictGnssVelocity(const State& x) { return x.X.v; }
        [[nodiscard]] S predictBarometer(const State& x) const { return ellipsoidalHeight(x.X.p) + x.baroBias; }

        /// Height above the WGS-84 ellipsoid of LCI position p.
        [[nodiscard]] S ellipsoidalHeight(const Vec3& p) const { return Environment::GeometricAltitude_m(frame_.toEci(p)); }

        /// d(ellipsoidalHeight)/dp, in LCI. Analytic derivative of
        /// GeometricAltitude_m: with s = z/r, D = 1 - e^2 + e^2 s^2 and
        /// b = a (1 - f),  grad = r_hat + b e^2 s D^-3/2 (e_z - s r_hat) / r  in ECI.
        [[nodiscard]] Vec3 ellipsoidalHeightGradient(const Vec3& p) const
        {
            namespace Core = ODE::RKMK::Core;
            const Vec3 r_W = frame_.toEci(p);
            const S r = Core::sqrt_s(r_W.dot(r_W));
            const Vec3 r_hat = r_W / r;
            const S s = r_W[2] / r;
            const S e2 = S(Environment::WGS84::kEccentricitySq);
            const S b = S(Environment::WGS84::kSemiMajorAxis_m * (1.0 - Environment::WGS84::kFlattening));
            const S D = S(1) - e2 + e2 * s * s;
            const S D32 = D * Core::sqrt_s(D);
            const Vec3 e_z(S(0), S(0), S(1));
            const Vec3 grad_W = r_hat + (b * e2 * s / (D32 * r)) * (e_z - s * r_hat);
            return frame_.rotateToLci(grad_W);
        }

        // ---- linearised updates ----------------------------------------------------
        // z: innovation, H: (m x n) with n = cfg.dimension(), R: (m x m)
        // measurement covariance in the convention's innovation coordinates.

        template <ErrorConvention C>
        void gnssPosition(const Configuration& cfg,
            const State& x,
            const Vec3& y_L,
            const Mat3& Sigma_L,
            VecX& z,
            MatX& H,
            MatX& R) const
        {
            leftInvariantObservation<C>(cfg, x, y_L - x.X.p, x.X.p, Sigma_L, NavCoreLayout::kPosition, z, H, R);
        }

        template <ErrorConvention C>
        void gnssVelocity(const Configuration& cfg,
            const State& x,
            const Vec3& y_L,
            const Mat3& Sigma_L,
            VecX& z,
            MatX& H,
            MatX& R) const
        {
            leftInvariantObservation<C>(cfg, x, y_L - x.X.v, x.X.v, Sigma_L, NavCoreLayout::kVelocity, z, H, R);
        }

        template <ErrorConvention C>
        void
        barometer(const Configuration& cfg, const State& x, const S& y_m, const S& sigma_m, VecX& z, MatX& H, MatX& R) const
        {
            const int n = cfg.dimension();
            const int nav = cfg.offset(BlockId::NavCore);
            const int iPhi = nav + NavCoreLayout::kAttitude;
            const int iRho = nav + NavCoreLayout::kPosition;
            const int bp = cfg.offset(BlockId::BaroBias);

            z = VecX::Constant(1, y_m - predictBarometer(x));
            H = MatX::Zero(1, n);
            const Eigen::Matrix<S, 1, 3> g = ellipsoidalHeightGradient(x.X.p).transpose();
            if constexpr (C == ErrorConvention::RightInvariant)
            {
                H.template block<1, 3>(0, iPhi) = -g * Spatial::skew(x.X.p);
                H.template block<1, 3>(0, iRho) = g;
            } else
            {
                H.template block<1, 3>(0, iRho) = g * x.X.R;
            }
            if (bp >= 0)
                H(0, bp) = S(1);
            R = MatX::Constant(1, 1, sigma_m * sigma_m);
        }

    private:
        LciFrame<S> frame_;

        // Y = X b observations (position b = (0,0,1), velocity b = (0,1,0)):
        // residual = y - q with q the predicted vector, `col` its block (nu or rho).
        template <ErrorConvention C>
        void leftInvariantObservation(const Configuration& cfg,
            const State& x,
            const Vec3& residual,
            const Vec3& q,
            const Mat3& Sigma_L,
            int col,
            VecX& z,
            MatX& H,
            MatX& R) const
        {
            const int n = cfg.dimension();
            const int nav = cfg.offset(BlockId::NavCore);
            const int iPhi = nav + NavCoreLayout::kAttitude;
            H = MatX::Zero(3, n);
            if constexpr (C == ErrorConvention::RightInvariant)
            {
                z = residual;
                H.template block<3, 3>(0, iPhi) = -Spatial::skew(q);
                H.template block<3, 3>(0, nav + col) = Mat3::Identity();
                R = Sigma_L;
            } else
            {
                const Mat3 Rt = x.X.R.transpose();
                z = Rt * residual;
                H.template block<3, 3>(0, nav + col) = Mat3::Identity();
                R = Rt * Sigma_L * x.X.R;
            }
        }
    };

} // namespace Aetherion::Estimation

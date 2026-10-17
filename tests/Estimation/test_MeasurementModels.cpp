// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Sensor fusion step 5, measurement models: the raw-to-LCI mappings against
// the coordinate library, the pressure-altitude inversion against the
// atmosphere it inverts, and every innovation Jacobian against central
// differences of the nonlinear maps in both error conventions -- then the
// barometer's once more through CppAD.
// ------------------------------------------------------------------------------

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <vector>

#include <cppad/cppad.hpp>

#include <Aetherion/Estimation/MeasurementModels.h>

using namespace Aetherion::Estimation;
namespace SO3 = Aetherion::ODE::RKMK::Lie::SO3;
namespace WGS84 = Aetherion::Environment::WGS84;

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using VecX = Eigen::VectorXd;
using MatX = Eigen::MatrixXd;
using AD = CppAD::AD<double>;

namespace {
    constexpr double kDeg = std::numbers::pi / 180.0;
    const Configuration& cfg = kNavBaroGnss16;

    LciFrame<double> testFrame() { return LciFrame<double>::FromGeodetic(36.0192 * kDeg, -75.6744 * kDeg, 3.0, 0.3); }

    EstimationState<double> testState()
    {
        EstimationState<double> x;
        x.X.R = SO3::Exp_R(Vec3(0.3, -0.2, 1.1));
        x.X.v = Vec3(120.0, 40.0, -5.0);
        x.X.p = Vec3(8000.0, -3000.0, -1500.0);
        x.gyroBias = Vec3(0.002, -0.001, 0.0015);
        x.accelBias = Vec3(0.05, -0.02, 0.03);
        x.baroBias = 12.0;
        return x;
    }

    template <class S>
    EstimationState<S> cast(const EstimationState<double>& x)
    {
        EstimationState<S> y;
        y.X.R = x.X.R.template cast<S>();
        y.X.v = x.X.v.template cast<S>();
        y.X.p = x.X.p.template cast<S>();
        y.gyroBias = x.gyroBias.template cast<S>();
        y.accelBias = x.accelBias.template cast<S>();
        y.baroBias = S(x.baroBias);
        return y;
    }

// H by central differences: the innovation at the estimate x, fed with the
// measurement a truth perturbed by +-eps e_j would produce.
    template <ErrorConvention C, class Innovation>
    MatX finiteDifferenceH(const EstimationState<double>& x, int m, Innovation innovation, double eps = 1e-6)
    {
        const int n = cfg.dimension();
        MatX H(m, n);
        for (int j = 0; j < n; ++j)
        {
            VecX d = VecX::Zero(n);
            d[j] = eps;
            const VecX plus = innovation(x, x.retract<C>(cfg, d));
            const VecX minus = innovation(x, x.retract<C>(cfg, -d));
            H.col(j) = (plus - minus) / (2.0 * eps);
        }
        return H;
    }
}  // namespace

// ---------------------------------------------------------------------------
TEST_CASE("pressure altitude inverts the standard atmosphere", "[estimation][measurement][baro]")
{
    for (const double h : { 0.0, 500.0, 5000.0, 10999.0, 11001.0, 15000.0, 25000.0, 40000.0, 60000.0, 80000.0 })
    {
        const double p = Aetherion::Environment::US1976Atmosphere(h).p;
        REQUIRE(pressureAltitude_m(p) == Catch::Approx(h).margin(1e-6));
    }
    REQUIRE(pressureAltitude_m(101325.0) == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(pressureAltitude_m(102325.0) < -80.0);  // a high-QNH day reads below sea level
    REQUIRE(pressureAltitude_m(102325.0) > -90.0);
  // Above the ceiling: the model's isothermal decay, continuous at 84.852 km.
    const double p_top = Aetherion::Environment::US1976Atmosphere(84852.0).p;
    REQUIRE(pressureAltitude_m(p_top) == Catch::Approx(84852.0).margin(1e-3));
    REQUIRE(pressureAltitude_m(Aetherion::Environment::US1976Atmosphere(90000.0).p) ==
            Catch::Approx(90000.0).margin(1e-3));
}

TEST_CASE("GNSS fix maps into LCI and back through the coordinate library", "[estimation][measurement][gnss]")
{
    const LciFrame<double> f = testFrame();
    const MeasurementModels<double> m(f);
    const GnssFix<double> fix{ 36.03 * kDeg, -75.66 * kDeg, 3000.0 };
    const double t = 100.0;
    const double era = f.earthRotationAngle(t);
    REQUIRE(era == Catch::Approx(0.3 + WGS84::kRotationRate_rad_s * t));

  // At the frame epoch the fix is a few km north-east and 3 km up:
  // (N, E, D) ~ (+1.2 km, +1.3 km, -3 km), the D a touch above -3 km from
  // the curvature drop over ~1.8 km (0.25 m) and the origin's own 3 m.
    const Vec3 p0 = m.gnssPositionLci(fix, f.era0_rad);
    REQUIRE(p0[0] == Catch::Approx(1200.0).margin(30.0));
    REQUIRE(p0[1] == Catch::Approx(1300.0).margin(30.0));
    REQUIRE(p0[2] == Catch::Approx(-2997.0).margin(1.0));

  // 100 s later the same Earth-fixed point has moved ~37 km east in the
  // inertial frame (w_e R cos(lat) * 100 s), and the curvature drop over
  // that distance (~107 m) shows in D. The frame is inertial: this is
  // correct, and it is why the ERA at the fix's time matters.
    const Vec3 p = m.gnssPositionLci(fix, era);
    REQUIRE(p[1] - p0[1] == Catch::Approx(WGS84::kRotationRate_rad_s * t * 6371e3 * std::cos(fix.lat_rad)).epsilon(0.02));
    REQUIRE(p[2] > p0[2] + 100.0);

  // Round trip: ECI -> ECEF(era) -> geodetic reproduces the fix.
    const Vec3 r_W = f.toEci(p);
    const auto r_E = Aetherion::Coordinate::ECIToECEF(std::array<double, 3>{ r_W[0], r_W[1], r_W[2] }, era);
    double lat{}, lon{}, h{};
    Aetherion::Coordinate::ECEFToGeodeticWGS84(r_E, lat, lon, h);
    REQUIRE(lat == Catch::Approx(fix.lat_rad).margin(1e-11));
    REQUIRE(lon == Catch::Approx(fix.lon_rad).margin(1e-11));
    REQUIRE(h == Catch::Approx(fix.height_m).margin(1e-5));

  // The barometer's height function sees the same point to its stated 0.1 m.
    REQUIRE(m.ellipsoidalHeight(p) == Catch::Approx(fix.height_m).margin(0.1));

  // NED -> LCI at the origin at t0 is the identity (the frame axes are NED there).
    const GnssFix<double> origin{ 36.0192 * kDeg, -75.6744 * kDeg, 3.0 };
    REQUIRE((m.nedToLci(origin, f.era0_rad) - Mat3::Identity()).norm() < 1e-12);
  // and 100 s later it has rotated by w_e * 100 s about the Earth's axis.
    const Mat3 C = m.nedToLci(origin, era);
    REQUIRE(C.isUnitary(1e-12));
    REQUIRE((C - Mat3::Identity()).norm() > 1e-3);
}

TEST_CASE("GNSS velocity carries the transport term", "[estimation][measurement][gnss]")
{
    const LciFrame<double> f = testFrame();
    const MeasurementModels<double> m(f);
    const GnssFix<double> fix{ 36.03 * kDeg, -75.66 * kDeg, 3000.0 };
    const double era = f.earthRotationAngle(100.0);

  // Stationary on the ground: inertial speed is w_e * distance from the axis.
    const Vec3 v0 = m.gnssVelocityLci(fix, Vec3::Zero(), era);
    const auto r_E = Aetherion::Coordinate::GeodeticToECEF(fix.lat_rad, fix.lon_rad, fix.height_m);
    const double expected = WGS84::kRotationRate_rad_s * std::hypot(r_E[0], r_E[1]);
    REQUIRE(v0.norm() == Catch::Approx(expected).epsilon(1e-9));
    REQUIRE(expected > 370.0);  // 465 m/s cos(36 deg)

  // A NED velocity adds linearly, through the same rotation as nedToLci.
    const Vec3 v_ned(100.0, -20.0, 5.0);
    const Vec3 v1 = m.gnssVelocityLci(fix, v_ned, era);
    REQUIRE((v1 - v0 - m.nedToLci(fix, era) * v_ned).norm() < 1e-9);
}

TEST_CASE("ellipsoidal height gradient matches central differences", "[estimation][measurement][baro]")
{
    const MeasurementModels<double> m(testFrame());
    const Vec3 p = testState().X.p;
    const Vec3 g = m.ellipsoidalHeightGradient(p);
    REQUIRE(g.norm() == Catch::Approx(1.0).margin(1e-6));  // a unit "up"
    REQUIRE(g[2] < -0.99);                                 // mostly -D
    for (int j = 0; j < 3; ++j)
    {
        Vec3 d = Vec3::Zero();
        d[j] = 1.0;
        const double fd = (m.ellipsoidalHeight(p + d) - m.ellipsoidalHeight(p - d)) / 2.0;
        REQUIRE(g[j] == Catch::Approx(fd).margin(1e-7));
    }
}

TEST_CASE("innovation Jacobians match finite differences in both conventions", "[estimation][measurement]")
{
    const LciFrame<double> f = testFrame();
    const MeasurementModels<double> m(f);
    const EstimationState<double> x = testState();
    Mat3 Sigma = Vec3(1.5 * 1.5, 1.5 * 1.5, 3.0 * 3.0).asDiagonal();

    auto check = [&](auto conv) {
        constexpr ErrorConvention C = decltype(conv)::value;
        VecX z;
        MatX H, R;

    // Position
        m.template gnssPosition<C>(cfg, x, x.X.p, Sigma, z, H, R);
        REQUIRE(z.norm() == 0.0);
        const MatX Hp = finiteDifferenceH<C>(x, 3, [&](const auto& est, const auto& truth) {
            VecX zz;
            MatX HH, RR;
            m.template gnssPosition<C>(cfg, est, MeasurementModels<double>::predictGnssPosition(truth), Sigma, zz, HH, RR);
            return zz;
        });
        INFO(toString(C) << " position: |H - H_fd| = " << (H - Hp).norm());
        REQUIRE((H - Hp).norm() < 1e-6 * std::max(1.0, Hp.norm()));

    // Velocity
        m.template gnssVelocity<C>(cfg, x, x.X.v, Sigma, z, H, R);
        const MatX Hv = finiteDifferenceH<C>(x, 3, [&](const auto& est, const auto& truth) {
            VecX zz;
            MatX HH, RR;
            m.template gnssVelocity<C>(cfg, est, MeasurementModels<double>::predictGnssVelocity(truth), Sigma, zz, HH, RR);
            return zz;
        });
        INFO(toString(C) << " velocity: |H - H_fd| = " << (H - Hv).norm());
        REQUIRE((H - Hv).norm() < 1e-6 * std::max(1.0, Hv.norm()));

    // Barometer. h = |r_W| - r_surface with |r_W| ~ 6.4e6 m carries ~1e-9 m
    // of round-off, which a 1e-6 step turns into 5e-4; a larger step runs
    // into the RI attitude columns' truncation, eps^2 |p|/6. 1e-5 balances
    // the two at ~1e-4; the CppAD test below is the tight one.
        m.template barometer<C>(cfg, x, m.predictBarometer(x), 2.0, z, H, R);
        REQUIRE(z.norm() == 0.0);
        REQUIRE(R(0, 0) == Catch::Approx(4.0));
        REQUIRE(H(0, cfg.offset(BlockId::BaroBias)) == 1.0);
        const MatX Hb = finiteDifferenceH<C>(
            x,
            1,
            [&](const auto& est, const auto& truth) {
                VecX zz;
                MatX HH, RR;
                m.template barometer<C>(cfg, est, m.predictBarometer(truth), 2.0, zz, HH, RR);
                return zz;
            },
            1e-5);
        INFO(toString(C) << " barometer: |H - H_fd| = " << (H - Hb).norm());
        REQUIRE((H - Hb).norm() < 3e-4);
    };

    check(std::integral_constant<ErrorConvention, ErrorConvention::RightInvariant>{});
    check(std::integral_constant<ErrorConvention, ErrorConvention::LeftInvariant>{});

  // The measurement covariance is rotated for LI, not for RI: same eigenvalues.
    VecX z;
    MatX H, Rri, Rli;
    m.template gnssPosition<ErrorConvention::RightInvariant>(cfg, x, x.X.p, Sigma, z, H, Rri);
    m.template gnssPosition<ErrorConvention::LeftInvariant>(cfg, x, x.X.p, Sigma, z, H, Rli);
    REQUIRE((Rri - Sigma).norm() == 0.0);
    REQUIRE((Rli - x.X.R.transpose() * Sigma * x.X.R).norm() < 1e-12);
    REQUIRE(Rli.trace() == Catch::Approx(Sigma.trace()));
}

TEST_CASE("CppAD differentiates the barometer innovation through Eigen", "[estimation][measurement][ad]")
{
    const LciFrame<double> frame = testFrame();
    const MeasurementModels<double> m(frame);
    const EstimationState<double> x = testState();
    const int n = cfg.dimension();

    auto tape = [&](auto conv) {
        constexpr ErrorConvention C = decltype(conv)::value;
        LciFrame<AD> frameAD;
        frameAD.r_W0 = frame.r_W0.cast<AD>();
        frameAD.R_WL = frame.R_WL.cast<AD>();
        frameAD.era0_rad = AD(frame.era0_rad);
        const MeasurementModels<AD> mAD(frameAD);
        const EstimationState<AD> xAD = cast<AD>(x);

        std::vector<AD> ax(n, AD(0.0));
        CppAD::Independent(ax);
        Eigen::Matrix<AD, Eigen::Dynamic, 1> delta(n);
        for (int i = 0; i < n; ++i)
            delta[i] = ax[i];
        const EstimationState<AD> truth = xAD.template retract<C>(cfg, delta);
        Eigen::Matrix<AD, Eigen::Dynamic, 1> z;
        Eigen::Matrix<AD, Eigen::Dynamic, Eigen::Dynamic> H, R;
        mAD.template barometer<C>(cfg, xAD, mAD.predictBarometer(truth), AD(2.0), z, H, R);
        std::vector<AD> ay{ z[0] };
        CppAD::ADFun<double> f(ax, ay);
        const std::vector<double> J = f.Jacobian(std::vector<double>(n, 0.0));

        VecX zd;
        MatX Hd, Rd;
        m.template barometer<C>(cfg, x, m.predictBarometer(x), 2.0, zd, Hd, Rd);
        for (int j = 0; j < n; ++j)
        {
            INFO(toString(C) << " column " << j);
            REQUIRE(J[j] == Catch::Approx(Hd(0, j)).margin(1e-9));
        }
    };
    tape(std::integral_constant<ErrorConvention, ErrorConvention::RightInvariant>{});
    tape(std::integral_constant<ErrorConvention, ErrorConvention::LeftInvariant>{});
}

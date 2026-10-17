// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Sensor fusion step 5: SE2(3), the LCI frame, the state's retract/error
// pair and the strapdown process model, in both error conventions. The
// decisive test is the last pair: the analytic discrete transition against
// central differences of the full nonlinear step (the check that found the
// trapezoidal-F requirement in the prototype), repeated through CppAD so the
// Eigen + CondExp path that step 8 will generate code from is exercised now.
// ------------------------------------------------------------------------------

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <vector>

#include <cppad/cppad.hpp>

#include <Aetherion/Estimation/ProcessModel.h>

using namespace Aetherion::Estimation;
namespace SO3 = Aetherion::ODE::RKMK::Lie::SO3;

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec9 = Eigen::Matrix<double, 9, 1>;
using VecX = Eigen::VectorXd;
using MatX = Eigen::MatrixXd;
using AD = CppAD::AD<double>;

namespace {
    constexpr double kDeg = std::numbers::pi / 180.0;
    const Configuration& cfg = kNavBaroGnss16;

    // Kitty Hawk (the F-16 cases), 3 m ellipsoidal height, an arbitrary ERA.
    LciFrame<double> testFrame() { return LciFrame<double>::FromGeodetic(36.0192 * kDeg, -75.6744 * kDeg, 3.0, 0.3); }

    // A state well away from the origin, so the right-invariant [p]x couplings
    // are large and a start-of-step F would show its first-order error.
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

    ImuSample<double> testImu() { return { Vec3(0.1, -0.05, 0.2), Vec3(2.0, -1.0, -9.0) }; }

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

    // Phi by central differences of xi -> error_C(propagate(x), propagate(retract_C(x, xi))).
    template <ErrorConvention C>
    MatX finiteDifferencePhi(const StrapdownProcessModel<double>& model,
        const EstimationState<double>& x,
        const ImuSample<double>& u,
        double dt,
        double eps = 1e-5)
    {
        const int n = cfg.dimension();
        const EstimationState<double> x1 = model.propagate(x, u, dt);
        MatX Phi(n, n);
        for (int j = 0; j < n; ++j)
        {
            VecX d = VecX::Zero(n);
            d[j] = eps;
            const VecX plus = x1.error<C>(cfg, model.propagate(x.retract<C>(cfg, d), u, dt));
            const VecX minus = x1.error<C>(cfg, model.propagate(x.retract<C>(cfg, -d), u, dt));
            Phi.col(j) = (plus - minus) / (2.0 * eps);
        }
        return Phi;
    }

    double relErr(const MatX& a, const MatX& ref) { return (a - ref).norm() / ref.norm(); }
    double absErr(const MatX& a, const MatX& ref) { return (a - ref).norm(); }
} // namespace

// ---------------------------------------------------------------------------
TEST_CASE("SO(3): Log inverts Exp, J_l^-1 inverts J_l, including small angles", "[estimation][se23]")
{
    for (const Vec3& w : { Vec3(0.3, -0.2, 1.1), Vec3(2.5, 0.0, 0.0), Vec3(1e-6, -2e-6, 3e-6), Vec3::Zero().eval() })
    {
        const Mat3 R = SO3::Exp_R(w);
        REQUIRE((SO3::Log_R(R) - w).norm() < 1e-12);
        REQUIRE((SO3::LeftJacobianInverse(w) * SO3::LeftJacobian(w) - Mat3::Identity()).norm() < 1e-12);
    }
    // Round-off past |cos| = 1 is clamped, not NaN.
    Mat3 I = Mat3::Identity();
    I(0, 0) += 1e-15;
    REQUIRE(SO3::Log_R(I).norm() < 1e-12);
}

TEST_CASE("SE2(3): composition, inverse, Exp/Log, adjoint", "[estimation][se23]")
{
    using G = SE23<double>;
    Vec9 xi;
    xi << 0.3, -0.2, 1.1, 5.0, -2.0, 1.0, 100.0, 50.0, -20.0;
    const G X = G::Exp(xi);

    REQUIRE((G::Log(X) - xi).norm() < 1e-9);

    const G I = X * X.inverse();
    REQUIRE((I.R - Mat3::Identity()).norm() < 1e-12);
    REQUIRE(I.v.norm() < 1e-9);
    REQUIRE(I.p.norm() < 1e-9);

    // X Exp(eta) X^-1 == Exp(Ad_X eta)
    Vec9 eta;
    eta << 0.01, 0.02, -0.03, 0.5, 0.1, -0.2, 3.0, -1.0, 2.0;
    const G lhs = X * G::Exp(eta) * X.inverse();
    const G rhs = G::Exp(G::Adjoint(X) * eta);
    REQUIRE((lhs.R - rhs.R).norm() < 1e-12);
    REQUIRE((lhs.v - rhs.v).norm() < 1e-9);
    REQUIRE((lhs.p - rhs.p).norm() < 1e-9);
}

TEST_CASE("LCI frame: mapping, gravity direction, gravity gradient", "[estimation][lci]")
{
    const LciFrame<double> f = testFrame();
    const Vec3 p(8000.0, -3000.0, -1500.0);
    REQUIRE((f.toLci(f.toEci(p)) - p).norm() < 1e-6);

    // At the origin the LCI axes are NED: gravity is (nearly) along +D, with
    // a small meridional (N) component from J2 and none to the east.
    const Vec3 g0 = f.gravity(Vec3::Zero());
    REQUIRE(g0[2] == Catch::Approx(9.80).margin(0.03));
    REQUIRE(std::abs(g0[0]) < 0.05);
    REQUIRE(std::abs(g0[1]) < 1e-6);

    // Analytic (central-term) gradient against central differences of the
    // full J2 gravity: J2's share of the gradient is ~1e-3 of ~1.5e-6 s^-2.
    const Mat3 G = f.gravityGradient(p);
    const double h = 10.0;
    for (int j = 0; j < 3; ++j)
    {
        Vec3 d = Vec3::Zero();
        d[j] = h;
        const Vec3 col = (f.gravity(p + d) - f.gravity(p - d)) / (2.0 * h);
        REQUIRE((G.col(j) - col).norm() < 1e-8);
    }
    REQUIRE(G.norm() > 1e-6); // and it is not trivially zero
}

TEST_CASE("retract and error are exact inverses in both conventions", "[estimation][state]")
{
    const EstimationState<double> x = testState();
    VecX d(cfg.dimension());
    d << 0.05, -0.03, 0.08, 2.0, -1.0, 0.5, 30.0, -10.0, 5.0, 1e-3, -2e-3, 5e-4, 0.01, 0.02, -0.03, 1.5;

    SECTION("right-invariant")
    {
        const auto y = x.retract<ErrorConvention::RightInvariant>(cfg, d);
        REQUIRE((x.error<ErrorConvention::RightInvariant>(cfg, y) - d).norm() < 1e-9);
    }
    SECTION("left-invariant")
    {
        const auto y = x.retract<ErrorConvention::LeftInvariant>(cfg, d);
        REQUIRE((x.error<ErrorConvention::LeftInvariant>(cfg, y) - d).norm() < 1e-9);
    }
    SECTION("the two conventions differ by the adjoint of the estimate")
    {
        // xi_R = Ad_Xhat xi_L for the same perturbed state (first-order identity;
        // exact for the group part because Xhat Exp(xi) Xhat^-1 = Exp(Ad xi)).
        const auto y = x.retract<ErrorConvention::LeftInvariant>(cfg, d);
        const VecX eR = x.error<ErrorConvention::RightInvariant>(cfg, y);
        const Vec9 expected = SE23<double>::Adjoint(x.X) * d.head<9>();
        REQUIRE((eR.head<9>() - expected).norm() < 1e-9);
        REQUIRE((eR.tail<7>() - d.tail<7>()).norm() < 1e-12);
    }
}

TEST_CASE("propagate: free fall and constant rate integrate as expected", "[estimation][process]")
{
    const StrapdownProcessModel<double> model(testFrame());
    EstimationState<double> x; // at rest at the origin, no biases
    const Vec3 w(0.0, 0.0, 0.5);
    const ImuSample<double> u{ w, Vec3::Zero() }; // zero specific force: free fall
    const Vec3 g0 = model.frame().gravity(Vec3::Zero());

    const double dt = 0.01;
    const int N = 100;
    for (int k = 0; k < N; ++k)
        x = model.propagate(x, u, dt);
    const double T = N * dt;

    REQUIRE((x.X.R - SO3::Exp_R(Vec3(w * T))).norm() < 1e-12); // constant twist: exact
    REQUIRE((x.X.v - g0 * T).norm() < 1e-4);                   // gravity changes by ~1e-6 over 5 m
    REQUIRE((x.X.p - 0.5 * g0 * T * T).norm() < 1e-4);
    REQUIRE(x.gyroBias.isZero());
    REQUIRE(x.baroBias == 0.0);
}

TEST_CASE("noise layout follows the configuration", "[estimation][process]")
{
    REQUIRE(noiseDimension(cfg) == 13);
    REQUIRE(noiseOffset(cfg, BlockId::GyroBias) == 6);
    REQUIRE(noiseOffset(cfg, BlockId::AccelBias) == 9);
    REQUIRE(noiseOffset(cfg, BlockId::BaroBias) == 12);
    REQUIRE(noiseOffset(cfg, BlockId::NavCore) == -1);

    const Configuration noBaro{ { BlockId::NavCore, BlockId::GyroBias, BlockId::AccelBias },
        { MeasurementId::GnssPosition } };
    REQUIRE(noiseDimension(noBaro) == 12);
    REQUIRE(noiseOffset(noBaro, BlockId::BaroBias) == -1);

    const StrapdownProcessModel<double> model(testFrame());
    const ProcessNoise<double> q{ 1e-3, 2e-3, 3e-5, 4e-4, 5e-2 };
    const MatX Qc = model.continuousNoiseCovariance(cfg, q);
    REQUIRE(Qc.rows() == 13);
    REQUIRE(Qc(0, 0) == Catch::Approx(1e-6));
    REQUIRE(Qc(5, 5) == Catch::Approx(4e-6));
    REQUIRE(Qc(8, 8) == Catch::Approx(9e-10));
    REQUIRE(Qc(11, 11) == Catch::Approx(1.6e-7));
    REQUIRE(Qc(12, 12) == Catch::Approx(2.5e-3));
    REQUIRE(Qc.diagonal().sum() == Catch::Approx(Qc.sum())); // diagonal
}

// The check the prototype's finite differences performed, now on the C++
// model: Phi from step<C>() against central differences of the nonlinear
// map, in both conventions. Also records why the trapezoidal F is required:
// with a start-of-step F the right-invariant Phi is an order worse.
TEST_CASE("discrete transition matches finite differences; trapezoidal F is required for RI", "[estimation][process]")
{
    const StrapdownProcessModel<double> model(testFrame());
    const EstimationState<double> x = testState();
    const ImuSample<double> u = testImu();
    const double dt = 0.01;
    const MatX Qc = model.continuousNoiseCovariance(cfg, ProcessNoise<double>{ 1e-3, 2e-3, 3e-5, 4e-4, 5e-2 });

    // The analytic Phi linearises the continuous flow; the finite differences
    // linearise the discrete propagate(). Both are second-order one-step maps,
    // so their difference is a local error of O(dt^3): the absolute residual
    // must fall by ~8 when dt is halved. Absolute, not relative: in RI the
    // norm of Phi itself is dominated by dt [p]x R, so a relative residual
    // would understate the order by one. A ratio of ~4 means something is
    // first-order consistent only (the test caught exactly that in
    // propagate() before it rotated by the mid-step attitude); ~2 would be a
    // wrong entry.
    auto check = [&](auto conv, double h) {
        constexpr ErrorConvention C = decltype(conv)::value;
        MatX Phi, Qd, F0, G0;
        const auto x1 = model.template step<C>(cfg, x, u, h, Qc, Phi, Qd);
        (void)x1;
        const MatX PhiFd = finiteDifferencePhi<C>(model, x, u, h);

        model.template errorJacobians<C>(cfg, x, u, F0, G0);
        const MatX Fd0 = F0 * h;
        const MatX PhiStart = MatX::Identity(16, 16) + Fd0 + 0.5 * Fd0 * Fd0;

        const double eTrap = absErr(Phi, PhiFd);
        const double eStart = absErr(PhiStart, PhiFd);
        INFO("convention " << toString(C) << ", dt " << h << ": trapezoidal abs err " << eTrap << " (rel "
                           << relErr(Phi, PhiFd) << "), start-of-step " << eStart);
        REQUIRE(relErr(Phi, PhiFd) < 1e-6);
        // Equal in LI up to the step's change of R^T G R (~1e-9 relative), so
        // the inequality is really the RI claim; the slack covers LI's noise.
        REQUIRE(eTrap <= eStart * (1.0 + 1e-6));

        REQUIRE(Qd.rows() == 16);
        REQUIRE((Qd - Qd.transpose()).norm() < 1e-12 * Qd.norm());
        REQUIRE(Qd.diagonal().minCoeff() >= 0.0);
        return std::pair{ eTrap, eStart };
    };

    constexpr auto RI = std::integral_constant<ErrorConvention, ErrorConvention::RightInvariant>{};
    constexpr auto LI = std::integral_constant<ErrorConvention, ErrorConvention::LeftInvariant>{};

    const auto [trapRI, startRI] = check(RI, dt);
    const auto [trapRI2, startRI2] = check(RI, dt / 2);
    const auto [trapLI, startLI] = check(LI, dt);
    const auto [trapLI2, startLI2] = check(LI, dt / 2);

    // Trapezoidal: local error O(dt^3) in both conventions.
    INFO("trapezoidal abs-err ratio dt/(dt/2): RI " << trapRI / trapRI2 << ", LI " << trapLI / trapLI2);
    REQUIRE(trapRI / trapRI2 > 6.0);
    REQUIRE(trapLI / trapLI2 > 6.0);

    // RI with a start-of-step F: the [p]x R bias coupling moves within the
    // step, so the local error is O(dt^2) -- ratio ~4 -- and far larger than
    // the trapezoidal one at the same dt.
    INFO("RI start-of-step abs-err ratio dt/(dt/2): " << startRI / startRI2);
    REQUIRE(startRI / startRI2 < 6.0);
    REQUIRE(startRI > 50.0 * trapRI);
    // LI carries that variation inside A_L: start-of-step keeps O(dt^3).
    INFO("LI start-of-step abs-err ratio dt/(dt/2): " << startLI / startLI2);
    REQUIRE(startLI / startLI2 > 6.0);
}

// The same map, differentiated by CppAD through Eigen and the CondExp
// branches of Exp/Log: the path the code generator (step 8) will take.
TEST_CASE("CppAD differentiates the process model through Eigen", "[estimation][process][ad]")
{
    const LciFrame<double> frame = testFrame();
    const StrapdownProcessModel<double> model(frame);
    const EstimationState<double> x = testState();
    const ImuSample<double> u = testImu();
    const double dt = 0.01;
    const int n = cfg.dimension();

    auto tape = [&](auto conv) {
        constexpr ErrorConvention C = decltype(conv)::value;

        LciFrame<AD> frameAD;
        frameAD.r_W0 = frame.r_W0.cast<AD>();
        frameAD.R_WL = frame.R_WL.cast<AD>();
        const StrapdownProcessModel<AD> modelAD(frameAD);
        const EstimationState<AD> xAD = cast<AD>(x);
        const ImuSample<AD> uAD{ u.gyro_rad_s.cast<AD>(), u.accel_m_s2.cast<AD>() };

        std::vector<AD> ax(n, AD(0.0));
        CppAD::Independent(ax);
        Eigen::Matrix<AD, Eigen::Dynamic, 1> delta(n);
        for (int i = 0; i < n; ++i)
            delta[i] = ax[i];
        const EstimationState<AD> x1 = modelAD.propagate(xAD, uAD, AD(dt));
        const EstimationState<AD> x1p = modelAD.propagate(xAD.template retract<C>(cfg, delta), uAD, AD(dt));
        const Eigen::Matrix<AD, Eigen::Dynamic, 1> xi = x1.template error<C>(cfg, x1p);
        std::vector<AD> ay(n);
        for (int i = 0; i < n; ++i)
            ay[i] = xi[i];
        CppAD::ADFun<double> f(ax, ay);

        const std::vector<double> J = f.Jacobian(std::vector<double>(n, 0.0));
        MatX PhiAD(n, n);
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                PhiAD(i, j) = J[i * n + j];
        REQUIRE(PhiAD.allFinite());

        const MatX PhiFd = finiteDifferencePhi<C>(model, x, u, dt);
        INFO("convention " << toString(C) << ": AD vs finite differences rel err " << relErr(PhiAD, PhiFd));
        REQUIRE(relErr(PhiAD, PhiFd) < 1e-7);

        // And the analytic Phi agrees with the AD one to discretisation order
        // (the order itself is established in the previous test case).
        MatX Phi, Qd;
        const MatX Qc = MatX::Identity(noiseDimension(cfg), noiseDimension(cfg));
        (void)model.template step<C>(cfg, x, u, dt, Qc, Phi, Qd);
        REQUIRE(relErr(Phi, PhiAD) < 1e-6);
    };

    tape(std::integral_constant<ErrorConvention, ErrorConvention::RightInvariant>{});
    tape(std::integral_constant<ErrorConvention, ErrorConvention::LeftInvariant>{});
}

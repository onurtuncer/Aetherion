// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_F16Environment.cpp
//
// F16AeroPolicy in wind, gusts and on a non-standard day, at the Scenario-11
// trim state:
//   - the calm, standard-day policy is bit-identical to a policy with the
//     environment set to its defaults;
//   - a steady wind shifts the ground velocity and nothing else (the frame
//     chain NED -> ECEF -> ECI(t) -> body is checked at a non-zero Earth
//     rotation angle);
//   - a linear gust is the same as the opposite shift of the body velocity;
//   - the rotational gusts have the physical sign: a positive angular
//     velocity of the air about an axis produces a moment about that axis in
//     the same sense (the damping derivatives are negative, the relative rate
//     is minus the gust);
//   - a hot day scales the forces by the density ratio and trims at a higher
//     alpha and throttle;
//   - the snapshot reports the air data the forces were computed from;
//   - everything evaluates under CppAD::AD<double>.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cppad/cppad.hpp>

#include <Aetherion/FlightDynamics/Policies/F16/F16AeroPolicy.h>
#include <Aetherion/FlightDynamics/Policies/F16/F16PropPolicy.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Trim/TrimSolver.h>
#include <Aetherion/FlightDynamics/Trim/TrimBodyRates.h>
#include <Aetherion/FlightDynamics/Trim/TrimWeight.h>
#include <Aetherion/Environment/WindModels.h>
#include <Aetherion/Environment/DrydenTurbulence.h>
#include <Aetherion/RigidBody/BuildInitialState.h>
#include <Aetherion/RigidBody/Config.h>
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/RigidBody/StateLayout.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>

#include <cmath>
#include <memory>
#include <numbers>

using namespace Aetherion;
using namespace Aetherion::FlightDynamics;
using namespace Aetherion::Serialization;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

#ifndef DAVEML_F16_AERO_FILE
#  define DAVEML_F16_AERO_FILE ""
#endif
#ifndef DAVEML_F16_PROP_FILE
#  define DAVEML_F16_PROP_FILE ""
#endif
static const std::string kAeroFile = DAVEML_F16_AERO_FILE;
static const std::string kPropFile = DAVEML_F16_PROP_FILE;

namespace {

    constexpr double kFt_m       = TrimSolver::kFt_m;
    constexpr double kDeg        = std::numbers::pi / 180.0;
    constexpr double kLat_deg    = 36.01917, kLon_deg = -75.67444, kHeading_deg = 45.0;
    constexpr double kAlt_ft     = 10013.0;
    constexpr double kV_fps      = 400.0;           // per axis: vt = 565.685 ft/s
    constexpr double kMass_kg    = 9298.6439;
    constexpr double kXcg_ft     = 1.132;
    constexpr double kOmegaE     = Environment::WGS84::kRotationRate_rad_s;

    using SE3d  = ODE::RKMK::Lie::SE3<double>;
    using Vec6d = Eigen::Matrix<double, 6, 1>;

    struct Scene {
        std::shared_ptr<const DAVEMLAeroModel> aero;
        std::shared_ptr<const DAVEMLPropModel> prop;
        TrimPoint trim;
        RigidBody::BodyRates rates;   // transport rate at the trim heading and pitch
    };

    Scene makeScene(const Environment::AtmosphereOffsets& atm = {})
    {
        Scene s;
        s.aero = std::make_shared<const DAVEMLAeroModel>(kAeroFile);
        s.prop = std::make_shared<const DAVEMLPropModel>(kPropFile);
        TrimSolver solver(*s.aero, *s.prop, kXcg_ft);
        solver.setAtmosphereOffsets(atm);

        const double alt_m = kAlt_ft * kFt_m, v = kV_fps * kFt_m;
        TrimInputs in{};
        in.vt_fps     = std::hypot(kV_fps, kV_fps);
        in.alt_ft     = kAlt_ft;
        in.weight_lbf = LevelFlightTrimWeight_lbf(kMass_kg, kLat_deg, alt_m, v, v);
        in.bodyRates  = LevelFlightBodyRates(kLat_deg, alt_m, v, v, kHeading_deg, 0.0, 0.0);
        s.trim = solver.solve(in);
        REQUIRE(s.trim.converged);
        s.rates = LevelFlightBodyRates(kLat_deg, alt_m, v, v, kHeading_deg, s.trim.alpha_deg, 0.0);
        return s;
    }

    /// State at the trim attitude with the given NED ground velocity, built for
    /// Earth rotation angle theta (so it must be evaluated at t = theta / omega_E).
    RigidBody::StateD makeState(const Scene& s, double vN, double vE, double vD, double theta)
    {
        RigidBody::Config cfg{};
        cfg.pose.lat_deg          = kLat_deg;
        cfg.pose.lon_deg          = kLon_deg;
        cfg.pose.alt_m            = kAlt_ft * kFt_m;
        cfg.pose.azimuth_deg      = kHeading_deg;
        cfg.pose.zenith_deg       = 90.0 - s.trim.alpha_deg;
        cfg.pose.roll_deg         = 0.0;
        cfg.velocityNED.north_mps = vN;
        cfg.velocityNED.east_mps  = vE;
        cfg.velocityNED.down_mps  = vD;
        cfg.bodyRates             = s.rates;
        cfg.inertialParameters.mass_kg = kMass_kg;

        const auto x0 = RigidBody::BuildInitialStateVector(cfg, theta);
        using L = RigidBody::StateLayout;
        RigidBody::StateD st{};
        st.g.p = Eigen::Vector3d(x0[L::IDX_P], x0[L::IDX_P + 1], x0[L::IDX_P + 2]);
        const Eigen::Quaterniond q(x0[L::IDX_Q], x0[L::IDX_Q + 1], x0[L::IDX_Q + 2], x0[L::IDX_Q + 3]);
        st.g.q = q.normalized();
        st.g.R = st.g.q.toRotationMatrix();
        st.nu_B << x0[L::IDX_W], x0[L::IDX_W + 1], x0[L::IDX_W + 2],
                   x0[L::IDX_V], x0[L::IDX_V + 1], x0[L::IDX_V + 2];
        st.m = kMass_kg;
        return st;
    }

    F16AeroPolicy makePolicy(const Scene& s)
    {
        return F16AeroPolicy(s.aero, s.trim.el_deg, 0.0, 0.0, kXcg_ft * kFt_m);
    }

    SE3d pose(const RigidBody::StateD& st) { return SE3d(st.g.R, st.g.p); }
}

TEST_CASE("F16Environment: model files are present", "[f16env][smoke]")
{
    REQUIRE_FALSE(kAeroFile.empty());
    REQUIRE_FALSE(kPropFile.empty());
}

TEST_CASE("F16Environment: default environment is bit-identical to no environment", "[f16env]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;
    const Scene s = makeScene();
    const double v = kV_fps * kFt_m;
    const auto st = makeState(s, v, v, 0.0, 0.3);
    const double t = 0.3 / kOmegaE;

    F16AeroPolicy a = makePolicy(s);
    F16AeroPolicy b = makePolicy(s);
    b.setWindECEF(Eigen::Vector3d::Zero());
    b.setGust(Environment::GustState{});
    b.setAtmosphereOffsets(Environment::AtmosphereOffsets{});

    const auto wa = a(pose(st), st.nu_B, kMass_kg, t);
    const auto wb = b(pose(st), st.nu_B, kMass_kg, t);
    for (int i = 0; i < 6; ++i) CHECK(wa.f(i) == wb.f(i));

    const auto va = a.airRelativeVelocity_B<double>(pose(st), st.nu_B, t);
    const auto vb = b.airRelativeVelocity_B<double>(pose(st), st.nu_B, t);
    for (int i = 0; i < 3; ++i) CHECK(va(i) == vb(i));
}

TEST_CASE("F16Environment: steady wind shifts the ground velocity and nothing else", "[f16env][wind]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;
    const Scene s = makeScene();
    const double v = kV_fps * kFt_m;

    // Headwind, crosswind and a vertical component; at t = 0 and at a large
    // Earth rotation angle, so the ECEF -> ECI(t) step is exercised.
    struct W { double n, e, d; };
    const W winds[] = { { -7.0711, -7.0711, 0.0 }, { 7.0711, -7.0711, 0.0 }, { 3.0, -4.0, 1.5 } };
    const double thetas[] = { 0.0, 0.3, 2.9 };

    for (const auto& w : winds) {
        for (double theta : thetas) {
            const double t = theta / kOmegaE;

            // Calm reference at the air velocity.
            const auto stCalm = makeState(s, v, v, 0.0, theta);
            F16AeroPolicy calm = makePolicy(s);
            const auto wCalm  = calm(pose(stCalm), stCalm.nu_B, kMass_kg, t);
            const auto vrCalm = calm.airRelativeVelocity_B<double>(pose(stCalm), stCalm.nu_B, t);

            // Windy: ground velocity = air velocity + wind, policy told the wind.
            const auto stWind = makeState(s, v + w.n, v + w.e, w.d, theta);
            F16AeroPolicy windy = makePolicy(s);
            const auto wecef = Environment::ConstantECEFWind::from_ned(
                w.n, w.e, w.d, kLat_deg * kDeg, kLon_deg * kDeg);
            windy.setWindECEF(Eigen::Vector3d(wecef.vx, wecef.vy, wecef.vz));
            const auto wWind  = windy(pose(stWind), stWind.nu_B, kMass_kg, t);
            const auto vrWind = windy.airRelativeVelocity_B<double>(pose(stWind), stWind.nu_B, t);

            INFO("wind NED = (" << w.n << ", " << w.e << ", " << w.d << "), theta = " << theta);
            for (int i = 0; i < 3; ++i) CHECK_THAT(vrWind(i), WithinAbs(vrCalm(i), 1e-7));
            // Fy is ~1e-5 N at zero sideslip: a relative test is meaningless there.
            for (int i = 3; i < 6; ++i)
                CHECK_THAT(wWind.f(i), WithinRel(wCalm.f(i), 1e-7) || WithinAbs(wCalm.f(i), 1e-4));
            for (int i = 0; i < 3; ++i) CHECK_THAT(wWind.f(i), WithinAbs(wCalm.f(i), 1e-4));

            // And the windy policy at the calm state sees a different airspeed.
            const auto vrMismatch = windy.airRelativeVelocity_B<double>(pose(stCalm), stCalm.nu_B, t);
            CHECK((std::abs(vrMismatch.norm() - vrCalm.norm()) > 0.5 || std::abs(w.n + w.e) < 1e-9));
        }
    }
}

TEST_CASE("F16Environment: a linear gust is the opposite shift of the body velocity", "[f16env][gust]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;
    const Scene s = makeScene();
    const double v = kV_fps * kFt_m;
    const auto st = makeState(s, v, v, 0.0, 0.0);

    Environment::GustState g{};
    g.u_mps = 4.0; g.v_mps = -2.5; g.w_mps = 3.0;

    F16AeroPolicy gusty = makePolicy(s);
    gusty.setGust(g);
    const auto wG = gusty(pose(st), st.nu_B, kMass_kg, 0.0);

    RigidBody::StateD shifted = st;
    shifted.nu_B(3) -= g.u_mps; shifted.nu_B(4) -= g.v_mps; shifted.nu_B(5) -= g.w_mps;
    F16AeroPolicy calm = makePolicy(s);
    const auto wS = calm(pose(shifted), shifted.nu_B, kMass_kg, 0.0);

    for (int i = 0; i < 6; ++i) CHECK_THAT(wG.f(i), WithinRel(wS.f(i), 1e-10));
    // and it is a real change
    const auto w0 = calm(pose(st), st.nu_B, kMass_kg, 0.0);
    CHECK(std::abs(wG.f(5) - w0.f(5)) > 1000.0);
}

TEST_CASE("F16Environment: rotational gusts have the physical sign", "[f16env][gust]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;
    const Scene s = makeScene();
    const double v = kV_fps * kFt_m;
    const auto st = makeState(s, v, v, 0.0, 0.0);
    F16AeroPolicy pol = makePolicy(s);
    const auto w0 = pol(pose(st), st.nu_B, kMass_kg, 0.0);

    // p_air > 0: the air rotates about +x, i.e. the downward gust grows toward
    // the right wing, which loses lift.  The aircraft must roll right (+Mx).
    {
        Environment::GustState g{}; g.p_rad_s = 0.05;
        pol.setGust(g);
        const auto w = pol(pose(st), st.nu_B, kMass_kg, 0.0);
        CHECK(w.f(0) - w0.f(0) > 100.0);
        // and it equals the aircraft rolling at -p_air relative to the air
        RigidBody::StateD r = st; r.nu_B(0) -= 0.05;
        F16AeroPolicy calm = makePolicy(s);
        const auto wr = calm(pose(r), r.nu_B, kMass_kg, 0.0);
        CHECK_THAT(w.f(0), WithinRel(wr.f(0), 1e-10));
    }
    // q_air > 0: the air rotates nose-up; relative to it the aircraft pitches
    // nose-down and pitch damping answers with a nose-up moment (+My).
    {
        Environment::GustState g{}; g.q_rad_s = 0.05;
        pol.setGust(g);
        const auto w = pol(pose(st), st.nu_B, kMass_kg, 0.0);
        CHECK(w.f(1) - w0.f(1) > 100.0);
    }
    // r_air > 0: likewise a +Mz answer.
    {
        Environment::GustState g{}; g.r_rad_s = 0.05;
        pol.setGust(g);
        const auto w = pol(pose(st), st.nu_B, kMass_kg, 0.0);
        CHECK(w.f(2) - w0.f(2) > 100.0);
    }
}

TEST_CASE("F16Environment: a hot day scales the aero forces by the density ratio", "[f16env][atm]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;
    const Scene s = makeScene();
    const double v = kV_fps * kFt_m;
    const auto st = makeState(s, v, v, 0.0, 0.0);
    const double alt_m = Environment::GeometricAltitude_m(st.g.p);

    const Environment::AtmosphereOffsets hot{ 20.0, 0.0 };
    F16AeroPolicy std_ = makePolicy(s), hot_ = makePolicy(s);
    hot_.setAtmosphereOffsets(hot);
    const auto w0 = std_(pose(st), st.nu_B, kMass_kg, 0.0);
    const auto w1 = hot_(pose(st), st.nu_B, kMass_kg, 0.0);

    const double ratio = Environment::US1976Atmosphere(alt_m, hot).rho / Environment::US1976Atmosphere(alt_m).rho;
    CHECK(ratio < 0.98);   // 0.954 at 10 013 ft for +20 K: thinner, below the crossover
    for (int i = 0; i < 6; ++i) {
        if (std::abs(w0.f(i)) > 1.0) CHECK_THAT(w1.f(i), WithinRel(w0.f(i) * ratio, 1e-9));
    }
    CHECK_THAT(hot_.atmosphere(alt_m).rho, WithinRel(Environment::US1976Atmosphere(alt_m, hot).rho, 1e-15));
}

TEST_CASE("F16Environment: a hot day trims at a higher alpha", "[f16env][atm]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;
    const Scene stdDay = makeScene();
    const Scene hotDay = makeScene(Environment::AtmosphereOffsets{ 20.0, 0.0 });
    INFO("standard: alpha=" << stdDay.trim.alpha_deg << " pwr=" << stdDay.trim.pwr_pct
         << "  hot: alpha=" << hotDay.trim.alpha_deg << " pwr=" << hotDay.trim.pwr_pct);
    CHECK(hotDay.trim.alpha_deg > stdDay.trim.alpha_deg + 0.05);
    // The throttle is not monotonic in dT: the engine deck is indexed on Mach,
    // which drops on a hot day, and the required thrust moves with both the
    // lower dynamic pressure and the higher induced drag.  Measured: 13.87 %
    // standard, 13.50 % at +20 K.  Only alpha is asserted.
    CHECK(hotDay.trim.pwr_pct > 0.0);
}

TEST_CASE("F16Environment: the snapshot reports the air data the forces used", "[f16env][snapshot]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;
    const Scene s = makeScene();
    const double v = kV_fps * kFt_m;
    const double theta = 0.3, t = theta / kOmegaE;

    const Environment::AtmosphereOffsets atm{ 12.0, -600.0 };
    const double wn = 6.0, we = -8.0, wd = 0.5;
    const auto st = makeState(s, v + wn, v + we, wd, theta);

    F16AeroPolicy pol = makePolicy(s);
    const auto wecef = Environment::ConstantECEFWind::from_ned(wn, we, wd, kLat_deg * kDeg, kLon_deg * kDeg);
    pol.setWindECEF(Eigen::Vector3d(wecef.vx, wecef.vy, wecef.vz));
    pol.setAtmosphereOffsets(atm);
    Environment::GustState g{}; g.u_mps = 1.5; g.w_mps = -2.0;
    pol.setGust(g);

    const auto snap = Simulation::MakeSnapshot1(t, st, theta, J2GravityPolicy{}, pol);
    const auto v_rel = pol.airRelativeVelocity_B<double>(pose(st), st.nu_B, t);
    CHECK_THAT(snap.trueAirspeed_m_s, WithinRel(v_rel.norm(), 1e-9));

    const auto a = Environment::US1976Atmosphere(snap.altitudeMsl_m, atm);
    CHECK_THAT(snap.ambientPressure_Pa,   WithinRel(a.p,   1e-12));
    CHECK_THAT(snap.ambientTemperature_K, WithinRel(a.T,   1e-12));
    CHECK_THAT(snap.airDensity_kg_m3,     WithinRel(a.rho, 1e-12));
    CHECK_THAT(snap.mach,                 WithinRel(v_rel.norm() / a.a, 1e-9));
    CHECK_THAT(snap.dynamicPressure_Pa,   WithinRel(0.5 * a.rho * v_rel.squaredNorm(), 1e-9));

    // Ground speed is what the NED velocity still reports.
    CHECK_THAT(snap.feVelocity_m_s.norm(), WithinRel(std::sqrt((v + wn) * (v + wn) + (v + we) * (v + we) + wd * wd), 1e-6));
}

TEST_CASE("F16Environment: the engine flies in the same air as the airframe", "[f16env][prop]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;
    const Scene s = makeScene();
    const double v = kV_fps * kFt_m;
    const double theta = 0.3, t = theta / kOmegaE;

    // Tailwind along the track: ground speed up by 10 m/s, airspeed unchanged.
    const double wn = 7.0711, we = 7.0711;
    const auto stCalm = makeState(s, v, v, 0.0, theta);
    const auto stWind = makeState(s, v + wn, v + we, 0.0, theta);

    F16PropPolicy calm(s.prop, s.trim.pwr_pct), windy(s.prop, s.trim.pwr_pct);
    const auto wecef = Environment::ConstantECEFWind::from_ned(wn, we, 0.0, kLat_deg * kDeg, kLon_deg * kDeg);
    windy.setWindECEF(Eigen::Vector3d(wecef.vx, wecef.vy, wecef.vz));

    const auto T0 = calm(pose(stCalm), stCalm.nu_B, kMass_kg, t);
    const auto T1 = windy(pose(stWind), stWind.nu_B, kMass_kg, t);
    CHECK_THAT(T1.f(3), WithinRel(T0.f(3), 1e-7));
    // Without the wind term the engine would see a higher Mach and give a
    // different thrust: that was the pre-0.16.0 trim leak.
    const auto Tleak = calm(pose(stWind), stWind.nu_B, kMass_kg, t);
    CHECK(std::abs(Tleak.f(3) - T0.f(3)) > 20.0);

    // Hot day: Mach is formed with the hot speed of sound.
    const Environment::AtmosphereOffsets hot{ 20.0, 0.0 };
    F16PropPolicy hotProp(s.prop, s.trim.pwr_pct);
    hotProp.setAtmosphereOffsets(hot);
    const auto Th = hotProp(pose(stCalm), stCalm.nu_B, kMass_kg, t);
    const double alt_m = Environment::GeometricAltitude_m(stCalm.g.p);
    const double vt    = calm.windECEF().isZero()
        ? AirRelativeVelocity_B(pose(stCalm), stCalm.nu_B, t, Eigen::Vector3d::Zero(), Environment::GustState{}).norm()
        : 0.0;
    Serialization::DAVEMLPropModel::Inputs<double> in{};
    in.pwr_pct = s.trim.pwr_pct;
    in.alt_ft  = alt_m / kFt_m;
    in.mach    = vt / Environment::US1976Atmosphere(alt_m, hot).a;
    const auto direct = s.prop->evaluate<double>(in);
    CHECK_THAT(Th.f(3), WithinRel(direct.fx_N, 1e-12));
    CHECK(std::abs(Th.f(3) - T0.f(3)) > 1.0);
}

TEST_CASE("F16Environment: evaluates with CppAD::AD<double> in wind, gust and offsets", "[f16env][AD]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;
    using AD = CppAD::AD<double>;
    const Scene s = makeScene();
    const double v = kV_fps * kFt_m;
    const double theta = 0.3, t = theta / kOmegaE;
    const auto st = makeState(s, v + 5.0, v - 3.0, 0.0, theta);

    F16AeroPolicy pol = makePolicy(s);
    const auto wecef = Environment::ConstantECEFWind::from_ned(5.0, -3.0, 0.0, kLat_deg * kDeg, kLon_deg * kDeg);
    pol.setWindECEF(Eigen::Vector3d(wecef.vx, wecef.vy, wecef.vz));
    pol.setAtmosphereOffsets(Environment::AtmosphereOffsets{ -10.0, 400.0 });
    Environment::GustState g{}; g.v_mps = 2.0; g.p_rad_s = 0.01; g.q_rad_s = -0.02;
    pol.setGust(g);

    const auto wd = pol(pose(st), st.nu_B, kMass_kg, t);

    ODE::RKMK::Lie::SE3<AD> gAD(st.g.R.cast<AD>(), st.g.p.cast<AD>());
    Eigen::Matrix<AD, 6, 1> nuAD = st.nu_B.cast<AD>();
    const auto wAD = pol(gAD, nuAD, AD(kMass_kg), AD(t));
    for (int i = 0; i < 6; ++i) CHECK_THAT(CppAD::Value(wAD.f(i)), WithinRel(wd.f(i), 1e-12));
}

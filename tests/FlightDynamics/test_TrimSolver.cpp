// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_TrimSolver.cpp
//
// Smoke tests for the Newton-Raphson trim solver using the F-16 DAVE-ML models.
//
// Reference flight condition: NASA TM-2015-218675, Check-Case 11
//   V = 565.685 ft/s  (335.15 KTAS)
//   h = 10 013 ft
//   α ≈ 2.643°,  δe ≈ −1.24°  (from reference initial state, simulation 02)
//   Body rates = 0,  β = 0
//
// The smoke tests use that figure with a 0.1° tolerance.  The [apparent] test
// is the tight one: it trims against the level-flight apparent weight and
// checks α to 1e-4° against the reference simulations that actually hold
// altitude — see the comment there on why simulation 02 is not one of them.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Aetherion/FlightDynamics/Trim/TrimSolver.h>
#include <Aetherion/FlightDynamics/Trim/TrimBodyRates.h>
#include <Aetherion/FlightDynamics/Trim/TrimWeight.h>
#include <Aetherion/FlightDynamics/Policies/F16/F16AeroPolicy.h>
#include <Aetherion/FlightDynamics/Policies/F16/F16PropPolicy.h>
#include <Aetherion/RigidBody/BuildInitialState.h>
#include <Aetherion/RigidBody/Config.h>
#include <Aetherion/RigidBody/StateLayout.h>

#include <memory>

using namespace Aetherion::FlightDynamics;
using namespace Aetherion::Serialization;
using Catch::Matchers::WithinAbs;

// Injected at CMake configure time (see CMakeLists.txt)
// AV Rule 30 deviation (#define for constants): see CODING_STANDARDS.md,
// Pre-Processing Directives.
#ifndef DAVEML_F16_AERO_FILE
#  define DAVEML_F16_AERO_FILE ""
#endif
#ifndef DAVEML_F16_PROP_FILE
#  define DAVEML_F16_PROP_FILE ""
#endif
static const std::string kAeroFile = DAVEML_F16_AERO_FILE;
static const std::string kPropFile = DAVEML_F16_PROP_FILE;

// ── Scenario-11 flight condition (English units) ──────────────────────────────
static constexpr double kVt_fps     = 565.685;   // 335.15 KTAS → ft/s
static constexpr double kAlt_ft     = 10013.0;
// Weight from DML mass (637.1596 slug) × local gravity (32.18876 ft/s²)
static constexpr double kWeight_lbf = 20509.4;  // [lbf]

// ── AC→CG offset: (35%−25%) × 11.32 ft = 1.132 ft ───────────────────────────
static constexpr double kXcg_ft     = 1.132;    // [ft]

// ── Reference trim values from NASA Atmos_11 ─────────────────────────────────
static constexpr double kRefAlpha_deg = 2.643;   // [deg]

// ── Tests ─────────────────────────────────────────────────────────────────────

TEST_CASE("TrimSolver: model files are present", "[trim][smoke]")
{
    REQUIRE_FALSE(kAeroFile.empty());
    REQUIRE_FALSE(kPropFile.empty());
}

TEST_CASE("TrimSolver: converges for Scenario-11 flight condition", "[trim][smoke]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;

    DAVEMLAeroModel aero(kAeroFile);
    DAVEMLPropModel prop(kPropFile);
    TrimSolver      solver(aero, prop, kXcg_ft);

    TrimInputs in{};
    in.vt_fps     = kVt_fps;
    in.alt_ft     = kAlt_ft;
    in.weight_lbf = kWeight_lbf;

    const TrimPoint tp = solver.solve(in);

    INFO("alpha=" << tp.alpha_deg << "  el=" << tp.el_deg
         << "  pwr=" << tp.pwr_pct << "  |r|=" << tp.residual_norm);

    CHECK(tp.converged);
    CHECK(tp.residual_norm < 0.1);  // residual in lbf
}

TEST_CASE("TrimSolver: trim alpha matches NASA reference", "[trim][smoke]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;

    DAVEMLAeroModel aero(kAeroFile);
    DAVEMLPropModel prop(kPropFile);
    TrimSolver      solver(aero, prop, kXcg_ft);

    TrimInputs in{};
    in.vt_fps     = kVt_fps;
    in.alt_ft     = kAlt_ft;
    in.weight_lbf = kWeight_lbf;

    const TrimPoint tp = solver.solve(in);

    CHECK_THAT(tp.alpha_deg, WithinAbs(kRefAlpha_deg, 0.1));
}

TEST_CASE("TrimSolver: apparent-weight trim matches the altitude-holding NESC simulations",
    "[trim][apparent]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;

    // Initial pitch attitude (= α in level flight) of NASA TM-2015-218675
    // reference simulation 05, which holds altitude to a fraction of a foot
    // through both trim checks.  Simulation 02 — the source of the 2.643° quoted
    // above — trims without the rotating-Earth relief and climbs 51 ft (Scenario
    // 11) and 244 ft (Scenario 12) over the same run, so it is not a trim
    // reference.
    struct Case {
        const char* name;
        double lat_deg, alt_ft, heading_deg, vNorth_fps, vEast_fps, refAlpha_deg;
    };

    const Case cases[] = {
        { "11 subsonic, heading 045",    36.01917, 10013.0, 45.0,  400.0,      400.0,      2.63893 },
        { "12 Mach 2, heading 045",      36.01917, 30013.0, 45.0, 1414.2136,  1414.2136,  -0.74158 },
        { "15 near pole, heading 090",   89.95,    10000.0, 90.0,    0.0,      563.643,    2.68722 },
        { "16 equator, heading 000",      0.0,     10000.0,  0.0,  563.643,      0.0,      2.66540 },
    };

    constexpr double kMass_kg = 9298.6439;  // 637.1596 slug, from F16_inertia.dml
    constexpr double kFt_m    = TrimSolver::kFt_m;

    DAVEMLAeroModel aero(kAeroFile);
    DAVEMLPropModel prop(kPropFile);
    TrimSolver      solver(aero, prop, kXcg_ft);

    for (const auto& c : cases) {
        TrimInputs in{};
        in.vt_fps    = std::hypot(c.vNorth_fps, c.vEast_fps);
        in.alt_ft    = c.alt_ft;
        in.bodyRates = LevelFlightBodyRates(c.lat_deg, c.alt_ft * kFt_m,
                                            c.vNorth_fps * kFt_m, c.vEast_fps * kFt_m,
                                            c.heading_deg, 0.0, 0.0);

        in.weight_lbf = TrimWeight_lbf(kMass_kg, c.lat_deg, c.alt_ft * kFt_m);
        const TrimPoint tpStatic = solver.solve(in);

        in.weight_lbf = LevelFlightTrimWeight_lbf(kMass_kg, c.lat_deg, c.alt_ft * kFt_m,
                                                  c.vNorth_fps * kFt_m, c.vEast_fps * kFt_m);
        const TrimPoint tp = solver.solve(in);

        INFO("Scenario " << c.name << ":  alpha=" << tp.alpha_deg
             << "  (static weight: " << tpStatic.alpha_deg << ")"
             << "  ref=" << c.refAlpha_deg
             << "  el=" << tp.el_deg << "  pwr=" << tp.pwr_pct);

        REQUIRE(tp.converged);
        CHECK_THAT(tp.alpha_deg, WithinAbs(c.refAlpha_deg, 1.0e-4));

        // The relief always lowers α.  Trimming against the static weight misses
        // the reference by 0.002° (near the pole, where there is no surface speed
        // to add to) up to 0.017° — outside the tolerance above in every case.
        CHECK(tp.alpha_deg < tpStatic.alpha_deg);
        CHECK(std::abs(tpStatic.alpha_deg - c.refAlpha_deg) > 1.5e-3);
    }
}

TEST_CASE("TrimSolver: the trim point is an equilibrium of F16AeroPolicy", "[trim][apparent]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;

    // The trim solver and the aero policy evaluate the same DAVE-ML model, but
    // each works out its own altitude, density, airspeed, α and body rates — the
    // solver from the flight condition, the policy from the ECI state.  Build
    // the initial state the examples build and ask the policy what it actually
    // produces there.  Mid-latitude on purpose: a 24 m altitude error in the
    // policy (0.25 % in density, 50 lbf) vanished at the equator and the poles.
    struct Case { const char* name; double alt_ft, v_fps; };
    const Case cases[] = { { "11", 10013.0,  400.0 }, { "12", 30013.0, 1414.2136 } };

    constexpr double kLat_deg = 36.01917, kLon_deg = -75.67444, kHeading_deg = 45.0;
    constexpr double kMass_kg = 9298.6439;
    constexpr double kFt_m    = TrimSolver::kFt_m;

    const auto aero = std::make_shared<const DAVEMLAeroModel>(kAeroFile);
    DAVEMLPropModel prop(kPropFile);
    TrimSolver      solver(*aero, prop, kXcg_ft);

    for (const auto& c : cases) {
        const double alt_m = c.alt_ft * kFt_m, v = c.v_fps * kFt_m;

        TrimInputs in{};
        in.vt_fps     = std::hypot(c.v_fps, c.v_fps);
        in.alt_ft     = c.alt_ft;
        in.weight_lbf = LevelFlightTrimWeight_lbf(kMass_kg, kLat_deg, alt_m, v, v);
        in.bodyRates  = LevelFlightBodyRates(kLat_deg, alt_m, v, v, kHeading_deg, 0.0, 0.0);
        const TrimPoint tp = solver.solve(in);
        REQUIRE(tp.converged);

        Aetherion::RigidBody::Config cfg{};
        cfg.pose.lat_deg          = kLat_deg;
        cfg.pose.lon_deg          = kLon_deg;
        cfg.pose.alt_m            = alt_m;
        cfg.pose.azimuth_deg      = kHeading_deg;
        cfg.pose.zenith_deg       = 90.0 - tp.alpha_deg;
        cfg.pose.roll_deg         = 0.0;
        cfg.velocityNED.north_mps = v;
        cfg.velocityNED.east_mps  = v;
        cfg.bodyRates = LevelFlightBodyRates(kLat_deg, alt_m, v, v, kHeading_deg, tp.alpha_deg, 0.0);
        cfg.inertialParameters.mass_kg = kMass_kg;

        const auto x0 = Aetherion::RigidBody::BuildInitialStateVector(cfg, 0.3);
        using L = Aetherion::RigidBody::StateLayout;

        const Eigen::Quaterniond q(x0[L::IDX_Q], x0[L::IDX_Q + 1], x0[L::IDX_Q + 2], x0[L::IDX_Q + 3]);
        const Aetherion::ODE::RKMK::Lie::SE3<double> pose(
            q.normalized().toRotationMatrix(),
            Eigen::Vector3d(x0[L::IDX_P], x0[L::IDX_P + 1], x0[L::IDX_P + 2]));
        Eigen::Matrix<double, 6, 1> nu_B;
        nu_B << x0[L::IDX_W], x0[L::IDX_W + 1], x0[L::IDX_W + 2],
                x0[L::IDX_V], x0[L::IDX_V + 1], x0[L::IDX_V + 2];

        const F16AeroPolicy policy(aero, tp.el_deg, 0.0, 0.0, kXcg_ft * kFt_m);
        const auto w = policy(pose, nu_B, kMass_kg, 0.0);

        const double fz_lbf   = w.f(5) / TrimSolver::kLbf_N;
        const double my_ftlbf = w.f(1) / TrimSolver::kFtLbf_Nm;
        INFO("Scenario " << c.name << ":  Fz=" << fz_lbf << " lbf  My_cg=" << my_ftlbf << " ft.lbf");

        CHECK_THAT(fz_lbf, WithinAbs(-in.weight_lbf * std::cos(tp.alpha_deg * TrimSolver::kDegRad), 0.2));
        CHECK_THAT(my_ftlbf, WithinAbs(0.0, 0.5));
    }
}

TEST_CASE("TrimSolver: force balance at trim", "[trim][smoke]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;

    DAVEMLAeroModel aero(kAeroFile);
    DAVEMLPropModel prop(kPropFile);
    TrimSolver      solver(aero, prop, kXcg_ft);

    TrimInputs in{};
    in.vt_fps     = kVt_fps;
    in.alt_ft     = kAlt_ft;
    in.weight_lbf = kWeight_lbf;

    const TrimPoint tp = solver.solve(in);
    const auto r = solver.residual(in, tp.alpha_deg, tp.el_deg, tp.pwr_pct);

    // Each force residual < 1 lbf (< 0.005% of weight)
    CHECK_THAT(r(0), WithinAbs(0.0, 1.0));  // body-X force [lbf]
    CHECK_THAT(r(1), WithinAbs(0.0, 1.0));  // body-Z force [lbf]
    CHECK_THAT(r(2), WithinAbs(0.0, 50.0)); // pitch moment [ft·lbf]
}

TEST_CASE("TrimSolver: throttle and elevator in physical range", "[trim][smoke]")
{
    if (kAeroFile.empty() || kPropFile.empty()) return;

    DAVEMLAeroModel aero(kAeroFile);
    DAVEMLPropModel prop(kPropFile);
    TrimSolver      solver(aero, prop, kXcg_ft);

    TrimInputs in{};
    in.vt_fps     = kVt_fps;
    in.alt_ft     = kAlt_ft;
    in.weight_lbf = kWeight_lbf;

    const TrimPoint tp = solver.solve(in);

    CHECK(tp.pwr_pct >= 0.0);
    CHECK(tp.pwr_pct <= 100.0);
    CHECK(tp.el_deg  >= -25.0);
    CHECK(tp.el_deg  <=  25.0);
}

TEST_CASE("TrimSolver: F16AeroPolicy concept conformance", "[trim][concept]")
{
    static_assert(AeroPolicy<F16AeroPolicy>,
        "F16AeroPolicy must satisfy AeroPolicy");
}

TEST_CASE("TrimSolver: F16PropPolicy concept conformance", "[trim][concept]")
{
    static_assert(PropulsionPolicy<F16PropPolicy>,
        "F16PropPolicy must satisfy PropulsionPolicy");
}

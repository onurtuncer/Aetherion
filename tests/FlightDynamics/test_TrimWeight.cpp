// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_TrimWeight.cpp
//
// Catch2 tests for Aetherion/FlightDynamics/Trim/TrimWeight.h
//
// The point of the helper is that the weight handed to TrimSolver is the same
// gravity J2GravityPolicy applies during integration.  These tests pin that
// agreement, the NASA TM-2015-218675 reference weight it reproduces, and the
// axisymmetry the (lat, alt)-only interface relies on.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <numbers>

#include <Aetherion/FlightDynamics/Trim/TrimBodyRates.h>
#include <Aetherion/FlightDynamics/Trim/TrimWeight.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>

using namespace Aetherion;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

    using SE3d = ODE::RKMK::Lie::SE3<double>;

    constexpr double kDegRad = std::numbers::pi / 180.0;
    constexpr double kFt_m   = 0.3048;

    // ── NASA TM-2015-218675 Scenario-11 trim point (Kitty Hawk, NC) ───────────
    constexpr double kLat_deg  = 36.01917;
    constexpr double kLon_deg  = -75.67444;
    constexpr double kAlt_ft   = 10013.0;
    constexpr double kAlt_m    = kAlt_ft * kFt_m;
    constexpr double kMass_kg  = 9298.6439;   // 637.1596 slug, from F16_inertia.dml

    // Reference local gravity and weight the NASA check case was built from.
    constexpr double kRefG_fps2    = 32.18876;  // [ft/s²]
    constexpr double kRefWeight_lbf = 20509.4;  // [lbf]

    /// Magnitude of the acceleration J2GravityPolicy applies to a unit mass at
    /// an ECI position — i.e. what the integrator actually pulls with.
    double policyGravityMagnitude(const Eigen::Vector3d& r_eci)
    {
        const FlightDynamics::J2GravityPolicy policy{};
        const SE3d pose(Eigen::Matrix3d::Identity(), r_eci);
        // Body frame is aligned with ECI here, so the wrench force is the ECI
        // force; unit mass makes it the acceleration.
        return policy(pose, 1.0).f.tail<3>().norm();
    }

    /// ECI position of a geodetic point at Earth rotation angle @p era_rad.
    Eigen::Vector3d geodeticToECI(double lat_deg, double lon_deg, double alt_m,
                                  double era_rad)
    {
        const auto r_ecef = Coordinate::GeodeticToECEF(lat_deg * kDegRad,
                                                       lon_deg * kDegRad, alt_m);
        const auto r_eci  = Coordinate::ECEFToECI(r_ecef, era_rad);
        return Eigen::Vector3d(r_eci[0], r_eci[1], r_eci[2]);
    }

} // namespace

// =============================================================================

TEST_CASE("TrimWeight: reproduces the NASA Scenario-11 local gravity and weight",
    "[trim][weight]")
{
    const double g = FlightDynamics::LocalGravityMagnitude_m_s2(kLat_deg, kAlt_m);

    // NASA quotes 32.18876 ft/s²; the WGS-84 J2 field gives 32.18857 ft/s².
    CHECK_THAT(g / kFt_m, WithinAbs(kRefG_fps2, 1.0e-3));

    const double w = FlightDynamics::TrimWeight_lbf(kMass_kg, kLat_deg, kAlt_m);
    CHECK_THAT(w, WithinAbs(kRefWeight_lbf, 0.5));

    // The sea-level constant this replaced is off by ~9 lbf — well outside the
    // tolerance above, so this test would fail if anyone reinstates it.
    const double w_g0 = kMass_kg * 9.80665 / FlightDynamics::kTrimLbf_N;
    CHECK(std::abs(w - w_g0) > 5.0);
}

TEST_CASE("TrimWeight: matches the gravity J2GravityPolicy applies at the same point",
    "[trim][weight]")
{
    struct Case { double lat_deg, lon_deg, alt_m, era_rad; };

    const Case cases[] = {
        { kLat_deg,  kLon_deg,  kAlt_m,           0.0   },  // Scenario 11
        { kLat_deg,  kLon_deg,  kAlt_m,           1.234 },  // ... hours later
        {  0.0,     -179.95,    10000.0 * kFt_m,  0.0   },  // equator, Scenario 16
        { 89.95,     -45.0,     10000.0 * kFt_m,  2.5   },  // near pole, Scenario 15
        {-42.0,       137.0,    30013.0 * kFt_m,  4.7   },  // southern, supersonic alt
    };

    for (const auto& c : cases) {
        INFO("lat=" << c.lat_deg << "  lon=" << c.lon_deg
             << "  alt=" << c.alt_m << "  era=" << c.era_rad);

        const double g_helper = FlightDynamics::LocalGravityMagnitude_m_s2(c.lat_deg,
                                                                          c.alt_m);
        const double g_policy = policyGravityMagnitude(
            geodeticToECI(c.lat_deg, c.lon_deg, c.alt_m, c.era_rad));

        // Longitude and Earth rotation angle only rotate the position about the
        // polar axis, which the J2 field is symmetric about — so the helper's
        // (lat, alt)-only interface loses nothing.
        CHECK_THAT(g_helper, WithinRel(g_policy, 1.0e-12));
    }
}

TEST_CASE("TrimWeight: latitude and altitude dependence has the right sign",
    "[trim][weight]")
{
    const double g_eq   = FlightDynamics::LocalGravityMagnitude_m_s2( 0.0, 0.0);
    const double g_45   = FlightDynamics::LocalGravityMagnitude_m_s2(45.0, 0.0);
    const double g_pole = FlightDynamics::LocalGravityMagnitude_m_s2(90.0, 0.0);

    // Oblateness: the poles sit closer to the centre, so gravity is stronger.
    CHECK(g_eq < g_45);
    CHECK(g_45 < g_pole);

    // Sanity band around standard gravity (no centrifugal term — this is mass
    // attraction only, matching J2GravityPolicy).
    CHECK(g_eq   > 9.7);
    CHECK(g_pole < 9.9);

    // Falls off with altitude.
    CHECK(FlightDynamics::LocalGravityMagnitude_m_s2(kLat_deg, 20000.0)
        < FlightDynamics::LocalGravityMagnitude_m_s2(kLat_deg, 0.0));
}

// =============================================================================
// Apparent weight in level flight
// =============================================================================

TEST_CASE("TrimWeight: apparent gravity at rest is plumb-bob gravity",
    "[trim][weight][apparent]")
{
    // WGS-84 normal gravity on the ellipsoid (Somigliana).  It carries the full
    // normal field where the helper has J2 only; the truncation is worth up to
    // 1.2e-4 m/s² (at the pole), hence the 2e-4 m/s² tolerance.
    for (const double lat_deg : { 0.0, 36.01917, 60.0, 89.95 }) {
        INFO("lat=" << lat_deg);
        const double s2    = std::pow(std::sin(lat_deg * kDegRad), 2);
        const double gamma = 9.7803253359 * (1.0 + 0.00193185265241 * s2)
                           / std::sqrt(1.0 - Environment::WGS84::kEccentricitySq * s2);

        CHECK_THAT(FlightDynamics::LevelFlightApparentGravity_m_s2(lat_deg, 0.0, 0.0, 0.0),
                   WithinAbs(gamma, 2.0e-4));
    }
}

TEST_CASE("TrimWeight: apparent gravity matches the path acceleration of level flight",
    "[trim][weight][apparent]")
{
    // Independent check of the kinematic relief: fly a constant-altitude path in
    // geodetic coordinates, map it to ECI with the library transforms, and take
    // the second difference.  Its component along the geodetic vertical is what
    // gravity has to supply, so  g_app = G_D − a_down.
    struct Case { double lat_deg, alt_m, vN, vE; };

    const Case cases[] = {
        { kLat_deg, kAlt_m,           121.92,   121.92 },  // Scenario 11, heading 045
        { kLat_deg, 30013.0 * kFt_m,  431.05,   431.05 },  // Scenario 12, Mach 2
        {  0.0,     10000.0 * kFt_m,    0.0,   -171.80 },  // equator, westbound
        { 60.0,     5000.0,          -200.0,     50.0  },  // southbound, high latitude
    };

    constexpr double kOmega = Environment::WGS84::kRotationRate_rad_s;
    constexpr double kE2    = Environment::WGS84::kEccentricitySq;
    constexpr double kA     = Environment::WGS84::kSemiMajorAxis_m;

    for (const auto& c : cases) {
        INFO("lat=" << c.lat_deg << "  alt=" << c.alt_m << "  vN=" << c.vN << "  vE=" << c.vE);

        const double lat0 = c.lat_deg * kDegRad;
        const double w2   = 1.0 - kE2 * std::sin(lat0) * std::sin(lat0);
        const double N    = kA / std::sqrt(w2);
        const double M    = N * (1.0 - kE2) / w2;

        const double latRate = c.vN / (M + c.alt_m);
        const double lonRate = c.vE / ((N + c.alt_m) * std::cos(lat0));

        const auto r_eci = [&](double t) {
            const auto r_ecef = Coordinate::GeodeticToECEF(lat0 + latRate * t,
                                                           lonRate * t, c.alt_m);
            const auto r      = Coordinate::ECEFToECI(r_ecef, kOmega * t);
            return Eigen::Vector3d(r[0], r[1], r[2]);
        };

        constexpr double dt = 1.0;
        const Eigen::Vector3d a_eci = (r_eci(dt) - 2.0 * r_eci(0.0) + r_eci(-dt)) / (dt * dt);
        const Eigen::Vector3d down(-std::cos(lat0), 0.0, -std::sin(lat0));

        const double g_static = FlightDynamics::LevelFlightApparentGravity_m_s2(
            c.lat_deg, c.alt_m, 0.0, 0.0)
            + std::pow(kOmega * (N + c.alt_m) * std::cos(lat0), 2) / (N + c.alt_m);  // = G_D
        const double g_app = FlightDynamics::LevelFlightApparentGravity_m_s2(
            c.lat_deg, c.alt_m, c.vN, c.vE);

        CHECK_THAT(g_static - g_app, WithinAbs(a_eci.dot(down), 1.0e-7));
    }
}

TEST_CASE("TrimWeight: Eotvos effect has the right sign", "[trim][weight][apparent]")
{
    const double g_rest = FlightDynamics::LevelFlightApparentGravity_m_s2(kLat_deg, kAlt_m, 0.0,    0.0);
    const double g_east = FlightDynamics::LevelFlightApparentGravity_m_s2(kLat_deg, kAlt_m, 0.0,  200.0);
    const double g_west = FlightDynamics::LevelFlightApparentGravity_m_s2(kLat_deg, kAlt_m, 0.0, -200.0);

    // Eastbound adds to the surface speed, westbound subtracts from it.
    CHECK(g_east < g_rest);
    CHECK(g_west > g_rest);

    // Direction of a meridional leg does not matter.
    CHECK_THAT(FlightDynamics::LevelFlightApparentGravity_m_s2(kLat_deg, kAlt_m,  200.0, 0.0),
               WithinRel(FlightDynamics::LevelFlightApparentGravity_m_s2(kLat_deg, kAlt_m, -200.0, 0.0),
                         1.0e-15));
}

TEST_CASE("TrimWeight: reproduces the NESC body-Z aero force at trim",
    "[trim][weight][apparent]")
{
    // NASA TM-2015-218675 reference simulations 04 and 05 hold altitude through
    // the whole trim check, so their t = 0 body-Z aero force is the apparent
    // weight resolved on body Z (the F-16 thrust line has no Z component):
    //   −FZ_aero = W_app · cos θ
    struct Case { double alt_m, v_mps, theta_deg, refFz_lbf; };

    const Case cases[] = {
        { kAlt_m,           400.0  * kFt_m,       2.6389, 20401.30 },  // Scenario 11
        { 30013.0 * kFt_m,  1414.2136 * kFt_m,   -0.7416, 20193.83 },  // Scenario 12
    };

    for (const auto& c : cases) {
        INFO("alt=" << c.alt_m << "  v=" << c.v_mps);
        const double w_app = FlightDynamics::LevelFlightTrimWeight_lbf(
            kMass_kg, kLat_deg, c.alt_m, c.v_mps, c.v_mps);

        CHECK_THAT(w_app * std::cos(c.theta_deg * kDegRad), WithinAbs(c.refFz_lbf, 1.0));

        // The static weight misses by the full relief — 86 lbf and 275 lbf.
        const double w_static = FlightDynamics::TrimWeight_lbf(kMass_kg, kLat_deg, c.alt_m);
        CHECK(w_static - w_app > 80.0);
    }
}

// =============================================================================
// Level-flight body rates
// =============================================================================

TEST_CASE("TrimBodyRates: reproduce the NESC initial inertial pitch and roll rates",
    "[trim][rates]")
{
    // bodyAngularRateWrtEi at t = 0 of NASA TM-2015-218675 reference simulation
    // 05, [deg/s].  Inertial rate = Earth rate + transport rate, both resolved
    // on body axes; BuildInitialStateVector supplies the first, the helper the
    // second.  Yaw is not compared: simulation 05 carries the rhumb-line term
    // −V_E tan φ/(N+h), which the helper leaves out on purpose.
    struct Case { double alt_m, v_mps, theta_deg, refRoll_deg_s, refPitch_deg_s; };

    const Case cases[] = {
        { kAlt_m,           400.0     * kFt_m,  2.63893, 0.002533, -0.003939 },  // Scenario 11
        { 30013.0 * kFt_m,  1414.2136 * kFt_m, -0.74158, 0.002309, -0.007864 },  // Scenario 12
    };

    constexpr double kHeading_deg = 45.0;
    constexpr double kOmega       = Environment::WGS84::kRotationRate_rad_s;

    for (const auto& c : cases) {
        INFO("alt=" << c.alt_m << "  v=" << c.v_mps);

        const auto rates = FlightDynamics::LevelFlightBodyRates(
            kLat_deg, c.alt_m, c.v_mps, c.v_mps, kHeading_deg, c.theta_deg, 0.0);

        // Earth rate in NED, then NED → body (yaw, pitch; roll = 0)
        const double lat = kLat_deg * kDegRad, psi = kHeading_deg * kDegRad,
                     the = c.theta_deg * kDegRad;
        const double eN = kOmega * std::cos(lat), eD = -kOmega * std::sin(lat);
        const double x1 =  std::cos(psi) * eN;
        const double y1 = -std::sin(psi) * eN;
        const double earthRoll  = std::cos(the) * x1 - std::sin(the) * eD;
        const double earthPitch = y1;

        CHECK_THAT((rates.pitch_rad_s + earthPitch) / kDegRad, WithinAbs(c.refPitch_deg_s, 2.0e-6));
        // Roll differs by the omitted vertical term times sin θ — under 1e-4 °/s.
        CHECK_THAT((rates.roll_rad_s  + earthRoll)  / kDegRad, WithinAbs(c.refRoll_deg_s,  1.0e-4));
    }
}

TEST_CASE("TrimBodyRates: pitch rate is the transport rate, nose-down", "[trim][rates]")
{
    // Due north at the equator: the whole transport rate is about body −y.
    const double v = 171.8, h = 3048.0;
    const auto rates = FlightDynamics::LevelFlightBodyRates(0.0, h, v, 0.0, 0.0, 0.0, 0.0);
    const auto radii = FlightDynamics::WGS84CurvatureRadii(0.0);

    CHECK_THAT(rates.pitch_rad_s, WithinRel(-v / (radii.meridian_m + h), 1.0e-14));
    CHECK_THAT(rates.roll_rad_s,  WithinAbs(0.0, 1.0e-18));
    CHECK_THAT(rates.yaw_rad_s,   WithinAbs(0.0, 1.0e-18));

    // Finite near the pole, where the rhumb-line yaw term would not be.
    const auto polar = FlightDynamics::LevelFlightBodyRates(89.95, h, 0.0, v, 90.0, 2.7, 0.0);
    CHECK(std::abs(polar.yaw_rad_s) < 1.0e-5);
}

TEST_CASE("TrimWeight: weight is linear in mass", "[trim][weight]")
{
    const double w1 = FlightDynamics::TrimWeight_lbf(1000.0, kLat_deg, kAlt_m);
    const double w3 = FlightDynamics::TrimWeight_lbf(3000.0, kLat_deg, kAlt_m);
    CHECK_THAT(w3, WithinRel(3.0 * w1, 1.0e-14));
}

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

TEST_CASE("TrimWeight: weight is linear in mass", "[trim][weight]")
{
    const double w1 = FlightDynamics::TrimWeight_lbf(1000.0, kLat_deg, kAlt_m);
    const double w3 = FlightDynamics::TrimWeight_lbf(3000.0, kLat_deg, kAlt_m);
    CHECK_THAT(w3, WithinRel(3.0 * w1, 1.0e-14));
}

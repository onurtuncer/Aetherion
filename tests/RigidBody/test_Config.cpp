// ------------------------------------------------------------------------------
// Project: Aetherion — Catch2 tests for RigidBody::Config and sub-structs
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Covers:
//   RigidBody::GeodeticPoseNED
//   RigidBody::VelocityNED
//   RigidBody::BodyRates
//   RigidBody::InertialParameters
//   RigidBody::AerodynamicParameters
//   RigidBody::Config  (aggregate of the above)
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Aetherion/RigidBody/Config.h>

using Catch::Approx;
namespace RB = Aetherion::RigidBody;

// =============================================================================
// GeodeticPoseNED
// =============================================================================

TEST_CASE("GeodeticPoseNED default-constructs to all zeros", "[GeodeticPoseNED][RigidBody]") {
    const RB::GeodeticPoseNED p{};
    REQUIRE(p.lat_deg      == Approx(0.0));
    REQUIRE(p.lon_deg      == Approx(0.0));
    REQUIRE(p.alt_m        == Approx(0.0));
    REQUIRE(p.azimuth_deg  == Approx(0.0));
    REQUIRE(p.zenith_deg   == Approx(0.0));
    REQUIRE(p.roll_deg     == Approx(0.0));
}

TEST_CASE("GeodeticPoseNED fields can be assigned independently", "[GeodeticPoseNED][RigidBody]") {
    RB::GeodeticPoseNED p{};
    p.lat_deg     = 28.6;
    p.lon_deg     = -80.6;
    p.alt_m       = 9144.0;
    p.azimuth_deg = 90.0;
    p.zenith_deg  = 45.0;
    p.roll_deg    = 10.0;

    REQUIRE(p.lat_deg     == Approx(28.6));
    REQUIRE(p.lon_deg     == Approx(-80.6));
    REQUIRE(p.alt_m       == Approx(9144.0));
    REQUIRE(p.azimuth_deg == Approx(90.0));
    REQUIRE(p.zenith_deg  == Approx(45.0));
    REQUIRE(p.roll_deg    == Approx(10.0));
}

TEST_CASE("GeodeticPoseNED fields are independent of each other", "[GeodeticPoseNED][RigidBody]") {
    RB::GeodeticPoseNED p{};
    p.lat_deg = 51.5;

    // All other fields must remain at their defaults
    REQUIRE(p.lon_deg     == Approx(0.0));
    REQUIRE(p.alt_m       == Approx(0.0));
    REQUIRE(p.azimuth_deg == Approx(0.0));
    REQUIRE(p.zenith_deg  == Approx(0.0));
    REQUIRE(p.roll_deg    == Approx(0.0));
}

TEST_CASE("GeodeticPoseNED supports negative coordinates", "[GeodeticPoseNED][RigidBody]") {
    RB::GeodeticPoseNED p{};
    p.lat_deg = -33.9;
    p.lon_deg = -70.6;
    p.alt_m   = -50.0;

    REQUIRE(p.lat_deg == Approx(-33.9));
    REQUIRE(p.lon_deg == Approx(-70.6));
    REQUIRE(p.alt_m   == Approx(-50.0));
}

// =============================================================================
// VelocityNED
// =============================================================================

TEST_CASE("VelocityNED default-constructs to all zeros", "[VelocityNED][RigidBody]") {
    const RB::VelocityNED v{};
    REQUIRE(v.north_mps == Approx(0.0));
    REQUIRE(v.east_mps  == Approx(0.0));
    REQUIRE(v.down_mps  == Approx(0.0));
}

TEST_CASE("VelocityNED fields can be assigned independently", "[VelocityNED][RigidBody]") {
    RB::VelocityNED v{};
    v.north_mps = 100.0;
    v.east_mps  = -50.0;
    v.down_mps  =  20.0;

    REQUIRE(v.north_mps == Approx(100.0));
    REQUIRE(v.east_mps  == Approx(-50.0));
    REQUIRE(v.down_mps  == Approx(20.0));
}

TEST_CASE("VelocityNED north does not affect east or down", "[VelocityNED][RigidBody]") {
    RB::VelocityNED v{};
    v.north_mps = 300.0;

    REQUIRE(v.east_mps == Approx(0.0));
    REQUIRE(v.down_mps == Approx(0.0));
}

// =============================================================================
// BodyRates
// =============================================================================

TEST_CASE("BodyRates default-constructs to all zeros", "[BodyRates][RigidBody]") {
    const RB::BodyRates r{};
    REQUIRE(r.roll_rad_s  == Approx(0.0));
    REQUIRE(r.pitch_rad_s == Approx(0.0));
    REQUIRE(r.yaw_rad_s   == Approx(0.0));
}

TEST_CASE("BodyRates fields can be assigned independently", "[BodyRates][RigidBody]") {
    RB::BodyRates r{};
    r.roll_rad_s  =  0.12;
    r.pitch_rad_s = -0.07;
    r.yaw_rad_s   =  0.03;

    REQUIRE(r.roll_rad_s  == Approx( 0.12));
    REQUIRE(r.pitch_rad_s == Approx(-0.07));
    REQUIRE(r.yaw_rad_s   == Approx( 0.03));
}

TEST_CASE("BodyRates roll does not affect pitch or yaw", "[BodyRates][RigidBody]") {
    RB::BodyRates r{};
    r.roll_rad_s = 1.0;

    REQUIRE(r.pitch_rad_s == Approx(0.0));
    REQUIRE(r.yaw_rad_s   == Approx(0.0));
}

// =============================================================================
// InertialParameters
// =============================================================================

TEST_CASE("InertialParameters default-constructs to all zeros", "[InertialParameters][RigidBody]") {
    const RB::InertialParameters ip{};
    REQUIRE(ip.mass_kg == Approx(0.0));
    REQUIRE(ip.Ixx     == Approx(0.0));
    REQUIRE(ip.Iyy     == Approx(0.0));
    REQUIRE(ip.Izz     == Approx(0.0));
    REQUIRE(ip.Ixy     == Approx(0.0));
    REQUIRE(ip.Iyz     == Approx(0.0));
    REQUIRE(ip.Ixz     == Approx(0.0));
    REQUIRE(ip.xbar_m  == Approx(0.0));
    REQUIRE(ip.ybar_m  == Approx(0.0));
    REQUIRE(ip.zbar_m  == Approx(0.0));
}

TEST_CASE("InertialParameters mass and moments can be assigned", "[InertialParameters][RigidBody]") {
    RB::InertialParameters ip{};
    ip.mass_kg = 14.5939;
    ip.Ixx     = 4.881;
    ip.Iyy     = 4.881;
    ip.Izz     = 4.881;

    REQUIRE(ip.mass_kg == Approx(14.5939));
    REQUIRE(ip.Ixx     == Approx(4.881));
    REQUIRE(ip.Iyy     == Approx(4.881));
    REQUIRE(ip.Izz     == Approx(4.881));
    // Products of inertia must remain at defaults
    REQUIRE(ip.Ixy     == Approx(0.0));
    REQUIRE(ip.Iyz     == Approx(0.0));
    REQUIRE(ip.Ixz     == Approx(0.0));
}

TEST_CASE("InertialParameters products of inertia and CoG offset can be assigned",
          "[InertialParameters][RigidBody]")
{
    RB::InertialParameters ip{};
    ip.Ixy    =  0.5;
    ip.Iyz    = -0.3;
    ip.Ixz    =  0.1;
    ip.xbar_m =  0.05;
    ip.ybar_m = -0.02;
    ip.zbar_m =  0.01;

    REQUIRE(ip.Ixy    == Approx( 0.5));
    REQUIRE(ip.Iyz    == Approx(-0.3));
    REQUIRE(ip.Ixz    == Approx( 0.1));
    REQUIRE(ip.xbar_m == Approx( 0.05));
    REQUIRE(ip.ybar_m == Approx(-0.02));
    REQUIRE(ip.zbar_m == Approx( 0.01));
    // Mass and diagonal moments must remain at defaults
    REQUIRE(ip.mass_kg == Approx(0.0));
    REQUIRE(ip.Ixx     == Approx(0.0));
}

// =============================================================================
// AerodynamicParameters
// =============================================================================

TEST_CASE("AerodynamicParameters default-constructs to all zeros", "[AerodynamicParameters][RigidBody]") {
    const RB::AerodynamicParameters ap{};
    REQUIRE(ap.S  == Approx(0.0));
    REQUIRE(ap.CL == Approx(0.0));
    REQUIRE(ap.CD == Approx(0.0));
    REQUIRE(ap.CY == Approx(0.0));
    REQUIRE(ap.Cl == Approx(0.0));
    REQUIRE(ap.Cm == Approx(0.0));
    REQUIRE(ap.Cn == Approx(0.0));
}

TEST_CASE("AerodynamicParameters fields can be assigned independently", "[AerodynamicParameters][RigidBody]") {
    RB::AerodynamicParameters ap{};
    ap.S  = 0.018241;
    ap.CD = 0.47;
    ap.CL = 0.0;
    ap.CY = 0.1;
    ap.Cl = -0.02;
    ap.Cm =  0.05;
    ap.Cn = -0.01;

    REQUIRE(ap.S  == Approx(0.018241));
    REQUIRE(ap.CD == Approx(0.47));
    REQUIRE(ap.CL == Approx(0.0));
    REQUIRE(ap.CY == Approx(0.1));
    REQUIRE(ap.Cl == Approx(-0.02));
    REQUIRE(ap.Cm == Approx( 0.05));
    REQUIRE(ap.Cn == Approx(-0.01));
}

TEST_CASE("AerodynamicParameters CD does not affect other coefficients", "[AerodynamicParameters][RigidBody]") {
    RB::AerodynamicParameters ap{};
    ap.CD = 0.47;

    REQUIRE(ap.S  == Approx(0.0));
    REQUIRE(ap.CL == Approx(0.0));
    REQUIRE(ap.CY == Approx(0.0));
    REQUIRE(ap.Cl == Approx(0.0));
    REQUIRE(ap.Cm == Approx(0.0));
    REQUIRE(ap.Cn == Approx(0.0));
}

// =============================================================================
// RigidBody::Config  (aggregate)
// =============================================================================

TEST_CASE("RigidBody::Config default-constructs with all sub-structs at their defaults",
          "[Config][RigidBody]")
{
    const RB::Config cfg{};

    // GeodeticPoseNED
    REQUIRE(cfg.pose.lat_deg     == Approx(0.0));
    REQUIRE(cfg.pose.lon_deg     == Approx(0.0));
    REQUIRE(cfg.pose.alt_m       == Approx(0.0));
    REQUIRE(cfg.pose.azimuth_deg == Approx(0.0));
    REQUIRE(cfg.pose.zenith_deg  == Approx(0.0));
    REQUIRE(cfg.pose.roll_deg    == Approx(0.0));

    // VelocityNED
    REQUIRE(cfg.velocityNED.north_mps == Approx(0.0));
    REQUIRE(cfg.velocityNED.east_mps  == Approx(0.0));
    REQUIRE(cfg.velocityNED.down_mps  == Approx(0.0));

    // BodyRates
    REQUIRE(cfg.bodyRates.roll_rad_s  == Approx(0.0));
    REQUIRE(cfg.bodyRates.pitch_rad_s == Approx(0.0));
    REQUIRE(cfg.bodyRates.yaw_rad_s   == Approx(0.0));

    // InertialParameters
    REQUIRE(cfg.inertialParameters.mass_kg == Approx(0.0));
    REQUIRE(cfg.inertialParameters.Ixx     == Approx(0.0));
    REQUIRE(cfg.inertialParameters.Iyy     == Approx(0.0));
    REQUIRE(cfg.inertialParameters.Izz     == Approx(0.0));

    // AerodynamicParameters
    REQUIRE(cfg.aerodynamicParameters.S  == Approx(0.0));
    REQUIRE(cfg.aerodynamicParameters.CD == Approx(0.0));
    REQUIRE(cfg.aerodynamicParameters.CL == Approx(0.0));
}

TEST_CASE("RigidBody::Config sub-struct fields can be set via the aggregate", "[Config][RigidBody]") {
    RB::Config cfg{};

    cfg.pose.lat_deg                = 28.6;
    cfg.pose.alt_m                  = 9144.0;
    cfg.velocityNED.north_mps       = 100.0;
    cfg.bodyRates.roll_rad_s        = -7.2921e-5;
    cfg.inertialParameters.mass_kg  = 14.5939;
    cfg.inertialParameters.Ixx      = 4.881;
    cfg.aerodynamicParameters.CD    = 0.47;
    cfg.aerodynamicParameters.S     = 0.018241;

    REQUIRE(cfg.pose.lat_deg               == Approx(28.6));
    REQUIRE(cfg.pose.alt_m                 == Approx(9144.0));
    REQUIRE(cfg.velocityNED.north_mps      == Approx(100.0));
    REQUIRE(cfg.bodyRates.roll_rad_s       == Approx(-7.2921e-5));
    REQUIRE(cfg.inertialParameters.mass_kg == Approx(14.5939));
    REQUIRE(cfg.inertialParameters.Ixx     == Approx(4.881));
    REQUIRE(cfg.aerodynamicParameters.CD   == Approx(0.47));
    REQUIRE(cfg.aerodynamicParameters.S    == Approx(0.018241));
}

TEST_CASE("RigidBody::Config sub-struct fields are independent", "[Config][RigidBody]") {
    RB::Config cfg{};
    cfg.inertialParameters.mass_kg = 500.0;

    // All other sub-struct fields must remain at their defaults
    REQUIRE(cfg.pose.lat_deg               == Approx(0.0));
    REQUIRE(cfg.velocityNED.north_mps      == Approx(0.0));
    REQUIRE(cfg.bodyRates.roll_rad_s       == Approx(0.0));
    REQUIRE(cfg.aerodynamicParameters.CD   == Approx(0.0));
}

TEST_CASE("RigidBody::Config is copyable and copies are independent", "[Config][RigidBody]") {
    RB::Config a{};
    a.pose.lat_deg               = 51.5;
    a.inertialParameters.mass_kg = 1000.0;
    a.aerodynamicParameters.CD   = 0.3;

    RB::Config b = a;
    b.pose.lat_deg               = 0.0;
    b.inertialParameters.mass_kg = 2000.0;
    b.aerodynamicParameters.CD   = 0.6;

    REQUIRE(a.pose.lat_deg               == Approx(51.5));
    REQUIRE(a.inertialParameters.mass_kg == Approx(1000.0));
    REQUIRE(a.aerodynamicParameters.CD   == Approx(0.3));

    REQUIRE(b.pose.lat_deg               == Approx(0.0));
    REQUIRE(b.inertialParameters.mass_kg == Approx(2000.0));
    REQUIRE(b.aerodynamicParameters.CD   == Approx(0.6));
}

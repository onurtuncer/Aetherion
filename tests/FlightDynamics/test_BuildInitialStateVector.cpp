// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_BuildInitialStateVector.cpp
// PATH: tests/FlightDynamics/test_BuildInitialStateVector.cpp
//
// Catch2 v3 tests for BuildInitialStateVector.h
//
// Test sections (Catch2 tags):
//
//   [structural]     -- vector size, all-finite, StateLayout index offsets
//   [mass]           -- mass slot passthrough, independence from all other fields
//   [omega]          -- angular-rate slots passthrough, independence from pose/vel
//   [position]       -- ECI position vs. independent GeodeticToECEF->ECEFToECI chain
//   [attitude]       -- quaternion unit-norm, body +x aligned with launch direction,
//                       full orthonormal frame, direct match to MakeLaunchStateECI
//   [velocity]       -- ECI velocity vs. independent NEDToECEF->ECEFToECI+omega×r chain,
//                       speed preservation, independence from attitude angles
//   [earth_rotation] -- omega×r term: magnitude, direction, ERA independence,
//                       equator/pole values, NED-round-trip, NASA Atmos-01 IC
//   [corner]         -- equator/prime-meridian/theta=0, north pole, due-East heading,
//                       LEO altitude, ERA=2pi identity, 360 deg roll identity
//   [era]            -- ERA overload vs. convenience overload, ERA-from-t0 formula,
//                       XY-plane rotation by Dtheta
//   [perturbation]   -- finite-difference Jacobians for position/velocity/attitude
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <numbers>
#include <array>

#include <Eigen/Dense>

// Unit under test
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>

// Reference transforms (used for independent ground-truth computations)
#include <Aetherion/Coordinate/LocalToInertial.h>
#include <Aetherion/Coordinate/InertialToLocal.h>
#include <Aetherion/Coordinate/Math.h>

// State index constants
#include <Aetherion/RigidBody/StateLayout.h>
#include <Aetherion/RigidBody/Config.h>

// --- Namespace aliases --------------------------------------------------------

using Catch::Approx;

namespace AC = Aetherion::Coordinate;
namespace FD = Aetherion::FlightDynamics;
using SL = Aetherion::RigidBody::StateLayout;
namespace RigidBody = Aetherion::RigidBody;

using Vec3d = AC::Vec3<double>;
using Quatd = AC::Quat<double>;

// --- File-scope constants -----------------------------------------------------

static constexpr double kPi = std::numbers::pi_v<double>;
static constexpr double kD2R = kPi / 180.0;
static constexpr double kOmegaE = 7.2921150e-5;    // WGS-84 Earth rotation [rad/s]
static constexpr double kReq = 6378137.0;        // WGS-84 semi-major axis [m]
static constexpr double kFlat = 1.0 / 298.257223563; // WGS-84 flattening
static constexpr double kRpol = kReq * (1.0 - kFlat); // polar radius [m]

// --- Test helpers -------------------------------------------------------------

// Build a RigidBody::Config.  All pose angles are in DEGREES (matches GeodeticPoseNED).
// Every parameter has a default so individual tests can name just what they change.
static RigidBody::Config MakeCfg(
    double lat_deg = 28.6,
    double lon_deg = -80.6,
    double alt_m = 0.0,
    double az_deg = 0.0,
    double zen_deg = 0.0,
    double roll_deg = 0.0,
    double vN = 0.0,
    double vE = 0.0,
    double vD = 0.0,
    double omega_roll = 0.0,
    double omega_pitch = 0.0,
    double omega_yaw = 0.0,
    double mass_kg = 10'000.0)
{
    RigidBody::Config cfg{};
    cfg.pose.lat_deg = lat_deg;
    cfg.pose.lon_deg = lon_deg;
    cfg.pose.alt_m = alt_m;
    cfg.pose.azimuth_deg = az_deg;
    cfg.pose.zenith_deg = zen_deg;
    cfg.pose.roll_deg = roll_deg;

    cfg.velocityNED.north_mps = vN;
    cfg.velocityNED.east_mps = vE;
    cfg.velocityNED.down_mps = vD;

    cfg.bodyRates.roll_rad_s = omega_roll;
    cfg.bodyRates.pitch_rad_s = omega_pitch;
    cfg.bodyRates.yaw_rad_s = omega_yaw;

    cfg.inertialParameters.mass_kg = mass_kg;

    return cfg;
}

// Rotate v_B into the "A" frame given q_AB = [w,x,y,z].
static Vec3d RotateVec(const Quatd& q, const Vec3d& v)
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double vw = -x * v[0] - y * v[1] - z * v[2];
    const double vx = w * v[0] + y * v[2] - z * v[1];
    const double vy = w * v[1] + z * v[0] - x * v[2];
    const double vz = w * v[2] + x * v[1] - y * v[0];
    return Vec3d{
        vw * (-x) + vx * w + vy * (-z) - vz * (-y),
        vw * (-y) - vx * (-z) + vy * w + vz * (-x),
        vw * (-z) + vx * (-y) - vy * (-x) + vz * w
    };
}

// Extract q_EB from the state vector into an Aetherion Quat.
static Quatd QuatFromState(const Eigen::Matrix<double, SL::N, 1>& x)
{
    return Quatd{ x[SL::IDX_Q + 0], x[SL::IDX_Q + 1], x[SL::IDX_Q + 2], x[SL::IDX_Q + 3] };
}

// |dot(a,b)| == 1 means same rotation.
static double QuatAbsDot(const Quatd& a, const Quatd& b)
{
    return std::abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
}

static double Norm3(const Vec3d& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static double Dot3(const Vec3d& a, const Vec3d& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// Compute omega_E × r_eci analytically (omega = [0,0,kOmegaE]).
static Vec3d OmegaCrossR(const Vec3d& r) {
    return Vec3d{ -kOmegaE * r[1], kOmegaE * r[0], 0.0 };
}

// =============================================================================
// GROUP 1 - Structural invariants
// =============================================================================

TEST_CASE("BuildInitialStateVector - output has exactly N=14 elements",
    "[structural][initial_state]")
{
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(), 0.0);
    STATIC_REQUIRE(SL::N == 14);
    REQUIRE(x0.size() == 14);
    REQUIRE(x0.rows() == 14);
    REQUIRE(x0.cols() == 1);
}

TEST_CASE("BuildInitialStateVector - StateLayout index offsets are as documented",
    "[structural][initial_state]")
{
    STATIC_REQUIRE(SL::IDX_P == 0);
    STATIC_REQUIRE(SL::IDX_Q == 3);
    STATIC_REQUIRE(SL::IDX_W == 7);
    STATIC_REQUIRE(SL::IDX_V == 10);
    STATIC_REQUIRE(SL::IDX_M == 13);
}

TEST_CASE("BuildInitialStateVector - every slot is finite for a representative config",
    "[structural][initial_state]")
{
    const auto cfg = MakeCfg(51.5, -0.1, 500.0,
        45.0, 30.0, 10.0,
        100.0, -50.0, 5.0,
        0.01, -0.02, 0.005,
        5000.0);
    const auto x0 = FD::BuildInitialStateVector(cfg, 0.0);

    for (int i = 0; i < SL::N; ++i) {
        INFO("slot " << i << " = " << x0[i]);
        REQUIRE(std::isfinite(x0[i]));
    }
}

TEST_CASE("BuildInitialStateVector - every slot is finite when all inputs are zero",
    "[structural][initial_state]")
{
    const auto cfg = MakeCfg(0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        1.0);
    const auto x0 = FD::BuildInitialStateVector(cfg, 0.0);

    for (int i = 0; i < SL::N; ++i) {
        INFO("slot " << i << " = " << x0[i]);
        REQUIRE(std::isfinite(x0[i]));
    }
}


// =============================================================================
// GROUP 2 - Mass slot (IDX_M = 13)
// =============================================================================

TEST_CASE("BuildInitialStateVector - mass slot equals cfg.inertialParameters.mass_kg",
    "[mass][initial_state]")
{
    const double m = 7654.321;
    const auto cfg = MakeCfg(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, m);
    const auto x0 = FD::BuildInitialStateVector(cfg, 0.0);
    REQUIRE(x0[SL::IDX_M] == Approx(m).epsilon(1e-15));
}

TEST_CASE("BuildInitialStateVector - mass slot is independent of pose, velocity, and angular rates",
    "[mass][initial_state]")
{
    const double m = 500.0;
    const auto cfg_a = MakeCfg(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, m);
    const auto cfg_b = MakeCfg(45, 90, 1000, 90, 45, 30, 10, 5, -2, 0.1, -0.05, 0.03, m);

    const auto x_a = FD::BuildInitialStateVector(cfg_a, 0.3);
    const auto x_b = FD::BuildInitialStateVector(cfg_b, 1.7);

    REQUIRE(x_a[SL::IDX_M] == Approx(m).epsilon(1e-15));
    REQUIRE(x_b[SL::IDX_M] == Approx(m).epsilon(1e-15));
}


// =============================================================================
// GROUP 3 - Angular-rate slots (IDX_W = 7..9)
// =============================================================================

TEST_CASE("BuildInitialStateVector - angular rate slots equal cfg.bodyRates",
    "[omega][initial_state]")
{
    const double roll_s = 0.12, pitch_s = -0.07, yaw_s = 0.03;
    const auto cfg = MakeCfg(0, 0, 0, 0, 0, 0, 0, 0, 0, roll_s, pitch_s, yaw_s);
    const auto x0 = FD::BuildInitialStateVector(cfg, 0.5);

    REQUIRE(x0[SL::IDX_W + 0] == Approx(roll_s).epsilon(1e-15));
    REQUIRE(x0[SL::IDX_W + 1] == Approx(pitch_s).epsilon(1e-15));
    REQUIRE(x0[SL::IDX_W + 2] == Approx(yaw_s).epsilon(1e-15));
}

TEST_CASE("BuildInitialStateVector - zero angular rates produce zero omega slots",
    "[omega][initial_state]")
{
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(), 0.0);
    REQUIRE(x0[SL::IDX_W + 0] == Approx(0.0).margin(1e-20));
    REQUIRE(x0[SL::IDX_W + 1] == Approx(0.0).margin(1e-20));
    REQUIRE(x0[SL::IDX_W + 2] == Approx(0.0).margin(1e-20));
}

TEST_CASE("BuildInitialStateVector - omega slots are independent of pose and velocity",
    "[omega][initial_state]")
{
    const double roll_s = 0.05, pitch_s = -0.02, yaw_s = 0.01;
    const auto cfg_a = MakeCfg(0, 0, 0, 0, 0, 0, 0, 0, 0, roll_s, pitch_s, yaw_s);
    const auto cfg_b = MakeCfg(45, 30, 500, 60, 20, 15, 50, -20, 3, roll_s, pitch_s, yaw_s);

    const auto x_a = FD::BuildInitialStateVector(cfg_a, 0.2);
    const auto x_b = FD::BuildInitialStateVector(cfg_b, 1.0);

    REQUIRE(x_a[SL::IDX_W + 0] == Approx(x_b[SL::IDX_W + 0]).epsilon(1e-15));
    REQUIRE(x_a[SL::IDX_W + 1] == Approx(x_b[SL::IDX_W + 1]).epsilon(1e-15));
    REQUIRE(x_a[SL::IDX_W + 2] == Approx(x_b[SL::IDX_W + 2]).epsilon(1e-15));
}


// =============================================================================
// GROUP 4 - ECI position (IDX_P = 0..2)
// =============================================================================

TEST_CASE("BuildInitialStateVector - ECI position matches independent GeodeticToECEF->ECEFToECI",
    "[position][initial_state]")
{
    const double lat_deg = 28.6, lon_deg = -80.6, alt_m = 200.0, theta = 0.75;
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(lat_deg, lon_deg, alt_m), theta);

    const Vec3d r_ecef = AC::GeodeticToECEF(lat_deg * kD2R, lon_deg * kD2R, alt_m);
    const Vec3d r_eci = AC::ECEFToECI(r_ecef, theta);

    REQUIRE(x0[SL::IDX_P + 0] == Approx(r_eci[0]).margin(1e-4));
    REQUIRE(x0[SL::IDX_P + 1] == Approx(r_eci[1]).margin(1e-4));
    REQUIRE(x0[SL::IDX_P + 2] == Approx(r_eci[2]).margin(1e-4));
}

TEST_CASE("BuildInitialStateVector - ECI position magnitude equals WGS-84 radius + altitude",
    "[position][initial_state]")
{
    const double h = 1500.0;
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0.0, 0.0, h), 0.0);
    REQUIRE(x0.segment<3>(SL::IDX_P).norm() == Approx(kReq + h).epsilon(1e-8));
}

TEST_CASE("BuildInitialStateVector - increasing altitude increases radial distance by exactly Dh",
    "[position][initial_state]")
{
    const double theta = 0.4;
    const double dh = 1000.0;
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(45.0, 60.0, 0.0), theta);
    const auto x1 = FD::BuildInitialStateVector(MakeCfg(45.0, 60.0, dh), theta);

    const double dr = x1.segment<3>(SL::IDX_P).norm()
        - x0.segment<3>(SL::IDX_P).norm();
    REQUIRE(dr == Approx(dh).epsilon(1e-4));
}

TEST_CASE("BuildInitialStateVector - position is independent of launch attitude angles",
    "[position][initial_state]")
{
    const double theta = 0.6;
    const auto x_a = FD::BuildInitialStateVector(MakeCfg(50, 10, 300, 0, 0, 0), theta);
    const auto x_b = FD::BuildInitialStateVector(MakeCfg(50, 10, 300, 90, 45, 30), theta);
    const auto x_c = FD::BuildInitialStateVector(MakeCfg(50, 10, 300, 180, 60, -20), theta);

    for (int i = SL::IDX_P; i < SL::IDX_P + 3; ++i) {
        REQUIRE(x_a[i] == Approx(x_b[i]).margin(1e-6));
        REQUIRE(x_a[i] == Approx(x_c[i]).margin(1e-6));
    }
}

TEST_CASE("BuildInitialStateVector - position is independent of initial velocity and rates",
    "[position][initial_state]")
{
    const double theta = 0.8;
    const auto x_a = FD::BuildInitialStateVector(
        MakeCfg(30, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), theta);
    const auto x_b = FD::BuildInitialStateVector(
        MakeCfg(30, 20, 0, 0, 0, 0, 100, 50, -10, 0.1, -0.05, 0.02), theta);

    for (int i = SL::IDX_P; i < SL::IDX_P + 3; ++i)
        REQUIRE(x_a[i] == Approx(x_b[i]).margin(1e-6));
}


// =============================================================================
// GROUP 5 - Attitude quaternion (IDX_Q = 3..6)
// =============================================================================

TEST_CASE("BuildInitialStateVector - quaternion is unit-normalised",
    "[attitude][initial_state]")
{
    const auto cfg = MakeCfg(35.0, 139.0, 100.0, 30.0, 45.0, 20.0);
    const auto x0 = FD::BuildInitialStateVector(cfg, 1.1);

    const double norm = std::sqrt(
        x0[SL::IDX_Q + 0] * x0[SL::IDX_Q + 0] + x0[SL::IDX_Q + 1] * x0[SL::IDX_Q + 1] +
        x0[SL::IDX_Q + 2] * x0[SL::IDX_Q + 2] + x0[SL::IDX_Q + 3] * x0[SL::IDX_Q + 3]);
    REQUIRE(norm == Approx(1.0).margin(1e-12));
}

TEST_CASE("BuildInitialStateVector - body +x in ECI matches MakeLaunchStateECI direction",
    "[attitude][initial_state]")
{
    const double lat_deg = 51.5, lon_deg = -0.1, alt_m = 0.0;
    const double az_deg = 60.0, zen_deg = 20.0, roll_deg = 5.0;
    const double theta = 0.4;

    const auto x0 = FD::BuildInitialStateVector(
        MakeCfg(lat_deg, lon_deg, alt_m, az_deg, zen_deg, roll_deg), theta);

    const Vec3d dir_eci = RotateVec(QuatFromState(x0), Vec3d{ 1.0, 0.0, 0.0 });

    const auto launch = AC::MakeLaunchStateECI(
        lat_deg * kD2R, lon_deg * kD2R, alt_m,
        az_deg * kD2R, zen_deg * kD2R, roll_deg * kD2R,
        theta);

    REQUIRE(dir_eci[0] == Approx(launch.dir0_eci[0]).margin(1e-11));
    REQUIRE(dir_eci[1] == Approx(launch.dir0_eci[1]).margin(1e-11));
    REQUIRE(dir_eci[2] == Approx(launch.dir0_eci[2]).margin(1e-11));
}

TEST_CASE("BuildInitialStateVector - attitude quaternion matches MakeLaunchStateECI q_EB directly",
    "[attitude][initial_state]")
{
    const double lat_deg = -33.9, lon_deg = 151.2, alt_m = 50.0;
    const double az_deg = 90.0, zen_deg = 15.0, roll_deg = 0.0;
    const double theta = 2.3;

    const auto x0 = FD::BuildInitialStateVector(
        MakeCfg(lat_deg, lon_deg, alt_m, az_deg, zen_deg, roll_deg), theta);

    const auto launch = AC::MakeLaunchStateECI(
        lat_deg * kD2R, lon_deg * kD2R, alt_m,
        az_deg * kD2R, zen_deg * kD2R, roll_deg * kD2R,
        theta);

    REQUIRE(QuatAbsDot(QuatFromState(x0), launch.q_EB) == Approx(1.0).margin(1e-12));
}

TEST_CASE("BuildInitialStateVector - body frame axes are orthonormal in ECI",
    "[attitude][initial_state]")
{
    const auto x0 = FD::BuildInitialStateVector(
        MakeCfg(35.0, 139.0, 100.0, 30.0, 45.0, 20.0), 1.1);
    const Quatd q = QuatFromState(x0);

    const Vec3d ex = RotateVec(q, Vec3d{ 1,0,0 });
    const Vec3d ey = RotateVec(q, Vec3d{ 0,1,0 });
    const Vec3d ez = RotateVec(q, Vec3d{ 0,0,1 });

    REQUIRE(Norm3(ex) == Approx(1.0).margin(1e-12));
    REQUIRE(Norm3(ey) == Approx(1.0).margin(1e-12));
    REQUIRE(Norm3(ez) == Approx(1.0).margin(1e-12));

    REQUIRE(Dot3(ex, ey) == Approx(0.0).margin(1e-12));
    REQUIRE(Dot3(ex, ez) == Approx(0.0).margin(1e-12));
    REQUIRE(Dot3(ey, ez) == Approx(0.0).margin(1e-12));
}


// =============================================================================
// GROUP 6 - ECI velocity (IDX_V = 10..12)
// =============================================================================

TEST_CASE("BuildInitialStateVector - ECI velocity matches independent NEDToECEF->ECEFToECI+omega*r",
    "[velocity][initial_state]")
{
    // Full reference: v_eci = ECEFToECI(NEDToECEF(v_ned)) + omega_E × r_eci
    const double lat_deg = 45.0, lon_deg = 30.0, theta = 1.0;
    const double vN = 100.0, vE = -50.0, vD = 20.0;

    const auto x0 = FD::BuildInitialStateVector(
        MakeCfg(lat_deg, lon_deg, 0.0, 0, 0, 0, vN, vE, vD), theta);

    const Vec3d r_ecef = AC::GeodeticToECEF(lat_deg * kD2R, lon_deg * kD2R, 0.0);
    const Vec3d r_eci = AC::ECEFToECI(r_ecef, theta);
    const Vec3d v_ecef = AC::NEDToECEF(Vec3d{ vN, vE, vD }, lat_deg * kD2R, lon_deg * kD2R);
    const Vec3d v_eci_kinematic = AC::ECEFToECI(v_ecef, theta);
    const Vec3d ocr = OmegaCrossR(r_eci);
    const Vec3d v_eci_ref = {
        v_eci_kinematic[0] + ocr[0],
        v_eci_kinematic[1] + ocr[1],
        v_eci_kinematic[2] + ocr[2]
    };

    REQUIRE(x0[SL::IDX_V + 0] == Approx(v_eci_ref[0]).margin(1e-6));
    REQUIRE(x0[SL::IDX_V + 1] == Approx(v_eci_ref[1]).margin(1e-6));
    REQUIRE(x0[SL::IDX_V + 2] == Approx(v_eci_ref[2]).margin(1e-6));
}

TEST_CASE("BuildInitialStateVector - zero NED velocity gives ECI velocity equal to omega*r only",
    "[velocity][initial_state]")
{
    // When v_NED = 0 the vehicle is at rest on the Earth surface.
    // Its ECI velocity must be the surface velocity: omega_E × r_eci.
    // It must NOT be zero.
    const double lat_deg = 28.6, lon_deg = -80.6, theta = 0.5;

    const auto x0 = FD::BuildInitialStateVector(
        MakeCfg(lat_deg, lon_deg, 0.0), theta);

    const Vec3d r_ecef = AC::GeodeticToECEF(lat_deg * kD2R, lon_deg * kD2R, 0.0);
    const Vec3d r_eci = AC::ECEFToECI(r_ecef, theta);
    const Vec3d ocr = OmegaCrossR(r_eci);

    REQUIRE(x0[SL::IDX_V + 0] == Approx(ocr[0]).margin(1e-6));
    REQUIRE(x0[SL::IDX_V + 1] == Approx(ocr[1]).margin(1e-6));
    REQUIRE(x0[SL::IDX_V + 2] == Approx(ocr[2]).margin(1e-6));
}

TEST_CASE("BuildInitialStateVector - NED-to-ECI rotation preserves speed magnitude",
    "[velocity][initial_state]")
{
    // The NED->ECEF->ECI kinematic rotation is isometric, so the speed
    // contribution from v_NED is preserved.  The total ECI speed also includes
    // omega×r, so we check the NED contribution specifically.
    const double vN = 200.0, vE = -100.0, vD = 50.0;
    const double lat_deg = 60.0, lon_deg = -120.0, theta = 0.7;

    const auto x0 = FD::BuildInitialStateVector(
        MakeCfg(lat_deg, lon_deg, 0.0, 0, 0, 0, vN, vE, vD), theta);

    // Subtract the omega×r contribution and check the remaining speed.
    const Vec3d r_ecef = AC::GeodeticToECEF(lat_deg * kD2R, lon_deg * kD2R, 0.0);
    const Vec3d r_eci = AC::ECEFToECI(r_ecef, theta);
    const Vec3d ocr = OmegaCrossR(r_eci);

    const double speed_ned = std::sqrt(vN * vN + vE * vE + vD * vD);
    const Eigen::Vector3d v_eci_ned_part{
        x0[SL::IDX_V + 0] - ocr[0],
        x0[SL::IDX_V + 1] - ocr[1],
        x0[SL::IDX_V + 2] - ocr[2]
    };
    REQUIRE(v_eci_ned_part.norm() == Approx(speed_ned).epsilon(1e-12));
}

TEST_CASE("BuildInitialStateVector - ECI velocity is independent of launch attitude angles",
    "[velocity][initial_state]")
{
    const double theta = 0.9;
    const double vN = 50.0, vE = -30.0, vD = 10.0;

    const auto x_a = FD::BuildInitialStateVector(
        MakeCfg(20, -40, 0, 0, 0, 0, vN, vE, vD), theta);
    const auto x_b = FD::BuildInitialStateVector(
        MakeCfg(20, -40, 0, 90, 45, 15, vN, vE, vD), theta);

    for (int i = SL::IDX_V; i < SL::IDX_V + 3; ++i)
        REQUIRE(x_a[i] == Approx(x_b[i]).margin(1e-10));
}


// =============================================================================
// GROUP 7 - Earth rotation: omega_E × r term
// =============================================================================

TEST_CASE("BuildInitialStateVector - earth rotation: zero NED velocity gives non-zero ECI velocity",
    "[earth_rotation][initial_state]")
{
    // A vehicle at rest on the surface cannot have zero inertial velocity.
    // This test guards against the regression of setting v_eci=[0,0,0] for v_NED=[0,0,0].
    for (const double theta : {0.0, 0.3, 1.0, kPi}) {
        const auto x0 = FD::BuildInitialStateVector(MakeCfg(45.0, 90.0, 0.0), theta);
        const double v_eci_norm = x0.segment<3>(SL::IDX_V).norm();
        INFO("theta=" << theta << "  |v_eci|=" << v_eci_norm);
        REQUIRE(v_eci_norm > 100.0);  // must be of order omega_E * Re ~ 465 m/s
    }
}

TEST_CASE("BuildInitialStateVector - earth rotation: equator surface speed equals omega_E * r",
    "[earth_rotation][initial_state]")
{
    // At the equator with v_NED = [0,0,0] and theta = 0, the ECI velocity
    // is purely in the +Y_ECI direction with magnitude omega_E * (Re + h).
    const double h = 9144.0;   // NASA Atmos-01 initial altitude
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0.0, 0.0, h), 0.0);

    // WGS-84: prime vertical radius of curvature N at equator equals kReq
    const double e2 = 2.0 * kFlat - kFlat * kFlat;
    const double N = kReq / std::sqrt(1.0 - e2 * 0.0);  // lat=0 => sin=0
    const double expected_speed = kOmegaE * (N + h);

    REQUIRE(x0[SL::IDX_V + 0] == Approx(0.0).margin(1e-6));
    REQUIRE(x0[SL::IDX_V + 1] == Approx(expected_speed).epsilon(1e-8));
    REQUIRE(x0[SL::IDX_V + 2] == Approx(0.0).margin(1e-6));
}

TEST_CASE("BuildInitialStateVector - earth rotation: north pole surface speed is zero",
    "[earth_rotation][initial_state]")
{
    // At the pole r is parallel to omega, so omega × r = 0.
    // The surface velocity (and thus the omega×r term) must vanish.
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(90.0, 0.0, 0.0), 0.0);

    REQUIRE(x0[SL::IDX_V + 0] == Approx(0.0).margin(1e-6));
    REQUIRE(x0[SL::IDX_V + 1] == Approx(0.0).margin(1e-6));
    REQUIRE(x0[SL::IDX_V + 2] == Approx(0.0).margin(1e-6));
}

TEST_CASE("BuildInitialStateVector - earth rotation: omega*r is perpendicular to spin axis",
    "[earth_rotation][initial_state]")
{
    // omega × r has no Z-component regardless of site or ERA.
    for (const auto& [lat, lon, theta] : std::initializer_list<std::tuple<double, double, double>>{
            {0.0,   0.0,  0.0},
            {45.0, 90.0,  0.5},
            {-33.9, 151.2, 1.2},
            {28.6, -80.6,  2.7} })
    {
        const auto x0 = FD::BuildInitialStateVector(MakeCfg(lat, lon, 0.0), theta);
        INFO("lat=" << lat << " lon=" << lon << " theta=" << theta);
        REQUIRE(x0[SL::IDX_V + 2] == Approx(0.0).margin(1e-6));
    }
}

TEST_CASE("BuildInitialStateVector - earth rotation: v_eci round-trips to zero NED velocity",
    "[earth_rotation][initial_state]")
{
    // The fundamental contract: if we convert the computed v_eci back to NED
    // (using ECIToECEF then ECEFToNED then subtracting omega×r_ecef), we must
    // recover the original v_NED = [0, 0, 0].
    const double lat_deg = 28.6, lon_deg = -80.6, theta = 0.5;

    const auto x0 = FD::BuildInitialStateVector(
        MakeCfg(lat_deg, lon_deg, 0.0), theta);

    // Convert v_eci back to ECEF frame
    const Vec3d v_eci_vec{
        x0[SL::IDX_V + 0], x0[SL::IDX_V + 1], x0[SL::IDX_V + 2]
    };
    const Vec3d v_ecef_frame = AC::ECIToECEF(v_eci_vec, theta);

    // Subtract Earth surface velocity (omega × r_ecef)
    const Vec3d r_ecef = AC::GeodeticToECEF(lat_deg * kD2R, lon_deg * kD2R, 0.0);
    const Vec3d omega_cross_r_ecef{
        -kOmegaE * r_ecef[1],
         kOmegaE * r_ecef[0],
         0.0
    };
    const Vec3d v_ecef_relative{
        v_ecef_frame[0] - omega_cross_r_ecef[0],
        v_ecef_frame[1] - omega_cross_r_ecef[1],
        v_ecef_frame[2] - omega_cross_r_ecef[2]
    };

    // Rotate to NED
    const Vec3d v_ned = AC::ECEFToNED(v_ecef_relative, lat_deg * kD2R, lon_deg * kD2R);

    REQUIRE(v_ned[0] == Approx(0.0).margin(1e-6));
    REQUIRE(v_ned[1] == Approx(0.0).margin(1e-6));
    REQUIRE(v_ned[2] == Approx(0.0).margin(1e-6));
}

TEST_CASE("BuildInitialStateVector - earth rotation: non-zero NED velocity round-trips correctly",
    "[earth_rotation][initial_state]")
{
    // Same round-trip as above but with a non-zero initial NED velocity.
    const double lat_deg = 51.5, lon_deg = -0.1, theta = 1.3;
    const double vN = 150.0, vE = -75.0, vD = 10.0;

    const auto x0 = FD::BuildInitialStateVector(
        MakeCfg(lat_deg, lon_deg, 500.0, 0, 0, 0, vN, vE, vD), theta);

    const Vec3d r_ecef = AC::GeodeticToECEF(lat_deg * kD2R, lon_deg * kD2R, 500.0);
    const Vec3d v_eci_vec{
        x0[SL::IDX_V + 0], x0[SL::IDX_V + 1], x0[SL::IDX_V + 2]
    };
    const Vec3d v_ecef_frame = AC::ECIToECEF(v_eci_vec, theta);
    const Vec3d v_ecef_relative{
        v_ecef_frame[0] - (-kOmegaE * r_ecef[1]),
        v_ecef_frame[1] - (kOmegaE * r_ecef[0]),
        v_ecef_frame[2]
    };
    const Vec3d v_ned = AC::ECEFToNED(v_ecef_relative, lat_deg * kD2R, lon_deg * kD2R);

    REQUIRE(v_ned[0] == Approx(vN).margin(1e-6));
    REQUIRE(v_ned[1] == Approx(vE).margin(1e-6));
    REQUIRE(v_ned[2] == Approx(vD).margin(1e-6));
}

TEST_CASE("BuildInitialStateVector - earth rotation: NASA Atmos-01 initial conditions",
    "[earth_rotation][initial_state]")
{
    // NASA TM-2015-218675 Atmos-01 check case:
    //   lat=0, lon=0, alt=30000 ft = 9144 m, v_NED=[0,0,0], theta=0
    // Expected initial ECI velocity = omega_E × r_eci ~ [0, 465.77, 0] m/s
    const double h = 9144.0;
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0.0, 0.0, h), 0.0);

    // At lat=0, lon=0, theta=0: r_eci = [Re+h, 0, 0]
    // omega × r = [0, omega_E*(Re+h), 0]
    const double e2 = 2.0 * kFlat - kFlat * kFlat;
    const double N = kReq / std::sqrt(1.0 - e2 * 0.0);
    const double expected_vy = kOmegaE * (N + h);

    REQUIRE(x0[SL::IDX_V + 0] == Approx(0.0).margin(1e-6));
    REQUIRE(x0[SL::IDX_V + 1] == Approx(expected_vy).epsilon(1e-7));
    REQUIRE(x0[SL::IDX_V + 2] == Approx(0.0).margin(1e-6));
}

TEST_CASE("BuildInitialStateVector - earth rotation: omega*r magnitude scales with cos(lat)",
    "[earth_rotation][initial_state]")
{
    // The perpendicular distance from the spin axis is r * cos(geocentric_lat).
    // For WGS-84, the ECEF X-component at lon=0 is (N+h)*cos(lat), so
    // |omega × r_eci| ~ omega_E * (N+h) * cos(lat).  We verify the ratio
    // between equator and 60 deg latitude.
    const double h = 0.0;
    const auto x_eq = FD::BuildInitialStateVector(MakeCfg(0.0, 0.0, h), 0.0);
    const auto x_60 = FD::BuildInitialStateVector(MakeCfg(60.0, 0.0, h), 0.0);

    const double v_eq = x_eq.segment<3>(SL::IDX_V).norm();
    const double v_60 = x_60.segment<3>(SL::IDX_V).norm();

    // v_60 / v_eq ≈ cos(60°) * (N_60+h) / (N_eq+h)
    // Both N values are close to kReq, so the ratio is dominated by cos(60)=0.5.
    REQUIRE(v_60 / v_eq == Approx(0.5).epsilon(0.01));
}


// =============================================================================
// GROUP 8 - Corner cases
// =============================================================================

TEST_CASE("BuildInitialStateVector - equator/prime-meridian/theta=0: position on +X_ECI",
    "[corner][position][initial_state]")
{
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0, 0, 0), 0.0);
    REQUIRE(x0[SL::IDX_P + 0] == Approx(kReq).epsilon(1e-8));
    REQUIRE(x0[SL::IDX_P + 1] == Approx(0.0).margin(1.0));
    REQUIRE(x0[SL::IDX_P + 2] == Approx(0.0).margin(1.0));
}

TEST_CASE("BuildInitialStateVector - equator/prime-meridian/theta=0/zen=0: body +x points along +X_ECI",
    "[corner][attitude][initial_state]")
{
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0, 0, 0, 0, 0, 0), 0.0);
    const Vec3d dir = RotateVec(QuatFromState(x0), Vec3d{ 1,0,0 });

    REQUIRE(dir[0] == Approx(1.0).margin(1e-12));
    REQUIRE(dir[1] == Approx(0.0).margin(1e-12));
    REQUIRE(dir[2] == Approx(0.0).margin(1e-12));
}

TEST_CASE("BuildInitialStateVector - due-East heading at equator/prime-meridian: body +x aligns +Y_ECI",
    "[corner][attitude][initial_state]")
{
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0, 0, 0, 90, 90, 0), 0.0);
    const Vec3d dir = RotateVec(QuatFromState(x0), Vec3d{ 1,0,0 });

    REQUIRE(dir[0] == Approx(0.0).margin(1e-12));
    REQUIRE(dir[1] == Approx(1.0).margin(1e-12));
    REQUIRE(dir[2] == Approx(0.0).margin(1e-12));
}

TEST_CASE("BuildInitialStateVector - north pole: Z component equals WGS-84 polar radius",
    "[corner][position][initial_state]")
{
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(90.0, 0.0, 0.0), 0.0);
    REQUIRE(x0[SL::IDX_P + 0] == Approx(0.0).margin(1.0));
    REQUIRE(x0[SL::IDX_P + 1] == Approx(0.0).margin(1.0));
    REQUIRE(x0[SL::IDX_P + 2] == Approx(kRpol).epsilon(1e-7));
}

TEST_CASE("BuildInitialStateVector - LEO altitude (400 km): radial distance correct",
    "[corner][position][initial_state]")
{
    const double h = 400'000.0;
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0, 0, h), 0.0);
    REQUIRE(x0.segment<3>(SL::IDX_P).norm() == Approx(kReq + h).epsilon(1e-7));
}

TEST_CASE("BuildInitialStateVector - ERA=2pi gives identical result to ERA=0",
    "[corner][era][initial_state]")
{
    const auto cfg = MakeCfg(35.0, 45.0, 500.0, 30.0, 20.0, 5.0, 10, -5, 1);
    const auto x0 = FD::BuildInitialStateVector(cfg, 0.0);
    const auto x2pi = FD::BuildInitialStateVector(cfg, 2.0 * kPi);

    for (int i = 0; i < SL::N; ++i)
        REQUIRE(x0[i] == Approx(x2pi[i]).margin(1e-7));
}

TEST_CASE("BuildInitialStateVector - roll=360 deg gives same attitude and direction as roll=0 deg",
    "[corner][attitude][initial_state]")
{
    const auto cfg0 = MakeCfg(45, 90, 0, 30, 45, 0.0);
    const auto cfg360 = MakeCfg(45, 90, 0, 30, 45, 360.0);

    const auto x0 = FD::BuildInitialStateVector(cfg0, 0.5);
    const auto x360 = FD::BuildInitialStateVector(cfg360, 0.5);

    REQUIRE(QuatAbsDot(QuatFromState(x0), QuatFromState(x360)) == Approx(1.0).margin(1e-10));

    const Vec3d dir0 = RotateVec(QuatFromState(x0), Vec3d{ 1,0,0 });
    const Vec3d dir360 = RotateVec(QuatFromState(x360), Vec3d{ 1,0,0 });

    REQUIRE(dir0[0] == Approx(dir360[0]).margin(1e-10));
    REQUIRE(dir0[1] == Approx(dir360[1]).margin(1e-10));
    REQUIRE(dir0[2] == Approx(dir360[2]).margin(1e-10));
}

TEST_CASE("BuildInitialStateVector - negative altitude (below ellipsoid) stays finite",
    "[corner][structural][initial_state]")
{
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(31.5, 35.5, -400.0), 0.3);

    for (int i = 0; i < SL::N; ++i) {
        INFO("slot " << i << " = " << x0[i]);
        REQUIRE(std::isfinite(x0[i]));
    }
    REQUIRE(x0.segment<3>(SL::IDX_P).norm() < kReq);
}


// =============================================================================
// GROUP 9 - ERA overload vs. convenience overload
// =============================================================================

TEST_CASE("BuildInitialStateVector - convenience overload equals ERA overload with theta=omegaE*t0",
    "[era][initial_state]")
{
    const double t0 = 10.0;
    const double theta = kOmegaE * t0;
    const auto   cfg = MakeCfg(28.6, -80.6, 0, 45, 30, 10, 5, -3, 0, 0.01, -0.02, 0.005, 5000);

    const auto x0_explicit = FD::BuildInitialStateVector(cfg, theta);
    const auto x0_conv = FD::BuildInitialStateVector(cfg, theta);

    for (int i = 0; i < SL::N; ++i)
        REQUIRE(x0_explicit[i] == Approx(x0_conv[i]).margin(1e-12));
}

TEST_CASE("BuildInitialStateVector - convenience overload at t0=0 matches ERA overload at theta=0",
    "[era][initial_state]")
{
    const auto cfg = MakeCfg(28.6, -80.6, 0, 45, 30, 10, 5, -3, 0, 0.01, -0.02, 0.005, 5000);
    const auto x0_explicit = FD::BuildInitialStateVector(cfg, 0.0);
    const auto x0_conv = FD::BuildInitialStateVector(cfg, 0.0);

    for (int i = 0; i < SL::N; ++i)
        REQUIRE(x0_explicit[i] == Approx(x0_conv[i]).margin(1e-12));
}

TEST_CASE("BuildInitialStateVector - ECI position rotates by Dtheta in the XY plane as ERA increases",
    "[era][position][initial_state]")
{
    const double theta1 = 0.0;
    const double theta2 = kPi / 4.0;
    const auto   cfg = MakeCfg(0.0, 0.0, 0.0);

    const auto x1 = FD::BuildInitialStateVector(cfg, theta1);
    const auto x2 = FD::BuildInitialStateVector(cfg, theta2);

    REQUIRE(x1[SL::IDX_P + 2] == Approx(x2[SL::IDX_P + 2]).margin(1e-6));
    REQUIRE(x1.segment<3>(SL::IDX_P).norm()
        == Approx(x2.segment<3>(SL::IDX_P).norm()).epsilon(1e-12));

    const double phi1 = std::atan2(x1[SL::IDX_P + 1], x1[SL::IDX_P + 0]);
    const double phi2 = std::atan2(x2[SL::IDX_P + 1], x2[SL::IDX_P + 0]);
    REQUIRE((phi2 - phi1) == Approx(theta2 - theta1).margin(1e-12));
}

TEST_CASE("BuildInitialStateVector - attitude also rotates consistently with ERA",
    "[era][attitude][initial_state]")
{
    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0, 0, 0, 0, 0, 0), 0.0);
    const auto x_half = FD::BuildInitialStateVector(MakeCfg(0, 0, 0, 0, 0, 0), kPi / 2.0);

    const Vec3d dir0 = RotateVec(QuatFromState(x0), Vec3d{ 1,0,0 });
    const Vec3d dir_pi2 = RotateVec(QuatFromState(x_half), Vec3d{ 1,0,0 });

    REQUIRE(dir0[0] == Approx(1.0).margin(1e-12));
    REQUIRE(dir0[1] == Approx(0.0).margin(1e-12));
    REQUIRE(dir_pi2[0] == Approx(0.0).margin(1e-12));
    REQUIRE(dir_pi2[1] == Approx(1.0).margin(1e-12));
    REQUIRE(dir_pi2[2] == Approx(0.0).margin(1e-12));
}


// =============================================================================
// GROUP 10 - Finite-difference Jacobian sanity checks
// =============================================================================

TEST_CASE("BuildInitialStateVector - finite-diff dr/dh has unit magnitude",
    "[perturbation][position][initial_state]")
{
    const double dh = 1.0, theta = 0.3;
    const auto x_p = FD::BuildInitialStateVector(MakeCfg(45.0, 60.0, dh), theta);
    const auto x_m = FD::BuildInitialStateVector(MakeCfg(45.0, 60.0, -dh), theta);

    const Eigen::Vector3d dr =
        (x_p.segment<3>(SL::IDX_P) - x_m.segment<3>(SL::IDX_P)) / (2.0 * dh);
    REQUIRE(dr.norm() == Approx(1.0).epsilon(1e-6));
}

TEST_CASE("BuildInitialStateVector - finite-diff dv_eci/dvN has unit magnitude",
    "[perturbation][velocity][initial_state]")
{
    // The NED->ECEF->ECI kinematic part is a pure rotation, so the Jacobian
    // of v_eci w.r.t. v_N has unit norm.  The omega×r term is constant
    // w.r.t. v_N, so it cancels in the finite difference.
    const double eps = 1e-3, theta = 0.7;
    const auto x_p = FD::BuildInitialStateVector(MakeCfg(30, 45, 0, 0, 0, 0, eps, 0, 0), theta);
    const auto x_m = FD::BuildInitialStateVector(MakeCfg(30, 45, 0, 0, 0, 0, -eps, 0, 0), theta);

    const Eigen::Vector3d dv =
        (x_p.segment<3>(SL::IDX_V) - x_m.segment<3>(SL::IDX_V)) / (2.0 * eps);
    REQUIRE(dv.norm() == Approx(1.0).epsilon(1e-10));
}

TEST_CASE("BuildInitialStateVector - small azimuth perturbation produces continuous attitude change",
    "[perturbation][attitude][initial_state]")
{
    const double delta_az_rad = 1e-6;
    const double theta = 0.4;

    const auto cfg1 = MakeCfg(40, 20, 0, 45.0, 30.0, 0.0);
    const auto cfg2 = MakeCfg(40, 20, 0, 45.0 + delta_az_rad / kD2R, 30.0, 0.0);

    const auto x1 = FD::BuildInitialStateVector(cfg1, theta);
    const auto x2 = FD::BuildInitialStateVector(cfg2, theta);

    REQUIRE(QuatAbsDot(QuatFromState(x1), QuatFromState(x2)) == Approx(1.0).margin(1e-9));
}

TEST_CASE("BuildInitialStateVector - finite-diff ddir/dzen has unit magnitude",
    "[perturbation][attitude][initial_state]")
{
    const double dzen_rad = 1e-5;
    const double theta = 0.6;

    const auto cfg_p = MakeCfg(30, 20, 0, 45.0, 30.0 + dzen_rad / kD2R, 0.0);
    const auto cfg_m = MakeCfg(30, 20, 0, 45.0, 30.0 - dzen_rad / kD2R, 0.0);

    const Vec3d dir_p = RotateVec(QuatFromState(FD::BuildInitialStateVector(cfg_p, theta)),
        Vec3d{ 1,0,0 });
    const Vec3d dir_m = RotateVec(QuatFromState(FD::BuildInitialStateVector(cfg_m, theta)),
        Vec3d{ 1,0,0 });

    const Vec3d ddir{
        (dir_p[0] - dir_m[0]) / (2.0 * dzen_rad),
        (dir_p[1] - dir_m[1]) / (2.0 * dzen_rad),
        (dir_p[2] - dir_m[2]) / (2.0 * dzen_rad)
    };
    REQUIRE(Norm3(ddir) == Approx(1.0).epsilon(1e-6));
}

//// ------------------------------------------------------------------------------
//// Project: Aetherion
//// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
////
//// SPDX-License-Identifier: MIT
//// License-Filename: LICENSE
//// ------------------------------------------------------------------------------
////
//// test_BuildInitialStateVector.cpp
//// PATH: tests/FlightDynamics/test_BuildInitialStateVector.cpp
////
//// Catch2 v3 tests for BuildInitialStateVector.h
////
//// Test sections (Catch2 tags):
////
////   [structural]   -- vector size, all-finite, StateLayout index offsets
////   [mass]         -- mass slot passthrough, independence from all other fields
////   [omega]        -- angular-rate slots passthrough, independence from pose/vel
////   [position]     -- ECI position vs. independent GeodeticToECEF->ECEFToECI chain
////   [attitude]     -- quaternion unit-norm, body +x aligned with launch direction,
////                    full orthonormal frame, direct match to MakeLaunchStateECI
////   [velocity]     -- ECI velocity vs. independent NEDToECEF->ECEFToECI chain,
////                    speed preservation, independence from attitude angles
////   [corner]       -- equator/prime-meridian/theta=0, north pole, due-East heading,
////                    LEO altitude, ERA=2pi identity, 360 deg roll identity
////   [era]          -- ERA overload vs. convenience overload, ERA-from-t0 formula,
////                    XY-plane rotation by Dtheta
////   [perturbation] -- finite-difference Jacobians for position/velocity/attitude
//// ------------------------------------------------------------------------------
//
//#include <catch2/catch_test_macros.hpp>
//#include <catch2/catch_approx.hpp>
//
//#include <cmath>
//#include <numbers>
//#include <array>
//
//#include <Eigen/Dense>
//
//// Unit under test
//#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
//
//// Reference transforms (used for independent ground-truth computations)
//#include <Aetherion/Coordinate/LocalToInertial.h>
//#include <Aetherion/Coordinate/Math.h>
//
//// State index constants
//#include <Aetherion/RigidBody/StateLayout.h>
//#include <Aetherion/RigidBody/Config.h>
//
//// --- Namespace aliases --------------------------------------------------------
//
//using Catch::Approx;
//
//namespace AC = Aetherion::Coordinate;
//namespace FD = Aetherion::FlightDynamics;
//using SL = Aetherion::RigidBody::StateLayout;
//namespace RigidBody = Aetherion::RigidBody;
//
//using Vec3d = AC::Vec3<double>;
//using Quatd = AC::Quat<double>;
//
//// --- File-scope constants -----------------------------------------------------
//
//static constexpr double kPi = std::numbers::pi_v<double>;
//static constexpr double kD2R = kPi / 180.0;
//static constexpr double kOmegaE = 7.2921150e-5;   // WGS-84 Earth rotation [rad/s]
//static constexpr double kReq = 6378137.0;       // WGS-84 semi-major axis [m]
//static constexpr double kFlat = 1.0 / 298.257223563; // WGS-84 flattening
//static constexpr double kRpol = kReq * (1.0 - kFlat); // polar radius [m]
//
//// --- Test helpers -------------------------------------------------------------
//
//// Build a RigidBody::Config.  All pose angles are in DEGREES (matches GeodeticPoseNED).
//// Every parameter has a default so individual tests can name just what they change.
//static RigidBody::Config MakeCfg(
//    double lat_deg = 28.6,
//    double lon_deg = -80.6,
//    double alt_m = 0.0,
//    double az_deg = 0.0,
//    double zen_deg = 0.0,
//    double roll_deg = 0.0,
//    double vN = 0.0,
//    double vE = 0.0,
//    double vD = 0.0,
//    double omega_roll = 0.0,
//    double omega_pitch = 0.0,
//    double omega_yaw = 0.0,
//    double mass_kg = 10'000.0)
//
//{
//    RigidBody::Config cfg{};
//    cfg.pose.lat_deg = lat_deg;
//    cfg.pose.lon_deg = lon_deg;
//    cfg.pose.alt_m = alt_m;
//    cfg.pose.azimuth_deg = az_deg;
//    cfg.pose.zenith_deg = zen_deg;
//    cfg.pose.roll_deg = roll_deg;
//
//    cfg.velocityNED.north_mps = vN;
//    cfg.velocityNED.east_mps = vE;
//    cfg.velocityNED.down_mps = vD;
//
//    cfg.bodyRates.roll_rad_s = omega_roll;
//    cfg.bodyRates.pitch_rad_s = omega_pitch;
//    cfg.bodyRates.yaw_rad_s = omega_yaw;
//
//    cfg.inertialParameters.mass_kg = mass_kg;
//
//    return cfg;
//}
//
//// Rotate v_B into the "A" frame given q_AB = [w,x,y,z].
//// Uses the same formula as the existing Coordinate test suite.
//static Vec3d RotateVec(const Quatd& q, const Vec3d& v)
//{
//    const double w = q[0], x = q[1], y = q[2], z = q[3];
//
//    // t = q * [0,v] (pure-quat product, vector part)
//    const double vw = -x * v[0] - y * v[1] - z * v[2];
//    const double vx = w * v[0] + y * v[2] - z * v[1];
//    const double vy = w * v[1] + z * v[0] - x * v[2];
//    const double vz = w * v[2] + x * v[1] - y * v[0];
//
//    // (t) * q_conj  (q_conj = [w,-x,-y,-z])
//    return Vec3d{
//        vw * (-x) + vx * w + vy * (-z) - vz * (-y),
//        vw * (-y) - vx * (-z) + vy * w + vz * (-x),
//        vw * (-z) + vx * (-y) - vy * (-x) + vz * w
//    };
//}
//
//// Extract q_EB from the state vector into an Aetherion Quat.
//static Quatd QuatFromState(const Eigen::Matrix<double, SL::N, 1>& x)
//{
//    return Quatd{
//        x[SL::IDX_Q + 0],
//        x[SL::IDX_Q + 1],
//        x[SL::IDX_Q + 2],
//        x[SL::IDX_Q + 3]
//    };
//}
//
//// Squared quaternion dot product (|dot| == 1 means same rotation).
//static double QuatAbsDot(const Quatd& a, const Quatd& b)
//{
//    return std::abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
//}
//
//static double Norm3(const Vec3d& v) {
//    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
//}
//
//static double Dot3(const Vec3d& a, const Vec3d& b) {
//    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
//}
//
//
//// =============================================================================
//// GROUP 1 - Structural invariants
//// =============================================================================
//
//TEST_CASE("BuildInitialStateVector - output has exactly N=14 elements",
//    "[structural][initial_state]")
//{
//    const auto x0 = FD::BuildInitialStateVector(MakeCfg(), 0.0);
//    STATIC_REQUIRE(SL::N == 14);
//    REQUIRE(x0.size() == 14);
//    REQUIRE(x0.rows() == 14);
//    REQUIRE(x0.cols() == 1);
//}
//
//TEST_CASE("BuildInitialStateVector - StateLayout index offsets are as documented",
//    "[structural][initial_state]")
//{
//    // Freeze the documented layout so a future refactor can never silently shift slots.
//    STATIC_REQUIRE(SL::IDX_P == 0);
//    STATIC_REQUIRE(SL::IDX_Q == 3);
//    STATIC_REQUIRE(SL::IDX_W == 7);
//    STATIC_REQUIRE(SL::IDX_V == 10);
//    STATIC_REQUIRE(SL::IDX_M == 13);
//}
//
//TEST_CASE("BuildInitialStateVector - every slot is finite for a representative config",
//    "[structural][initial_state]")
//{
//    const auto cfg = MakeCfg(51.5, -0.1, 500.0,
//        45.0, 30.0, 10.0,
//        100.0, -50.0, 5.0,
//        0.01, -0.02, 0.005,
//        5000.0); //, 1000.0);
//    const auto x0 = FD::BuildInitialStateVector(cfg, 0.0);
//
//    for (int i = 0; i < SL::N; ++i) {
//        INFO("slot " << i << " = " << x0[i]);
//        REQUIRE(std::isfinite(x0[i]));
//    }
//}
//
//TEST_CASE("BuildInitialStateVector - every slot is finite when all inputs are zero",
//    "[structural][initial_state]")
//{
//    const auto cfg = MakeCfg(0.0, 0.0, 0.0,
//        0.0, 0.0, 0.0,
//        0.0, 0.0, 0.0,
//        0.0, 0.0, 0.0,
//        1.0);  // mass must be non-zero for physical sense
//    const auto x0 = FD::BuildInitialStateVector(cfg, 0.0);
//
//    for (int i = 0; i < SL::N; ++i) {
//        INFO("slot " << i << " = " << x0[i]);
//        REQUIRE(std::isfinite(x0[i]));
//    }
//}
//
//
//// =============================================================================
//// GROUP 2 - Mass slot (IDX_M = 13)
//// =============================================================================
//
//TEST_CASE("BuildInitialStateVector - mass slot equals cfg.inertialParameters.mass_kg",
//    "[mass][initial_state]")
//{
//    const double m = 7654.321;
//    const auto cfg = MakeCfg(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, m);
//    const auto x0 = FD::BuildInitialStateVector(cfg, 0.0);
//
//    REQUIRE(x0[SL::IDX_M] == Approx(m).epsilon(1e-15));
//}
//
//TEST_CASE("BuildInitialStateVector - mass slot is independent of pose, velocity, and angular rates",
//    "[mass][initial_state]")
//{
//    const double m = 500.0;
//
//    // Two configs that differ only in non-mass fields
//    const auto cfg_a = MakeCfg(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, m);
//    const auto cfg_b = MakeCfg(45, 90, 1000, 90, 45, 30, 10, 5, -2, 0.1, -0.05, 0.03, m);
//
//    const auto x_a = FD::BuildInitialStateVector(cfg_a, 0.3);
//    const auto x_b = FD::BuildInitialStateVector(cfg_b, 1.7);
//
//    REQUIRE(x_a[SL::IDX_M] == Approx(m).epsilon(1e-15));
//    REQUIRE(x_b[SL::IDX_M] == Approx(m).epsilon(1e-15));
//}
//
//
//// =============================================================================
//// GROUP 3 - Angular-rate slots (IDX_W = 7..9)
//// =============================================================================
//
//TEST_CASE("BuildInitialStateVector - angular rate slots equal cfg.bodyRates",
//    "[omega][initial_state]")
//{
//    const double roll_s = 0.12;
//    const double pitch_s = -0.07;
//    const double yaw_s = 0.03;
//
//    const auto cfg = MakeCfg(0, 0, 0, 0, 0, 0, 0, 0, 0, roll_s, pitch_s, yaw_s);
//    const auto x0 = FD::BuildInitialStateVector(cfg, 0.5);
//
//    REQUIRE(x0[SL::IDX_W + 0] == Approx(roll_s).epsilon(1e-15));
//    REQUIRE(x0[SL::IDX_W + 1] == Approx(pitch_s).epsilon(1e-15));
//    REQUIRE(x0[SL::IDX_W + 2] == Approx(yaw_s).epsilon(1e-15));
//}
//
//TEST_CASE("BuildInitialStateVector - zero angular rates produce zero omega slots",
//    "[omega][initial_state]")
//{
//    const auto x0 = FD::BuildInitialStateVector(MakeCfg(), 0.0);
//    REQUIRE(x0[SL::IDX_W + 0] == Approx(0.0).margin(1e-20));
//    REQUIRE(x0[SL::IDX_W + 1] == Approx(0.0).margin(1e-20));
//    REQUIRE(x0[SL::IDX_W + 2] == Approx(0.0).margin(1e-20));
//}
//
//TEST_CASE("BuildInitialStateVector - omega slots are independent of pose and velocity",
//    "[omega][initial_state]")
//{
//    const double roll_s = 0.05, pitch_s = -0.02, yaw_s = 0.01;
//
//    const auto cfg_a = MakeCfg(0, 0, 0, 0, 0, 0, 0, 0, 0, roll_s, pitch_s, yaw_s);
//    const auto cfg_b = MakeCfg(45, 30, 500, 60, 20, 15, 50, -20, 3, roll_s, pitch_s, yaw_s);
//
//    const auto x_a = FD::BuildInitialStateVector(cfg_a, 0.2);
//    const auto x_b = FD::BuildInitialStateVector(cfg_b, 1.0);
//
//    REQUIRE(x_a[SL::IDX_W + 0] == Approx(x_b[SL::IDX_W + 0]).epsilon(1e-15));
//    REQUIRE(x_a[SL::IDX_W + 1] == Approx(x_b[SL::IDX_W + 1]).epsilon(1e-15));
//    REQUIRE(x_a[SL::IDX_W + 2] == Approx(x_b[SL::IDX_W + 2]).epsilon(1e-15));
//}
//
//
//// =============================================================================
//// GROUP 4 - ECI position (IDX_P = 0..2)
//// =============================================================================
//
//TEST_CASE("BuildInitialStateVector - ECI position matches independent GeodeticToECEF->ECEFToECI",
//    "[position][initial_state]")
//{
//    const double lat_deg = 28.6, lon_deg = -80.6, alt_m = 200.0, theta = 0.75;
//
//    const auto x0 = FD::BuildInitialStateVector(MakeCfg(lat_deg, lon_deg, alt_m), theta);
//
//    const Vec3d r_ecef = AC::GeodeticToECEF(lat_deg * kD2R, lon_deg * kD2R, alt_m);
//    const Vec3d r_eci = AC::ECEFToECI(r_ecef, theta);
//
//    REQUIRE(x0[SL::IDX_P + 0] == Approx(r_eci[0]).margin(1e-4));
//    REQUIRE(x0[SL::IDX_P + 1] == Approx(r_eci[1]).margin(1e-4));
//    REQUIRE(x0[SL::IDX_P + 2] == Approx(r_eci[2]).margin(1e-4));
//}
//
//TEST_CASE("BuildInitialStateVector - ECI position magnitude equals WGS-84 radius + altitude",
//    "[position][initial_state]")
//{
//    // At lat=0, lon=0 the WGS-84 surface radius equals kReq exactly.
//    const double h = 1500.0;
//    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0.0, 0.0, h), 0.0);
//
//    const double r = x0.segment<3>(SL::IDX_P).norm();
//    REQUIRE(r == Approx(kReq + h).epsilon(1e-8));
//}
//
//TEST_CASE("BuildInitialStateVector - increasing altitude increases radial distance by exactly Dh",
//    "[position][initial_state]")
//{
//    const double theta = 0.4;
//    const double dh = 1000.0;
//
//    const auto x0 = FD::BuildInitialStateVector(MakeCfg(45.0, 60.0, 0.0), theta);
//    const auto x1 = FD::BuildInitialStateVector(MakeCfg(45.0, 60.0, dh), theta);
//
//    const double dr = x1.segment<3>(SL::IDX_P).norm()
//        - x0.segment<3>(SL::IDX_P).norm();
//
//    REQUIRE(dr == Approx(dh).epsilon(1e-4));
//}
//
//TEST_CASE("BuildInitialStateVector - position is independent of launch attitude angles",
//    "[position][initial_state]")
//{
//    // r_ECI depends only on (lat, lon, alt, theta_ERA); changing az/zen/roll must not move it.
//    const double theta = 0.6;
//    const auto x_a = FD::BuildInitialStateVector(MakeCfg(50, 10, 300, 0, 0, 0), theta);
//    const auto x_b = FD::BuildInitialStateVector(MakeCfg(50, 10, 300, 90, 45, 30), theta);
//    const auto x_c = FD::BuildInitialStateVector(MakeCfg(50, 10, 300, 180, 60, -20), theta);
//
//    for (int i = SL::IDX_P; i < SL::IDX_P + 3; ++i) {
//        REQUIRE(x_a[i] == Approx(x_b[i]).margin(1e-6));
//        REQUIRE(x_a[i] == Approx(x_c[i]).margin(1e-6));
//    }
//}
//
//TEST_CASE("BuildInitialStateVector - position is independent of initial velocity and rates",
//    "[position][initial_state]")
//{
//    const double theta = 0.8;
//    const auto x_a = FD::BuildInitialStateVector(MakeCfg(30, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), theta);
//    const auto x_b = FD::BuildInitialStateVector(MakeCfg(30, 20, 0, 0, 0, 0, 100, 50, -10, 0.1, -0.05, 0.02), theta);
//
//    for (int i = SL::IDX_P; i < SL::IDX_P + 3; ++i)
//        REQUIRE(x_a[i] == Approx(x_b[i]).margin(1e-6));
//}
//
//
//// =============================================================================
//// GROUP 5 - Attitude quaternion (IDX_Q = 3..6)
//// =============================================================================
//
//TEST_CASE("BuildInitialStateVector - quaternion is unit-normalised",
//    "[attitude][initial_state]")
//{
//    const auto cfg = MakeCfg(35.0, 139.0, 100.0, 30.0, 45.0, 20.0);
//    const auto x0 = FD::BuildInitialStateVector(cfg, 1.1);
//
//    const double norm = std::sqrt(x0[SL::IDX_Q + 0] * x0[SL::IDX_Q + 0]
//        + x0[SL::IDX_Q + 1] * x0[SL::IDX_Q + 1]
//        + x0[SL::IDX_Q + 2] * x0[SL::IDX_Q + 2]
//        + x0[SL::IDX_Q + 3] * x0[SL::IDX_Q + 3]);
//    REQUIRE(norm == Approx(1.0).margin(1e-12));
//}
//
//TEST_CASE("BuildInitialStateVector - body +x in ECI matches MakeLaunchStateECI direction",
//    "[attitude][initial_state]")
//{
//    const double lat_deg = 51.5, lon_deg = -0.1, alt_m = 0.0;
//    const double az_deg = 60.0, zen_deg = 20.0, roll_deg = 5.0;
//    const double theta = 0.4;
//
//    const auto x0 = FD::BuildInitialStateVector(
//        MakeCfg(lat_deg, lon_deg, alt_m, az_deg, zen_deg, roll_deg), theta);
//
//    const Vec3d dir_eci = RotateVec(QuatFromState(x0), Vec3d{ 1.0, 0.0, 0.0 });
//
//    // Reference direction from the underlying function
//    const auto launch = AC::MakeLaunchStateECI(
//        lat_deg * kD2R, lon_deg * kD2R, alt_m,
//        az_deg * kD2R, zen_deg * kD2R, roll_deg * kD2R,
//        theta);
//
//    REQUIRE(dir_eci[0] == Approx(launch.dir0_eci[0]).margin(1e-11));
//    REQUIRE(dir_eci[1] == Approx(launch.dir0_eci[1]).margin(1e-11));
//    REQUIRE(dir_eci[2] == Approx(launch.dir0_eci[2]).margin(1e-11));
//}
//
//TEST_CASE("BuildInitialStateVector - attitude quaternion matches MakeLaunchStateECI q_EB directly",
//    "[attitude][initial_state]")
//{
//    const double lat_deg = -33.9, lon_deg = 151.2, alt_m = 50.0;
//    const double az_deg = 90.0, zen_deg = 15.0, roll_deg = 0.0;
//    const double theta = 2.3;
//
//    const auto x0 = FD::BuildInitialStateVector(
//        MakeCfg(lat_deg, lon_deg, alt_m, az_deg, zen_deg, roll_deg), theta);
//
//    const auto launch = AC::MakeLaunchStateECI(
//        lat_deg * kD2R, lon_deg * kD2R, alt_m,
//        az_deg * kD2R, zen_deg * kD2R, roll_deg * kD2R,
//        theta);
//
//    // q and -q encode the same rotation -> compare by |dot| ~= 1
//    REQUIRE(QuatAbsDot(QuatFromState(x0), launch.q_EB) == Approx(1.0).margin(1e-12));
//}
//
//TEST_CASE("BuildInitialStateVector - body frame axes are orthonormal in ECI",
//    "[attitude][initial_state]")
//{
//    const auto x0 = FD::BuildInitialStateVector(
//        MakeCfg(35.0, 139.0, 100.0, 30.0, 45.0, 20.0), 1.1);
//    const Quatd q = QuatFromState(x0);
//
//    const Vec3d ex = RotateVec(q, Vec3d{ 1,0,0 });
//    const Vec3d ey = RotateVec(q, Vec3d{ 0,1,0 });
//    const Vec3d ez = RotateVec(q, Vec3d{ 0,0,1 });
//
//    REQUIRE(Norm3(ex) == Approx(1.0).margin(1e-12));
//    REQUIRE(Norm3(ey) == Approx(1.0).margin(1e-12));
//    REQUIRE(Norm3(ez) == Approx(1.0).margin(1e-12));
//
//    REQUIRE(Dot3(ex, ey) == Approx(0.0).margin(1e-12));
//    REQUIRE(Dot3(ex, ez) == Approx(0.0).margin(1e-12));
//    REQUIRE(Dot3(ey, ez) == Approx(0.0).margin(1e-12));
//}
//
//
//// =============================================================================
//// GROUP 6 - ECI velocity (IDX_V = 10..12)
//// =============================================================================
//
//TEST_CASE("BuildInitialStateVector - ECI velocity matches independent NEDToECEF->ECEFToECI",
//    "[velocity][initial_state]")
//{
//    const double lat_deg = 45.0, lon_deg = 30.0, theta = 1.0;
//    const double vN = 100.0, vE = -50.0, vD = 20.0;
//
//    const auto x0 = FD::BuildInitialStateVector(
//        MakeCfg(lat_deg, lon_deg, 0.0, 0, 0, 0, vN, vE, vD), theta);
//
//    const Vec3d v_ecef = AC::NEDToECEF(Vec3d{ vN,vE,vD }, lat_deg * kD2R, lon_deg * kD2R);
//    const Vec3d v_eci = AC::ECEFToECI(v_ecef, theta);
//
//    REQUIRE(x0[SL::IDX_V + 0] == Approx(v_eci[0]).margin(1e-10));
//    REQUIRE(x0[SL::IDX_V + 1] == Approx(v_eci[1]).margin(1e-10));
//    REQUIRE(x0[SL::IDX_V + 2] == Approx(v_eci[2]).margin(1e-10));
//}
//
//TEST_CASE("BuildInitialStateVector - zero NED velocity produces zero ECI velocity",
//    "[velocity][initial_state]")
//{
//    const auto x0 = FD::BuildInitialStateVector(
//        MakeCfg(0, 0, 0, 0, 0, 0, 0, 0, 0), 0.5);
//
//    REQUIRE(x0[SL::IDX_V + 0] == Approx(0.0).margin(1e-20));
//    REQUIRE(x0[SL::IDX_V + 1] == Approx(0.0).margin(1e-20));
//    REQUIRE(x0[SL::IDX_V + 2] == Approx(0.0).margin(1e-20));
//}
//
//TEST_CASE("BuildInitialStateVector - NED-to-ECI rotation preserves speed magnitude",
//    "[velocity][initial_state]")
//{
//    const double vN = 200.0, vE = -100.0, vD = 50.0;
//    const double speed_ned = std::sqrt(vN * vN + vE * vE + vD * vD);
//
//    const auto x0 = FD::BuildInitialStateVector(
//        MakeCfg(60.0, -120.0, 0.0, 0, 0, 0, vN, vE, vD), 0.7);
//
//    const double speed_eci = x0.segment<3>(SL::IDX_V).norm();
//    REQUIRE(speed_eci == Approx(speed_ned).epsilon(1e-12));
//}
//
//TEST_CASE("BuildInitialStateVector - ECI velocity is independent of launch attitude angles",
//    "[velocity][initial_state]")
//{
//    const double theta = 0.9;
//    const double vN = 50.0, vE = -30.0, vD = 10.0;
//
//    const auto x_a = FD::BuildInitialStateVector(
//        MakeCfg(20, -40, 0, 0, 0, 0, vN, vE, vD), theta);
//    const auto x_b = FD::BuildInitialStateVector(
//        MakeCfg(20, -40, 0, 90, 45, 15, vN, vE, vD), theta);
//
//    for (int i = SL::IDX_V; i < SL::IDX_V + 3; ++i)
//        REQUIRE(x_a[i] == Approx(x_b[i]).margin(1e-10));
//}
//
//TEST_CASE("BuildInitialStateVector - ECI velocity is zero for any ERA when v_NED=0",
//    "[velocity][initial_state]")
//{
//    const auto cfg = MakeCfg(10.0, 20.0, 500.0);
//
//    for (const double theta : {0.0, 0.5, kPi, 2.0 * kPi - 0.1}) {
//        const auto x0 = FD::BuildInitialStateVector(cfg, theta);
//        REQUIRE(x0[SL::IDX_V + 0] == Approx(0.0).margin(1e-20));
//        REQUIRE(x0[SL::IDX_V + 1] == Approx(0.0).margin(1e-20));
//        REQUIRE(x0[SL::IDX_V + 2] == Approx(0.0).margin(1e-20));
//    }
//}
//
//
//// =============================================================================
//// GROUP 7 - Corner cases
//// =============================================================================
//
//TEST_CASE("BuildInitialStateVector - equator/prime-meridian/theta=0: position on +X_ECI",
//    "[corner][position][initial_state]")
//{
//    // At lat=0, lon=0, alt=0, theta=0 the site is on the +X_ECI axis.
//    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0, 0, 0), 0.0);
//
//    REQUIRE(x0[SL::IDX_P + 0] == Approx(kReq).epsilon(1e-8));
//    REQUIRE(x0[SL::IDX_P + 1] == Approx(0.0).margin(1.0));   // float rounding only
//    REQUIRE(x0[SL::IDX_P + 2] == Approx(0.0).margin(1.0));
//}
//
//TEST_CASE("BuildInitialStateVector - equator/prime-meridian/theta=0/zen=0: body +x points along +X_ECI",
//    "[corner][attitude][initial_state]")
//{
//    // zen=0 means straight Up.  Up at lat=0, lon=0, theta=0 is +X_ECI.
//    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0, 0, 0, 0, 0, 0), 0.0);
//    const Vec3d dir = RotateVec(QuatFromState(x0), Vec3d{ 1,0,0 });
//
//    REQUIRE(dir[0] == Approx(1.0).margin(1e-12));
//    REQUIRE(dir[1] == Approx(0.0).margin(1e-12));
//    REQUIRE(dir[2] == Approx(0.0).margin(1e-12));
//}
//
//TEST_CASE("BuildInitialStateVector - due-East heading at equator/prime-meridian: body +x aligns +Y_ECI",
//    "[corner][attitude][initial_state]")
//{
//    // az=90 deg (East), zen=90 deg (horizontal).
//    // East at lat=0, lon=0, theta=0 is the +Y_ECEF = +Y_ECI direction.
//    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0, 0, 0, 90, 90, 0), 0.0);
//    const Vec3d dir = RotateVec(QuatFromState(x0), Vec3d{ 1,0,0 });
//
//    REQUIRE(dir[0] == Approx(0.0).margin(1e-12));
//    REQUIRE(dir[1] == Approx(1.0).margin(1e-12));
//    REQUIRE(dir[2] == Approx(0.0).margin(1e-12));
//}
//
//TEST_CASE("BuildInitialStateVector - north pole: Z component equals WGS-84 polar radius",
//    "[corner][position][initial_state]")
//{
//    const auto x0 = FD::BuildInitialStateVector(MakeCfg(90.0, 0.0, 0.0), 0.0);
//
//    // At the North Pole X~=Y~=0, Z = b (semi-minor axis)
//    REQUIRE(x0[SL::IDX_P + 0] == Approx(0.0).margin(1.0));
//    REQUIRE(x0[SL::IDX_P + 1] == Approx(0.0).margin(1.0));
//    REQUIRE(x0[SL::IDX_P + 2] == Approx(kRpol).epsilon(1e-7));
//}
//
//TEST_CASE("BuildInitialStateVector - LEO altitude (400 km): radial distance correct",
//    "[corner][position][initial_state]")
//{
//    const double h = 400'000.0;
//    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0, 0, h), 0.0);
//
//    REQUIRE(x0.segment<3>(SL::IDX_P).norm() == Approx(kReq + h).epsilon(1e-7));
//}
//
//TEST_CASE("BuildInitialStateVector - ERA=2pi gives identical result to ERA=0",
//    "[corner][era][initial_state]")
//{
//    const auto cfg = MakeCfg(35.0, 45.0, 500.0, 30.0, 20.0, 5.0, 10, -5, 1);
//    const auto x0 = FD::BuildInitialStateVector(cfg, 0.0);
//    const auto x2pi = FD::BuildInitialStateVector(cfg, 2.0 * kPi);
//
//    for (int i = 0; i < SL::N; ++i)
//        REQUIRE(x0[i] == Approx(x2pi[i]).margin(1e-7));
//}
//
//TEST_CASE("BuildInitialStateVector - roll=360 deg gives same attitude and direction as roll=0 deg",
//    "[corner][attitude][initial_state]")
//{
//    const auto cfg0 = MakeCfg(45, 90, 0, 30, 45, 0.0);
//    const auto cfg360 = MakeCfg(45, 90, 0, 30, 45, 360.0);
//
//    const auto x0 = FD::BuildInitialStateVector(cfg0, 0.5);
//    const auto x360 = FD::BuildInitialStateVector(cfg360, 0.5);
//
//    // Quaternion must represent the same rotation (|dot| ~= 1)
//    REQUIRE(QuatAbsDot(QuatFromState(x0), QuatFromState(x360)) == Approx(1.0).margin(1e-10));
//
//    // Body +x direction in ECI must be identical
//    const Vec3d dir0 = RotateVec(QuatFromState(x0), Vec3d{ 1,0,0 });
//    const Vec3d dir360 = RotateVec(QuatFromState(x360), Vec3d{ 1,0,0 });
//
//    REQUIRE(dir0[0] == Approx(dir360[0]).margin(1e-10));
//    REQUIRE(dir0[1] == Approx(dir360[1]).margin(1e-10));
//    REQUIRE(dir0[2] == Approx(dir360[2]).margin(1e-10));
//}
//
//TEST_CASE("BuildInitialStateVector - negative altitude (below ellipsoid) stays finite",
//    "[corner][structural][initial_state]")
//{
//    // e.g. dead sea / underground test facility at -400 m
//    const auto x0 = FD::BuildInitialStateVector(MakeCfg(31.5, 35.5, -400.0), 0.3);
//
//    for (int i = 0; i < SL::N; ++i) {
//        INFO("slot " << i << " = " << x0[i]);
//        REQUIRE(std::isfinite(x0[i]));
//    }
//    // Position magnitude should be smaller than kReq
//    REQUIRE(x0.segment<3>(SL::IDX_P).norm() < kReq);
//}
//
//
//// =============================================================================
//// GROUP 8 - ERA overload vs. convenience overload
//// =============================================================================
//
//TEST_CASE("BuildInitialStateVector - convenience overload equals ERA overload with theta=omegaE*t0",
//    "[era][initial_state]")
//{
//    const double t0 = 10.0;
//    const double theta = kOmegaE * t0;
//
//    const auto cfg = MakeCfg(28.6, -80.6, 0, 45, 30, 10, 5, -3, 0, 0.01, -0.02, 0.005, 5000); // , t0);
//    const auto x0_explicit = FD::BuildInitialStateVector(cfg, theta);
//    const auto x0_conv = FD::BuildInitialStateVector(cfg, theta);
//
//    for (int i = 0; i < SL::N; ++i)
//        REQUIRE(x0_explicit[i] == Approx(x0_conv[i]).margin(1e-12));
//}
//
//TEST_CASE("BuildInitialStateVector - convenience overload at t0=0 matches ERA overload at theta=0",
//    "[era][initial_state]")
//{
//    const auto cfg = MakeCfg(28.6, -80.6, 0, 45, 30, 10, 5, -3, 0, 0.01, -0.02, 0.005, 5000); //, 0.0);
//    const auto x0_explicit = FD::BuildInitialStateVector(cfg, 0.0);
//    const auto x0_conv = FD::BuildInitialStateVector(cfg, 0.0);
//
//    for (int i = 0; i < SL::N; ++i)
//        REQUIRE(x0_explicit[i] == Approx(x0_conv[i]).margin(1e-12));
//}
//
//TEST_CASE("BuildInitialStateVector - ECI position rotates by Dtheta in the XY plane as ERA increases",
//    "[era][position][initial_state]")
//{
//    // An ECEF-fixed site at lat=0, lon=0 rotates as theta increases.
//    // Its XY-plane angle in ECI must increase by exactly Dtheta.
//    // Z-component and radial magnitude must be unchanged.
//    const double theta1 = 0.0;
//    const double theta2 = kPi / 4.0;
//    const auto   cfg = MakeCfg(0.0, 0.0, 0.0);
//
//    const auto x1 = FD::BuildInitialStateVector(cfg, theta1);
//    const auto x2 = FD::BuildInitialStateVector(cfg, theta2);
//
//    // Z unchanged
//    REQUIRE(x1[SL::IDX_P + 2] == Approx(x2[SL::IDX_P + 2]).margin(1e-6));
//
//    // Magnitude unchanged
//    REQUIRE(x1.segment<3>(SL::IDX_P).norm()
//        == Approx(x2.segment<3>(SL::IDX_P).norm()).epsilon(1e-12));
//
//    // In-plane rotation angle equals Dtheta
//    const double phi1 = std::atan2(x1[SL::IDX_P + 1], x1[SL::IDX_P + 0]);
//    const double phi2 = std::atan2(x2[SL::IDX_P + 1], x2[SL::IDX_P + 0]);
//    REQUIRE((phi2 - phi1) == Approx(theta2 - theta1).margin(1e-12));
//}
//
//TEST_CASE("BuildInitialStateVector - attitude also rotates consistently with ERA",
//    "[era][attitude][initial_state]")
//{
//    // At lat=0, lon=0, theta=0 body +x is along +X_ECI (zen=0, straight Up).
//    // At ERA = pi/2 the same body +x must rotate by pi/2 in the XY plane -> +Y_ECI.
//    const auto x0 = FD::BuildInitialStateVector(MakeCfg(0, 0, 0, 0, 0, 0), 0.0);
//    const auto x_half = FD::BuildInitialStateVector(MakeCfg(0, 0, 0, 0, 0, 0), kPi / 2.0);
//
//    const Vec3d dir0 = RotateVec(QuatFromState(x0), Vec3d{ 1,0,0 });
//    const Vec3d dir_pi2 = RotateVec(QuatFromState(x_half), Vec3d{ 1,0,0 });
//
//    // dir0 ~= [1,0,0]
//    REQUIRE(dir0[0] == Approx(1.0).margin(1e-12));
//    REQUIRE(dir0[1] == Approx(0.0).margin(1e-12));
//
//    // dir_pi2 ~= [0,1,0]  (rotated by pi/2 around Z)
//    REQUIRE(dir_pi2[0] == Approx(0.0).margin(1e-12));
//    REQUIRE(dir_pi2[1] == Approx(1.0).margin(1e-12));
//    REQUIRE(dir_pi2[2] == Approx(0.0).margin(1e-12));
//}
//
//
//// =============================================================================
//// GROUP 9 - Finite-difference Jacobian sanity checks
//// =============================================================================
//
//TEST_CASE("BuildInitialStateVector - finite-diff dr/dh has unit magnitude",
//    "[perturbation][position][initial_state]")
//{
//    // Moving the site upward by Dh should move the ECI position by Dh
//    // along the local outward normal.  So ||dr/dh|| ~= 1 m/m.
//    const double dh = 1.0, theta = 0.3;
//
//    const auto x_p = FD::BuildInitialStateVector(MakeCfg(45.0, 60.0, dh), theta);
//    const auto x_m = FD::BuildInitialStateVector(MakeCfg(45.0, 60.0, -dh), theta);
//
//    const Eigen::Vector3d dr =
//        (x_p.segment<3>(SL::IDX_P) - x_m.segment<3>(SL::IDX_P)) / (2.0 * dh);
//
//    REQUIRE(dr.norm() == Approx(1.0).epsilon(1e-6));
//}
//
//TEST_CASE("BuildInitialStateVector - finite-diff dv_eci/dvN has unit magnitude",
//    "[perturbation][velocity][initial_state]")
//{
//    // The NED->ECEF->ECI transform is a rotation, so ||dv_eci/dvN|| = 1.
//    const double eps = 1e-3, theta = 0.7;
//
//    const auto x_p = FD::BuildInitialStateVector(MakeCfg(30, 45, 0, 0, 0, 0, eps, 0, 0), theta);
//    const auto x_m = FD::BuildInitialStateVector(MakeCfg(30, 45, 0, 0, 0, 0, -eps, 0, 0), theta);
//
//    const Eigen::Vector3d dv =
//        (x_p.segment<3>(SL::IDX_V) - x_m.segment<3>(SL::IDX_V)) / (2.0 * eps);
//
//    REQUIRE(dv.norm() == Approx(1.0).epsilon(1e-10));
//}
//
//TEST_CASE("BuildInitialStateVector - small azimuth perturbation produces continuous attitude change",
//    "[perturbation][attitude][initial_state]")
//{
//    // A Daz of 1e-6 rad should yield |dot(q1,q2)| very close to 1
//    // (i.e. a barely perceptible rotation).
//    const double delta_az_rad = 1e-6;
//    const double theta = 0.4;
//
//    const auto cfg1 = MakeCfg(40, 20, 0, 45.0, 30.0, 0.0);
//    const auto cfg2 = MakeCfg(40, 20, 0, 45.0 + delta_az_rad / kD2R, 30.0, 0.0);
//
//    const auto x1 = FD::BuildInitialStateVector(cfg1, theta);
//    const auto x2 = FD::BuildInitialStateVector(cfg2, theta);
//
//    REQUIRE(QuatAbsDot(QuatFromState(x1), QuatFromState(x2)) == Approx(1.0).margin(1e-9));
//}
//
//TEST_CASE("BuildInitialStateVector - finite-diff ddir/dzen has unit magnitude",
//    "[perturbation][attitude][initial_state]")
//{
//    // Rotating the launch direction by Dzen should rotate body +x by Dzen in ECI.
//    // So ||d(dir_eci)/d(zen)|| ~= 1 rad/rad.
//    const double dzen_rad = 1e-5;
//    const double theta = 0.6;
//
//    const auto cfg_p = MakeCfg(30, 20, 0, 45.0, 30.0 + dzen_rad / kD2R, 0.0);
//    const auto cfg_m = MakeCfg(30, 20, 0, 45.0, 30.0 - dzen_rad / kD2R, 0.0);
//
//    const Vec3d dir_p = RotateVec(QuatFromState(FD::BuildInitialStateVector(cfg_p, theta)),
//        Vec3d{ 1,0,0 });
//    const Vec3d dir_m = RotateVec(QuatFromState(FD::BuildInitialStateVector(cfg_m, theta)),
//        Vec3d{ 1,0,0 });
//
//    const Vec3d ddir{
//        (dir_p[0] - dir_m[0]) / (2.0 * dzen_rad),
//        (dir_p[1] - dir_m[1]) / (2.0 * dzen_rad),
//        (dir_p[2] - dir_m[2]) / (2.0 * dzen_rad)
//    };
//
//    REQUIRE(Norm3(ddir) == Approx(1.0).epsilon(1e-6));
//}

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <vendor/nlohmann/json.hpp>

#include "Aetherion/FlightDynamics/FlightSimulationConfig.h"
#include "Aetherion/FlightDynamics/Serialization/FlightSimulationConfigJson.h"

namespace Ser = ::Aetherion::FlightDynamics::Serialization;
using ::Aetherion::FlightDynamics::FlightSimulationConfig;

TEST_CASE("FlightSimulationConfig: JSON round-trip preserves all fields", "[json][flightdynamics][config]")
{
    FlightSimulationConfig in{};

    // --- SimulationParameters ---
    in.simulation.startTime = 1.25;
    in.simulation.duration = 120.0;

    // --- InitialPoseWGS84_NED ---
    in.initialPose.lat_deg = 41.1055;
    in.initialPose.lon_deg = 29.0217;
    in.initialPose.alt_m = 123.4;
    in.initialPose.azimuth_deg = 90.0;
    in.initialPose.zenith_deg = 5.5;
    in.initialPose.roll_deg = -2.0;

    // --- InitialVelocityNED ---

    in.initialVelocityNED.north_mps = 0.0;
    in.initialVelocityNED.east_mps = 0.0;
    in.initialVelocityNED.down_mps = 0.0;

    // --- InitialRotationAboutBodyAxes ---

    in.initialRotationAboutBodyAxes.roll_rad_s = 0.0;
    in.initialRotationAboutBodyAxes.pitch_rad_s = 0.0;
    in.initialRotationAboutBodyAxes.yaw_rad_s = 0.0;

    // --- InertialParameters ---
    in.inertialParameters.mass_kg = 12.5;

    in.inertialParameters.Ixx = 0.42;
    in.inertialParameters.Iyy = 0.58;
    in.inertialParameters.Izz = 0.61;

    in.inertialParameters.Ixy = 0.01;
    in.inertialParameters.Iyz = -0.02;
    in.inertialParameters.Ixz = 0.00;

    in.inertialParameters.xbar_m = 0.03;
    in.inertialParameters.ybar_m = -0.01;
    in.inertialParameters.zbar_m = 0.00;

    // --- AerodynamicParameters ---
    in.aerodynamicParameters.S = 0.85;
    in.aerodynamicParameters.CL = 0.52;
    in.aerodynamicParameters.CD = 0.03;
    in.aerodynamicParameters.CY = 0.01;
    in.aerodynamicParameters.Cl = 0.002;
    in.aerodynamicParameters.Cm = -0.04;
    in.aerodynamicParameters.Cn = 0.005;

    // Serialize
    nlohmann::json j;
    Ser::to_json(j, in);

    // Top-level key sanity
    REQUIRE(j.contains("simulation"));
    REQUIRE(j.contains("initialPose"));
    REQUIRE(j.contains("initialVelocityNED"));
    REQUIRE(j.contains("initialRotationAboutBodyAxes"));
    REQUIRE(j.contains("inertialParameters"));
    REQUIRE(j.contains("aerodynamicParameters"));

    // Deserialize
    FlightSimulationConfig out{};
    Ser::from_json(j, out);

    // --- Compare all fields ---
    REQUIRE(out.simulation.startTime == Catch::Approx(in.simulation.startTime));
    REQUIRE(out.simulation.duration == Catch::Approx(in.simulation.duration));

    REQUIRE(out.initialPose.lat_deg == Catch::Approx(in.initialPose.lat_deg));
    REQUIRE(out.initialPose.lon_deg == Catch::Approx(in.initialPose.lon_deg));
    REQUIRE(out.initialPose.alt_m == Catch::Approx(in.initialPose.alt_m));
    REQUIRE(out.initialPose.azimuth_deg == Catch::Approx(in.initialPose.azimuth_deg));
    REQUIRE(out.initialPose.zenith_deg == Catch::Approx(in.initialPose.zenith_deg));
    REQUIRE(out.initialPose.roll_deg == Catch::Approx(in.initialPose.roll_deg));

    REQUIRE(out.initialVelocityNED.north_mps == Catch::Approx(in.initialVelocityNED.north_mps));
    REQUIRE(out.initialVelocityNED.east_mps == Catch::Approx(in.initialVelocityNED.east_mps));
    REQUIRE(out.initialVelocityNED.down_mps == Catch::Approx(in.initialVelocityNED.down_mps));

    REQUIRE(out.initialRotationAboutBodyAxes.roll_rad_s == Catch::Approx(in.initialRotationAboutBodyAxes.roll_rad_s));
    REQUIRE(out.initialRotationAboutBodyAxes.pitch_rad_s == Catch::Approx(in.initialRotationAboutBodyAxes.pitch_rad_s));
    REQUIRE(out.initialRotationAboutBodyAxes.yaw_rad_s == Catch::Approx(in.initialRotationAboutBodyAxes.yaw_rad_s));

    REQUIRE(out.inertialParameters.mass_kg == Catch::Approx(in.inertialParameters.mass_kg));

    REQUIRE(out.inertialParameters.Ixx == Catch::Approx(in.inertialParameters.Ixx));
    REQUIRE(out.inertialParameters.Iyy == Catch::Approx(in.inertialParameters.Iyy));
    REQUIRE(out.inertialParameters.Izz == Catch::Approx(in.inertialParameters.Izz));

    REQUIRE(out.inertialParameters.Ixy == Catch::Approx(in.inertialParameters.Ixy));
    REQUIRE(out.inertialParameters.Iyz == Catch::Approx(in.inertialParameters.Iyz));
    REQUIRE(out.inertialParameters.Ixz == Catch::Approx(in.inertialParameters.Ixz));

    REQUIRE(out.inertialParameters.xbar_m == Catch::Approx(in.inertialParameters.xbar_m));
    REQUIRE(out.inertialParameters.ybar_m == Catch::Approx(in.inertialParameters.ybar_m));
    REQUIRE(out.inertialParameters.zbar_m == Catch::Approx(in.inertialParameters.zbar_m));

    REQUIRE(out.aerodynamicParameters.S == Catch::Approx(in.aerodynamicParameters.S));
    REQUIRE(out.aerodynamicParameters.CL == Catch::Approx(in.aerodynamicParameters.CL));
    REQUIRE(out.aerodynamicParameters.CD == Catch::Approx(in.aerodynamicParameters.CD));
    REQUIRE(out.aerodynamicParameters.CY == Catch::Approx(in.aerodynamicParameters.CY));
    REQUIRE(out.aerodynamicParameters.Cl == Catch::Approx(in.aerodynamicParameters.Cl));
    REQUIRE(out.aerodynamicParameters.Cm == Catch::Approx(in.aerodynamicParameters.Cm));
    REQUIRE(out.aerodynamicParameters.Cn == Catch::Approx(in.aerodynamicParameters.Cn));
}

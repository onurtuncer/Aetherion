// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Aetherion/Estimation/LogContract.h>

using namespace Aetherion::Estimation;
using namespace Aetherion::Estimation::LogContract;
using Catch::Matchers::ContainsSubstring;

// Header lines exactly as Hemerion c4a3178 writes them
// (examples/f16_trim_ecos/f16_flight_computer.cpp, cosim_host_main.cpp).
namespace
{
constexpr std::string_view kGpsHeader = "fix_index,nominal_time_s,host_time_s,latitude_deg,longitude_deg,altitude_m,"
                                        "ground_speed_mps,course_deg,"
                                        "horizontal_accuracy_m,vertical_accuracy_m,num_satellites,fix_type";
constexpr std::string_view kImuHeader = "sample_index,part_time_s,host_time_s,accel_x_mps2,accel_y_mps2,accel_z_mps2,"
                                        "gyro_x_rad_s,gyro_y_rad_s,"
                                        "gyro_z_rad_s";
constexpr std::string_view kBaroHeader = "sample_index,part_time_s,host_time_s,pressure_pa,temperature_c";
constexpr std::string_view kF16TruthHeader = "iterations, time, f16::out.alt_m[REAL], f16::out.lat_deg[REAL], "
                                             "f16::out.lon_deg[REAL], "
                                             "f16::out.v_north_m_s[REAL], f16::out.v_east_m_s[REAL], "
                                             "f16::out.v_down_m_s[REAL], "
                                             "f16::out.yaw_rad[REAL], f16::out.pitch_rad[REAL], "
                                             "f16::out.roll_rad[REAL], f16::out.p_rad_s[REAL]";
constexpr std::string_view kRocketTruthHeader = "iterations, time, rocket::out.alt_m[REAL], rocket::out.lat_deg[REAL], "
                                                "rocket::out.lon_deg[REAL], "
                                                "rocket::out.v_north_m_s[REAL], rocket::out.v_east_m_s[REAL], "
                                                "rocket::out.v_down_m_s[REAL], "
                                                "rocket::out.p_rad_s[REAL], rocket::out.staged[BOOL]";

const Configuration kGnssPosBaro{ { BlockId::NavCore, BlockId::GyroBias, BlockId::AccelBias, BlockId::BaroBias },
                                  { MeasurementId::GnssPosition, MeasurementId::Barometer } };
}  // namespace

TEST_CASE("Current IMU, baro and F-16 truth logs bind", "[estimation][logcontract]")
{
  const auto imu = bindColumns(Log::ImuSamples, kImuHeader, kNavBaroGnss16);
  REQUIRE(imu.at("gyro_z_rad_s") == 8);

  const auto baro = bindColumns(Log::BaroSamples, kBaroHeader, kNavBaroGnss16);
  REQUIRE(baro.at("pressure_pa") == 3);

  const auto truth = bindColumns(Log::Truth, kF16TruthHeader, kNavBaroGnss16);
  REQUIRE(truth.at("time") == 1);
  REQUIRE(truth.at("out.lat_deg") == 3);
  REQUIRE(truth.at("out.roll_rad") == 10);
}

TEST_CASE("GNSS position binds against the current GPS log", "[estimation][logcontract]")
{
  const auto gps = bindColumns(Log::GpsFixes, kGpsHeader, kGnssPosBaro);
  REQUIRE(gps.at("latitude_deg") == 3);
  REQUIRE(gps.at("fix_type") == 11);
}

TEST_CASE("GNSS velocity fails loudly until Hemerion logs velN/E/D", "[estimation][logcontract]")
{
  REQUIRE_THROWS_WITH(bindColumns(Log::GpsFixes, kGpsHeader, kNavBaroGnss16),
                      ContainsSubstring("vel_north_mps") && ContainsSubstring("vel_east_mps") &&
                          ContainsSubstring("vel_down_mps") && ContainsSubstring("speed_accuracy_mps"));
}

TEST_CASE("Rocket truth log lacks attitude", "[estimation][logcontract]")
{
  REQUIRE_THROWS_WITH(bindColumns(Log::Truth, kRocketTruthHeader, kNavBaroGnss16),
                      ContainsSubstring("out.yaw_rad") && ContainsSubstring("out.roll_rad"));
}

TEST_CASE("A renamed column is reported, not shifted", "[estimation][logcontract]")
{
  const std::string renamed = "sample_index,part_time_s,host_time_s,pressure_hpa,temperature_c";
  REQUIRE_THROWS_WITH(bindColumns(Log::BaroSamples, renamed, kNavBaroGnss16), ContainsSubstring("pressure_pa"));
}

TEST_CASE("Logs a configuration does not use are not required", "[estimation][logcontract]")
{
  const Configuration noBaro{ { BlockId::NavCore }, { MeasurementId::GnssPosition } };
  REQUIRE(requiredColumns(Log::BaroSamples, noBaro).empty());
  REQUIRE_NOTHROW(bindColumns(Log::BaroSamples, "anything", noBaro));
}

TEST_CASE("Sidecar parsing", "[estimation][logcontract]")
{
  const auto kv = parseSidecar("check_case=11\nlat0_deg=36.5\nlon0_deg=-121.25\nalt0_ft=10000\n"
                               "gps_latency_s=0\nrealtime_factor=0\n");
  REQUIRE(kv.at("lat0_deg") == "36.5");
  REQUIRE(kv.at("check_case") == "11");

  REQUIRE_THROWS_WITH(parseSidecar("lat0_deg=1\n"),
                      ContainsSubstring("lon0_deg") && ContainsSubstring("realtime_factor"));
}

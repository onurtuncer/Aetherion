// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// LogContract.h
//
// The columns this repository reads from Hemerion's co-simulation logs
// (sensor fusion step 2). Aetherion does not depend on Hemerion: it replays
// the logs Hemerion's hosts write, and this file is the whole interface.
//
// Every column is looked up by name when a log is opened, and every missing
// one is reported at once, so a rename on the Hemerion side fails loudly
// instead of shifting a column.
//
// Logs (Hemerion examples/*_ecos, as of Hemerion c4a3178):
//   gps_fixes.csv     flight computer, decoded UBX-NAV-PVT
//   imu_samples.csv   flight computer, decoded IMU frames
//   baro_samples.csv  flight computer, decoded BMP390 samples
//   <truth>.csv       co-simulation host (f16_truth.csv, rocket_truth.csv);
//                     ", "-separated, columns "<instance>::<variable>[REAL]"
//   <truth>.config    key=value run configuration beside the truth log
//
// Time base: host_time_s is the flight computer's clock and equals simulation
// time only under realtime_factor=1; part_time_s is the sensor's own clock.
// The truth log's `time` is simulation time.
// ------------------------------------------------------------------------------

#pragma once

#include <cstddef>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <Aetherion/Estimation/Configuration.h>

namespace Aetherion::Estimation::LogContract
{

enum class Log : std::uint8_t
{
  GpsFixes,
  ImuSamples,
  BaroSamples,
  Truth
};

[[nodiscard]] constexpr std::string_view defaultFileName(Log log)
{
  switch (log)
  {
    case Log::GpsFixes:
      return "gps_fixes.csv";
    case Log::ImuSamples:
      return "imu_samples.csv";
    case Log::BaroSamples:
      return "baro_samples.csv";
    case Log::Truth:
      return "truth.csv";  // f16_truth.csv / rocket_truth.csv in practice
  }
  return "?";
}

// ── Column names ----------------------------------------------------------

namespace Gps
{
inline constexpr std::string_view kHostTime = "host_time_s";
inline constexpr std::string_view kLatitude = "latitude_deg";
inline constexpr std::string_view kLongitude = "longitude_deg";
// NAV-PVT `height` (ellipsoidal): Hemerion's ubxParser reads payload
// offset 32, although GpsFix::altitude_m is documented as MSL.
inline constexpr std::string_view kAltitude = "altitude_m";
inline constexpr std::string_view kHorizAcc = "horizontal_accuracy_m";
inline constexpr std::string_view kVertAcc = "vertical_accuracy_m";
inline constexpr std::string_view kFixType = "fix_type";
// Not yet written by Hemerion: GpsFix decodes only gSpeed and course,
// although the emitter already encodes velN/E/D and sAcc. Proposed
// names; a configuration with GnssVelocity cannot replay until they
// exist.
inline constexpr std::string_view kVelNorth = "vel_north_mps";
inline constexpr std::string_view kVelEast = "vel_east_mps";
inline constexpr std::string_view kVelDown = "vel_down_mps";
inline constexpr std::string_view kSpeedAcc = "speed_accuracy_mps";
}  // namespace Gps

namespace Imu
{
inline constexpr std::string_view kPartTime = "part_time_s";
inline constexpr std::string_view kHostTime = "host_time_s";
inline constexpr std::string_view kAccelX = "accel_x_mps2";
inline constexpr std::string_view kAccelY = "accel_y_mps2";
inline constexpr std::string_view kAccelZ = "accel_z_mps2";
inline constexpr std::string_view kGyroX = "gyro_x_rad_s";
inline constexpr std::string_view kGyroY = "gyro_y_rad_s";
inline constexpr std::string_view kGyroZ = "gyro_z_rad_s";
}  // namespace Imu

namespace Baro
{
inline constexpr std::string_view kPartTime = "part_time_s";
inline constexpr std::string_view kHostTime = "host_time_s";
inline constexpr std::string_view kPressure = "pressure_pa";
inline constexpr std::string_view kTemperature = "temperature_c";
}  // namespace Baro

// Truth columns after normalisation (instance prefix and [TYPE] suffix
// stripped): "f16::out.lat_deg[REAL]" -> "out.lat_deg".
namespace Truth
{
inline constexpr std::string_view kTime = "time";
inline constexpr std::string_view kLatitude = "out.lat_deg";
inline constexpr std::string_view kLongitude = "out.lon_deg";
inline constexpr std::string_view kAltitude = "out.alt_m";
inline constexpr std::string_view kVelNorth = "out.v_north_m_s";
inline constexpr std::string_view kVelEast = "out.v_east_m_s";
inline constexpr std::string_view kVelDown = "out.v_down_m_s";
// ZYX Euler, body -> NED. Adequate for the F-16 cases; singular at
// 90 deg pitch, and not logged at all by rocket_gps_ecos.
inline constexpr std::string_view kYaw = "out.yaw_rad";
inline constexpr std::string_view kPitch = "out.pitch_rad";
inline constexpr std::string_view kRoll = "out.roll_rad";
}  // namespace Truth

// Run-configuration sidecar keys. lat0/lon0/alt0 define the LCI origin.
namespace Sidecar
{
inline constexpr std::string_view kLat0 = "lat0_deg";
inline constexpr std::string_view kLon0 = "lon0_deg";
inline constexpr std::string_view kAlt0Ft = "alt0_ft";
inline constexpr std::string_view kGpsLatency = "gps_latency_s";
inline constexpr std::string_view kRealtimeFactor = "realtime_factor";
}  // namespace Sidecar

// ── Requirements per configuration ----------------------------------------

/// Columns this repository reads from `log` when replaying `cfg`.
[[nodiscard]] inline std::vector<std::string_view> requiredColumns(Log log, const Configuration& cfg)
{
  switch (log)
  {
    case Log::GpsFixes:
    {
      std::vector<std::string_view> c;
      if (cfg.has(MeasurementId::GnssPosition) || cfg.has(MeasurementId::GnssVelocity))
        c.insert(c.end(), { Gps::kHostTime, Gps::kFixType });
      if (cfg.has(MeasurementId::GnssPosition))
        c.insert(c.end(), { Gps::kLatitude, Gps::kLongitude, Gps::kAltitude, Gps::kHorizAcc, Gps::kVertAcc });
      if (cfg.has(MeasurementId::GnssVelocity))
        c.insert(c.end(), { Gps::kVelNorth, Gps::kVelEast, Gps::kVelDown, Gps::kSpeedAcc });
      return c;
    }
    case Log::ImuSamples:
      return { Imu::kPartTime, Imu::kHostTime, Imu::kAccelX, Imu::kAccelY,
               Imu::kAccelZ,   Imu::kGyroX,    Imu::kGyroY,  Imu::kGyroZ };
    case Log::BaroSamples:
      if (!cfg.has(MeasurementId::Barometer))
        return {};
      return { Baro::kPartTime, Baro::kHostTime, Baro::kPressure, Baro::kTemperature };
    case Log::Truth:
      return { Truth::kTime,    Truth::kLatitude, Truth::kLongitude, Truth::kAltitude, Truth::kVelNorth,
               Truth::kVelEast, Truth::kVelDown,  Truth::kYaw,       Truth::kPitch,    Truth::kRoll };
  }
  return {};
}

[[nodiscard]] inline std::vector<std::string_view> requiredSidecarKeys()
{
  return { Sidecar::kLat0, Sidecar::kLon0, Sidecar::kAlt0Ft, Sidecar::kGpsLatency, Sidecar::kRealtimeFactor };
}

// ── Header binding ---------------------------------------------------------

namespace detail
{
[[nodiscard]] inline std::string_view trim(std::string_view s)
{
  constexpr std::string_view ws = " \t\r\n";
  const auto b = s.find_first_not_of(ws);
  if (b == std::string_view::npos)
    return {};
  return s.substr(b, s.find_last_not_of(ws) - b + 1);
}

/// "f16::out.lat_deg[REAL]" -> "out.lat_deg"; plain names unchanged.
[[nodiscard]] inline std::string_view normalise(std::string_view s)
{
  s = trim(s);
  if (!s.empty() && s.back() == ']')
    if (auto lb = s.rfind('['); lb != std::string_view::npos)
      s = s.substr(0, lb);
  if (auto sep = s.rfind("::"); sep != std::string_view::npos)
    s = s.substr(sep + 2);
  return s;
}
}  // namespace detail

/// Column name -> index in the header.
using ColumnMap = std::map<std::string, std::size_t, std::less<>>;

/// Splits `header` on commas, normalises each name, and checks that every
/// required column for (log, cfg) is present. Throws std::runtime_error
/// naming all missing columns.
[[nodiscard]] inline ColumnMap bindColumns(Log log, std::string_view header, const Configuration& cfg)
{
  ColumnMap map;
  std::size_t index = 0;
  for (std::size_t start = 0;; ++index)
  {
    const auto comma = header.find(',', start);
    const auto name = detail::normalise(header.substr(start, comma - start));
    map.try_emplace(std::string(name), index);
    if (comma == std::string_view::npos)
      break;
    start = comma + 1;
  }

  std::string missing;
  for (std::string_view col : requiredColumns(log, cfg))
  {
    if (map.contains(col))
      continue;
    if (!missing.empty())
      missing.append(", ");
    missing.append(col);
  }
  if (!missing.empty())
    throw std::runtime_error(std::string(defaultFileName(log)) + ": missing column(s): " + missing);
  return map;
}

/// Parses `key=value` lines; blank lines and lines without '=' are ignored.
/// Throws std::runtime_error naming all required keys that are absent.
[[nodiscard]] inline std::map<std::string, std::string, std::less<>> parseSidecar(std::string_view text)
{
  std::map<std::string, std::string, std::less<>> kv;
  while (!text.empty())
  {
    const auto nl = text.find('\n');
    const auto line = detail::trim(text.substr(0, nl));
    if (auto eq = line.find('='); eq != std::string_view::npos)
      kv.insert_or_assign(std::string(detail::trim(line.substr(0, eq))),
                          std::string(detail::trim(line.substr(eq + 1))));
    if (nl == std::string_view::npos)
      break;
    text.remove_prefix(nl + 1);
  }

  std::string missing;
  for (std::string_view key : requiredSidecarKeys())
  {
    if (kv.contains(key))
      continue;
    if (!missing.empty())
      missing.append(", ");
    missing.append(key);
  }
  if (!missing.empty())
    throw std::runtime_error("run-configuration sidecar: missing key(s): " + missing);
  return kv;
}

}  // namespace Aetherion::Estimation::LogContract

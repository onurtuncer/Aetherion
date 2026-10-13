// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Logs.h
//
// Hemerion's co-simulation logs as typed records (sensor fusion step 6).
// Column names come from LogContract.h and nowhere else; units are the
// logs' own (degrees for latitude and longitude, metres, m/s, rad/s,
// pascals). Nothing is converted or re-timed here: the replay loop decides
// which clock a record lives on (TimeBase.h) and converts units as it
// builds measurements.
//
// A log set is loaded from a directory holding gps_fixes.csv,
// imu_samples.csv and baro_samples.csv, plus the truth CSV and its
// `.config` sidecar, or from streams (tests). Logs the configuration does
// not use are not required to exist.
// ------------------------------------------------------------------------------

#pragma once

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <istream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <Aetherion/Estimation/Configuration.h>
#include <Aetherion/Estimation/LogContract.h>
#include <Aetherion/Estimation/Replay/Csv.h>

namespace Aetherion::Estimation::Replay
{

struct ImuRecord
{
  double partTime = 0.0;  ///< sensor clock [s]
  double hostTime = 0.0;  ///< flight computer clock [s]
  Eigen::Vector3d gyro_rad_s;
  Eigen::Vector3d accel_m_s2;
};

struct GpsRecord
{
  bool valid = false;       ///< false: an epoch without a fix (empty fields), nothing below is meaningful
  double nominalTime = 0.0; ///< NaN if the log has no nominal_time_s column
  double hostTime = 0.0;
  double lat_deg = 0.0, lon_deg = 0.0, alt_m = 0.0;
  double horizontalAccuracy_m = 0.0, verticalAccuracy_m = 0.0;
  int fixType = 0;
  bool hasVelocity = false;
  Eigen::Vector3d velNed_m_s = Eigen::Vector3d::Zero();
  double speedAccuracy_m_s = 0.0;
};

struct BaroRecord
{
  double partTime = 0.0;
  double hostTime = 0.0;
  double pressure_Pa = 0.0;
  double temperature_C = 0.0;
};

struct TruthRecord
{
  double time = 0.0;  ///< simulation time [s]
  double lat_deg = 0.0, lon_deg = 0.0, alt_m = 0.0;
  Eigen::Vector3d vNed_m_s;
  double yaw_rad = 0.0, pitch_rad = 0.0, roll_rad = 0.0;
};

struct RunConfiguration
{
  double lat0_deg = 0.0;
  double lon0_deg = 0.0;
  double alt0_m = 0.0;  ///< the sidecar's alt0_ft, converted
  double gpsLatency_s = 0.0;
  double realtimeFactor = 0.0;
  std::map<std::string, std::string, std::less<>> raw;  ///< every key the sidecar had
};

struct LogSet
{
  std::vector<ImuRecord> imu;
  std::vector<GpsRecord> gps;
  std::vector<BaroRecord> baro;
  std::vector<TruthRecord> truth;
  RunConfiguration run;
};

// ---- table -> records -------------------------------------------------------

[[nodiscard]] inline std::vector<ImuRecord> imuRecords(const CsvTable& t)
{
  namespace C = LogContract::Imu;
  std::vector<ImuRecord> out;
  out.reserve(t.rows.size());
  for (std::size_t i = 0; i < t.rows.size(); ++i)
  {
    ImuRecord r;
    r.partTime = t.at(i, C::kPartTime);
    r.hostTime = t.at(i, C::kHostTime);
    r.gyro_rad_s = Eigen::Vector3d(t.at(i, C::kGyroX), t.at(i, C::kGyroY), t.at(i, C::kGyroZ));
    r.accel_m_s2 = Eigen::Vector3d(t.at(i, C::kAccelX), t.at(i, C::kAccelY), t.at(i, C::kAccelZ));
    out.push_back(r);
  }
  return out;
}

[[nodiscard]] inline std::vector<GpsRecord> gpsRecords(const CsvTable& t)
{
  namespace C = LogContract::Gps;
  const bool vel = t.has(C::kVelNorth) && t.has(C::kVelEast) && t.has(C::kVelDown) && t.has(C::kSpeedAcc);
  const bool nominal = t.has(C::kNominalTime);
  std::vector<GpsRecord> out;
  out.reserve(t.rows.size());
  for (std::size_t i = 0; i < t.rows.size(); ++i)
  {
    GpsRecord r;
    r.nominalTime = nominal ? t.at(i, C::kNominalTime) : std::numeric_limits<double>::quiet_NaN();
    r.hostTime = t.at(i, C::kHostTime);
    r.lat_deg = t.at(i, C::kLatitude);
    r.lon_deg = t.at(i, C::kLongitude);
    r.alt_m = t.at(i, C::kAltitude);
    r.horizontalAccuracy_m = t.at(i, C::kHorizAcc);
    r.verticalAccuracy_m = t.at(i, C::kVertAcc);
    const double ft = t.at(i, C::kFixType);
    r.valid = std::isfinite(r.lat_deg) && std::isfinite(r.lon_deg) && std::isfinite(r.alt_m) && std::isfinite(ft);
    r.fixType = r.valid ? static_cast<int>(std::lround(ft)) : 0;
    r.hasVelocity = vel && r.valid;
    if (r.hasVelocity)
    {
      r.velNed_m_s = Eigen::Vector3d(t.at(i, C::kVelNorth), t.at(i, C::kVelEast), t.at(i, C::kVelDown));
      r.speedAccuracy_m_s = t.at(i, C::kSpeedAcc);
    }
    out.push_back(r);
  }
  return out;
}

[[nodiscard]] inline std::vector<BaroRecord> baroRecords(const CsvTable& t)
{
  namespace C = LogContract::Baro;
  std::vector<BaroRecord> out;
  out.reserve(t.rows.size());
  for (std::size_t i = 0; i < t.rows.size(); ++i)
    out.push_back({ t.at(i, C::kPartTime), t.at(i, C::kHostTime), t.at(i, C::kPressure), t.at(i, C::kTemperature) });
  return out;
}

[[nodiscard]] inline std::vector<TruthRecord> truthRecords(const CsvTable& t)
{
  namespace C = LogContract::Truth;
  std::vector<TruthRecord> out;
  out.reserve(t.rows.size());
  for (std::size_t i = 0; i < t.rows.size(); ++i)
  {
    TruthRecord r;
    r.time = t.at(i, C::kTime);
    r.lat_deg = t.at(i, C::kLatitude);
    r.lon_deg = t.at(i, C::kLongitude);
    r.alt_m = t.at(i, C::kAltitude);
    r.vNed_m_s = Eigen::Vector3d(t.at(i, C::kVelNorth), t.at(i, C::kVelEast), t.at(i, C::kVelDown));
    r.yaw_rad = t.at(i, C::kYaw);
    r.pitch_rad = t.at(i, C::kPitch);
    r.roll_rad = t.at(i, C::kRoll);
    out.push_back(r);
  }
  return out;
}

[[nodiscard]] inline RunConfiguration runConfiguration(std::string_view sidecarText)
{
  namespace K = LogContract::Sidecar;
  RunConfiguration c;
  c.raw = LogContract::parseSidecar(sidecarText);
  auto num = [&](std::string_view key) {
    const std::string& s = c.raw.at(std::string(key));
    try
    {
      return std::stod(s);
    }
    catch (const std::exception&)
    {
      throw std::runtime_error("run-configuration sidecar: " + std::string(key) + " is not a number: " + s);
    }
  };
  c.lat0_deg = num(K::kLat0);
  c.lon0_deg = num(K::kLon0);
  c.alt0_m = num(K::kAlt0Ft) * 0.3048;
  c.gpsLatency_s = num(K::kGpsLatency);
  c.realtimeFactor = num(K::kRealtimeFactor);
  return c;
}

// ---- loading ----------------------------------------------------------------

/// Streams in place of files; a null stream means "not provided".
struct LogStreams
{
  std::istream* gps = nullptr;
  std::istream* imu = nullptr;
  std::istream* baro = nullptr;
  std::istream* truth = nullptr;
  std::string_view sidecar;
};

[[nodiscard]] inline LogSet loadLogs(const LogStreams& s, const Configuration& cfg)
{
  using LogContract::Log;
  LogSet logs;
  if (!s.imu)
    throw std::runtime_error("imu_samples.csv is required");
  if (!s.truth)
    throw std::runtime_error("the truth log is required");
  logs.imu = imuRecords(readCsv(*s.imu, Log::ImuSamples, cfg, "imu_samples.csv"));
  logs.truth = truthRecords(readCsv(*s.truth, Log::Truth, cfg, "truth"));
  logs.run = runConfiguration(s.sidecar);

  const bool wantGps = cfg.has(MeasurementId::GnssPosition) || cfg.has(MeasurementId::GnssVelocity);
  if (wantGps)
  {
    if (!s.gps)
      throw std::runtime_error("gps_fixes.csv is required by this configuration");
    logs.gps = gpsRecords(readCsv(*s.gps, Log::GpsFixes, cfg, "gps_fixes.csv"));
  }
  if (cfg.has(MeasurementId::Barometer))
  {
    if (!s.baro)
      throw std::runtime_error("baro_samples.csv is required by this configuration");
    logs.baro = baroRecords(readCsv(*s.baro, Log::BaroSamples, cfg, "baro_samples.csv"));
  }
  return logs;
}

/// `dir` holds the three sensor logs under their default names; `truth` is
/// the truth CSV and `sidecar` its `.config` (default: `truth` with the
/// extension replaced).
[[nodiscard]] inline LogSet loadLogs(const std::filesystem::path& dir,
                                     const std::filesystem::path& truth,
                                     const Configuration& cfg,
                                     std::filesystem::path sidecar = {})
{
  using LogContract::Log;
  if (sidecar.empty())
    sidecar = std::filesystem::path(truth).replace_extension(".config");

  auto open = [](const std::filesystem::path& p, bool required) {
    auto f = std::make_unique<std::ifstream>(p);
    if (!*f)
    {
      if (required)
        throw std::runtime_error("cannot open " + p.string());
      return std::unique_ptr<std::ifstream>{};
    }
    return f;
  };
  const bool wantGps = cfg.has(MeasurementId::GnssPosition) || cfg.has(MeasurementId::GnssVelocity);
  auto gps = open(dir / LogContract::defaultFileName(Log::GpsFixes), wantGps);
  auto imu = open(dir / LogContract::defaultFileName(Log::ImuSamples), true);
  auto baro = open(dir / LogContract::defaultFileName(Log::BaroSamples), cfg.has(MeasurementId::Barometer));
  auto tr = open(truth, true);
  std::ifstream sc(sidecar);
  if (!sc)
    throw std::runtime_error("cannot open " + sidecar.string());
  std::stringstream scText;
  scText << sc.rdbuf();
  const std::string text = scText.str();

  return loadLogs(LogStreams{ gps.get(), imu.get(), baro.get(), tr.get(), text }, cfg);
}

}  // namespace Aetherion::Estimation::Replay

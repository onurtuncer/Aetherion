// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// EkfReplay.cpp
//
// Replays the reference invariant EKF over a set of Hemerion co-simulation
// logs and judges it against the truth log (sensor fusion step 6;
// doc/sensor_fusion.rst "Replay and log contract").
//
//   EkfReplay --logs <dir> --truth <truth.csv> [--sidecar <truth.config>]
//             [--convention ri|li] [--config nav-baro-gnss16|nav-baro-gnss]
//             [--output <prefix>] [--every <s>]
//             [--gyro-density <rad/s/sqrt(Hz)>] [--accel-density <m/s^2/sqrt(Hz)>]
//             [--baro-sigma <m>] [--min-fix-type <n>]
//
// <dir> holds gps_fixes.csv, imu_samples.csv and baro_samples.csv; the
// sidecar defaults to the truth file with the extension replaced by
// `.config`. Writes <prefix>_states.csv, <prefix>_updates.csv and prints
// the summary. The `nav-baro-gnss` configuration drops GNSS velocity,
// which Hemerion's GPS log does not carry yet.
// ------------------------------------------------------------------------------

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <Aetherion/Estimation/Replay/Replay.h>
#include <Aetherion/Simulation/ArgumentParser.h>

namespace
{
using namespace Aetherion::Estimation;

const Configuration kNavBaroGnss{ { BlockId::NavCore, BlockId::GyroBias, BlockId::AccelBias, BlockId::BaroBias },
                                  { MeasurementId::GnssPosition, MeasurementId::Barometer } };
}  // namespace

int main(int argc, char** argv)
{
  std::string logsDir, truthFile, sidecarFile, conventionName = "ri", configName = "nav-baro-gnss";
  std::string outputPrefix = "ekf_replay";
  Replay::ReplayOptions opt;

  Aetherion::Simulation::ArgumentParser parser(argv[0]);
  parser.addArgument("--logs",
                     "Directory with gps_fixes.csv, imu_samples.csv, baro_samples.csv",
                     [&](const std::string& v) { logsDir = v; });
  parser.addArgument(
      "--truth", "Truth CSV written by the co-simulation host", [&](const std::string& v) { truthFile = v; });
  parser.addArgument("--sidecar", "Run-configuration sidecar (default: truth with .config)", [&](const std::string& v) {
    sidecarFile = v;
  });
  parser.addArgument(
      "--convention", "Error convention: ri (default) or li", [&](const std::string& v) { conventionName = v; });
  parser.addArgument("--config",
                     "nav-baro-gnss (default; no GNSS velocity) or nav-baro-gnss16",
                     [&](const std::string& v) { configName = v; });
  parser.addArgument("--output", "Output prefix (default ekf_replay)", [&](const std::string& v) { outputPrefix = v; });
  parser.addArgument("--every",
                     "Write a state row every N seconds (default: every IMU sample)",
                     [&](const std::string& v) { opt.outputEvery_s = std::stod(v); });
  parser.addArgument("--gyro-density", "Gyro noise density [rad/s/sqrt(Hz)]", [&](const std::string& v) {
    opt.noise.gyroNoise = std::stod(v);
  });
  parser.addArgument("--accel-density", "Accelerometer noise density [m/s^2/sqrt(Hz)]", [&](const std::string& v) {
    opt.noise.accelNoise = std::stod(v);
  });
  parser.addArgument("--baro-sigma", "Pressure-altitude noise given to the filter [m]", [&](const std::string& v) {
    opt.baroSigma_m = std::stod(v);
  });
  parser.addArgument("--min-fix-type",
                     "Skip fixes with a smaller NAV-PVT fixType (default 3)",
                     [&](const std::string& v) { opt.minFixType = std::stoi(v); });

  try
  {
    parser.parse(argc, argv);
    if (logsDir.empty() || truthFile.empty())
    {
      parser.printUsage();
      return 2;
    }
    const Configuration& cfg = (configName == "nav-baro-gnss16") ? kNavBaroGnss16 : kNavBaroGnss;
    const ErrorConvention conv =
        (conventionName == "li") ? ErrorConvention::LeftInvariant : ErrorConvention::RightInvariant;

    const Replay::LogSet logs = Replay::loadLogs(logsDir, truthFile, cfg, sidecarFile);
    const Replay::ReplayResult result = Replay::replay(conv, logs, cfg, opt);

    std::ofstream states(outputPrefix + "_states.csv");
    Replay::writeStates(states, result, cfg);
    std::ofstream updates(outputPrefix + "_updates.csv");
    Replay::writeUpdates(updates, result);
    Replay::writeSummary(std::cout, result.summary);
    std::cout << "wrote " << outputPrefix << "_states.csv and " << outputPrefix << "_updates.csv\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "EkfReplay: " << e.what() << '\n';
    return 1;
  }
}

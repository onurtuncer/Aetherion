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
//             [--gyro-bias-walk <rad/s^2/sqrt(Hz)>] [--accel-bias-walk <m/s^3/sqrt(Hz)>]
//             [--baro-bias-walk <m/s/sqrt(Hz)>]
//             [--gnss-horiz-scale <k>] [--gnss-vert-scale <k>] [--gnss-vel-scale <k>]
//             [--init-attitude-sigma-deg <deg>] [--init-velocity-sigma <m/s>]
//             [--init-position-sigma <m>] [--init-gyro-bias-sigma <rad/s>]
//             [--init-accel-bias-sigma <m/s^2>] [--init-baro-bias-sigma <m>]
//             [--imu-time-shift <s>] [--csv on|off]
//
// <dir> holds gps_fixes.csv, imu_samples.csv and baro_samples.csv; the
// sidecar defaults to the truth file with the extension replaced by
// `.config`. Writes <prefix>_states.csv, <prefix>_updates.csv and prints
// the summary. The `nav-baro-gnss` configuration drops GNSS velocity,
// which Hemerion's GPS log does not carry yet. The tuning options (step 7)
// default to the values of ReplayOptions; --csv off prints the summary only.
// ------------------------------------------------------------------------------

#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <optional>
#include <string>

#include <Aetherion/Estimation/Replay/Replay.h>
#include <Aetherion/Simulation/ArgumentParser.h>

namespace {
    using namespace Aetherion::Estimation;

    const Configuration kNavBaroGnss{ { BlockId::NavCore, BlockId::GyroBias, BlockId::AccelBias, BlockId::BaroBias },
        { MeasurementId::GnssPosition, MeasurementId::Barometer } };
} // namespace

int main(int argc, char** argv)
{
    std::string logsDir, truthFile, sidecarFile, conventionName = "ri", configName = "nav-baro-gnss";
    std::string outputPrefix = "ekf_replay";
    Replay::ReplayOptions opt;
    bool writeCsv = true;
    // Initial 1-sigma overrides, physical coordinates; unset keeps the replay's default.
    std::optional<double> sigAtt_deg, sigVel, sigPos, sigGyroBias, sigAccelBias, sigBaroBias;

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
    parser.addArgument("--gyro-bias-walk", "Gyro bias random walk [rad/s^2/sqrt(Hz)]", [&](const std::string& v) {
        opt.noise.gyroBiasWalk = std::stod(v);
    });
    parser.addArgument("--accel-bias-walk", "Accelerometer bias random walk [m/s^3/sqrt(Hz)]", [&](const std::string& v) {
        opt.noise.accelBiasWalk = std::stod(v);
    });
    parser.addArgument("--baro-bias-walk", "Barometer bias random walk [m/s/sqrt(Hz)]", [&](const std::string& v) {
        opt.noise.baroBiasWalk = std::stod(v);
    });
    parser.addArgument("--gnss-horiz-scale", "Factor on the fix's horizontal accuracy (default 1)", [&](const std::string& v) {
        opt.gnssHorizScale = std::stod(v);
    });
    parser.addArgument("--gnss-vert-scale", "Factor on the fix's vertical accuracy (default 1)", [&](const std::string& v) {
        opt.gnssVertScale = std::stod(v);
    });
    parser.addArgument("--gnss-vel-scale", "Factor on the fix's speed accuracy (default 1)", [&](const std::string& v) {
        opt.gnssVelScale = std::stod(v);
    });
    parser.addArgument("--init-attitude-sigma-deg", "Initial attitude 1-sigma [deg]", [&](const std::string& v) {
        sigAtt_deg = std::stod(v);
    });
    parser.addArgument("--init-velocity-sigma", "Initial velocity 1-sigma [m/s]", [&](const std::string& v) {
        sigVel = std::stod(v);
    });
    parser.addArgument("--init-position-sigma", "Initial position 1-sigma [m]", [&](const std::string& v) {
        sigPos = std::stod(v);
    });
    parser.addArgument("--init-gyro-bias-sigma", "Initial gyro bias 1-sigma [rad/s]", [&](const std::string& v) {
        sigGyroBias = std::stod(v);
    });
    parser.addArgument("--init-accel-bias-sigma", "Initial accelerometer bias 1-sigma [m/s^2]", [&](const std::string& v) {
        sigAccelBias = std::stod(v);
    });
    parser.addArgument("--init-baro-bias-sigma", "Initial barometer bias 1-sigma [m]", [&](const std::string& v) {
        sigBaroBias = std::stod(v);
    });
    parser.addArgument("--imu-time-shift",
        "Added to every IMU part_time_s [s] (default 0); a diagnostic for a stamp offset in the log",
        [&](const std::string& v) {
            const double shift = std::stod(v);
            opt.timing.imu = [shift](const Replay::ImuRecord& r, const Replay::TimeBaseFit&, const Replay::RunConfiguration&) {
                return r.partTime + shift;
            };
        });
    parser.addArgument("--csv", "on (default) or off: off prints the summary only", [&](const std::string& v) {
        writeCsv = (v != "off");
    });
    parser.addArgument("--min-fix-type",
        "Skip fixes with a smaller fix_type; Hemerion's enum 0 none, 1 2D, 2 3D (default 2)",
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

        if (sigAtt_deg || sigVel || sigPos || sigGyroBias || sigAccelBias || sigBaroBias)
        {
            Eigen::VectorXd sig = Replay::detail::defaultInitialSigma(cfg);
            const int nav = cfg.offset(BlockId::NavCore);
            if (sigAtt_deg)
                sig.segment<3>(nav + NavCoreLayout::kAttitude).setConstant(*sigAtt_deg * std::numbers::pi / 180.0);
            if (sigVel)
                sig.segment<3>(nav + NavCoreLayout::kVelocity).setConstant(*sigVel);
            if (sigPos)
                sig.segment<3>(nav + NavCoreLayout::kPosition).setConstant(*sigPos);
            if (const int o = cfg.offset(BlockId::GyroBias); o >= 0 && sigGyroBias)
                sig.segment<3>(o).setConstant(*sigGyroBias);
            if (const int o = cfg.offset(BlockId::AccelBias); o >= 0 && sigAccelBias)
                sig.segment<3>(o).setConstant(*sigAccelBias);
            if (const int o = cfg.offset(BlockId::BaroBias); o >= 0 && sigBaroBias)
                sig[o] = *sigBaroBias;
            opt.initialSigma = sig;
        }

        const Replay::LogSet logs = Replay::loadLogs(logsDir, truthFile, cfg, sidecarFile);
        const Replay::ReplayResult result = Replay::replay(conv, logs, cfg, opt);

        Replay::writeSummary(std::cout, result.summary);
        if (!writeCsv)
            return 0;
        std::ofstream states(outputPrefix + "_states.csv");
        Replay::writeStates(states, result, cfg);
        std::ofstream updates(outputPrefix + "_updates.csv");
        Replay::writeUpdates(updates, result);
        std::cout << "wrote " << outputPrefix << "_states.csv and " << outputPrefix << "_updates.csv\n";
        return 0;
    } catch (const std::exception& e)
    {
        std::cerr << "EkfReplay: " << e.what() << '\n';
        return 1;
    }
}

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include "Aetherion/Simulation/Application.h"

#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/SnapshotTraits.h>

#include <stdexcept>
#include <cstdlib>
#include <fstream>

namespace Aetherion::Simulation {

    // ── writeCsvHeader default ────────────────────────────────────────────────
    void Application::writeCsvHeader(std::ofstream& csv) const
    {
        SnapshotTraits<SnapshotFormat::One>::write_header(csv);
    }

    // ── Constructor ───────────────────────────────────────────────────────────
    Application::Application(int argc, char* argv[])
        : parser_(argv[0])
    {
        registerArguments();
        try {
            parser_.parse(argc, argv);
        }
        catch (const std::invalid_argument&) {
            throw;
        }
    }

    // ── registerArguments ─────────────────────────────────────────────────────
    void Application::registerArguments()
    {
        parser_.addArgument(
            "--timeStep",
            "<double>   Time step value (default: 0.01)",
            [this](const std::string& val) {
                try {
                    double ts = std::stod(val);
                    if (ts <= 0.0)
                        throw std::invalid_argument("--timeStep must be positive.");
                    config_.timeStep = ts;
                }
                catch (const std::invalid_argument& e) {
                    throw std::invalid_argument(
                        std::string("Invalid --timeStep value: ") + e.what());
                }
            });

        parser_.addArgument(
            "--startTime",
            "<double>   Simulation start time (default: 0.0)",
            [this](const std::string& val) {
                try {
                    config_.startTime = std::stod(val);
                }
                catch (const std::exception&) {
                    throw std::invalid_argument(
                        "Invalid --startTime value: must be a number.");
                }
            });

        parser_.addArgument(
            "--endTime",
            "<double>   Simulation end time (default: 1.0)",
            [this](const std::string& val) {
                try {
                    double et = std::stod(val);
                    if (et <= config_.startTime)
                        throw std::invalid_argument(
                            "--endTime must be greater than --startTime.");
                    config_.endTime = et;
                }
                catch (const std::invalid_argument& e) {
                    throw std::invalid_argument(
                        std::string("Invalid --endTime value: ") + e.what());
                }
            });

        parser_.addArgument(
            "--inputFileName",
            "<string>   Input file name (required)",
            [this](const std::string& val) {
                if (val.empty())
                    throw std::invalid_argument(
                        "--inputFileName must not be empty.");
                config_.inputFileName = val;
            });

        parser_.addArgument(
            "--outputFileName",
            "<string>   Output file name (default: output.txt)",
            [this](const std::string& val) {
                if (val.empty())
                    throw std::invalid_argument(
                        "--outputFileName must not be empty.");
                config_.outputFileName = val;
            });

        parser_.addArgument(
            "--writeInterval",
            "<int>      Write a CSV row every N steps (default: 1 — write every step)",
            [this](const std::string& val) {
                try {
                    const int n = std::stoi(val);
                    if (n < 1)
                        throw std::invalid_argument("--writeInterval must be >= 1.");
                    config_.writeInterval = static_cast<std::size_t>(n);
                }
                catch (const std::invalid_argument& e) {
                    throw std::invalid_argument(
                        std::string("Invalid --writeInterval value: ") + e.what());
                }
            });
    }

    // ── logSimulationParameters ───────────────────────────────────────────────
    void Application::logSimulationParameters() const
    {
        AE_CORE_INFO("Simulation parameters:");
        AE_CORE_INFO("  startTime      = {:.6f} s", config_.startTime);
        AE_CORE_INFO("  endTime        = {:.6f} s", config_.endTime);
        AE_CORE_INFO("  timeStep       = {:.6f} s", config_.timeStep);
        AE_CORE_INFO("  writeInterval  = {} step(s)", config_.writeInterval);
        AE_CORE_INFO("  inputFileName  = {}", config_.inputFileName);
        AE_CORE_INFO("  outputFileName = {}", config_.outputFileName);
    }

    // ── openOutputCsv ─────────────────────────────────────────────────────────
    std::ofstream Application::openOutputCsv() const
    {
        AE_CORE_INFO("Opening output file '{}'", config_.outputFileName);
        std::ofstream csv(config_.outputFileName);
        if (!csv.is_open())
            throw std::runtime_error(
                "Cannot open output file: " + config_.outputFileName);
        return csv;
    }

    // ── run ───────────────────────────────────────────────────────────────────
    void Application::run() const
    {
        Simulation::Log::Init();

        logStartupBanner();
        logSimulationParameters();
        prepareSimulation();

        std::ofstream csv = openOutputCsv();
        writeCsvHeader(csv);

        writeInitialSnapshot(csv);
        runTimeStepLoop(csv);

        csv.close();
        logFinalSummary();
    }

    // ── runTimeStepLoop ───────────────────────────────────────────────────────
    void Application::runTimeStepLoop(std::ofstream& csv) const
    {
        const auto& cfg = getConfig();
        const double dt = cfg.timeStep;
        const double t_end = cfg.endTime;

        const std::size_t writeInterval = cfg.writeInterval;

        AE_CORE_INFO("Starting integration: t_end={:.3f} s, dt={:.4f} s, writeInterval={} step(s)",
            t_end, dt, writeInterval);

        std::size_t step_count = 0;
        std::size_t failed_steps = 0;
        double      t_current = cfg.startTime;

        while (t_current < t_end - 1e-12)
        {
            const double h = std::min(dt, t_end - t_current);
            ++step_count;
            const bool   doWrite = (step_count % writeInterval == 0);
            const auto   obs = stepAndRecord(csv, h, doWrite);

            t_current = obs.time_s;

            if (!obs.converged)
            {
                ++failed_steps;
                AE_CORE_WARN(
                    "Step {} did not converge at t={:.6f} s  (residual={:.3e}). Continuing.",
                    step_count, obs.time_s, obs.residual);
            }

            if (step_count % 10 == 0 || t_current >= t_end - 1e-12)
            {
                constexpr double kToDeg = 180.0 / 3.14159265358979323846;
                AE_CORE_INFO(
                    "  t={:.4f} s  alt={:.3f} m  |v_NED|={:.4f} m/s  "
                    "lat={:.6f} deg  lon={:.6f} deg",
                    obs.time_s, obs.altitude_m, obs.speed_mps,
                    obs.latitude_rad * kToDeg, obs.longitude_rad * kToDeg);
            }
        }

        AE_CORE_INFO("  Total steps : {}  (failed: {})", step_count, failed_steps);
    }

} // namespace Aetherion::Simulation
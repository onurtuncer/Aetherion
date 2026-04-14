// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Simulation/Config.h>
#include <Aetherion/Simulation/ArgumentParser.h>

#include <fstream>
#include <string>

namespace Aetherion::Simulation {

    // ─────────────────────────────────────────────────────────────
    // Application — base class for all Aetherion simulations.
    //
    // Implements the Template Method pattern:
    //   run() orchestrates the loop and calls virtual hooks.
    //   Subclasses override hooks to inject simulation-specific
    //   behaviour without re-implementing the loop skeleton.
    // ─────────────────────────────────────────────────────────────
    class Application {
    public:
        explicit Application(int argc, char* argv[]);
        virtual ~Application() = default;

        // Non-copyable
        Application(const Application&) = delete;
        Application& operator=(const Application&) = delete;

        // ── Template-method entry point ───────────────────────────
        // Final: subclasses extend via the virtual hooks below,
        // not by overriding run() directly.
        void run() const;

        const Config& getConfig() const { return config_; }

    protected:
        // ── Virtual hooks (override in subclass) ─────────────────

        // print an application-specific startup banner.
        virtual void logStartupBanner() const {}

        // Steps 1-4 — build the simulator from the loaded config.
        // Called once before the time-step loop. Subclasses must
        // initialise whatever internal simulator state they own.
        virtual void prepareSimulation() const = 0;

        // write the t = startTime snapshot to the CSV and
        // emit the initial INFO log line.
        virtual void writeInitialSnapshot(std::ofstream& csv) const = 0;

        // Called inside the loop after each accepted step.
        // Writes one CSV row and returns the values needed by the
        // generic INFO log (time, altitude, speed, lat, lon).
        struct StepObservation {
            double time_s = 0.0;
            double altitude_m = 0.0;
            double speed_mps = 0.0;
            double latitude_rad = 0.0;
            double longitude_rad = 0.0;
            bool   converged = true;
            double residual = 0.0;
        };
        // doWrite: true when this step falls on a writeInterval boundary and
        // the row should be written to csv; false when the step should only
        // advance the simulation without producing CSV output.
        virtual StepObservation stepAndRecord(std::ofstream& csv, double h, bool doWrite) const = 0;

        // emit the final summary INFO block.
        virtual void logFinalSummary() const = 0;

        // ── Non-virtual helpers (available to all subclasses) ─────

        // Logs the five Config fields at INFO level.
        void logSimulationParameters() const;

        // Opens cfg.outputFileName for writing; throws on failure.
        // Writes a simulation-generic header comment line.
        // Returns the open stream (caller writes the column header).
        [[nodiscard]] std::ofstream openOutputCsv() const;

    private:
        Config         config_;
        ArgumentParser parser_;

        void registerArguments();

        // ── Core loop (non-virtual) ───────────────────────────────
        // Called by run() after prepareSimulation() and the initial
        // snapshot. Drives the dt loop and calls stepAndRecord().
        void runTimeStepLoop(std::ofstream& csv) const;
    };

} // namespace Aetherion::Simulation
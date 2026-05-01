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
#include <Aetherion/Simulation/SnapshotTraits.h>

#include <fstream>
#include <string>

namespace Aetherion::Simulation {

/// @brief Abstract base class for all Aetherion simulations.
///
/// Implements the **Template Method** pattern: run() provides the fixed integration
/// loop skeleton (argument parsing, CSV setup, time-step loop, logging) and delegates
/// simulation-specific behaviour to a set of protected virtual hooks that concrete
/// subclasses must or may override.
///
/// Subclasses must implement:
/// - prepareSimulation() — build the simulator from the loaded Config.
/// - writeInitialSnapshot() — write the t = startTime CSV row.
/// - stepAndRecord() — advance one step and optionally write a CSV row.
/// - logFinalSummary() — emit the end-of-run log block.
///
/// Application is non-copyable; one instance per process is expected.
class Application {
public:
    /// @brief Constructs the Application, registers CLI flags, and parses argv.
    /// @param argc Argument count from main().
    /// @param argv Argument vector from main(); must remain valid for the lifetime of this object.
    /// @throws std::invalid_argument if unrecognised flags are present or required values are missing.
    explicit Application(int argc, char* argv[]);

    virtual ~Application() = default;

    Application(const Application&)            = delete;
    Application& operator=(const Application&) = delete;

    /// @brief Runs the full simulation lifecycle.
    ///
    /// Orchestrates: startup logging → prepareSimulation() → CSV open →
    /// writeInitialSnapshot() → time-step loop → logFinalSummary().
    /// @note Declared `final` in intent: subclasses extend via the virtual hooks, not by overriding run().
    void run() const;

    /// @brief Returns the simulation Config populated during construction.
    /// @return Const reference to the loaded Config.
    const Config& getConfig() const { return config_; }

protected:
    // ── Virtual hooks (override in subclass) ─────────────────────────────────

    /// @brief Prints an application-specific startup banner to the log.
    ///        Default implementation is a no-op.
    virtual void logStartupBanner() const {}

    /// @brief Writes the CSV column-header row.
    ///
    /// Default: ``SnapshotTraits<SnapshotFormat::One>::write_header()``
    /// (38-column Snapshot1).  Override to use Snapshot2 format:
    ///
    /// @code
    /// void MyApp::writeCsvHeader(std::ofstream& csv) const override {
    ///     SnapshotTraits<SnapshotFormat::Two>::write_header(csv);
    /// }
    /// @endcode
    ///
    /// The corresponding ``writeInitialSnapshot()`` and ``stepAndRecord()``
    /// overrides must call the matching
    /// ``SnapshotTraits<SnapshotFormat::Two>::write_row(csv, sim->snapshot2())``
    /// to maintain header/row consistency.
    virtual void writeCsvHeader(std::ofstream& csv) const;

    /// @brief Builds the simulator from the loaded Config (steps 1–4 of the run protocol).
    ///
    /// Called once by run() before the time-step loop. Subclasses must initialise all
    /// internal simulator state they own (e.g. allocate the integrator, set initial conditions).
    virtual void prepareSimulation() const = 0;

    /// @brief Writes the t = startTime snapshot to the CSV and emits the initial INFO log line.
    /// @param csv Open output file stream positioned after the column-header row.
    virtual void writeInitialSnapshot(std::ofstream& csv) const = 0;

    /// @brief Observable quantities returned by stepAndRecord() for the generic loop log line.
    struct StepObservation {
        double time_s       = 0.0;  ///< Simulation time at the end of the step [s].
        double altitude_m   = 0.0;  ///< Geometric altitude above MSL [m].
        double speed_mps    = 0.0;  ///< Earth-relative speed [m/s].
        double latitude_rad = 0.0;  ///< Geodetic latitude [rad].
        double longitude_rad= 0.0;  ///< Geodetic longitude [rad].
        bool   converged    = true; ///< Whether the implicit solver converged on this step.
        double residual     = 0.0;  ///< Final Newton residual norm (0 for explicit integrators).
    };

    /// @brief Advances the simulation by one step and optionally writes a CSV row.
    /// @param csv Open output file stream.
    /// @param h Integration step size [s].
    /// @param doWrite `true` when the step index is on a writeInterval boundary and a CSV row should be written; `false` to advance without output.
    /// @return StepObservation containing the observables needed by the generic INFO log line.
    virtual StepObservation stepAndRecord(std::ofstream& csv, double h, bool doWrite) const = 0;

    /// @brief Emits the final summary INFO block after the integration loop completes.
    virtual void logFinalSummary() const = 0;

    // ── Non-virtual helpers (available to all subclasses) ─────────────────────

    /// @brief Logs the five Config fields (timeStep, startTime, endTime, writeInterval, outputFileName) at INFO level.
    void logSimulationParameters() const;

    /// @brief Opens the output CSV file for writing and writes a simulation-generic comment header.
    ///
    /// The returned stream is positioned for the caller to write the column-name row next.
    /// @return Open std::ofstream ready for column-header and data rows.
    /// @throws std::runtime_error if the file cannot be opened.
    [[nodiscard]] std::ofstream openOutputCsv() const;

private:
    Config         config_;
    ArgumentParser parser_;

    void registerArguments();

    /// @brief Drives the dt loop, calling stepAndRecord() at each step.
    ///        Called by run() after prepareSimulation() and writeInitialSnapshot().
    void runTimeStepLoop(std::ofstream& csv) const;
};

} // namespace Aetherion::Simulation

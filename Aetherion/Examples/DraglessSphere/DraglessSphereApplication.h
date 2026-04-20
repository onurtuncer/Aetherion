// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// DraglessSphereApplication.h
//
// Simulation::Application subclass for the NASA TM-2015-218675 Atmospheric
// Scenario 1 (dragless sphere, J2 gravitation).
//
// Design
// ──────
// Template Method pattern:
//   - Application::run()    — fixed skeleton (logging, CSV, loop)
//   - Virtual hooks         — overridden here for NASA Atmos-01 specifics
//   - Private helpers       — one function per numbered run-loop step
//
//  Hook / helper mapping
//  ─────────────────────────────────────────────────────────────────────
//  logStartupBanner()         → prints the NASA TM banner              [step 0]
//  prepareSimulation()        → steps 1-4: load config, ERA, state,
//                               construct DraglessSphereSimulator
//  writeInitialSnapshot()     → step 6: write t=startTime CSV row      [step 6]
//  stepAndRecord()            → inner loop body: advance + CSV row     [step 7]
//  logFinalSummary()          → step 9: final INFO block               [step 9]
//
//  Private helpers (called only by the hooks above):
//    loadVehicleConfig()      → step 1: deserialise RigidBody::Config
//    computeInitialERA()      → step 2: θ₀ = ω_E * t₀
//    buildInitialState()      → step 3: flat vector → StateD
//    constructSimulator()     → step 4: new DraglessSphereSimulator
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Simulation/Application.h>
#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Serialization/LoadConfig.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/Examples/DraglessSphere/DraglessSphereSimulator.h>

#include <Aetherion/RigidBody/State.h>
#include <Aetherion/RigidBody/StateLayout.h>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

namespace Aetherion::Examples::DraglessSphere {

/// @brief Concrete Application subclass for the NASA TM-2015-218675 Atmospheric Scenario 1 validation case.
///
/// Implements the Template Method hooks defined by Simulation::Application to run a
/// dragless sphere under J2 gravity, reproducing the NASA Atmos-01 reference trajectory.
/// The simulator is constructed lazily inside prepareSimulation() and stored as a
/// `mutable` unique_ptr so the `const` hook contract of the base class is respected.
///
/// Reference: NASA TM-2015-218675, Section B.1.1 / Table 25.
class DraglessSphereApplication : public Simulation::Application
{
public:
    /// @brief Constructs the application and forwards command-line arguments to Application for parsing.
    ///
    /// The simulator is not created here; it is allocated lazily by prepareSimulation().
    /// @param argc Argument count from main().
    /// @param argv Argument vector from main().
    explicit DraglessSphereApplication(int argc, char* argv[])
        : Application(argc, argv)
    {
    }

protected:
    // ── Hook overrides ────────────────────────────────────────────────────────

    /// @brief Prints the NASA TM-2015-218675 validation-case startup banner via the core logger.
    void logStartupBanner() const override;

    /// @brief Runs steps 1–4: deserialise JSON config → compute initial ERA → build StateD → allocate simulator.
    ///
    /// After this call `m_Simulator` is non-null and ready to integrate.
    void prepareSimulation() const override;

    /// @brief Writes the t = startTime snapshot row to @p csv and emits the initial INFO log line.
    /// @param csv Open output CSV stream positioned after the column-header row.
    void writeInitialSnapshot(std::ofstream& csv) const override;

    /// @brief Advances the simulator by one step @p h and, if @p doWrite is true, writes a CSV row.
    /// @param csv Open output CSV stream.
    /// @param h Integration step size [s].
    /// @param doWrite `true` on write-interval boundaries; `false` for intermediate steps.
    /// @return StepObservation containing time, altitude, speed, latitude, longitude, and Newton convergence info.
    StepObservation stepAndRecord(std::ofstream& csv, double h, bool doWrite) const override;

    /// @brief Emits the final INFO summary block (end time, final altitude, convergence statistics).
    void logFinalSummary() const override;

private:
    // ── Per-step helpers (one function per numbered run-loop step) ────────────

    /// @brief Step 1 — deserialises the RigidBody::Config from the input JSON file.
    /// @return Fully populated RigidBody::Config.
    /// @throws std::runtime_error if the file cannot be opened or parsed.
    [[nodiscard]] RigidBody::Config loadVehicleConfig() const;

    /// @brief Step 2 — computes the initial Earth Rotation Angle (ERA) from simulation start time.
    ///
    /// Uses a linear approximation: @f$ \theta_0 = \omega_E \cdot t_0 @f$.
    /// @param t0 Simulation start time [s].
    /// @return Initial ERA @f$ \theta_0 @f$ [rad].
    [[nodiscard]] static double computeInitialERA(double t0) noexcept;

    /// @brief Step 3 — builds the flat SE(3) state vector from the RigidBody::Config and initial ERA.
    /// @param rb_cfg Deserialised rigid-body configuration.
    /// @param theta0 Initial Earth Rotation Angle [rad] from computeInitialERA().
    /// @return Initial StateD (double-precision flat state vector).
    [[nodiscard]] static RigidBody::StateD
        buildInitialState(const RigidBody::Config& rb_cfg, double theta0);

    /// @brief Step 4 — allocates and configures the DraglessSphereSimulator.
    /// @param ip Inertial parameters (mass and inertia tensor).
    /// @param x0 Initial state vector.
    /// @param theta0 Initial Earth Rotation Angle [rad].
    /// @return Heap-allocated DraglessSphereSimulator ready to integrate.
    [[nodiscard]] static std::unique_ptr<DraglessSphereSimulator>
        constructSimulator(const RigidBody::InertialParameters& ip,
            const RigidBody::StateD& x0,
            double                   theta0);

    // ── Mutable simulation state (lazy-initialised in prepareSimulation) ──────
    // `mutable` because run() and all hooks are const (run() is const in
    // the base class contract). This is the only shared mutable state.
    mutable std::unique_ptr<DraglessSphereSimulator> m_Simulator;
};

} // namespace Aetherion::Examples::DraglessSphere

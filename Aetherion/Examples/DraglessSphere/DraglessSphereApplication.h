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

#include <Aetherion/RigidBody/State.h>        // ← ADD: RigidBody::StateD
#include <Aetherion/RigidBody/StateLayout.h>  // ← ADD: StateLayout::IDX_*

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

namespace Aetherion::Examples::DraglessSphere {

    class DraglessSphereApplication : public Simulation::Application
    {
    public:
        // ── Construction ─────────────────────────────────────────────────────
        // Forwards argc/argv to Application (parses CLI flags).
        // The simulator is constructed lazily inside prepareSimulation().
        explicit DraglessSphereApplication(int argc, char* argv[])
            : Application(argc, argv)
        {
        }

    protected:
        // ── Hook overrides ────────────────────────────────────────────────────

        void logStartupBanner() const override;

        // Steps 1-4: load JSON → ERA → StateD → simulator
        void prepareSimulation() const override;

        // Step 6: write t = startTime snapshot row and log it
        void writeInitialSnapshot(std::ofstream& csv) const override;

        // Inner loop: advance one step, write CSV row, return observables
        StepObservation stepAndRecord(std::ofstream& csv, double h) const override;

        // Step 9: emit the final INFO block
        void logFinalSummary() const override;

    private:
        // ── Per-step helpers (one function per numbered run-loop step) ────────

        // Step 1 — deserialise RigidBody::Config from inputFileName.
        [[nodiscard]] RigidBody::Config loadVehicleConfig() const;

        // Step 2 — θ₀ = ω_E * t₀  (linear ERA from J2000-like epoch).
        [[nodiscard]] static double computeInitialERA(double t0) noexcept;

        // Step 3 — flatten RigidBody::Config + ERA into a StateD.
        [[nodiscard]] static RigidBody::StateD
            buildInitialState(const RigidBody::Config& rb_cfg, double theta0);

        // Step 4 — allocate the DraglessSphereSimulator.
        [[nodiscard]] static std::unique_ptr<DraglessSphereSimulator>
            constructSimulator(const RigidBody::InertialParameters& ip,
                const RigidBody::StateD& x0,
                double                               theta0);

        // ── Mutable simulation state (lazy-initialised in prepareSimulation) ──
        // `mutable` because run() and all hooks are const (run() is const in
        // the base class contract). This is the only shared mutable state.
        mutable std::unique_ptr<DraglessSphereSimulator> m_Simulator;
    };

} // namespace Aetherion::Examples::DraglessSphere
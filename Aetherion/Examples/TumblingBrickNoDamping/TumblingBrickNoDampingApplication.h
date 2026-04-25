// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TumblingBrickNoDampingApplication.h
//
// Simulation::Application subclass for the NASA TM-2015-218675 Atmospheric
// Scenario 2 (tumbling brick, no aerodynamic damping, J2 gravitation).
//
// Design
// ──────
// Template Method pattern — same skeleton as DraglessSphereApplication.
//
//  Hook / helper mapping
//  ─────────────────────────────────────────────────────────────────────
//  logStartupBanner()         → NASA TM Scenario 2 banner              [step 0]
//  prepareSimulation()        → load config, ERA, state, construct sim  [steps 1-4]
//  writeInitialSnapshot()     → write t=startTime CSV row              [step 6]
//  stepAndRecord()            → advance one step + CSV row             [step 7]
//  logFinalSummary()          → final INFO block                       [step 9]
//
//  Private helpers:
//    loadVehicleConfig()      → step 1: deserialise RigidBody::Config
//    computeInitialERA()      → step 2: θ₀ = ω_E * t₀
//    buildInitialState()      → step 3: flat vector → StateD
//    constructSimulator()     → step 4: new TumblingBrickNoDampingSimulator
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Simulation/Application.h>
#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Serialization/LoadConfig.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/Examples/TumblingBrickNoDamping/TumblingBrickNoDampingSimulator.h>

#include <Aetherion/RigidBody/State.h>
#include <Aetherion/RigidBody/StateLayout.h>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

namespace Aetherion::Examples::TumblingBrickNoDamping {

/// @brief Concrete Application for the NASA TM-2015-218675 Atmospheric Scenario 2 validation case.
///
/// Runs a non-symmetric rigid body (1:2:3 inertia ratio) under J2 gravity with
/// zero aerodynamic forces, reproducing the torque-free tumbling reference trajectory.
///
/// Reference: NASA TM-2015-218675, Section B.1.2 / Table 26.
class TumblingBrickNoDampingApplication : public Simulation::Application
{
public:
    explicit TumblingBrickNoDampingApplication(int argc, char* argv[])
        : Application(argc, argv)
    {
    }

protected:
    void logStartupBanner()                                              const override;
    void prepareSimulation()                                             const override;
    void writeInitialSnapshot(std::ofstream& csv)                        const override;
    StepObservation stepAndRecord(std::ofstream& csv, double h, bool doWrite) const override;
    void logFinalSummary()                                               const override;

private:
    [[nodiscard]] RigidBody::Config loadVehicleConfig() const;
    [[nodiscard]] static double     computeInitialERA(double t0) noexcept;
    [[nodiscard]] static RigidBody::StateD
        buildInitialState(const RigidBody::Config& rb_cfg, double theta0);
    [[nodiscard]] static std::unique_ptr<TumblingBrickNoDampingSimulator>
        constructSimulator(const RigidBody::InertialParameters& ip,
                           const RigidBody::StateD&             x0,
                           double                               theta0);

    mutable std::unique_ptr<TumblingBrickNoDampingSimulator> m_Simulator;
};

} // namespace Aetherion::Examples::TumblingBrickNoDamping

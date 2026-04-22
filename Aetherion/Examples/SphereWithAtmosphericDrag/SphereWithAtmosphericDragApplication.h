// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// SphereWithAtmosphericDragApplication.h
//
// Simulation::Application subclass for the NASA TM-2015-218675 Atmospheric
// Scenario 6 (sphere with aerodynamic drag, J2 gravitation, US 1976 atmosphere).
//
// Design
// ──────
// Template Method pattern — mirrors DraglessSphereApplication exactly:
//
//  Hook / helper mapping
//  ─────────────────────────────────────────────────────────────────────
//  logStartupBanner()         → prints the NASA TM Atmos-06 banner      [step 0]
//  prepareSimulation()        → steps 1-4: load config, ERA, state,
//                               construct SphereWithAtmosphericDragSimulator
//  writeInitialSnapshot()     → step 6: write t=startTime CSV row       [step 6]
//  stepAndRecord()            → inner loop: advance + CSV row            [step 7]
//  logFinalSummary()          → step 9: final INFO block                 [step 9]
//
//  Private helpers:
//    loadVehicleConfig()      → step 1: deserialise RigidBody::Config
//    computeInitialERA()      → step 2: θ₀ = ω_E * t₀
//    buildInitialState()      → step 3: geodetic → SE(3) StateD
//    constructSimulator()     → step 4: allocate SphereWithAtmosphericDragSimulator
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Simulation/Application.h>
#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Serialization/LoadConfig.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/Examples/SphereWithAtmosphericDrag/SphereWithAtmosphericDragSimulator.h>

#include <Aetherion/RigidBody/State.h>
#include <Aetherion/RigidBody/StateLayout.h>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

namespace Aetherion::Examples::SphereWithAtmosphericDrag {

/// @brief Concrete Application subclass for the NASA TM-2015-218675 Atmospheric Scenario 6.
///
/// Runs a sphere with aerodynamic drag (DragOnlyAeroPolicy, CD from JSON config)
/// under J2 gravity and the US 1976 atmosphere, reproducing the Atmos-06 reference
/// trajectory.
///
/// Reference: NASA TM-2015-218675, Section B.1.6 / Table 30.
class SphereWithAtmosphericDragApplication : public Simulation::Application
{
public:
    explicit SphereWithAtmosphericDragApplication(int argc, char* argv[])
        : Application(argc, argv)
    {
    }

protected:
    void logStartupBanner()                                     const override;
    void prepareSimulation()                                    const override;
    void writeInitialSnapshot(std::ofstream& csv)               const override;
    StepObservation stepAndRecord(std::ofstream& csv,
                                  double h, bool doWrite)       const override;
    void logFinalSummary()                                      const override;

private:
    [[nodiscard]] RigidBody::Config loadVehicleConfig() const;

    [[nodiscard]] static double computeInitialERA(double t0) noexcept;

    [[nodiscard]] static RigidBody::StateD
        buildInitialState(const RigidBody::Config& rb_cfg, double theta0);

    [[nodiscard]] static std::unique_ptr<SphereWithAtmosphericDragSimulator>
        constructSimulator(const RigidBody::InertialParameters&    ip,
                           const RigidBody::AerodynamicParameters& aero,
                           const RigidBody::StateD&                x0,
                           double                                  theta0);

    mutable std::unique_ptr<SphereWithAtmosphericDragSimulator> m_Simulator;
};

} // namespace Aetherion::Examples::SphereWithAtmosphericDrag
// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// CircleEquatorDateLineApplication.h
//
// Application driver for NASA TM-2015-218675 Atmospheric Scenario 16:
// F-16 CCW circumnavigation of the equator/international date line intersection
// under J2 gravity with GNC autopilot.
//
// Reuses F16AltitudeChangeSimulator unchanged — the difference from Scenario 15
// is circlePoleSW = 0.0 (equator/IDL navigator) and the initial conditions:
//   lat = 0.0°, lon = −179.95°, heading = 0° (north)
//
// Autopilot commands:
//   altCmd        = 10 000 ft  (hold initial altitude)
//   keasCmd       = computed from US1976 atmosphere at 10 000 ft trim altitude
//   baseChiCmd    = 0°         (north placeholder; circumnavigator overrides)
//   circlePoleSW  = 0.0        (engage equator/IDL circumnavigator)
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Simulation/Application.h>
#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot2.h>
#include <Aetherion/Simulation/SnapshotTraits.h>
#include <Aetherion/RigidBody/BuildInitialState.h>
#include <Aetherion/Examples/F16AltitudeChange/F16AltitudeChangeSimulator.h>

#include <fstream>
#include <memory>
#include <stdexcept>

namespace Aetherion::Examples::CircleEquatorDateLine {

class CircleEquatorDateLineApplication : public Simulation::Application
{
public:
    explicit CircleEquatorDateLineApplication(int argc, char* argv[])
        : Application(argc, argv) {}

protected:
    void logStartupBanner()                                                const override;
    void writeCsvHeader(std::ofstream& csv)                                const override;
    void prepareSimulation()                                               const override;
    void writeInitialSnapshot(std::ofstream& csv)                          const override;
    StepObservation stepAndRecord(std::ofstream& csv, double h, bool doWrite) const override;
    void logFinalSummary()                                                 const override;

private:
    [[nodiscard]] static double computeInitialERA(double t0) noexcept;

    [[nodiscard]] static RigidBody::StateD
        buildInitialState(const RigidBody::Config& cfg, double theta0);

    [[nodiscard]] static std::unique_ptr<F16AltitudeChange::F16AltitudeChangeSimulator>
        constructSimulator(
            const RigidBody::InertialParameters&                      ip,
            const FlightDynamics::TrimPoint&                          trim,
            const std::shared_ptr<const Serialization::DAVEMLAeroModel>&     aero,
            const std::shared_ptr<const Serialization::DAVEMLPropModel>&     prop,
            const std::shared_ptr<const Serialization::DAVEMLControlModel>&  ctrl,
            const RigidBody::StateD& x0,
            double theta0,
            const F16AltitudeChange::F16AltitudeChangeSimulator::AutopilotCmds& cmds,
            double xcg_from_ac_m = 0.0);

    mutable std::unique_ptr<F16AltitudeChange::F16AltitudeChangeSimulator> m_Simulator;
};

} // namespace Aetherion::Examples::CircleEquatorDateLine

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16AltitudeChangeApplication.h
//
// Application driver for NASA TM-2015-218675 Atmospheric Scenario 13.1:
// F-16 subsonic altitude change (+100 ft step), closed-loop with LQR SAS
// and altitude / airspeed / heading autopilot (F16_control.dml).
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

namespace Aetherion::Examples::F16AltitudeChange {

class F16AltitudeChangeApplication : public Simulation::Application
{
public:
    explicit F16AltitudeChangeApplication(int argc, char* argv[])
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

    [[nodiscard]] static std::unique_ptr<F16AltitudeChangeSimulator>
        constructSimulator(
            const RigidBody::InertialParameters&                      ip,
            const FlightDynamics::TrimPoint&                          trim,
            std::shared_ptr<const Serialization::DAVEMLAeroModel>     aero,
            std::shared_ptr<const Serialization::DAVEMLPropModel>     prop,
            std::shared_ptr<const Serialization::DAVEMLControlModel>  ctrl,
            const RigidBody::StateD& x0,
            double theta0,
            const F16AltitudeChangeSimulator::AutopilotCmds& cmds,
            double xcg_from_ac_m = 0.0);

    mutable std::unique_ptr<F16AltitudeChangeSimulator> m_Simulator;
};

} // namespace Aetherion::Examples::F16AltitudeChange

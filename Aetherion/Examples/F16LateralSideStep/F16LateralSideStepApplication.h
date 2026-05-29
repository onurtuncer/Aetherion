// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16LateralSideStepApplication.h
//
// Application driver for NASA TM-2015-218675 Atmospheric Scenario 13.4:
// F-16 subsonic lateral side step (2 000 ft right-of-course offset at t = 20 s).
//
// Reuses F16AltitudeChangeSimulator with dynamic latOffset computation:
//   altCmd     = 10 013 ft  (hold altitude)
//   keasCmd    = computed from US1976 atmosphere  (hold trim airspeed)
//   baseChiCmd = 45°        (hold NE course)
//   latStepOffset_ft = 2 000 ft right of course, applied at t = 20 s
//
// The simulator computes latOffset at each step as:
//   latOffset = lateral_deviation_from_original_courseline − 2 000 ft
// so that latOffset → 0 as the aircraft reaches the new parallel course,
// naturally restoring baseChiCmd = 45° when on track.
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

namespace Aetherion::Examples::F16LateralSideStep {

class F16LateralSideStepApplication : public Simulation::Application
{
public:
    explicit F16LateralSideStepApplication(int argc, char* argv[])
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
            std::shared_ptr<const Serialization::DAVEMLAeroModel>     aero,
            std::shared_ptr<const Serialization::DAVEMLPropModel>     prop,
            std::shared_ptr<const Serialization::DAVEMLControlModel>  ctrl,
            const RigidBody::StateD& x0,
            double theta0,
            const F16AltitudeChange::F16AltitudeChangeSimulator::AutopilotCmds& cmds,
            double xcg_from_ac_m = 0.0);

    mutable std::unique_ptr<F16AltitudeChange::F16AltitudeChangeSimulator> m_Simulator;
};

} // namespace Aetherion::Examples::F16LateralSideStep

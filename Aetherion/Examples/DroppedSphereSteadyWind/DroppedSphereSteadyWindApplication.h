// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Simulation/Application.h>
#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Serialization/LoadConfig.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/Examples/DroppedSphereSteadyWind/DroppedSphereSteadyWindSimulator.h>

#include <Aetherion/RigidBody/State.h>
#include <Aetherion/RigidBody/StateLayout.h>

#include <fstream>
#include <memory>
#include <stdexcept>

namespace Aetherion::Examples::DroppedSphereSteadyWind {

/// @brief Application for NASA TM-2015-218675 Atmospheric Scenario 7.
///
/// Sphere (CD=0.1) dropped from 9144 m in a constant 20 ft/s eastward wind
/// under J₂ gravity.  The wind creates horizontal drift and an initial
/// non-zero TAS even before the sphere starts falling.
class DroppedSphereSteadyWindApplication : public Simulation::Application
{
public:
    explicit DroppedSphereSteadyWindApplication(int argc, char* argv[])
        : Application(argc, argv) {}

protected:
    void logStartupBanner()                                               const override;
    void prepareSimulation()                                              const override;
    void writeInitialSnapshot(std::ofstream& csv)                         const override;
    StepObservation stepAndRecord(std::ofstream& csv, double h, bool doWrite) const override;
    void logFinalSummary()                                                const override;

private:
    [[nodiscard]] RigidBody::Config loadVehicleConfig() const;
    [[nodiscard]] static double     computeInitialERA(double t0) noexcept;
    [[nodiscard]] static RigidBody::StateD
        buildInitialState(const RigidBody::Config& rb_cfg, double theta0);
    [[nodiscard]] static std::unique_ptr<DroppedSphereSteadyWindSimulator>
        constructSimulator(const RigidBody::Config& rb_cfg,
                           const RigidBody::StateD& x0,
                           double                   theta0);

    mutable std::unique_ptr<DroppedSphereSteadyWindSimulator> m_Simulator;
};

} // namespace Aetherion::Examples::DroppedSphereSteadyWind

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TumblingBrickWithDampingApplication.h
//
// Simulation::Application subclass for the NASA TM-2015-218675 Atmospheric
// Scenario 3 (tumbling brick, with aerodynamic drag and rotary damping).
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Simulation/Application.h>
#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Serialization/LoadConfig.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/Examples/TumblingBrickWithDamping/TumblingBrickWithDampingSimulator.h>

#include <Aetherion/RigidBody/State.h>
#include <Aetherion/RigidBody/StateLayout.h>

#include <fstream>
#include <memory>
#include <stdexcept>

namespace Aetherion::Examples::TumblingBrickWithDamping {

/// @brief Concrete Application for NASA TM-2015-218675 Atmospheric Scenario 3.
///
/// Tumbling US standard face brick (8×4×2.25 in) under J₂ gravity with
/// aerodynamic drag (CD=0.01) and rotary damping (Clp=Cmq=Cnr=−1).
///
/// Reference: NASA TM-2015-218675, Section B.1.3 / Tables 4 & 5.
class TumblingBrickWithDampingApplication : public Simulation::Application
{
public:
    explicit TumblingBrickWithDampingApplication(int argc, char* argv[])
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
    [[nodiscard]] static std::unique_ptr<TumblingBrickWithDampingSimulator>
        constructSimulator(const RigidBody::InertialParameters&    ip,
                           const RigidBody::AerodynamicParameters& aero,
                           const RigidBody::StateD&                x0,
                           double                                  theta0);

    mutable std::unique_ptr<TumblingBrickWithDampingSimulator> m_Simulator;
};

} // namespace Aetherion::Examples::TumblingBrickWithDamping

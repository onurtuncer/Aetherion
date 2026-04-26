// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// NorthwardCannonballApplication.h
//
// Simulation::Application subclass for the NASA TM-2015-218675 Atmospheric
// Scenario 10 (dragless sphere fired northward from sea level at 45°, J2 gravity).
//
// Physics: identical to Scenario 1 (DraglessSphere) — J2 gravity, ZeroAeroPolicy.
// Difference: initial altitude = 0 m, v_NED = [304.8, 0, −304.8] m/s.
// Reuses DraglessSphereSimulator directly.
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

namespace Aetherion::Examples::NorthwardCannonball {

/// @brief Application for NASA TM-2015-218675 Atmospheric Scenario 10.
///
/// Dragless sphere fired northward from sea level at 45° under J₂ gravity.
/// Reuses DraglessSphereSimulator (same VF: J2 + ZeroAeroPolicy).
class NorthwardCannonballApplication : public Simulation::Application
{
public:
    explicit NorthwardCannonballApplication(int argc, char* argv[])
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
    [[nodiscard]] static std::unique_ptr<DraglessSphere::DraglessSphereSimulator>
        constructSimulator(const RigidBody::InertialParameters& ip,
                           const RigidBody::StateD&             x0,
                           double                               theta0);

    mutable std::unique_ptr<DraglessSphere::DraglessSphereSimulator> m_Simulator;
};

} // namespace Aetherion::Examples::NorthwardCannonball

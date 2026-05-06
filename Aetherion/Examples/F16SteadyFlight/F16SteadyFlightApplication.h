// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16SteadyFlightApplication.h
//
// Application driver for NASA TM-2015-218675 Check-Case 11:
// steady straight-and-level flight of a subsonic F-16.
//
// DML file paths are injected at CMake configure time via compile definitions:
//   F16_INERTIA_FILE  — path to F16_inertia.dml
//   F16_AERO_FILE     — path to F16_aero.dml
//   F16_PROP_FILE     — path to F16_prop.dml
//
// Initial conditions (NASA Atmos_11 reference):
//   Location  : 36.01917° N, 75.67444° W  (Kitty Hawk, NC)
//   Altitude  : 10 013 ft (3 051.96 m)
//   Heading   : 45° (NE)
//   TAS       : 335.15 KTAS = 565.685 ft/s
//   v_NED     : [400, 400, 0] ft/s = [121.92, 121.92, 0] m/s
//   Body rates: all zero  (trim condition)
//
// The trim solver runs at construction and finds (α, δe, throttle) for the
// loaded mass and flight condition.  The resulting attitude and control
// settings initialise the six-DoF integrator.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Simulation/Application.h>
#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot2.h>
#include <Aetherion/Simulation/SnapshotTraits.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/Examples/F16SteadyFlight/F16SteadyFlightSimulator.h>

#include <fstream>
#include <memory>
#include <stdexcept>

namespace Aetherion::Examples::F16SteadyFlight {

/// @brief Application for NASA TM-2015-218675 Check-Case 11.
///
/// F-16 at 335.15 KTAS, 10 013 ft, heading 45°, Kitty Hawk NC.
/// Inertia, aerodynamics, and propulsion are all loaded from DAVE-ML files.
class F16SteadyFlightApplication : public Simulation::Application
{
public:
    explicit F16SteadyFlightApplication(int argc, char* argv[])
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

    [[nodiscard]] static std::unique_ptr<F16SteadyFlightSimulator>
        constructSimulator(const RigidBody::InertialParameters& ip,
                           const FlightDynamics::TrimPoint&     trim,
                           std::shared_ptr<const Serialization::DAVEMLAeroModel> aero,
                           std::shared_ptr<const Serialization::DAVEMLPropModel> prop,
                           const RigidBody::StateD& x0,
                           double theta0,
                           double z_engine_m = 0.0);

    mutable std::unique_ptr<F16SteadyFlightSimulator> m_Simulator;
};

} // namespace Aetherion::Examples::F16SteadyFlight

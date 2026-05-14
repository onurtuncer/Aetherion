// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16SupersonicTrimApplication.h
//
// Application driver for NASA TM-2015-218675 Atmospheric Check-Case 12:
// F-16 supersonic steady straight-and-level flight (Mach ≈ 2.01, 30 013 ft).
//
// DML file paths are injected at CMake configure time via compile definitions:
//   F16_INERTIA_FILE  — path to F16_inertia.dml
//   F16_AERO_FILE     — path to F16_aero.dml
//   F16_PROP_FILE     — path to F16_prop.dml
//
// Initial conditions (NASA Atmos_12 reference):
//   Location  : 36.01917° N, 75.67444° W  (Kitty Hawk, NC)
//   Altitude  : 30 013 ft (9 147.56 m)
//   Heading   : 45° (NE)
//   TAS       : ≈ 2 000 ft/s  (≈ 1 185 KTAS,  Mach ≈ 2.01)
//   v_NED     : [1414.2, 1414.2, 0] ft/s = [430.9, 430.9, 0] m/s
//   Body rates: all zero  (trim condition)
//
// Reuses F16SteadyFlightSimulator unchanged; the only differences from
// Check-Case 11 are the flight-condition constants (altitude, velocity).
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Simulation/Application.h>
#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot2.h>
#include <Aetherion/Simulation/SnapshotTraits.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/Examples/F16SteadyFlight/F16SteadyFlightSimulator.h>

#include <fstream>
#include <memory>
#include <stdexcept>

namespace Aetherion::Examples::F16SupersonicTrim {

class F16SupersonicTrimApplication : public Simulation::Application
{
public:
    explicit F16SupersonicTrimApplication(int argc, char* argv[])
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

    [[nodiscard]] static std::unique_ptr<F16SteadyFlight::F16SteadyFlightSimulator>
        constructSimulator(
            const RigidBody::InertialParameters&                  ip,
            const FlightDynamics::TrimPoint&                      trim,
            std::shared_ptr<const Serialization::DAVEMLAeroModel> aero,
            std::shared_ptr<const Serialization::DAVEMLPropModel> prop,
            const RigidBody::StateD& x0,
            double theta0,
            double xcg_from_ac_m = 0.0);

    mutable std::unique_ptr<F16SteadyFlight::F16SteadyFlightSimulator> m_Simulator;
};

} // namespace Aetherion::Examples::F16SupersonicTrim

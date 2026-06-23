// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TwoStageRocketApplication.h
//
// Application driver for NASA TM-2015-218675 Atmospheric Scenario 17:
// Two-stage rocket to orbit — gravity-turn ascent from the equator.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Simulation/Application.h>
#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot2.h>
#include <Aetherion/Simulation/SnapshotTraits.h>

#include <Aetherion/Examples/TwoStageRocket/TwoStageRocketSimulator.h>

#include <fstream>
#include <memory>
#include <stdexcept>

namespace Aetherion::Examples::TwoStageRocket {

class TwoStageRocketApplication : public Simulation::Application
{
public:
    explicit TwoStageRocketApplication(int argc, char** argv)
        : Application(argc, argv) {}

protected:
    void logStartupBanner()                                                const override;
    void writeCsvHeader(std::ofstream& csv)                                const override;
    void prepareSimulation()                                               const override;
    void writeInitialSnapshot(std::ofstream& csv)                          const override;
    StepObservation stepAndRecord(std::ofstream& csv, double h, bool doWrite) const override;
    void logFinalSummary()                                                 const override;

private:
    mutable std::unique_ptr<TwoStageRocketSimulator<>> m_Simulator;
};

} // namespace Aetherion::Examples::TwoStageRocket

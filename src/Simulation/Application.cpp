// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
#include <Aetherion/Simulation/Application.h>

#include <iostream>
#include <stdexcept>
#include <cstdlib>

namespace Aetherion::Simulation {

    Application::Application(int argc, char* argv[])
        : parser_(argv[0])
    {
        registerArguments();

        try {
            parser_.parse(argc, argv);
        }
        catch (const std::invalid_argument& e) {
            std::cerr << "Error: " << e.what() << "\n\n";
            parser_.printUsage();
            throw;   // let the caller (EntryPoint / test) decide what to do
        }
    }

    void Application::run() const {
        config_.print();
        // Your program logic here ...
    }

    void Application::registerArguments() {
        parser_.addArgument(
            "--timeStep",
            "<double>   Time step value (default: 0.01)",
            [this](const std::string& val) {
                try {
                    double ts = std::stod(val);
                    if (ts <= 0.0)
                        throw std::invalid_argument("--timeStep must be positive.");
                    config_.timeStep = ts;
                }
                catch (const std::invalid_argument& e) {
                    throw std::invalid_argument(
                        std::string("Invalid --timeStep value: ") + e.what());
                }
            }
        );

        parser_.addArgument(
            "--startTime",
            "<double>   Simulation start time (default: 0.0)",
            [this](const std::string& val) {
                try {
                    config_.startTime = std::stod(val);
                }
                catch (const std::exception&) {
                    throw std::invalid_argument("Invalid --startTime value: must be a number.");
                }
            }
        );

        parser_.addArgument(
            "--endTime",
            "<double>   Simulation end time (default: 1.0)",
            [this](const std::string& val) {
                try {
                    double et = std::stod(val);
                    if (et <= config_.startTime)
                        throw std::invalid_argument("--endTime must be greater than --startTime.");
                    config_.endTime = et;
                }
                catch (const std::invalid_argument& e) {
                    throw std::invalid_argument(
                        std::string("Invalid --endTime value: ") + e.what());
                }
            }
        );

        parser_.addArgument(
            "--inputFileName",
            "<string>   Input file name (required)",
            [this](const std::string& val) {
                if (val.empty())
                    throw std::invalid_argument("--inputFileName must not be empty.");
                config_.inputFileName = val;
            }
        );

        parser_.addArgument(
            "--outputFileName",
            "<string>   Output file name (default: output.txt)",
            [this](const std::string& val) {
                if (val.empty())
                    throw std::invalid_argument("--outputFileName must not be empty.");
                config_.outputFileName = val;
            }
        );
    }

} // namespace Aetherion::Simulation
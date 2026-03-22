// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include "ArgumentParser.h"
#include <iostream>
#include <stdexcept>
#include <cstdlib>

namespace Aetherion::Simulation {

ArgumentParser::ArgumentParser(std::string programName)
    : programName_(std::move(programName)) {}

void ArgumentParser::addArgument(const std::string& flag,
                                 const std::string& description,
                                 Handler            handler) {
    handlers_[flag]     = std::move(handler);
    descriptions_[flag] = description;
    order_.push_back(flag);   // keep insertion order for help text
}

void ArgumentParser::parse(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsage();
            std::exit(EXIT_SUCCESS);
        }

        auto it = handlers_.find(arg);
        if (it == handlers_.end())
            throw std::invalid_argument("Unknown argument: " + arg);

        if (i + 1 >= argc)
            throw std::invalid_argument(arg + " requires a value.");

        it->second(argv[++i]);   // invoke the registered handler
    }
}

void ArgumentParser::printUsage() const {
    std::cerr << "Usage: " << programName_ << " [options]\n\nOptions:\n";
    for (const auto& flag : order_)
        std::cerr << "  " << flag << "\t" << descriptions_.at(flag) << "\n";
    std::cerr << "  --help\t\tShow this help message\n";
}

} // namespace Aetherion::Simulation

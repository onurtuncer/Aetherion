// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace Aetherion::Simulation {

// ─────────────────────────────────────────────────────────────
// ArgumentParser  –  registers flags and parses argv
// ─────────────────────────────────────────────────────────────
class ArgumentParser {
public:
    // A handler receives the next token (the flag's value)
    using Handler = std::function<void(const std::string&)>;

    explicit ArgumentParser(std::string programName);

    // Register a flag together with its description and value handler
    void addArgument(const std::string& flag,
                     const std::string& description,
                     Handler            handler);

    // Parse argc/argv; throws std::invalid_argument on error
    void parse(int argc, char* argv[]);

    void printUsage() const;

private:
    std::string programName_;
    std::unordered_map<std::string, Handler>     handlers_;
    std::unordered_map<std::string, std::string> descriptions_;
    std::vector<std::string>                     order_;
};

} // namespace Aetherion::Simulation

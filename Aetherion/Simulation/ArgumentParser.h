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

/// @brief Lightweight command-line argument parser that maps named flags to handler callbacks.
///
/// Flags are registered with addArgument() together with a description and a callable that
/// receives the flag's value token. Calling parse() walks argv, invokes handlers, and throws
/// on unrecognised flags or missing values. printUsage() emits a formatted help message.
///
/// Example usage:
/// @code
/// ArgumentParser parser("my_sim");
/// std::string configPath;
/// parser.addArgument("--config", "Path to JSON config file",
///     [&](const std::string& v){ configPath = v; });
/// parser.parse(argc, argv);
/// @endcode
class ArgumentParser {
public:
    /// @brief Callable invoked with the string token that immediately follows a registered flag.
    using Handler = std::function<void(const std::string&)>;

    /// @brief Constructs the parser with the executable name shown in usage output.
    /// @param programName Name of the program (typically argv[0]).
    explicit ArgumentParser(std::string programName);

    /// @brief Registers a flag, its help text, and the callback to invoke when the flag is encountered.
    /// @param flag Flag string including any leading dashes, e.g. "--config".
    /// @param description Human-readable description shown by printUsage().
    /// @param handler Callback receiving the token that follows @p flag in argv.
    void addArgument(const std::string& flag,
                     const std::string& description,
                     Handler            handler);

    /// @brief Parses the command-line arguments and invokes the registered handlers.
    /// @param argc Argument count as received by main().
    /// @param argv Argument vector as received by main().
    /// @throws std::invalid_argument if an unrecognised flag is encountered or a value token is missing.
    void parse(int argc, char* argv[]);

    /// @brief Prints a formatted usage message listing all registered flags and their descriptions to stdout.
    void printUsage() const;

private:
    std::string programName_;
    std::unordered_map<std::string, Handler>     handlers_;
    std::unordered_map<std::string, std::string> descriptions_;
    std::vector<std::string>                     order_;
};

} // namespace Aetherion::Simulation

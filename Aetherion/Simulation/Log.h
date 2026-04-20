// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#include <memory>

namespace Aetherion::Simulation {

/// @brief Thin wrapper around spdlog that provides two named loggers: core (engine-internal)
///        and client (user application).
///
/// Call Log::Init() once at program startup before using any logging macros.
/// After initialisation, use the @c AE_CORE_* macros for engine-side messages and
/// the @c AE_* macros for application-side messages.
class Log
{
public:
    /// @brief Initialises the core and client spdlog loggers with coloured console output.
    ///        Must be called once before any logging macro is used.
    static void Init();

    /// @brief Returns a reference to the shared core (engine-internal) logger.
    /// @return Shared pointer to the spdlog core logger.
    static std::shared_ptr<spdlog::logger>& GetCoreLogger() { return s_CoreLogger; }

    /// @brief Returns a reference to the shared client (user application) logger.
    /// @return Shared pointer to the spdlog client logger.
    static std::shared_ptr<spdlog::logger>& GetClientLogger() { return s_ClientLogger; }

private:
    static std::shared_ptr<spdlog::logger> s_CoreLogger;   ///< Engine-internal logger instance.
    static std::shared_ptr<spdlog::logger> s_ClientLogger; ///< User-application logger instance.
};

} // namespace Aetherion::Simulation

/// @defgroup CoreLogMacros Core logging macros (engine-internal)
/// @{
#define AE_CORE_TRACE(...)    ::Aetherion::Simulation::Log::GetCoreLogger()->trace(__VA_ARGS__)    ///< Log a TRACE-level message on the core logger.
#define AE_CORE_INFO(...)     ::Aetherion::Simulation::Log::GetCoreLogger()->info(__VA_ARGS__)     ///< Log an INFO-level message on the core logger.
#define AE_CORE_WARN(...)     ::Aetherion::Simulation::Log::GetCoreLogger()->warn(__VA_ARGS__)     ///< Log a WARN-level message on the core logger.
#define AE_CORE_ERROR(...)    ::Aetherion::Simulation::Log::GetCoreLogger()->error(__VA_ARGS__)    ///< Log an ERROR-level message on the core logger.
#define AE_CORE_CRITICAL(...) ::Aetherion::Simulation::Log::GetCoreLogger()->critical(__VA_ARGS__) ///< Log a CRITICAL-level message on the core logger.
/// @}

/// @defgroup ClientLogMacros Client logging macros (user application)
/// @{
#define AE_TRACE(...)         ::Aetherion::Simulation::Log::GetClientLogger()->trace(__VA_ARGS__)    ///< Log a TRACE-level message on the client logger.
#define AE_INFO(...)          ::Aetherion::Simulation::Log::GetClientLogger()->info(__VA_ARGS__)     ///< Log an INFO-level message on the client logger.
#define AE_WARN(...)          ::Aetherion::Simulation::Log::GetClientLogger()->warn(__VA_ARGS__)     ///< Log a WARN-level message on the client logger.
#define AE_ERROR(...)         ::Aetherion::Simulation::Log::GetClientLogger()->error(__VA_ARGS__)    ///< Log an ERROR-level message on the client logger.
#define AE_CRITICAL(...)      ::Aetherion::Simulation::Log::GetClientLogger()->critical(__VA_ARGS__) ///< Log a CRITICAL-level message on the client logger.
/// @}

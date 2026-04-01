// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#include <memory>

namespace Aetherion::Simulation {

	class Log
	{
	public:
		static void Init();

		static std::shared_ptr<spdlog::logger>& GetCoreLogger() { return s_CoreLogger; }
		static std::shared_ptr<spdlog::logger>& GetClientLogger() { return s_ClientLogger; }
	private:
		static std::shared_ptr<spdlog::logger> s_CoreLogger;
		static std::shared_ptr<spdlog::logger> s_ClientLogger;
	};

} // namespace Aetherion::Simulation

// Core log macros
#define AE_CORE_TRACE(...)    ::Aetherion::Simulation::Log::GetCoreLogger()->trace(__VA_ARGS__)
#define AE_CORE_INFO(...)     ::Aetherion::Simulation::Log::GetCoreLogger()->info(__VA_ARGS__)
#define AE_CORE_WARN(...)     ::Aetherion::Simulation::Log::GetCoreLogger()->warn(__VA_ARGS__)
#define AE_CORE_ERROR(...)    ::Aetherion::Simulation::Log::GetCoreLogger()->error(__VA_ARGS__)
#define AE_CORE_CRITICAL(...) ::Aetherion::Simulation::Log::GetCoreLogger()->critical(__VA_ARGS__)

// Client log macros
#define AE_TRACE(...)         ::Aetherion::Simulation::Log::GetClientLogger()->trace(__VA_ARGS__)
#define AE_INFO(...)          ::Aetherion::Simulation::Log::GetClientLogger()->info(__VA_ARGS__)
#define AE_WARN(...)          ::Aetherion::Simulation::Log::GetClientLogger()->warn(__VA_ARGS__)
#define AE_ERROR(...)         ::Aetherion::Simulation::Log::GetClientLogger()->error(__VA_ARGS__)
#define AE_CRITICAL(...)      ::Aetherion::Simulation::Log::GetClientLogger()->critical(__VA_ARGS__)

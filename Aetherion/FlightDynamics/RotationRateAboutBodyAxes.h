// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::FlightDynamics {

	struct InitialRotationAboutBodyAxes
	{
		double roll_rad_s{ 0.0 };
		double pitch_rad_s{ 0.0 };
		double yaw_rad_s{ 0.0 };
	};
} // namespace Aetherion::FlightDynamics

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::RigidBody {

	struct VelocityNED
	{
		double north_mps{ 0.0 };
		double east_mps{ 0.0 };
		double down_mps{ 0.0 };
	};
} // namespace Aetherion::RigidBody
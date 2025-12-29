// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::FlightDynamics {

	struct AeroParams {
		double Cd{ 0.0 };
		double Aref{ 0.0 };
		// later: CLalpha, CMalpha, etc.
	};

} // namespace Aetherion::FlightDynamics


// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::FlightDynamics {

	struct AerodynamicParameters {
		double S{ 0.0 }; // Reference area (m^2)
		double CL{ 0.0 }; // Lift coefficient
		double CD{ 0.0 }; // Drag coefficient
		double CY{ 0.0 }; // Side force coefficient
		double Cl{ 0.0 }; // Roll moment coefficient
		double Cm{ 0.0 }; // Pitch moment coefficient
		double Cn{ 0.0 }; // Yaw moment coefficient
	};

} // namespace Aetherion::FlightDynamics


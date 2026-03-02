// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once 

#include "StateLayout.h"

#include "Aetherion/FlightDynamics/AerodynamicParameters.h"
#include "Aetherion/FlightDynamics/InertialParameters.h"

#include "Aetherion/Spatial/Inertia.h"
#include "Aetherion/Spatial/Wrench.h"

namespace Aetherion::FlightDynamics {

	struct ODEContextBase {

		Spatial::Inertia M;
		
		double mdot; // [kg/s] Mass flow rate (due to fuel burn) //TODO this needs to be a function pointer

		ODEContextBase(InertialParameters ip) {

		}

		Spatial::Wrench CalculateAerodynamicWrench() = 0;
		Spatial::Wrench CalculateGravitationalWrench() = 0;
		Spatial::Wrench CalculatePropulsiveWrench() = 0;
	};

} // namespace Aetherion::FlignhtDynamics


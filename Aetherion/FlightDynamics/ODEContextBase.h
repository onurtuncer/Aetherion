// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once 

#include "Aetherion/FlightDynamics/AerodynamicParameters.h"
#include "Aetherion/FlightDynamics/InertialParameters.h"

#include "Aetherion/Spatial/Inertia.h"

namespace Aetherion::FlightDynamics {

	struct ODEContextBase {

		Spatial::Inertia M;
		double m; // [kg] Mass
		Eigen::Matrix3d I_com; // [kg·m^2] Inertia w.r.t center-of-mass (CoM)
		Eigen::Vector3d c; // [m] Kütle merkezi ofseti
		Eigen::Vector3d tau; // [N·m] Body torque (control)  //TODO this needs to be function pointer!
		Eigen::Vector3d force; // [N] Body force         //TODO this needs to be a function pointer!
		double mdot; // [kg/s] Mass flow rate (due to fuel burn) //TODO this needs to be a function pointer
		// ... other parameters ...

		ODEContextBase(Inertial) {

		}
	};

}


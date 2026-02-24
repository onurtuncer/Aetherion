

#pragma once 

namespace Aetherion::FlightDynamics {

	struct ODEContext {
		double m; // [kg] Mass
		Eigen::Matrix3d I_com; // [kg·m^2] Inertia w.r.t center-of-mass (CoM)
		Eigen::Vector3d c; // [m] Kütle merkezi ofseti
		Eigen::Vector3d tau; // [N·m] Body torque (control)  //TODO this needs to be function pointer!
		Eigen::Vector3d force; // [N] Body force         //TODO this needs to be a function pointer!
		double mdot; // [kg/s] Mass flow rate (due to fuel burn) //TODO this needs to be a function pointer
		// ... other parameters ...
	};

}


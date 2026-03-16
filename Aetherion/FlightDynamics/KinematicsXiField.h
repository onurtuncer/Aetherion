// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// KinematicsXiField.h
//
// Stateless kinematics field for a free rigid body expressed in body frame.
//
// Implements the "where" equation on SE(3):
//
//   g_dot = g * hat(xi)
//
// where xi = [omega; v] is the body-frame twist and hat() is the se(3)
// isomorphism mapping R^6 -> se(3).  In the RKMK setting this field returns
// the Lie algebra element that drives the exponential update of g -- the
// integrator never sees g_dot directly.
//
// Scalar template parameter
// -------------------------
// The field is templated on Scalar so that the Radau IIA Newton solver can
// instantiate it with CppAD::AD<double> (or a nested AD type for higher-order
// derivatives) when assembling the Jacobian of the coupled kinematics+dynamics
// RHS.  The default Scalar=double is the runtime path.
//
// Convention (body frame, right-trivialised):
//
//   xi[0..2]  =  omega_B   angular velocity in body frame  (rad/s)
//   xi[3..5]  =  v_B       linear  velocity in body frame  (m/s)
//
// This matches the layout of nu_B in RigidBodyStateD and the head<6>()
// slice packed into the Euclidean state vector x by RigidBody6DoFStepper.
//
// Models: ODE::RKMK::KinematicsFieldOnSE3<KinematicsXiField<Scalar>, Scalar>
//
#pragma once

#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <cppad/cppad.hpp>
#include <Eigen/Core>

namespace Aetherion::FlightDynamics {

    template<typename Scalar = double>
    class KinematicsXiField
    {
    public:
        // ------------------------------------------------------------------
        // Convenience aliases -- mirror the pattern used by RigidBodyVectorField
        // so call-sites can write KinematicsXiField<Scalar>::Vector6 etc.
        // ------------------------------------------------------------------
        using ScalarType = Scalar;
        using SE3Type = ODE::RKMK::Lie::SE3<Scalar>;
        using Vector6 = Eigen::Matrix<Scalar, 6, 1>;

        // ------------------------------------------------------------------
        // operator()
        //
        // Parameters
        //   t    -- current time (unused; present for concept conformance and
        //           future time-varying kinematics e.g. moving reference frames)
        //   g    -- current pose in SE(3) (unused for standard body-frame
        //           kinematics; present for concept conformance and future
        //           configuration-dependent models such as left-trivialised
        //           world-frame:  return Ad(g) * xi )
        //   xi   -- body-frame twist [omega_B; v_B] in R^6
        //
        // Returns
        //   xi unchanged -- in the right-trivialised RKMK formulation the Lie
        //   algebra element driving g is exactly the body-frame twist.
        //
        //   When Scalar = CppAD::AD<double> this pass-through is differentiable
        //   at zero cost: CppAD records the identity operation on the tape and
        //   the resulting Jacobian column is a unit vector, which the Newton
        //   solver gets for free.
        // ------------------------------------------------------------------
        [[nodiscard]]
        Vector6 operator()(
            const Scalar&   /*t*/,
            const SE3Type&  /*g*/,
            const Vector6& xi) const noexcept
        {
            return xi;
        }
    };

    // --------------------------------------------------------------------------
    // Deduction aliases
    //
    // RigidBody6DoFStepper uses KinematicsXiField<double> at runtime and
    // KinematicsXiField<CppAD::AD<double>> inside the Newton Jacobian assembly.
    // These aliases keep the stepper header readable.
    // --------------------------------------------------------------------------
    using KinematicsXiFieldd = KinematicsXiField<double>;
    using KinematicsXiFieldAD = KinematicsXiField<CppAD::AD<double>>;

} // namespace Aetherion::FlightDynamics
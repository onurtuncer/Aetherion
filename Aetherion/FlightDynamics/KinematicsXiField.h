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
// The call operator is templated on Scalar S (not the class) so that
// StageResidualIRKProductSE3 can call it with S = CppAD::AD<double> when
// building the Newton Jacobian tape, and with S = double at runtime.
// This matches the pattern used by RigidBodyVectorField::operator().
//
// Convention (body frame, right-trivialised):
//
//   xi[0..2]  =  omega_B   angular velocity in body frame  (rad/s)
//   xi[3..5]  =  v_B       linear  velocity in body frame  (m/s)
//
#pragma once

#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <cppad/cppad.hpp>
#include <Eigen/Core>

namespace Aetherion::FlightDynamics {

    class KinematicsXiField
    {
    public:
        // ------------------------------------------------------------------
        // operator()<S>
        //
        // Templated on S so the same field instance works for both:
        //   S = double            -- runtime integration path
        //   S = CppAD::AD<double> -- Newton Jacobian tape path
        //
        // Returns xi unchanged -- in the right-trivialised RKMK formulation
        // the Lie algebra element driving g is exactly the body-frame twist.
        // The identity pass-through records a trivial operation on the CppAD
        // tape and contributes a free identity block to the Jacobian.
        // ------------------------------------------------------------------
        template<class S>
        [[nodiscard]]
        Eigen::Matrix<S, 6, 1> operator()(
            const S&                         /*t*/,
            const ODE::RKMK::Lie::SE3<S>&    /*g*/,
            const Eigen::Matrix<S, 6, 1>& xi) const noexcept
        {
            return xi;
        }
    };

    // Alias kept for call-site readability -- no longer a template itself.
    using KinematicsXiFieldd = KinematicsXiField;

} // namespace Aetherion::FlightDynamics
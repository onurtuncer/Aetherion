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

/// @brief Stateless kinematics field for a free rigid body (right-trivialised SE(3)).
///
/// Implements the "where" equation on SE(3):
/// @f[ \dot{g} = g\,\hat{\xi} @f]
/// where @f$\xi = [\omega_B;\,v_B] \in \mathbb{R}^6@f$ is the body-frame twist and
/// @f$\hat{\cdot}: \mathbb{R}^6 \to \mathfrak{se}(3)@f$ is the isomorphism.
///
/// In the RKMK setting the integrator calls this field to obtain the Lie-algebra
/// element that drives the exponential update of @f$g@f$.  The result is @f$\xi@f$
/// itself — the identity map — because the body-frame twist *is* the right-trivialised
/// velocity.
///
/// The call operator is templated on @c S (not the class) so that the same instance
/// is callable for both @c double (runtime) and @c CppAD::AD<double> (Newton Jacobian
/// tape) without duplication.
    class KinematicsXiField
    {
    public:
        /// @brief Return the Lie-algebra element (identity pass-through).
        ///
        /// @tparam S     Scalar type (@c double or @c CppAD::AD<double>).
        /// @param t      Current time [s] (unused — kinematics are time-invariant).
        /// @param g      Current SE(3) pose (unused — kinematics are state-independent).
        /// @param xi     Body-frame twist @f$[\omega_B(3);\,v_B(3)]@f$ [rad/s | m/s].
        /// @return       @c xi unchanged.
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

    /// @brief Convenience alias — @c KinematicsXiField is already double-precision only.
    using KinematicsXiFieldd = KinematicsXiField;

} // namespace Aetherion::FlightDynamics
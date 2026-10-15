// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// VectorField.h
//
// Euclidean part of the 6-DoF rigid body ODE on SE(3) x R^7.
//
//   x    = [nu_B(6); m(1)]  in R^7
//   xdot = operator()(t, g, x)
//        = [dnu_B/dt(6); dm/dt(1)]
//
// Newton-Euler equations (body frame, at CG):
//
//   M * dnu_B/dt = W_ext - ad*(nu_B) * M * nu_B
//   dm/dt        = mdot(t, m)
//
// where W_ext = sum of gravity + aero + propulsion wrenches in body frame.
//
// Templated on Scalar S -- works for both double and CppAD::AD<double>.
//
// Mass properties. M is Featherstone's spatial inertia about the body origin
// (eq. 2.63), built by Spatial::Inertia from InertialParameters: the inertia
// tensor is about the centre of mass and is shifted to the origin by the
// parallel-axis term m [c]x [c]x^T, with c the centre of mass relative to the
// origin; the off-diagonal blocks are m [c]x. Both M and M_inv are stored.
// The mass handed to the wrench policies is the one M was built with, so the
// gravitational acceleration is g whatever the mass state says; x(6) is
// integrated by the mass policy and is for the owner to read. Mass properties
// change only through setInertialParameters(), which a variable-mass vehicle
// calls before each step (TwoStageRocketSimulator does, zero-order hold).
//
#pragma once

#include <concepts>

#include <Eigen/Dense>

#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>
#include <Aetherion/RigidBody/KinematicsXiField.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/Spatial/Adjoint.h>
#include <Aetherion/Spatial/Inertia.h>
#include <Aetherion/Spatial/Twist.h>

namespace Aetherion::RigidBody {

    namespace FD = Aetherion::FlightDynamics;

/// @brief Euclidean part of the 6-DoF rigid-body ODE on @f$ SE(3) \times \mathbb{R}^7 @f$.
///
/// Implements the Newton-Euler equations of motion in the body frame:
/// @f[
///   \mathbf{M}\,\dot{\nu}_B = \mathbf{W}_\mathrm{ext} - \mathrm{ad}^*(\nu_B)\,\mathbf{M}\,\nu_B
/// @f]
/// @f[
///   \dot{m} = \dot{m}(t, m)
/// @f]
/// where @f$\mathbf{M}@f$ is the 6×6 spatial inertia, @f$\nu_B = [\omega_B;\,v_B]@f$
/// is the body twist, and @f$\mathbf{W}_\mathrm{ext}@f$ is the sum of gravity,
/// aerodynamic, and propulsion wrenches (all in body frame).
///
/// The callable @c operator()(t, g, x) returns @f$\dot{\mathbf{x}} \in \mathbb{R}^7@f$
/// with layout @f$[\dot{\nu}_B(6);\, \dot{m}(1)]@f$.
///
/// @tparam Gravity    Gravity policy satisfying @c GravityPolicy concept.
/// @tparam Aero       Aerodynamic policy satisfying @c AeroPolicy concept (default: zero).
/// @tparam Thrust     Propulsion policy satisfying @c PropulsionPolicy concept (default: zero).
/// @tparam MassMdot   Mass-rate policy satisfying @c MassPolicy concept (default: constant).
    template<
        FD::GravityPolicy    Gravity,
        FD::AeroPolicy       Aero = FD::ZeroAeroPolicy,
        FD::PropulsionPolicy Thrust = FD::ZeroPropulsionPolicy,
        FD::MassPolicy       MassMdot = FD::ConstantMassPolicy
    >
    struct VectorField {
        Eigen::Matrix<double, 6, 6> M;     ///< 6×6 spatial inertia about the body origin (body frame).
        Eigen::Matrix<double, 6, 6> M_inv; ///< Inverse of @c M.
        double                      mass_kg{ 0.0 }; ///< Mass @c M was built with [kg]; handed to the wrench policies.

        Gravity   gravity;     ///< Gravity wrench policy instance.
        Aero      aero;        ///< Aerodynamic wrench policy instance.
        Thrust    thrust;      ///< Propulsion wrench policy instance.
        MassMdot  mass_model;  ///< Mass-rate model policy instance.

        /// @brief Construct from inertial parameters and all policy instances.
        /// @param ip  Inertial parameters (mass, inertia tensor, CoG offset).
        /// @param g   Gravity policy instance.
        /// @param a   Aerodynamic policy instance.
        /// @param th  Propulsion policy instance.
        /// @param mm  Mass-rate policy instance.
        explicit VectorField(
            const InertialParameters& ip,
            Gravity   g,
            Aero      a,
            Thrust    th,
            MassMdot  mm)
            : gravity(std::move(g))
            , aero(std::move(a))
            , thrust(std::move(th))
            , mass_model(std::move(mm))
        {
            setInertialParameters(ip);
        }

        /// @brief Rebuild @c M, @c M_inv and @c mass_kg from new mass properties.
        ///
        /// The inertia tensor in @p ip is about the centre of mass and
        /// @c (xbar_m, ybar_m, zbar_m) is the centre of mass relative to the body
        /// origin, so @c M is the Featherstone spatial inertia about the origin:
        /// the tensor shifted by the parallel-axis term @f$ m[c]_\times[c]_\times^{\mathsf T} @f$
        /// in the rotational block and @f$ m[c]_\times @f$ in the coupling blocks.
        /// Without the shift the matrix is not the kinetic-energy metric of any
        /// rigid body (it is indefinite for a large enough offset). This is the
        /// only way the mass properties change; call it before each step for a
        /// variable-mass vehicle.
        void setInertialParameters(const InertialParameters& ip)
        {
            const Spatial::Inertia<double> inertia(
                ip.mass_kg, inertiaTensor(ip), Eigen::Vector3d(ip.xbar_m, ip.ybar_m, ip.zbar_m));
            M       = inertia.M;
            M_inv   = M.inverse();
            mass_kg = ip.mass_kg;
        }

        // Convenience constructors for the common case where trailing policy types are
        // default-constructible.  Guarded by requires so that concept checks (e.g.
        // ConstructibleFromInertialParameters) yield a soft SFINAE failure rather than
        // a hard error when a policy (such as F16AeroPolicy) has no default constructor.

        explicit VectorField(
            const InertialParameters& ip,
            Gravity   g,
            Aero      a,
            Thrust    th)
            requires std::default_initializable<MassMdot>
            : VectorField(ip, std::move(g), std::move(a), std::move(th), MassMdot{}) {}

        explicit VectorField(
            const InertialParameters& ip,
            Gravity   g,
            Aero      a)
            requires std::default_initializable<Thrust> && std::default_initializable<MassMdot>
            : VectorField(ip, std::move(g), std::move(a), Thrust{}, MassMdot{}) {}

        explicit VectorField(
            const InertialParameters& ip,
            Gravity   g)
            requires std::default_initializable<Aero>
                  && std::default_initializable<Thrust>
                  && std::default_initializable<MassMdot>
            : VectorField(ip, std::move(g), Aero{}, Thrust{}, MassMdot{}) {}

        explicit VectorField(const InertialParameters& ip)
            requires std::default_initializable<Gravity>
                  && std::default_initializable<Aero>
                  && std::default_initializable<Thrust>
                  && std::default_initializable<MassMdot>
            : VectorField(ip, Gravity{}, Aero{}, Thrust{}, MassMdot{}) {}

        /// @brief Evaluate the ODE right-hand side @f$\dot{\mathbf{x}}@f$.
        ///
        /// @param t  Current time [s].
        /// @param g  Current SE(3) pose (rotation + ECI position).
        /// @param x  Current Euclidean state @f$[\nu_B(6);\, m(1)]@f$.
        /// @return   Time derivative @f$[\dot{\nu}_B(6);\, \dot{m}(1)]@f$.
        template<class S>
        Eigen::Matrix<S, 7, 1>
            operator()(S t,
                const ODE::RKMK::Lie::SE3<S>& g,
                const Eigen::Matrix<S, 7, 1>& x) const
        {
            const Eigen::Matrix<S, 6, 1> nu_B = x.template head<6>();

            // 1. External wrenches, with the mass M was built with: gravity
            //    divided by M must give g, whatever the integrated mass state
            //    x(6) says (it only feeds the mass policy below).
            const S m = S(mass_kg);
            Spatial::Wrench<S> W_ext{};
            W_ext.f = gravity(g, m).f;
            W_ext.f += aero(g, nu_B, m, t).f;
            W_ext.f += thrust(g, nu_B, m, t).f;

            // 2. Newton-Euler:  M_inv * (W_ext - ad*(nu_B) * M * nu_B)
            const Eigen::Matrix<S, 6, 6> M_S = M.template cast<S>();
            const Eigen::Matrix<S, 6, 6> M_inv_S = M_inv.template cast<S>();

            const Eigen::Matrix<S, 6, 1> h = M_S * nu_B;

            Spatial::Twist<S> nu_twist;
            nu_twist.v = nu_B;
            const Eigen::Matrix<S, 6, 1> bias = Spatial::ad_star_times(nu_twist, h);

            const Eigen::Matrix<S, 6, 1> dnu = M_inv_S * (W_ext.f - bias);

            // 3. Mass equation, on the integrated mass state
            const S dm = mass_model.mdot(t, x(6));

            Eigen::Matrix<S, 7, 1> xdot;
            xdot.template head<6>() = dnu;
            xdot(6) = dm;
            return xdot;
        }
    };

} // namespace Aetherion::RigidBody

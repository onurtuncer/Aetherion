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
// Changes vs original:
//   1. Both M and M_inv are stored (built once at construction).
//      The original stored only M_inv then called M_inv_S.inverse() on every
//      Newton stage to recover M -- a full 6x6 inversion per call.
//      Now operator() uses M.cast<S>(), which is a free type-widening.
//   2. The CTAD deduction guide with defaulted template arguments has been
//      removed.  MSVC (C2572) and the C++ standard forbid default arguments
//      on deduction guide template parameter lists.
//
#pragma once

#include <Eigen/Dense>

#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>
#include <Aetherion/FlightDynamics/KinematicsXiField.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/Spatial/Adjoint.h>
#include <Aetherion/Spatial/Twist.h>

namespace Aetherion::RigidBody {

    namespace FD = Aetherion::FlightDynamics;

    template<
        FD::GravityPolicy    Gravity,
        FD::AeroPolicy       Aero = FD::ZeroAeroPolicy,
        FD::PropulsionPolicy Thrust = FD::ZeroPropulsionPolicy,
        FD::MassPolicy       MassMdot = FD::ConstantMassPolicy
    >
    class VectorField {
    public:
        Eigen::Matrix<double, 6, 6> M;
        Eigen::Matrix<double, 6, 6> M_inv;

        Gravity   gravity;
        Aero      aero;
        Thrust    thrust;
        MassMdot  mass_model;

        explicit VectorField(
            const InertialParameters& ip,
            Gravity   g = {},
            Aero      a = {},
            Thrust    th = {},
            MassMdot  mm = {})
            : gravity(std::move(g))
            , aero(std::move(a))
            , thrust(std::move(th))
            , mass_model(std::move(mm))
        {
            const double m = ip.mass_kg;
            const double Ixx = ip.Ixx, Iyy = ip.Iyy, Izz = ip.Izz;
            const double Ixy = ip.Ixy, Iyz = ip.Iyz, Ixz = ip.Ixz;
            const double rx = ip.xbar_m, ry = ip.ybar_m, rz = ip.zbar_m;

            M = Eigen::Matrix<double, 6, 6>::Zero();

            // Rotational inertia block (top-left 3x3)
            M(0, 0) = Ixx;  M(0, 1) = -Ixy;  M(0, 2) = -Ixz;
            M(1, 0) = -Ixy;  M(1, 1) = Iyy;  M(1, 2) = -Iyz;
            M(2, 0) = -Ixz;  M(2, 1) = -Iyz;  M(2, 2) = Izz;

            // Translational inertia block (bottom-right 3x3)
            M(3, 3) = m;  M(4, 4) = m;  M(5, 5) = m;

            // CG offset cross-coupling
            M(0, 4) = m * rz;  M(0, 5) = -m * ry;
            M(1, 3) = -m * rz;  M(1, 5) = m * rx;
            M(2, 3) = m * ry;  M(2, 4) = -m * rx;
            M(3, 1) = -m * rz;  M(3, 2) = m * ry;
            M(4, 0) = m * rz;  M(4, 2) = -m * rx;
            M(5, 0) = -m * ry;  M(5, 1) = m * rx;

            M_inv = M.inverse();
        }

        template<class S>
        Eigen::Matrix<S, 7, 1>
            operator()(S t,
                const ODE::RKMK::Lie::SE3<S>& g,
                const Eigen::Matrix<S, 7, 1>& x) const
        {
            const Eigen::Matrix<S, 6, 1> nu_B = x.template head<6>();
            const S                      m = x(6);

            // 1. External wrenches
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

            // 3. Mass equation
            const S dm = mass_model.mdot(t, m);

            Eigen::Matrix<S, 7, 1> xdot;
            xdot.template head<6>() = dnu;
            xdot(6) = dm;
            return xdot;
        }
    };

} // namespace Aetherion::RigidBody

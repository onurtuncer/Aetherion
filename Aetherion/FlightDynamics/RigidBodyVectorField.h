// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// RigidBodyVectorField.h
//
// Euclidean part of the 6-DoF rigid body ODE on SE(3) x R^7.
//
//   x    = [nu_B(6); m(1)]  ∈ R^7
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
// M_inv stored as double; cast<S>() inside operator().
//
#pragma once

#include <Eigen/Dense>

#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>
#include <Aetherion/FlightDynamics/InertialParameters.h>
#include <Aetherion/Spatial/Adjoint.h>
#include <Aetherion/Spatial/Twist.h>

namespace Aetherion::FlightDynamics {

    // -------------------------------------------------------------------------
    // KinematicsXiField
    //
    // The SE(3) part of the ODE.  The kinematic equation on SE(3) is:
    //
    //   g_dot = g * hat(nu_B)
    //
    // In RKMK form the velocity field xi(t, g, x) is simply nu_B -- the first
    // 6 components of the Euclidean state x.
    //
    // This struct satisfies the XiField concept expected by
    // StageResidualIRK_ProductSE3.
    // -------------------------------------------------------------------------
    struct KinematicsXiField {
        template<class S>
        Eigen::Matrix<S, 6, 1>
            operator()(S, const ODE::RKMK::Lie::SE3<S>&,
                const Eigen::Matrix<S, 7, 1>& x) const
        {
            return x.template head<6>();  // xi = nu_B
        }
    };

    // -------------------------------------------------------------------------
    // RigidBodyVectorField
    //
    // Four policies as template parameters; all have compile-time defaults so
    // that the dragless sphere case requires only GravityPolicy:
    //
    //   using VF = RigidBodyVectorField<CentralGravityPolicy>;
    //
    // -------------------------------------------------------------------------
    template<
        GravityPolicy    Gravity,
        AeroPolicy       Aero = ZeroAeroPolicy,
        PropulsionPolicy Thrust = ZeroPropulsionPolicy,
        MassPolicy       MassMdot = ConstantMassPolicy
    >
    class RigidBodyVectorField {
    public:
        // Pre-inverted 6x6 spatial inertia matrix (double, built once).
        Eigen::Matrix<double, 6, 6> M_inv;

        Gravity   gravity;
        Aero      aero;
        Thrust    thrust;
        MassMdot  mass_model;

        // Build M_inv from InertialParameters.
        explicit RigidBodyVectorField(
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
            // Build 6x6 spatial inertia  M = [ J_B  -m*[r]x ; m*[r]x  m*I ]
            // (Featherstone convention, body frame at CG)
            const double m = ip.mass_kg;
            const double Ixx = ip.Ixx, Iyy = ip.Iyy, Izz = ip.Izz;
            const double Ixy = ip.Ixy, Iyz = ip.Iyz, Ixz = ip.Ixz;

            Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();

            // Rotational inertia block (3x3, top-left)
            M(0, 0) = Ixx; M(0, 1) = -Ixy; M(0, 2) = -Ixz;
            M(1, 0) = -Ixy; M(1, 1) = Iyy; M(1, 2) = -Iyz;
            M(2, 0) = -Ixz; M(2, 1) = -Iyz; M(2, 2) = Izz;

            // Translational inertia block (3x3, bottom-right)
            M(3, 3) = m; M(4, 4) = m; M(5, 5) = m;

            // CG offset cross-coupling (off-diagonal 3x3 blocks)
            // r_cg = [xbar, ybar, zbar]; [r]x skew-symmetric
            const double rx = ip.xbar_m, ry = ip.ybar_m, rz = ip.zbar_m;
            // top-right:  -m * [r]x
            M(0, 4) = m * rz; M(0, 5) = -m * ry;
            M(1, 3) = -m * rz; M(1, 5) = m * rx;
            M(2, 3) = m * ry; M(2, 4) = -m * rx;
            // bottom-left: m * [r]x  (transpose of top-right)
            M(3, 1) = -m * rz; M(3, 2) = m * ry;
            M(4, 0) = m * rz; M(4, 2) = -m * rx;
            M(5, 0) = -m * ry; M(5, 1) = m * rx;

            M_inv = M.inverse();
        }

        // ---------------------------------------------------------------------
        // operator()(t, g, x) -> xdot
        //
        // x    = [nu_B(6); m(1)]
        // xdot = [dnu_B(6); dm(1)]
        //
        // Works for S = double and S = CppAD::AD<double>.
        // ---------------------------------------------------------------------
        template<class S>
        Eigen::Matrix<S, 7, 1>
            operator()(S t,
                const ODE::RKMK::Lie::SE3<S>& g,
                const Eigen::Matrix<S, 7, 1>& x) const
        {
            const Eigen::Matrix<S, 6, 1> nu_B = x.template head<6>();
            const S                    m = x(6);

            // 1. External wrenches -- all in body frame at CG
            Spatial::Wrench<S> W_ext{};
            W_ext.f = gravity(g, m).f;
            W_ext.f += aero(g, nu_B, m, t).f;
            W_ext.f += thrust(g, nu_B, m, t).f;

            // 2. Newton-Euler:  M_inv * (W_ext - ad*(nu_B) * M * nu_B)
            const Eigen::Matrix<S, 6, 6> M_inv_S = M_inv.template cast<S>();
            const Eigen::Matrix<S, 6, 6> M_S = M_inv_S.inverse();  // 6x6, negligible cost
            const Eigen::Matrix<S, 6, 1> h = M_S * nu_B;        // generalised momentum

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

    // CTAD deduction guide -- omit trailing defaulted policies at call site.
    template<class G,
        class A = ZeroAeroPolicy,
        class T = ZeroPropulsionPolicy,
        class M = ConstantMassPolicy>
    RigidBodyVectorField(const InertialParameters&, G, A = {}, T = {}, M = {})
        -> RigidBodyVectorField<G, A, T, M>;

} // namespace Aetherion::FlightDynamics
// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

// Aetherion/FlightDynamics/RigidBodyVectorField.h
#pragma once
#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>
#include <Aetherion/Spatial/Adjoint.h>
#include <Eigen/Dense>

namespace Aetherion::FlightDynamics {

    template
        GravityPolicy     Gravity,
        AeroPolicy        Aero = ZeroAeroPolicy,
        PropulsionPolicy  Thrust = ZeroPropulsionPolicy,
        MassPolicy        MassMdot = ConstantMassPolicy
    >
        class RigidBodyVectorField {
        public:
            // Baked-in physics parameters — double only, cast<S>() inside operator().
            Eigen::Matrix<double, 6, 6> M_inv;
            Gravity    gravity;
            Aero       aero;
            Thrust     thrust;
            MassMdot   mass_model;

            // Constructed from policies and pre-inverted inertia.
            explicit RigidBodyVectorField(
                Eigen::Matrix<double, 6, 6> M_inv_in,
                Gravity  g = {},
                Aero     a = {},
                Thrust   th = {},
                MassMdot mm = {})
                : M_inv(std::move(M_inv_in))
                , gravity(std::move(g))
                , aero(std::move(a))
                , thrust(std::move(th))
                , mass_model(std::move(mm))
            {
            }

            // ------------------------------------------------------------------
            // operator()(t, g, x) -> xdot
            //
            // x    = [nu_B(6); m(1)]   in R^7
            // xdot = [dnu_B(6); dm(1)] in R^7
            //
            // Satisfies EuclidField<RigidBodyVectorField, SE3<S>, Matrix<S,7,1>>.
            // Works for S = double and S = CppAD::AD<double>.
            // ------------------------------------------------------------------
            template<class S>
            Eigen::Matrix<S, 7, 1>
                operator()(S t,
                    const ODE::RKMK::Lie::SE3<S>& g,
                    const Eigen::Matrix<S, 7, 1>& x) const
            {
                const Eigen::Matrix<S, 6, 1> nu_B = x.template head<6>();
                const S                    m = x(6);

                // 1. Sum all wrenches in body frame at CG
                Spatial::Wrench<S> W_total{};
                W_total.f = gravity(g, m).f;
                W_total.f += aero(g, nu_B, m, t).f;
                W_total.f += thrust(g, nu_B, m, t).f;

                // 2. Newton-Euler: M_inv * (W - ad*(nu) * M * nu)
                const Eigen::Matrix<S, 6, 6> M_inv_S = M_inv.template cast<S>();
                // M = M_inv^{-1}: 6x6, cheap
                const Eigen::Matrix<S, 6, 6> M_S = M_inv_S.inverse();
                const Eigen::Matrix<S, 6, 1> h = M_S * nu_B;

                Spatial::Twist<S> nu_twist; nu_twist.v = nu_B;
                const Eigen::Matrix<S, 6, 1> bias = Spatial::ad_star_times(nu_twist, h);

                const Eigen::Matrix<S, 6, 1> dnu = M_inv_S * (W_total.f - bias);

                // 3. Mass equation
                const S dm = mass_model.mdot(t, m);

                Eigen::Matrix<S, 7, 1> xdot;
                xdot.template head<6>() = dnu;
                xdot(6) = dm;
                return xdot;
            }
    };

    // ------------------------------------------------------------------
    // CTAD deduction guide — lets you write:
    //   RigidBodyVectorField vf(M_inv, CentralGravityPolicy{...});
    // without spelling out all four policy types.
    // ------------------------------------------------------------------
    template<class G, class A = ZeroAeroPolicy,
        class T = ZeroPropulsionPolicy, class M = ConstantMassPolicy>
    RigidBodyVectorField(Eigen::Matrix<double, 6, 6>, G, A = {}, T = {}, M = {})
        -> RigidBodyVectorField<G, A, T, M>;

} // namespace Aetherion::FlightDynamics
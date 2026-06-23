// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// File: Aetherion/ODE/RKMK/Integrators/ExplicitRK4_RKMK_ProductSE3.h
//
// Classical explicit RK4 Runge-Kutta-Munthe-Kaas stepper on SE(3) x R^EuclidDim.
//
// Butcher tableau (explicit, 4-stage, order 4):
//   c = [0, 1/2, 1/2, 1]
//   b = [1/6, 1/3, 1/3, 1/6]
//   A = lower-triangular (explicit)
//
// RKMK adaptation:
//   At each stage the group element is computed via the SE(3) exponential map;
//   the dexp^{-1} correction is applied to convert the left-trivialized velocity
//   into the algebra-coordinate rate K_i.  No Newton solve is required.
//
// Satisfies the RKMKIntegratorOnProductSE3<I, EuclidDim, double> concept and
// the IntegratorFor<I, XiField, FField, EuclidDim, double> concept, making it
// a drop-in replacement for RadauIIA_RKMK_ProductSE3 in SixDoFStepper.
//
// Primary use: convergence-order baseline comparison against Radau IIA RKMK.
//
#pragma once

#include <Eigen/Dense>

#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>

namespace Aetherion::ODE::RKMK::Integrators {

    namespace Lie = Aetherion::ODE::RKMK::Lie;
    namespace Core = Aetherion::ODE::RKMK::Core;

    template<class XiField, class FField, int EuclidDim = 7>
    class ExplicitRK4_RKMK_ProductSE3 final {
    public:
        using Scalar = double;

        using SE3d = Lie::SE3<Scalar>;
        using Vec6 = Eigen::Matrix<Scalar, 6, 1>;
        using VecE = Eigen::Matrix<Scalar, EuclidDim, 1>;

        struct StepResult {
            SE3d g1;
            VecE x1{ VecE::Zero() };
            bool converged{ true };  // explicit: always "converged"
        };

        explicit ExplicitRK4_RKMK_ProductSE3(XiField xi, FField f)
            : xi_field_(std::move(xi))
            , f_field_(std::move(f))
        {
        }

        // step() -- explicit RK4 RKMK on SE(3) x R^EuclidDim.
        //
        // The NewtonOptions argument is accepted to satisfy the IntegratorFor
        // concept but is not used (explicit methods require no Newton solve).
        StepResult step(
            double t0,
            const SE3d& g0,
            const VecE& x0,
            double h,
            const Core::NewtonOptions& /*opt*/ = Core::NewtonOptions{}) const
        {
            // ------------------------------------------------------------------
            // Stage 1  (at t0, g0, x0)
            // ------------------------------------------------------------------
            const Vec6 xi0 = x0.template head<6>();
            const Vec6 K1  = xi_field_(t0, g0, xi0);          // dexp_inv(0)=I
            const VecE k1  = f_field_(t0, g0, x0);

            // ------------------------------------------------------------------
            // Stage 2  (at t0 + h/2, g0*Exp(h/2*K1), x0 + h/2*k1)
            // ------------------------------------------------------------------
            const Vec6 eta2 = (h * 0.5) * K1;
            const SE3d g2   = g0 * SE3d::Exp(eta2);
            const VecE x2   = x0 + (h * 0.5) * k1;
            const Vec6 K2   = SE3d::dexp_inv(eta2, xi_field_(t0 + h * 0.5, g2, x2.template head<6>().eval()));
            const VecE k2   = f_field_(t0 + h * 0.5, g2, x2);

            // ------------------------------------------------------------------
            // Stage 3  (at t0 + h/2, g0*Exp(h/2*K2), x0 + h/2*k2)
            // ------------------------------------------------------------------
            const Vec6 eta3 = (h * 0.5) * K2;
            const SE3d g3   = g0 * SE3d::Exp(eta3);
            const VecE x3   = x0 + (h * 0.5) * k2;
            const Vec6 K3   = SE3d::dexp_inv(eta3, xi_field_(t0 + h * 0.5, g3, x3.template head<6>().eval()));
            const VecE k3   = f_field_(t0 + h * 0.5, g3, x3);

            // ------------------------------------------------------------------
            // Stage 4  (at t0 + h, g0*Exp(h*K3), x0 + h*k3)
            // ------------------------------------------------------------------
            const Vec6 eta4 = h * K3;
            const SE3d g4   = g0 * SE3d::Exp(eta4);
            const VecE x4   = x0 + h * k3;
            const Vec6 K4   = SE3d::dexp_inv(eta4, xi_field_(t0 + h, g4, x4.template head<6>().eval()));
            const VecE k4   = f_field_(t0 + h, g4, x4);

            // ------------------------------------------------------------------
            // Update
            // ------------------------------------------------------------------
            const Vec6 Delta = (h / 6.0) * (K1 + 2.0 * K2 + 2.0 * K3 + K4);
            const VecE Delta_x = (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

            StepResult out;
            out.g1 = g0 * SE3d::Exp(Delta);
            out.x1 = x0 + Delta_x;
            return out;
        }

    private:
        XiField xi_field_;
        FField  f_field_;
    };

} // namespace Aetherion::ODE::RKMK::Integrators

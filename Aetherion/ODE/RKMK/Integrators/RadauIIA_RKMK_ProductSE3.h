// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// RadauIIA_RKMK_ProductSE3.h
//
// Radau IIA (3-stage, order 5) RKMK stepper on SE(3) x R^EuclidDim.
//
// Mirrors RadauIIA_RKMK_SE3 exactly; only difference is that the residual
// functor is StageResidualIRK_ProductSE3 and StepResult carries both g1
// and x1 (the Euclidean state after the step).
//
#pragma once

#include <cassert>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include <Aetherion/ODE/RKMK/Core/Tableau.h>
#include <Aetherion/ODE/RKMK/Core/Newton.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
//#include <Aetherion/ODE/RKMK/Core/CppADResidualJacobian.h>
#include <Aetherion/ODE/RKMK/Core/StageResidualIRKProductSE3.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <Aetherion/ODE/RKMK/Lie/SE3EigenInterop.h>

namespace Aetherion::ODE::RKMK::Integrators {

    namespace Lie = Aetherion::ODE::RKMK::Lie;
    namespace Core = Aetherion::ODE::RKMK::Core;

    template<class XiField, class FField, int EuclidDim = 7>
    class RadauIIA_RKMK_ProductSE3 final {
    public:
        using Scalar = double;
        static constexpr int Stages = 3;

        using Tableau = Core::ButcherTableau<Scalar, Stages>;
        using SE3d = Lie::SE3<Scalar>;
        using VecE = Eigen::Matrix<Scalar, EuclidDim, 1>;

        // Hoisted to class scope -- MSVC resolves constexpr members here
        // but not inside function-body using-declarations.
        using Residual = Core::StageResidualIRK_ProductSE3
            <Stages, EuclidDim, XiField, FField, Tableau>;

        static constexpr int BlockDim = 6 + EuclidDim;
        static constexpr int NewtonDim = Stages * BlockDim;

        struct StepResult {
            SE3d   g1{};
            VecE   x1{ VecE::Zero() };
            bool   converged{ false };
            int    iters{ 0 };
            double residual_norm{ 0.0 };
        };

        explicit RadauIIA_RKMK_ProductSE3(XiField xi, FField f)
            : xi_field_(std::move(xi))
            , f_field_(std::move(f))
            , tableau_(Tableau::radau_iia_3stage_order5())
        {
        }

        StepResult step(
            double             t0,
            const SE3d& g0,
            const VecE& x0,
            double             h,
            const Core::NewtonOptions& opt = Core::NewtonOptions{}) const
        {
            const Eigen::Quaterniond q0 = Lie::detail::extract_q_double(g0);
            const Eigen::Vector3d    p0 = Lie::detail::extract_p_double(g0);

            Eigen::VectorXd x0d(EuclidDim);
            for (int i = 0; i < EuclidDim; ++i) x0d(i) = x0(i);

            Eigen::VectorXd sol(NewtonDim);
            sol.setZero();
            {
                const Eigen::Matrix<double, 6, 1> v0 =
                    xi_field_(t0, g0, x0).template head<6>();
                const VecE f0 = f_field_(t0, g0, x0);
                for (int i = 0; i < Stages; ++i) {
                    const int base = i * BlockDim;
                    sol.template segment<6>(base) = h * v0;
                    sol.template segment<EuclidDim>(base + 6) = h * f0;
                }
            }

            Residual residual(q0, p0, x0d, t0, h, tableau_, xi_field_, f_field_);
            Core::CppADResidualJacobian<Residual> eval(residual, NewtonDim);
            const auto newton_res = Core::NewtonSolve(eval, sol, opt);

            StepResult out;
            if constexpr (requires { newton_res.converged; })
                out.converged = newton_res.converged;
            if constexpr (requires { newton_res.iters; })
                out.iters = newton_res.iters;

            {
                Residual residual_chk(q0, p0, x0d, t0, h, tableau_, xi_field_, f_field_);
                out.residual_norm = residual_chk(sol).norm();
            }

            Eigen::Matrix<double, 6, 1> Delta_xi = Eigen::Matrix<double, 6, 1>::Zero();
            VecE Delta_u = VecE::Zero();
            for (int i = 0; i < Stages; ++i) {
                const int    base = i * BlockDim;
                const double bi = tableau_.b(i);
                Delta_xi.noalias() += bi * sol.template segment<6>(base);
                Delta_u.noalias() += bi * sol.template segment<EuclidDim>(base + 6);
            }

            out.g1 = g0 * SE3d::Exp(Delta_xi);
            out.x1 = x0 + Delta_u;
            return out;
        }

    private:
        XiField xi_field_;
        FField  f_field_;
        Tableau tableau_;
    };

} // namespace Aetherion::ODE::RKMK::Integrators
//// ------------------------------------------------------------------------------
//// Project: Aetherion
//// SPDX-License-Identifier: MIT
//// ------------------------------------------------------------------------------
////
//// RadauIIA_RKMK_ProductSE3.h
////
//// Radau IIA (3-stage, order 5) RKMK stepper on SE(3) x R^EuclidDim.
////
//// Mirrors RadauIIA_RKMK_SE3 exactly; only difference is that the residual
//// functor is StageResidualIRK_ProductSE3 and StepResult carries both g1
//// and x1 (the Euclidean state after the step).
////
//#pragma once
//
//#include <cassert>
//#include <utility>
//#include <vector>
//
//#include <Eigen/Dense>
//#include <cppad/cppad.hpp>
//
//#include <Aetherion/ODE/RKMK/Core/Tableau.h>
//#include <Aetherion/ODE/RKMK/Core/Newton.h>
//#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
//#include <Aetherion/ODE/RKMK/Core/StageResidualIRKProductSE3.h>
//#include <Aetherion/ODE/RKMK/Lie/SE3.h>
//#include <Aetherion/ODE/RKMK/Lie/SE3EigenInterop.h>
//
//namespace Aetherion::ODE::RKMK::Integrators {
//
//    namespace Lie = Aetherion::ODE::RKMK::Lie;
//
//    template<class XiField, class FField, int EuclidDim = 7>
//    class RadauIIA_RKMK_ProductSE3 final {
//    public:
//        using Scalar = double;
//        static constexpr int Stages = 3;
//
//        using Tableau = Aetherion::ODE::RKMK::Core::ButcherTableau<Scalar, Stages>;
//        using SE3d = Lie::SE3<Scalar>;
//        using VecE = Eigen::Matrix<Scalar, EuclidDim, 1>;
//
//        static constexpr int BlockDim = 6 + EuclidDim;
//        static constexpr int NewtonDim = Stages * BlockDim;
//
//        struct StepResult {
//            SE3d g1{};
//            VecE x1{ VecE::Zero() };      // [nu_B(6); m(1)] after step
//            bool   converged{ false };
//            int    iters{ 0 };
//            double residual_norm{ 0.0 };
//        };
//
//        explicit RadauIIA_RKMK_ProductSE3(XiField xi, FField f)
//            : xi_field_(std::move(xi))
//            , f_field_(std::move(f))
//            , tableau_(Tableau::radau_iia_3stage_order5())
//        {
//        }
//
//        // step(t0, g0, x0, h, opt) -> StepResult
//        StepResult step(double t0,
//            const SE3d& g0,
//            const VecE& x0,
//            double h,
//            const Aetherion::ODE::RKMK::Core::NewtonOptions& opt =
//            Aetherion::ODE::RKMK::Core::NewtonOptions{}) const
//        {
//            namespace Core = Aetherion::ODE::RKMK::Core;
//
//            const Eigen::Quaterniond q0 = Lie::detail::extract_q_double(g0);
//            const Eigen::Vector3d    p0 = Lie::detail::extract_p_double(g0);
//
//            // x0 as dynamic vector for residual constructor
//            Eigen::VectorXd x0d(EuclidDim);
//            for (int i = 0; i < EuclidDim; ++i) x0d(i) = x0(i);
//
//            using Residual = Core::StageResidualIRK_ProductSE3<
//                Stages, EuclidDim, XiField, FField, Tableau>;
//
//            Residual residual(q0, p0, x0d, t0, h, tableau_, xi_field_, f_field_);
//
//            // Initial guess: xi_i = h*xi_field(t0,g0,x0), u_i = h*f_field(t0,g0,x0)
//            Eigen::VectorXd sol(NewtonDim);
//            sol.setZero();
//            {
//                const Eigen::Matrix<double, EuclidDim, 1> v0 =
//                    xi_field_(static_cast<double>(t0), g0, x0);
//                const Eigen::Matrix<double, EuclidDim, 1> f0 =
//                    f_field_(static_cast<double>(t0), g0, x0);
//                for (int i = 0; i < Stages; ++i) {
//                    const int base = i * BlockDim;
//                    sol.segment<6>(base) = h * v0.template head<6>();
//                    sol.segment<EuclidDim>(base + 6) = h * f0;
//                }
//            }
//
//            // CppAD tape + Newton solve
//            detail::CppADResidualJacobian<Residual> eval(std::move(residual), NewtonDim);
//            const auto newton_res = Core::NewtonSolve(eval, sol, opt);
//
//            StepResult out;
//            if constexpr (requires { newton_res.converged; })
//                out.converged = newton_res.converged;
//            if constexpr (requires { newton_res.iters; })
//                out.iters = newton_res.iters;
//
//            // Residual norm check
//            {
//                Residual residual_chk(q0, p0, x0d, t0, h, tableau_, xi_field_, f_field_);
//                out.residual_norm =
//                    residual_chk.template operator() < double > (sol).norm();
//            }
//
//            // SE(3) update:  g1 = g0 * Exp(Delta_xi)
//            Eigen::Matrix<double, 6, 1> Delta_xi = Eigen::Matrix<double, 6, 1>::Zero();
//            VecE Delta_u = VecE::Zero();
//            for (int i = 0; i < Stages; ++i) {
//                const int base = i * BlockDim;
//                const double bi = tableau_.b(i);
//                Delta_xi.noalias() += bi * sol.template segment<6>(base);
//                Delta_u.noalias() += bi * sol.template segment<EuclidDim>(base + 6);
//            }
//
//            out.g1 = g0 * SE3d::Exp(Delta_xi);
//            out.x1 = x0 + Delta_u;
//            return out;
//        }
//
//    private:
//        XiField xi_field_;
//        FField  f_field_;
//        Tableau tableau_;
//    };
//
//} // namespace Aetherion::ODE::RKMK::Integrators
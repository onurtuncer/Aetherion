// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// File: Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_SE3.h
//
// Radau IIA (3-stage, order 5) RKMK stepper on SE(3), using CppAD to obtain
// the dense Jacobian for Newton (NO finite differences).
//
#pragma once

#include <cassert>
#include <cmath>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cppad/cppad.hpp>

#include <Aetherion/ODE/RKMK/Core/Tableau.h>
#include <Aetherion/ODE/RKMK/Core/StagePack.h>
#include <Aetherion/ODE/RKMK/Core/Newton.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <Aetherion/ODE/RKMK/Core/StageResidualRadauIIASE3.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <Aetherion/ODE/RKMK/Lie/SE3EigenInterop.h>

namespace Aetherion::ODE::RKMK::Integrators {

    namespace detail {

        template<class...>
        inline constexpr bool always_false_v = false;

        template<class S>
        using Vec6 = Eigen::Matrix<S, 6, 1>;

         // Generic residual+Jacobian evaluator using CppAD dense Jacobian.
        // Expects ResidualFunctor to provide:
        //   template<class S> Eigen::Matrix<S,Dynamic,1> operator()(const Eigen::Matrix<S,Dynamic,1>& x) const;
        template<class ResidualFunctor>
        class CppADResidualJacobian final {
        public:
            explicit CppADResidualJacobian(ResidualFunctor residual, int n)
                : residual_(std::move(residual)), n_(n) {
                assert(n_ > 0);
            }

            void operator()(const Eigen::VectorXd& x, Eigen::VectorXd& r, Eigen::MatrixXd& J) const {
                assert(x.size() == n_);
                tape_if_needed_();

                std::vector<double> xd((std::size_t)n_);
                for (int i = 0; i < n_; ++i) xd[(std::size_t)i] = x(i);

                const std::vector<double> yd = fun_.Forward(0, xd);
                r.resize(n_);
                for (int i = 0; i < n_; ++i) r(i) = yd[(std::size_t)i];

                const std::vector<double> jd = fun_.Jacobian(xd);
                J.resize(n_, n_);
                for (int rr = 0; rr < n_; ++rr) {
                    for (int cc = 0; cc < n_; ++cc) {
                        J(rr, cc) = jd[(std::size_t)(rr * n_ + cc)];
                    }
                }
            }

        private:
            void tape_if_needed_() const {
                if (taped_) return;

                using AD = CppAD::AD<double>;

                CppAD::vector<AD> X((std::size_t)n_);
                for (int i = 0; i < n_; ++i) X[(std::size_t)i] = AD(0.0);
                CppAD::Independent(X);

                Eigen::Matrix<AD, Eigen::Dynamic, 1> x_ad(n_);
                for (int i = 0; i < n_; ++i) x_ad(i) = X[(std::size_t)i];

                const Eigen::Matrix<AD, Eigen::Dynamic, 1> y_ad =
                    residual_.template operator() < AD > (x_ad);

                CppAD::vector<AD> Y((std::size_t)n_);
                for (int i = 0; i < n_; ++i) Y[(std::size_t)i] = y_ad(i);

                fun_.Dependent(X, Y);
                taped_ = true;
            }

            ResidualFunctor residual_;
            int n_{ 0 };

            mutable bool taped_{ false };
            mutable CppAD::ADFun<double> fun_;
        };

    } // namespace detail

    // -----------------------------------------------------------------------------
    // RadauIIA_RKMK_SE3
    // -----------------------------------------------------------------------------

    namespace Lie = Aetherion::ODE::RKMK::Lie;

    template<class VectorField>
    class RadauIIA_RKMK_SE3 final {
    public:
        using Scalar = double;
        static constexpr int Stages = 3;

        using Tableau = Aetherion::ODE::RKMK::Core::ButcherTableau<Scalar, Stages>;
        using SE3d = Aetherion::ODE::RKMK::Lie::SE3<Scalar>;
        using Vec6d = detail::Vec6<Scalar>;

      

        struct StepResult final {
            SE3d            g1{};
            Eigen::VectorXd stage_xi;      // stacked xi (size 6*s)
            bool            converged{ false };
            int             iters{ 0 };
            double          residual_norm{ 0.0 };
        };

        explicit RadauIIA_RKMK_SE3(VectorField f)
            : f_(std::move(f))
            , tableau_(Tableau::radau_iia_3stage_order5()) {
        }

        StepResult step(double t0, const SE3d& g0, double h,
            const Aetherion::ODE::RKMK::Core::NewtonOptions& opt) const {
            namespace Core = Aetherion::ODE::RKMK::Core;

            using Lie::detail::extract_q_double;
            using Lie::detail::extract_p_double;

            const Eigen::Quaterniond q0 = Lie::detail::extract_q_double(g0);
            const Eigen::Vector3d    p0 = Lie::detail::extract_p_double(g0);

            using Residual = Core::StageResidualIRK_SE3<Stages, VectorField, Tableau>;
            Residual residual(q0, p0, t0, h, tableau_, f_);

            Core::StagePack<double, Stages, 0> pack;
            Eigen::VectorXd x(pack.total_dim());
            x.setZero();

            // Initial guess: xi_i = h * f(t0, g0) for all stages.
            const Vec6d v0 = f_(t0, g0);
            const Vec6d hxi = h * v0;
            for (int i = 0; i < Stages; ++i) {
                pack.xi(x, i).noalias() = hxi;
            }

            detail::CppADResidualJacobian<Residual> eval(std::move(residual), (int)x.size());

            const auto newton_res = Core::NewtonSolve(eval, x, opt);

            StepResult out;
            out.stage_xi = x;

            if constexpr (requires { newton_res.converged; }) out.converged = newton_res.converged;
            if constexpr (requires { newton_res.iters; })     out.iters = newton_res.iters;

            // Residual norm at the found x
            {
                Residual residual_chk(q0, p0, t0, h, tableau_, f_);
                out.residual_norm = residual_chk.template operator() < double > (x).norm();
            }

            // Update: Delta = sum_i b_i * xi_i
            Vec6d Delta = Vec6d::Zero();
            for (int i = 0; i < Stages; ++i) {
                const double bi = tableau_.b(i);
                Delta.noalias() += bi * pack.xi(x, i);
            }


            out.g1 = g0 * SE3d::Exp(Delta);
            return out;
        }

    private:
        VectorField f_;
        Tableau     tableau_;
    };

} // namespace Aetherion::ODE::RKMK::Integrators


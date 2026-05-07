// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

//
// File: Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_SE3.h
//
// Radau IIA (3-stage, order 5) RKMK stepper on SE(3), using CppAD to obtain
// the dense Jacobian for Newton (NO finite differences).
//
// Allocation policy
// -----------------
// All Eigen objects are fixed-size (stack-resident). The only heap traffic in
// the hot path is the two std::vector round-trips required by CppAD's
// Forward()/Jacobian() interface inside FixedCppADResidualJacobian; these are
// structurally unavoidable without changing the AD library.
//
// NOTE: SE(3) as a single Lie group, not a product manifold.
//
// Although SE(3) is diffeomorphic to SO(3) x R^3 as a smooth manifold,
// this integrator treats it as a single Lie group throughout. Concretely:
//
//   - There is one se(3)-valued stage vector xi_i in R^6 per stage,
//     with no splitting between the rotational (omega) and translational (v)
//     components.
//   - The update g1 = g0 * Exp(sum_i b_i * xi_i) uses the single SE(3)
//     exponential map, which couples rotation and translation in the
//     screw-motion sense.
//
// A product-manifold integrator would instead maintain separate stage
// packs for SO(3) and R^3, apply independent exponential maps to each,
// and could use different methods (or step sizes) per factor. That is
// not done here: every degree of freedom advances under the same Radau
// IIA tableau and the same group operation.
//
#pragma once

#include <utility>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <Aetherion/ODE/RKMK/Core/Tableau.h>
#include <Aetherion/ODE/RKMK/Core/StagePack.h>
#include <Aetherion/ODE/RKMK/Core/Newton.h>           // NewtonSolve<N>, FixedCppADResidualJacobian<N,...>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <Aetherion/ODE/RKMK/Core/StageResidualRadauIIASE3.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <Aetherion/ODE/RKMK/Lie/SE3EigenInterop.h>

namespace Aetherion::ODE::RKMK::Integrators {

    namespace detail {
        template<class S>
        using Vec6 = Eigen::Matrix<S, 6, 1>;
    } // namespace detail

    namespace Lie = Aetherion::ODE::RKMK::Lie;

    template<class VectorField>
    class RadauIIA_RKMK_SE3 final {
    public:
        using Scalar = double;
        static constexpr int Stages = 3;

        // Compile-time dimension of the stacked Newton system: one xi in R^6
        // per stage, all coupled through the implicit Butcher tableau.
        static constexpr int XDim = 6 * Stages;  // = 18

        using Tableau = Aetherion::ODE::RKMK::Core::ButcherTableau<Scalar, Stages>;
        using SE3d = Aetherion::ODE::RKMK::Lie::SE3<Scalar>;
        using Vec6d = detail::Vec6<Scalar>;
        using VecX = Eigen::Matrix<double, XDim, 1>;     // full stage vector, stack-resident
        using MatX = Eigen::Matrix<double, XDim, XDim>;  // Jacobian, stack-resident

          // NOTE: SE(3) as a single Lie group, not a product manifold.
          //
          // Although SE(3) is diffeomorphic to SO(3) x R^3 as a smooth manifold,
          // this integrator treats it as a single Lie group throughout. Concretely:
          //
          //   - There is one se(3)-valued stage vector xi_i in R^6 per stage,
          //     with no splitting between the rotational (omega) and translational (v)
          //     components.
          //   - The update g1 = g0 * Exp(sum_i b_i * xi_i) uses the single SE(3)
          //     exponential map, which couples rotation and translation in the
          //     screw-motion sense.
          //
          // A product-manifold integrator would instead maintain separate stage
          // packs for SO(3) and R^3, apply independent exponential maps to each,
          // and could use different methods (or step sizes) per factor. That is
          // not done here: every degree of freedom advances under the same Radau
          // IIA tableau and the same group operation.
        struct StepResult final {
            SE3d   g1{};
            VecX   stage_xi{ VecX::Zero() };  // fixed-size: no heap allocation
            bool   converged{ false };
            int    iters{ 0 };
            double residual_norm{ 0.0 };
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

            const Eigen::Quaterniond q0 = extract_q_double(g0);
            const Eigen::Vector3d    p0 = extract_p_double(g0);

            using Residual = Core::StageResidualIRK_SE3<Stages, VectorField, Tableau>;
            Residual residual(q0, p0, t0, h, tableau_, f_);

            // Stack-resident stage vector; StagePack provides xi(x, i) views
            // into the appropriate 6-element segment without any allocation.
            Core::StagePack<double, Stages, 0> pack;
            VecX x = VecX::Zero();

            // Initial guess: xi_i = h * f(t0, g0) for all stages.
            const Vec6d hxi = h * f_(t0, g0);
            for (int i = 0; i < Stages; ++i)
                pack.xi(x, i).noalias() = hxi;

            // FixedCppADResidualJacobian: r and J are fixed-size, zero heap.
            Core::FixedCppADResidualJacobian<XDim, Residual> eval(std::move(residual));
            const auto newton_res = Core::NewtonSolve<XDim>(eval, x, opt);

            StepResult out;
            out.stage_xi = x;   // fixed-size copy, no heap

            if constexpr (requires { newton_res.converged; }) out.converged = newton_res.converged;
            if constexpr (requires { newton_res.iters; })     out.iters = newton_res.iters;

            // Residual norm at the found x.
            {
                Residual residual_chk(q0, p0, t0, h, tableau_, f_);
                out.residual_norm =
                    residual_chk.template operator() < double > (x).norm();
            }

            // Update: Delta = sum_i b_i * xi_i
            Vec6d Delta = Vec6d::Zero();
            for (int i = 0; i < Stages; ++i)
                Delta.noalias() += tableau_.b(i) * pack.xi(x, i);

            out.g1 = g0 * SE3d::Exp(Delta);
            return out;
        }

    private:
        VectorField f_;
        Tableau     tableau_;
    };

} // namespace Aetherion::ODE::RKMK::Integrators

//// ------------------------------------------------------------------------------
//// Project: Aetherion
//// Copyright(c) 2025, Onur Tuncer, PhD,
//// Istanbul Technical University
////
//// SPDX - License - Identifier: MIT
//// License - Filename: LICENSE
//// ------------------------------------------------------------------------------
////
//// File: Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_SE3.h
////
//// Radau IIA (3-stage, order 5) RKMK stepper on SE(3), using CppAD to obtain
//// the dense Jacobian for Newton (NO finite differences).
////
//// Allocation policy
//// -----------------
//// All Eigen objects are fixed-size (stack-resident). The only heap traffic in
//// the hot path is the two std::vector round-trips required by CppAD's
//// Forward()/Jacobian() interface; these are structurally unavoidable without
//// changing the AD library. Everything else -- stage vector x, residual r,
//// Jacobian J, and the CppAD tape itself -- lives on the stack or in
//// statically-sized members.
////
//#pragma once
//
//#include <cassert>
//#include <cmath>
//#include <type_traits>
//#include <utility>
//#include <vector>
//
//#include <Eigen/Dense>
//#include <Eigen/Geometry>
//
//#include <cppad/cppad.hpp>
//
//#include <Aetherion/ODE/RKMK/Core/Tableau.h>
//#include <Aetherion/ODE/RKMK/Core/StagePack.h>
//#include <Aetherion/ODE/RKMK/Core/Newton.h>
//#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
//#include <Aetherion/ODE/RKMK/Core/StageResidualRadauIIASE3.h>
//#include <Aetherion/ODE/RKMK/Lie/SE3.h>
//#include <Aetherion/ODE/RKMK/Lie/SE3EigenInterop.h>
//
//namespace Aetherion::ODE::RKMK::Integrators {
//
//    namespace detail {
//
//        template<class...>
//        inline constexpr bool always_false_v = false;
//
//        template<class S>
//        using Vec6 = Eigen::Matrix<S, 6, 1>;
//
//        // Generic residual+Jacobian evaluator using CppAD dense Jacobian.
//        //
//        // Template parameter N is the compile-time dimension of the Newton
//        // system (= 6 * Stages for Radau IIA on SE(3)). All Eigen objects are
//        // fixed-size. The two std::vector copies (xd in, jd out) are the
//        // irreducible cost of CppAD's Forward()/Jacobian() API.
//        //
//        // ResidualFunctor must provide:
//        //   template<class S>
//        //   Eigen::Matrix<S, N, 1> operator()(const Eigen::Matrix<S, N, 1>& x) const;
//        template<int N, class ResidualFunctor>
//        class CppADResidualJacobian final {
//        public:
//            using VecN = Eigen::Matrix<double, N, 1>;
//            using MatN = Eigen::Matrix<double, N, N>;
//
//            explicit CppADResidualJacobian(ResidualFunctor residual)
//                : residual_(std::move(residual)) {
//                static_assert(N > 0, "CppADResidualJacobian requires N > 0 at compile time");
//            }
//
//            void operator()(const VecN& x, VecN& r, MatN& J) const {
//                tape_if_needed_(x);
//
//                // CppAD's interface requires std::vector -- unavoidable copies.
//                std::vector<double> xd(N);
//                for (int i = 0; i < N; ++i) xd[(std::size_t)i] = x(i);
//
//                const std::vector<double> yd = fun_.Forward(0, xd);
//                for (int i = 0; i < N; ++i) r(i) = yd[(std::size_t)i];
//
//                const std::vector<double> jd = fun_.Jacobian(xd);
//                for (int rr = 0; rr < N; ++rr)
//                    for (int cc = 0; cc < N; ++cc)
//                        J(rr, cc) = jd[(std::size_t)(rr * N + cc)];
//            }
//
//        private:
//            // Record the tape at the current x. Re-taping every call would be
//            // correct but wasteful; taping once at the initial guess is sound
//            // for smooth residuals (the graph topology does not change).
//            void tape_if_needed_(const VecN& x) const {
//                if (taped_) return;
//
//                using AD = CppAD::AD<double>;
//                using VecAD = Eigen::Matrix<AD, N, 1>;
//
//                CppAD::vector<AD> X(N);
//                for (int i = 0; i < N; ++i) X[(std::size_t)i] = AD(x(i));
//                CppAD::Independent(X);
//
//                VecAD x_ad;
//                for (int i = 0; i < N; ++i) x_ad(i) = X[(std::size_t)i];
//
//                const VecAD y_ad = residual_.template operator() < AD > (x_ad);
//
//                CppAD::vector<AD> Y(N);
//                for (int i = 0; i < N; ++i) Y[(std::size_t)i] = y_ad(i);
//
//                fun_.Dependent(X, Y);
//                taped_ = true;
//            }
//
//            ResidualFunctor residual_;
//
//            mutable bool              taped_{ false };
//            mutable CppAD::ADFun<double> fun_;
//        };
//
//    } // namespace detail
//
//    // -------------------------------------------------------------------------
//    // RadauIIA_RKMK_SE3
//    // -------------------------------------------------------------------------
//    //
//    // NOTE: SE(3) as a single Lie group, not a product manifold.
//    //
//    // Although SE(3) is diffeomorphic to SO(3) x R^3 as a smooth manifold,
//    // this integrator treats it as a single Lie group throughout. Concretely:
//    //
//    //   - There is one se(3)-valued stage vector xi_i in R^6 per stage,
//    //     with no splitting between the rotational (omega) and translational (v)
//    //     components.
//    //   - The update g1 = g0 * Exp(sum_i b_i * xi_i) uses the single SE(3)
//    //     exponential map, which couples rotation and translation in the
//    //     screw-motion sense.
//    //
//    // A product-manifold integrator would instead maintain separate stage
//    // packs for SO(3) and R^3, apply independent exponential maps to each,
//    // and could use different methods (or step sizes) per factor. That is
//    // not done here: every degree of freedom advances under the same Radau
//    // IIA tableau and the same group operation.
//
//    namespace Lie = Aetherion::ODE::RKMK::Lie;
//
//    template<class VectorField>
//    class RadauIIA_RKMK_SE3 final {
//    public:
//        using Scalar = double;
//        static constexpr int Stages = 3;
//
//        // Compile-time dimension of the stacked Newton system: one xi in R^6
//        // per stage, all coupled through the implicit Butcher tableau.
//        static constexpr int XDim = 6 * Stages;  // = 18
//
//        using Tableau = Aetherion::ODE::RKMK::Core::ButcherTableau<Scalar, Stages>;
//        using SE3d = Aetherion::ODE::RKMK::Lie::SE3<Scalar>;
//        using Vec6d = detail::Vec6<Scalar>;
//        using VecX = Eigen::Matrix<double, XDim, 1>;   // full stage vector, stack-resident
//        using MatX = Eigen::Matrix<double, XDim, XDim>; // Jacobian, stack-resident
//
//        struct StepResult final {
//            SE3d   g1{};
//            VecX   stage_xi{ VecX::Zero() };  // fixed-size: no heap allocation
//            bool   converged{ false };
//            int    iters{ 0 };
//            double residual_norm{ 0.0 };
//        };
//
//        explicit RadauIIA_RKMK_SE3(VectorField f)
//            : f_(std::move(f))
//            , tableau_(Tableau::radau_iia_3stage_order5()) {
//        }
//
//        StepResult step(double t0, const SE3d& g0, double h,
//            const Aetherion::ODE::RKMK::Core::NewtonOptions& opt) const {
//            namespace Core = Aetherion::ODE::RKMK::Core;
//
//            using Lie::detail::extract_q_double;
//            using Lie::detail::extract_p_double;
//
//            const Eigen::Quaterniond q0 = extract_q_double(g0);
//            const Eigen::Vector3d    p0 = extract_p_double(g0);
//
//            using Residual = Core::StageResidualIRK_SE3<Stages, VectorField, Tableau>;
//            Residual residual(q0, p0, t0, h, tableau_, f_);
//
//            // Stack-resident stage vector; StagePack provides xi(x, i) views
//            // into the appropriate 6-element segment without any allocation.
//            Core::StagePack<double, Stages, 0> pack;
//            VecX x = VecX::Zero();
//
//            // Initial guess: xi_i = h * f(t0, g0) for all stages.
//            const Vec6d hxi = h * f_(t0, g0);
//            for (int i = 0; i < Stages; ++i)
//                pack.xi(x, i).noalias() = hxi;
//
//            // CppADResidualJacobian is templated on XDim: r and J are fixed-size.
//            detail::CppADResidualJacobian<XDim, Residual> eval(std::move(residual));
//
//            const auto newton_res = Core::NewtonSolve(eval, x, opt);
//
//            StepResult out;
//            out.stage_xi = x;   // fixed-size copy, no heap
//
//            if constexpr (requires { newton_res.converged; }) out.converged = newton_res.converged;
//            if constexpr (requires { newton_res.iters; })     out.iters = newton_res.iters;
//
//            // Residual norm at the found x.
//            {
//                Residual residual_chk(q0, p0, t0, h, tableau_, f_);
//                out.residual_norm =
//                    residual_chk.template operator() < double > (x).norm();
//            }
//
//            // Update: Delta = sum_i b_i * xi_i
//            Vec6d Delta = Vec6d::Zero();
//            for (int i = 0; i < Stages; ++i)
//                Delta.noalias() += tableau_.b(i) * pack.xi(x, i);
//
//            out.g1 = g0 * SE3d::Exp(Delta);
//            return out;
//        }
//
//    private:
//        VectorField f_;
//        Tableau     tableau_;
//    };
//
//} // namespace Aetherion::ODE::RKMK::Integrators


//// ------------------------------------------------------------------------------
//// Project: Aetherion
//// Copyright(c) 2025, Onur Tuncer, PhD,
//// Istanbul Technical University
////
//// SPDX - License - Identifier: MIT
//// License - Filename: LICENSE
//// ------------------------------------------------------------------------------
////
//// File: Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_SE3.h
////
//// Radau IIA (3-stage, order 5) RKMK stepper on SE(3), using CppAD to obtain
//// the dense Jacobian for Newton (NO finite differences).
////
//#pragma once
//
//#include <cassert>
//#include <cmath>
//#include <type_traits>
//#include <utility>
//#include <vector>
//
//#include <Eigen/Dense>
//#include <Eigen/Geometry>
//
//#include <cppad/cppad.hpp>
//
//#include <Aetherion/ODE/RKMK/Core/Tableau.h>
//#include <Aetherion/ODE/RKMK/Core/StagePack.h>
//#include <Aetherion/ODE/RKMK/Core/Newton.h>
//#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
//#include <Aetherion/ODE/RKMK/Core/StageResidualRadauIIASE3.h>
//#include <Aetherion/ODE/RKMK/Lie/SE3.h>
//#include <Aetherion/ODE/RKMK/Lie/SE3EigenInterop.h>
//
//namespace Aetherion::ODE::RKMK::Integrators {
//
//    namespace detail {
//
//        template<class...>
//        inline constexpr bool always_false_v = false;
//
//        template<class S>
//        using Vec6 = Eigen::Matrix<S, 6, 1>;
//
//         // Generic residual+Jacobian evaluator using CppAD dense Jacobian.
//        // Expects ResidualFunctor to provide:
//        //   template<class S> Eigen::Matrix<S,Dynamic,1> operator()(const Eigen::Matrix<S,Dynamic,1>& x) const;
//        template<class ResidualFunctor>
//        class CppADResidualJacobian final {
//        public:
//            explicit CppADResidualJacobian(ResidualFunctor residual, int n)
//                : residual_(std::move(residual)), n_(n) {
//                assert(n_ > 0);
//            }
//
//            void operator()(const Eigen::VectorXd& x, Eigen::VectorXd& r, Eigen::MatrixXd& J) const {
//                assert(x.size() == n_);
//                tape_if_needed_();
//
//                std::vector<double> xd((std::size_t)n_);
//                for (int i = 0; i < n_; ++i) xd[(std::size_t)i] = x(i);
//
//                const std::vector<double> yd = fun_.Forward(0, xd);
//                r.resize(n_);
//                for (int i = 0; i < n_; ++i) r(i) = yd[(std::size_t)i];
//
//                const std::vector<double> jd = fun_.Jacobian(xd);
//                J.resize(n_, n_);
//                for (int rr = 0; rr < n_; ++rr) {
//                    for (int cc = 0; cc < n_; ++cc) {
//                        J(rr, cc) = jd[(std::size_t)(rr * n_ + cc)];
//                    }
//                }
//            }
//
//        private:
//            void tape_if_needed_() const {
//                if (taped_) return;
//
//                using AD = CppAD::AD<double>;
//
//                CppAD::vector<AD> X((std::size_t)n_);
//                for (int i = 0; i < n_; ++i) X[(std::size_t)i] = AD(0.0);
//                CppAD::Independent(X);
//
//                Eigen::Matrix<AD, Eigen::Dynamic, 1> x_ad(n_);
//                for (int i = 0; i < n_; ++i) x_ad(i) = X[(std::size_t)i];
//
//                const Eigen::Matrix<AD, Eigen::Dynamic, 1> y_ad =
//                    residual_.template operator() < AD > (x_ad);
//
//                CppAD::vector<AD> Y((std::size_t)n_);
//                for (int i = 0; i < n_; ++i) Y[(std::size_t)i] = y_ad(i);
//
//                fun_.Dependent(X, Y);
//                taped_ = true;
//            }
//
//            ResidualFunctor residual_;
//            int n_{ 0 };
//
//            mutable bool taped_{ false };
//            mutable CppAD::ADFun<double> fun_;
//        };
//
//    } // namespace detail
//
//    // -----------------------------------------------------------------------------
//    // RadauIIA_RKMK_SE3
//    // -----------------------------------------------------------------------------
//
//    namespace Lie = Aetherion::ODE::RKMK::Lie;
//
//    template<class VectorField>
//    class RadauIIA_RKMK_SE3 final {
//    public:
//        using Scalar = double;
//        static constexpr int Stages = 3;
//
//        using Tableau = Aetherion::ODE::RKMK::Core::ButcherTableau<Scalar, Stages>;
//        using SE3d = Aetherion::ODE::RKMK::Lie::SE3<Scalar>;
//        using Vec6d = detail::Vec6<Scalar>;
//
//      
//        // NOTE: SE(3) as a single Lie group, not a product manifold.
//        //
//        // Although SE(3) is diffeomorphic to SO(3) x R^3 as a smooth manifold,
//        // this integrator treats it as a single Lie group throughout. Concretely:
//        //
//        //   - There is one se(3)-valued stage vector xi_i in R^6 per stage,
//        //     with no splitting between the rotational (omega) and translational (v)
//        //     components.
//        //   - The update g1 = g0 * Exp(sum_i b_i * xi_i) uses the single SE(3)
//        //     exponential map, which couples rotation and translation in the
//        //     screw-motion sense.
//        //
//        // A product-manifold integrator would instead maintain separate stage
//        // packs for SO(3) and R^3, apply independent exponential maps to each,
//        // and could use different methods (or step sizes) per factor. That is
//        // not done here: every degree of freedom advances under the same Radau
//        // IIA tableau and the same group operation.
//        struct StepResult final {
//            SE3d            g1{};
//            Eigen::VectorXd stage_xi;      // stacked xi (size 6*s)
//            bool            converged{ false };
//            int             iters{ 0 };
//            double          residual_norm{ 0.0 };
//        };
//
//        explicit RadauIIA_RKMK_SE3(VectorField f)
//            : f_(std::move(f))
//            , tableau_(Tableau::radau_iia_3stage_order5()) {
//        }
//
//        StepResult step(double t0, const SE3d& g0, double h,
//            const Aetherion::ODE::RKMK::Core::NewtonOptions& opt) const {
//            namespace Core = Aetherion::ODE::RKMK::Core;
//
//            using Lie::detail::extract_q_double;
//            using Lie::detail::extract_p_double;
//
//            const Eigen::Quaterniond q0 = Lie::detail::extract_q_double(g0);
//            const Eigen::Vector3d    p0 = Lie::detail::extract_p_double(g0);
//
//            using Residual = Core::StageResidualIRK_SE3<Stages, VectorField, Tableau>;
//            Residual residual(q0, p0, t0, h, tableau_, f_);
//
//            Core::StagePack<double, Stages, 0> pack;
//            Eigen::VectorXd x(pack.total_dim());
//            x.setZero();
//
//            // Initial guess: xi_i = h * f(t0, g0) for all stages.
//            const Vec6d v0 = f_(t0, g0);
//            const Vec6d hxi = h * v0;
//            for (int i = 0; i < Stages; ++i) {
//                pack.xi(x, i).noalias() = hxi;
//            }
//
//            detail::CppADResidualJacobian<Residual> eval(std::move(residual), (int)x.size());
//
//            const auto newton_res = Core::NewtonSolve(eval, x, opt);
//
//            StepResult out;
//            out.stage_xi = x;
//
//            if constexpr (requires { newton_res.converged; }) out.converged = newton_res.converged;
//            if constexpr (requires { newton_res.iters; })     out.iters = newton_res.iters;
//
//            // Residual norm at the found x
//            {
//                Residual residual_chk(q0, p0, t0, h, tableau_, f_);
//                out.residual_norm = residual_chk.template operator() < double > (x).norm();
//            }
//
//            // Update: Delta = sum_i b_i * xi_i
//            Vec6d Delta = Vec6d::Zero();
//            for (int i = 0; i < Stages; ++i) {
//                const double bi = tableau_.b(i);
//                Delta.noalias() += bi * pack.xi(x, i);
//            }
//
//            out.g1 = g0 * SE3d::Exp(Delta);
//            return out;
//        }
//
//    private:
//        VectorField f_;
//        Tableau     tableau_;
//    };
//
//} // namespace Aetherion::ODE::RKMK::Integrators
//

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// File: Aetherion/ODE/RKMK/Core/StageResidual_RadauIIA_SE3.h
//
// Implicit stage residual for an IRK tableau (e.g., Radau IIA) in an RKMK
// canonical-coordinates form on SE(3).
//
// Unknowns are stacked stage increments xi_i in se(3), each already scaled by h:
//
//   x = [ xi_0 ; xi_1 ; ... ; xi_{s-1} ],   xi_i in R^6
//
// For each stage i:
//   Eta_i = sum_j A(i,j) * xi_j
//   g_i   = g0 * Exp(Eta_i)
//   v_i   = f(t0 + c(i)*h, g_i)                   (left-trivialized body twist)
//   rhs_i = dexp_inv(Eta_i) * v_i
//   r_i   = xi_i - h * rhs_i
//
// This header only assembles residuals; use your Core::CppADResidualJacobian
// wrapper (Newton.h) to obtain Jacobians via CppAD when needed.
//
#pragma once

#include <cassert>
#include <type_traits>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <Aetherion/ODE/RKMK/Core/StagePack.h>
#include <Aetherion/ODE/RKMK/Core/Tableau.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

namespace Aetherion::ODE::RKMK::Core {

    namespace detail {

        template<class...>
        inline constexpr bool always_false_v = false;

        template<class S>
        using Vec6 = Eigen::Matrix<S, 6, 1>;

        template<class S>
        using Mat6 = Eigen::Matrix<S, 6, 6>;

        // Construct SE3<S> from stored double pose (q0_, p0_).
        template<class S>
        [[nodiscard]] inline auto make_se3_from_qp(const Eigen::Quaterniond& qd,
            const Eigen::Vector3d& pd) {
            using G = Aetherion::ODE::RKMK::Lie::SE3<S>;

            const Eigen::Quaternion<S> q(static_cast<S>(qd.w()),
                static_cast<S>(qd.x()),
                static_cast<S>(qd.y()),
                static_cast<S>(qd.z()));
            const Eigen::Matrix<S, 3, 1> p = pd.template cast<S>();

            if constexpr (requires { G{ q, p }; }) {
                return G{ q, p };
            }
            else if constexpr (requires { G(q, p); }) {
                return G(q, p);
            }
            else if constexpr (requires { G::FromQuatTranslation(q, p); }) {
                return G::FromQuatTranslation(q, p);
            }
            else if constexpr (requires { G::from_quat_translation(q, p); }) {
                return G::from_quat_translation(q, p);
            }
            else {
                static_assert(always_false_v<S>,
                    "Lie::SE3<S> must be constructible from (Eigen::Quaternion<S>, Vec3) "
                    "or provide FromQuatTranslation / from_quat_translation.");
            }
        }

        template<class S>
        [[nodiscard]] inline auto se3_exp(const Vec6<S>& eta) {
            using G = Aetherion::ODE::RKMK::Lie::SE3<S>;
            if constexpr (requires { G::Exp(eta); }) {
                return G::Exp(eta);
            }
            else if constexpr (requires { G::exp(eta); }) {
                return G::exp(eta);
            }
            else {
                static_assert(always_false_v<S>, "Lie::SE3<S> must provide static Exp(eta) or exp(eta).");
            }
        }

        template<class S>
        [[nodiscard]] inline Mat6<S> se3_dexp_inv(const Vec6<S>& eta) {
            using G = Aetherion::ODE::RKMK::Lie::SE3<S>;
            if constexpr (requires { G::dexp_inv(eta); }) {
                return G::dexp_inv(eta);
            }
            else if constexpr (requires { G::dexpInv(eta); }) {
                return G::dexpInv(eta);
            }
            else if constexpr (requires { G::left_jacobian_inv(eta); }) {
                return G::left_jacobian_inv(eta);
            }
            else if constexpr (requires { G::LeftJacobianInv(eta); }) {
                return G::LeftJacobianInv(eta);
            }
            else {
                static_assert(always_false_v<S>,
                    "Lie::SE3<S> must provide dexp_inv(eta) (or dexpInv/left_jacobian_inv).");
            }
        }

        template<class G>
        [[nodiscard]] inline auto se3_compose(const G& a, const G& b) {
            if constexpr (requires { a* b; }) {
                return a * b;
            }
            else if constexpr (requires { a.compose(b); }) {
                return a.compose(b);
            }
            else if constexpr (requires { G::compose(a, b); }) {
                return G::compose(a, b);
            }
            else {
                static_assert(always_false_v<G>, "SE3 must support composition (operator* or compose).");
            }
        }

    } // namespace detail

    // -----------------------------------------------------------------------------
    // StageResidualIRK_SE3
    // -----------------------------------------------------------------------------

    template<int Stages, class VectorField, class Tableau>
    class StageResidualIRK_SE3 final {
    public:
        static_assert(Stages > 0, "Stages must be > 0");

        // Store g0 as (q0,p0) in double; we will cast to scalar S inside operator().
        StageResidualIRK_SE3(Eigen::Quaterniond q0,
            Eigen::Vector3d p0,
            double t0,
            double h,
            Tableau tableau,
            VectorField f)
            : q0_(std::move(q0))
            , p0_(std::move(p0))
            , t0_(t0)
            , h_(h)
            , tableau_(std::move(tableau))
            , f_(std::move(f)) {
        }

        [[nodiscard]] constexpr int stages() const noexcept { return Stages; }

        template<class S>
        [[nodiscard]] Eigen::Matrix<S, Eigen::Dynamic, 1>
            operator()(const Eigen::Matrix<S, Eigen::Dynamic, 1>& x) const {
            using detail::Vec6;
            using detail::Mat6;

            // SE3-only: EuclidDim = 0 => total_dim = Stages * 6
            const StagePack<S, Stages, 0> pack;

            assert(x.cols() == 1);
            assert(x.rows() == pack.total_dim());

            Eigen::Matrix<S, Eigen::Dynamic, 1> r(pack.total_dim());
            r.setZero();

            const auto g0 = detail::make_se3_from_qp<S>(q0_, p0_);

            const S t0s = static_cast<S>(t0_);
            const S hs = static_cast<S>(h_);

            // We expect tableau_.A, tableau_.c members as in your ButcherTableau.
            static_assert(std::is_same_v<decltype(tableau_.A), decltype(tableau_.A)>,
                "Tableau must have member A (Eigen matrix).");
            static_assert(std::is_same_v<decltype(tableau_.c), decltype(tableau_.c)>,
                "Tableau must have member c (Eigen vector).");

            for (int i = 0; i < Stages; ++i) {
                // Eta_i = sum_j A(i,j) * xi_j
                Vec6<S> Eta = Vec6<S>::Zero();
                for (int j = 0; j < Stages; ++j) {
                    const S aij = static_cast<S>(tableau_.A(i, j));
                    Eta.noalias() += aij * pack.xi(x, j);
                }

                // g_i = g0 * Exp(Eta_i)
                const auto dGi = detail::se3_exp<S>(Eta);
                const auto Gi = detail::se3_compose(g0, dGi);

                // t_i = t0 + c(i)*h
                const S ci = static_cast<S>(tableau_.c(i));
                const S ti = t0s + ci * hs;

                // v_i = f(t_i, g_i)   (expected Vec6<S>)
                const Vec6<S> vi = f_(ti, Gi);

                // rhs_i = dexp_inv(Eta_i) * v_i
                const Mat6<S> Jinv = detail::se3_dexp_inv<S>(Eta);
                const Vec6<S> rhs = Jinv * vi;

                // r_i = xi_i - h * rhs_i
                pack.xi(r, i).noalias() = pack.xi(x, i) - hs * rhs;
            }

            return r;
        }

    private:
        Eigen::Quaterniond q0_;
        Eigen::Vector3d    p0_;
        double             t0_{ 0.0 };
        double             h_{ 0.0 };
        Tableau            tableau_;
        VectorField        f_;
    };

    // Convenience alias for your tableau type
    template<class ScalarCoeff, int Stages, class VectorField>
    using StageResidualRadauIIA_SE3 =
        StageResidualIRK_SE3<Stages, VectorField, ButcherTableau<ScalarCoeff, Stages>>;

} // namespace Aetherion::ODE::RKMK::Core


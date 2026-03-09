// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// StageResidualIRK_ProductSE3.h
//
// Implicit IRK stage residual on the product manifold SE(3) x R^EuclidDim.
//
// Unknowns per stage i:
//   [xi_i(6); u_i(EuclidDim)]   (scaled by h, as in the SE3-only case)
//
// Residual per stage:
//   Eta_i  = sum_j A(i,j) * xi_j          (se(3) increment)
//   Phi_i  = sum_j A(i,j) * u_j           (R^m increment)
//   g_i    = g0 * Exp(Eta_i)
//   x_i    = x0 + Phi_i
//   v_i    = xi_field(t_i, g_i, x_i)      -> R^6  (kinematics)
//   f_i    = f_field (t_i, g_i, x_i)      -> R^m  (Euclidean dynamics)
//
//   r_xi_i = xi_i - h * dexp_inv(Eta_i) * v_i
//   r_u_i  = u_i  - h * f_i
//
// For 3-stage Radau IIA + EuclidDim=7: 39x39 Newton system.
//
#pragma once

#include <cassert>
#include <utility>

#include <Eigen/Dense>

#include <Aetherion/ODE/RKMK/Core/StagePack.h>
#include <Aetherion/ODE/RKMK/Core/Tableau.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <Aetherion/ODE/RKMK/Core/StageResidualRadauIIASE3.h>  // re-use detail helpers

namespace Aetherion::ODE::RKMK::Core {

    template<int Stages, int EuclidDim, class XiField, class FField, class Tableau>
    class StageResidualIRK_ProductSE3 final {
    public:
        static_assert(Stages > 0, "Stages must be > 0");
        static_assert(EuclidDim > 0, "EuclidDim must be > 0");

        static constexpr int XiDim = 6;
        static constexpr int BlockDim = XiDim + EuclidDim;   // per stage
        static constexpr int TotalDim = Stages * BlockDim;

        StageResidualIRK_ProductSE3(
            Eigen::Quaterniond       q0,
            Eigen::Vector3d          p0,
            Eigen::Matrix<double, Eigen::Dynamic, 1> x0,  // R^EuclidDim initial Eucl. state
            double                   t0,
            double                   h,
            Tableau                  tableau,
            XiField                  xi_field,
            FField                   f_field)
            : q0_(std::move(q0))
            , p0_(std::move(p0))
            , x0_(std::move(x0))
            , t0_(t0), h_(h)
            , tableau_(std::move(tableau))
            , xi_field_(std::move(xi_field))
            , f_field_(std::move(f_field))
        {
            assert(x0_.size() == EuclidDim);
        }

        [[nodiscard]] static constexpr int total_dim() noexcept { return TotalDim; }

        // ---------------------------------------------------------------------
        // operator()(x) -> residual
        //
        // x is stacked as:
        //   [ xi_0(6), u_0(EuclidDim), xi_1(6), u_1(EuclidDim), ... ]
        // ---------------------------------------------------------------------
        template<class S>
        [[nodiscard]] Eigen::Matrix<S, Eigen::Dynamic, 1>
            operator()(const Eigen::Matrix<S, Eigen::Dynamic, 1>& x) const
        {
            using Vec6 = Eigen::Matrix<S, 6, 1>;
            using VecE = Eigen::Matrix<S, EuclidDim, 1>;

            assert(x.size() == TotalDim);

            Eigen::Matrix<S, Eigen::Dynamic, 1> r(TotalDim);
            r.setZero();

            const auto g0 = detail::make_se3_from_qp<S>(q0_, p0_);
            const VecE  x0_S = x0_.template cast<S>();
            const S t0s = static_cast<S>(t0_);
            const S hs = static_cast<S>(h_);

            // Precompute all Eta_i and Phi_i (inner product over stages)
            Eigen::Matrix<S, 6, Stages>          Eta_all;
            Eigen::Matrix<S, EuclidDim, Stages>  Phi_all;
            Eta_all.setZero(); Phi_all.setZero();

            for (int i = 0; i < Stages; ++i) {
                const int base_i = i * BlockDim;
                const Vec6 xi_i = x.template segment<6>(base_i);
                const VecE u_i = x.template segment<EuclidDim>(base_i + XiDim);

                for (int j = 0; j < Stages; ++j) {
                    const S aij = static_cast<S>(tableau_.A(i, j));
                    const int base_j = j * BlockDim;
                    Eta_all.col(i).noalias() +=
                        aij * x.template segment<6>(base_j);
                    Phi_all.col(i).noalias() +=
                        aij * x.template segment<EuclidDim>(base_j + XiDim);
                }
                (void)xi_i; (void)u_i;  // used below stage-by-stage
            }

            for (int i = 0; i < Stages; ++i) {
                const int base_i = i * BlockDim;
                const Vec6 xi_i = x.template segment<6>(base_i);
                const VecE u_i = x.template segment<EuclidDim>(base_i + XiDim);

                const Vec6 Eta_i = Eta_all.col(i);
                const VecE Phi_i = Phi_all.col(i);

                // g_i = g0 * Exp(Eta_i)
                const auto dGi = detail::se3_exp<S>(Eta_i);
                const auto g_i = detail::se3_compose(g0, dGi);

                // x_i = x0 + Phi_i
                const VecE x_i = x0_S + Phi_i;

                // Stage time
                const S ti = t0s + static_cast<S>(tableau_.c(i)) * hs;

                // Build full R^(6+EuclidDim) state for field calls
                Eigen::Matrix<S, 6 + EuclidDim, 1> state_i;
                state_i.template head<6>() = xi_i;   // not used by xi_field, but x_i
                // xi_field only needs x_i (the Euclidean state)
                Eigen::Matrix<S, EuclidDim, 1> xe_i = x_i;

                // Kinematics:  v_i = xi_field(t_i, g_i, [nu_B=...; m=...])
                // We pack x_i (size EuclidDim) into a vector for the field.
                Eigen::Matrix<S, EuclidDim, 1> x_full = x_i;
                const Vec6 v_i = xi_field_(ti, g_i, x_full);

                // Euclidean dynamics: f_i = f_field(t_i, g_i, x_i)
                const VecE f_i = f_field_(ti, g_i, x_full);

                // dexp_inv(Eta_i) * v_i
                const auto Jinv = detail::se3_dexp_inv<S>(Eta_i);
                const Vec6 rhs = Jinv * v_i;

                // r_xi_i = xi_i - h * rhs
                r.template segment<6>(base_i).noalias() = xi_i - hs * rhs;

                // r_u_i = u_i - h * f_i
                r.template segment<EuclidDim>(base_i + XiDim).noalias() = u_i - hs * f_i;
            }

            return r;
        }

    private:
        Eigen::Quaterniond q0_;
        Eigen::Vector3d    p0_;
        Eigen::Matrix<double, Eigen::Dynamic, 1> x0_;
        double  t0_{ 0.0 }, h_{ 0.0 };
        Tableau  tableau_;
        XiField  xi_field_;
        FField   f_field_;
    };

} // namespace Aetherion::ODE::RKMK::Core
// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// File: Aetherion/ODE/RKMK/Core/StagePack.h
//
// Utilities to pack/unpack stacked stage unknowns/residuals/Jacobians for
// implicit RK (e.g., Radau IIA) on a product manifold SE(3) x R^m.
//
// Convention (per stage i = 0..s-1):
//   x_i = [ xi_i ; u_i ]
// where:
//   xi_i : se(3) tangent increment (dim 6)   -> (omega, v) packed as 6x1
//   u_i  : Euclidean increment (dim m)       -> optional; m can be fixed or dynamic
//
// The full stacked unknown is:
//   x = [ x_0 ; x_1 ; ... ; x_{s-1} ]  in R^{ s*(6+m) }.
//
// This header is intentionally "layout-only": no SE3 math here.

#pragma once

#include <cassert>
#include <type_traits>

#include <Eigen/Dense>

namespace Aetherion::ODE::RKMK::Core {

    template<class ScalarT, int Stages, int EuclidDim = Eigen::Dynamic>
    struct StagePack final {
        static_assert(Stages > 0, "StagePack: Stages must be > 0");

        using Scalar = ScalarT;

        static constexpr int kStages = Stages;
        static constexpr int kLieDim = 6; // se(3) tangent dimension (omega, v)
        static constexpr int kEuclidDimCT =
            (EuclidDim == Eigen::Dynamic) ? Eigen::Dynamic : EuclidDim;

        static constexpr int kStageDimCT =
            (EuclidDim == Eigen::Dynamic) ? Eigen::Dynamic : (kLieDim + EuclidDim);

        static constexpr int kTotalDimCT =
            (EuclidDim == Eigen::Dynamic) ? Eigen::Dynamic : (Stages * (kLieDim + EuclidDim));

        using Vector = Eigen::Matrix<Scalar, kTotalDimCT, 1>;
        using Matrix = Eigen::Matrix<Scalar, kTotalDimCT, kTotalDimCT>;

        // If EuclidDim is dynamic, you must pass m at runtime.
        constexpr StagePack()
            requires (EuclidDim != Eigen::Dynamic)
        : euclid_dim_(EuclidDim) {
        }

        explicit constexpr StagePack(int euclid_dim)
            requires (EuclidDim == Eigen::Dynamic)
        : euclid_dim_(euclid_dim) {
            assert(euclid_dim_ >= 0);
        }

        [[nodiscard]] constexpr int stages() const noexcept { return kStages; }
        [[nodiscard]] constexpr int lie_dim() const noexcept { return kLieDim; }

        [[nodiscard]] constexpr int euclid_dim() const noexcept {
            if constexpr (EuclidDim == Eigen::Dynamic) return euclid_dim_;
            else return EuclidDim;
        }

        [[nodiscard]] constexpr int stage_dim() const noexcept { return kLieDim + euclid_dim(); }
        [[nodiscard]] constexpr int total_dim() const noexcept { return kStages * stage_dim(); }

        // ---------------------------------------------------------------------
        // Allocation helpers (nice for tests / host code).
        // For fixed sizes, these return fixed-size objects.
        // For dynamic, they resize to total_dim().
        // ---------------------------------------------------------------------
        [[nodiscard]] Vector make_vector() const {
            if constexpr (kTotalDimCT == Eigen::Dynamic) {
                Vector v(total_dim());
                v.setZero();
                return v;
            }
            else {
                Vector v;
                v.setZero();
                return v;
            }
        }

        [[nodiscard]] Matrix make_matrix() const {
            if constexpr (kTotalDimCT == Eigen::Dynamic) {
                Matrix M(total_dim(), total_dim());
                M.setZero();
                return M;
            }
            else {
                Matrix M;
                M.setZero();
                return M;
            }
        }

        // ---------------------------------------------------------------------
        // Offsets
        // ---------------------------------------------------------------------
        [[nodiscard]] constexpr int stage_offset(int i) const noexcept {
            return i * stage_dim();
        }

        [[nodiscard]] constexpr int xi_offset(int i) const noexcept {
            return stage_offset(i);
        }

        [[nodiscard]] constexpr int u_offset(int i) const noexcept {
            return stage_offset(i) + kLieDim;
        }

        // ---------------------------------------------------------------------
        // Vector access (x or r): xi_i (6) and u_i (m)
        // ---------------------------------------------------------------------
        template<class Derived>
        [[nodiscard]] auto xi(Eigen::MatrixBase<Derived>& x, int i) const {
            check_vector_(x.derived());
            check_stage_(i);
            return x.derived().segment(xi_offset(i), kLieDim);
        }

        template<class Derived>
        [[nodiscard]] auto xi(const Eigen::MatrixBase<Derived>& x, int i) const {
            check_vector_(x.derived());
            check_stage_(i);
            return x.derived().segment(xi_offset(i), kLieDim);
        }

        template<class Derived>
        [[nodiscard]] auto u(Eigen::MatrixBase<Derived>& x, int i) const {
            check_vector_(x.derived());
            check_stage_(i);
            return x.derived().segment(u_offset(i), euclid_dim());
        }

        template<class Derived>
        [[nodiscard]] auto u(const Eigen::MatrixBase<Derived>& x, int i) const {
            check_vector_(x.derived());
            check_stage_(i);
            return x.derived().segment(u_offset(i), euclid_dim());
        }

        // Stage block [xi_i; u_i] (size stage_dim)
        template<class Derived>
        [[nodiscard]] auto stage(Eigen::MatrixBase<Derived>& x, int i) const {
            check_vector_(x.derived());
            check_stage_(i);
            return x.derived().segment(stage_offset(i), stage_dim());
        }

        template<class Derived>
        [[nodiscard]] auto stage(const Eigen::MatrixBase<Derived>& x, int i) const {
            check_vector_(x.derived());
            check_stage_(i);
            return x.derived().segment(stage_offset(i), stage_dim());
        }

        // ---------------------------------------------------------------------
        // Matrix access (Jacobian): stage blocks and sub-blocks
        //
        // J is assumed to be sized (total_dim x total_dim) with the same stacking
        // as the vector. These helpers return Eigen::Block views.
        // ---------------------------------------------------------------------
        template<class Derived>
        [[nodiscard]] auto stage_block(Eigen::MatrixBase<Derived>& J, int i, int j) const {
            check_square_(J.derived());
            check_stage_(i);
            check_stage_(j);
            return J.derived().block(stage_offset(i), stage_offset(j), stage_dim(), stage_dim());
        }

        template<class Derived>
        [[nodiscard]] auto stage_block(const Eigen::MatrixBase<Derived>& J, int i, int j) const {
            check_square_(J.derived());
            check_stage_(i);
            check_stage_(j);
            return J.derived().block(stage_offset(i), stage_offset(j), stage_dim(), stage_dim());
        }

        // xi-xi block: 6x6
        template<class Derived>
        [[nodiscard]] auto J_xi_xi(Eigen::MatrixBase<Derived>& J, int i, int j) const {
            check_square_(J.derived());
            check_stage_(i);
            check_stage_(j);
            return J.derived().block(xi_offset(i), xi_offset(j), kLieDim, kLieDim);
        }

        template<class Derived>
        [[nodiscard]] auto J_xi_xi(const Eigen::MatrixBase<Derived>& J, int i, int j) const {
            check_square_(J.derived());
            check_stage_(i);
            check_stage_(j);
            return J.derived().block(xi_offset(i), xi_offset(j), kLieDim, kLieDim);
        }

        // xi-u block: 6xm
        template<class Derived>
        [[nodiscard]] auto J_xi_u(Eigen::MatrixBase<Derived>& J, int i, int j) const {
            check_square_(J.derived());
            check_stage_(i);
            check_stage_(j);
            return J.derived().block(xi_offset(i), u_offset(j), kLieDim, euclid_dim());
        }

        template<class Derived>
        [[nodiscard]] auto J_xi_u(const Eigen::MatrixBase<Derived>& J, int i, int j) const {
            check_square_(J.derived());
            check_stage_(i);
            check_stage_(j);
            return J.derived().block(xi_offset(i), u_offset(j), kLieDim, euclid_dim());
        }

        // u-xi block: mx6
        template<class Derived>
        [[nodiscard]] auto J_u_xi(Eigen::MatrixBase<Derived>& J, int i, int j) const {
            check_square_(J.derived());
            check_stage_(i);
            check_stage_(j);
            return J.derived().block(u_offset(i), xi_offset(j), euclid_dim(), kLieDim);
        }

        template<class Derived>
        [[nodiscard]] auto J_u_xi(const Eigen::MatrixBase<Derived>& J, int i, int j) const {
            check_square_(J.derived());
            check_stage_(i);
            check_stage_(j);
            return J.derived().block(u_offset(i), xi_offset(j), euclid_dim(), kLieDim);
        }

        // u-u block: mxm
        template<class Derived>
        [[nodiscard]] auto J_u_u(Eigen::MatrixBase<Derived>& J, int i, int j) const {
            check_square_(J.derived());
            check_stage_(i);
            check_stage_(j);
            return J.derived().block(u_offset(i), u_offset(j), euclid_dim(), euclid_dim());
        }

        template<class Derived>
        [[nodiscard]] auto J_u_u(const Eigen::MatrixBase<Derived>& J, int i, int j) const {
            check_square_(J.derived());
            check_stage_(i);
            check_stage_(j);
            return J.derived().block(u_offset(i), u_offset(j), euclid_dim(), euclid_dim());
        }

    private:
        int euclid_dim_{ 0 };

        void check_stage_(int i) const noexcept {
            assert(i >= 0 && i < kStages);
        }

        template<class Derived>
        void check_vector_(const Eigen::MatrixBase<Derived>& x) const noexcept {
            assert(x.cols() == 1);
            assert(x.rows() == total_dim());
        }

        template<class Derived>
        void check_square_(const Eigen::MatrixBase<Derived>& J) const noexcept {
            assert(J.rows() == total_dim() && J.cols() == total_dim());
        }
    };

} // namespace Aetherion::ODE::RKMK::Core

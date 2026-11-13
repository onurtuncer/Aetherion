// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::Spatial {

    /// @brief Skew-symmetric (cross-product) matrix of a 3-vector.
    ///
    /// Returns @f$ [\mathbf{v}]_\times @f$ such that
    /// @f$ [\mathbf{v}]_\times \mathbf{u} = \mathbf{v} \times \mathbf{u} @f$.
    ///
    /// @tparam S  Scalar type.
    /// @param  v  Input 3-vector.
    /// @return    3×3 skew-symmetric matrix.
    template <class S>
    Eigen::Matrix<S, 3, 3> skew(const Eigen::Matrix<S, 3, 1>& v)
    {
        // S(0), not 0: an int converts to CppAD::AD<double> through one
        // user-defined conversion, but to AD<CppAD::cg::CG<double>> only
        // through two, which the comma initialiser cannot take.
        const S zero(0);
        Eigen::Matrix<S, 3, 3> M;
        M << zero, -v(2), v(1),
            v(2), zero, -v(0),
            -v(1), v(0), zero;
        return M;
    }

} // namespace Aetherion::Spatial

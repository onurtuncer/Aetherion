// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Aetherion/ODE/RKMK/Lie/SO3.h"

namespace Aetherion::ODE::RKMK::Lie {

/// @brief Rigid-body transformation group SE(3): rotation + translation in 3D space.
///
/// Represents an element of the Special Euclidean group
/// @f$ SE(3) = \{ (R, \mathbf{p}) \mid R \in SO(3),\; \mathbf{p} \in \mathbb{R}^3 \} @f$.
/// The rotation is stored as both a unit quaternion @f$ q @f$ (for efficient composition)
/// and an explicit 3×3 rotation matrix @f$ R @f$ (for direct matrix arithmetic).
/// The translational part is stored as a 3-vector @f$ \mathbf{p} @f$.
///
/// The tangent space (Lie algebra @f$ \mathfrak{se}(3) @f$) is represented as a 6-vector
/// @f$ \xi = [\omega^\top,\; v^\top]^\top @f$ where @f$ \omega \in \mathbb{R}^3 @f$ is the
/// angular velocity and @f$ v \in \mathbb{R}^3 @f$ is the translational velocity.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
template <class Scalar>
struct SE3 {
    using Mat3    = Eigen::Matrix<Scalar, 3, 3>; ///< 3×3 rotation matrix type.
    using Vec3    = Eigen::Matrix<Scalar, 3, 1>; ///< 3-vector type.
    using Tangent = Eigen::Matrix<Scalar, 6, 1>; ///< Lie algebra element: [omega; v].
    using Mat6    = Eigen::Matrix<Scalar, 6, 6>; ///< 6×6 matrix type (adjoint, dexp).

    Eigen::Quaternion<Scalar> q = Eigen::Quaternion<Scalar>::Identity(); ///< Unit quaternion representing the rotation part.
    Mat3                      R = Mat3::Identity();                       ///< Rotation matrix (kept in sync with q).
    Vec3                      p = Vec3::Zero();                           ///< Translation vector [m] (or dimensionless, context-dependent).

    SE3() = default;

    /// @brief Constructs an SE3 element from a rotation matrix and a translation vector.
    /// @param R_in 3×3 rotation matrix; must be a valid SO(3) element.
    /// @param p_in Translation vector.
    SE3(const Mat3& R_in, const Vec3& p_in);

    /// @brief Constructs an SE3 element from a unit quaternion and a translation vector.
    /// @param q_in Unit quaternion representing the rotation; will be normalised internally.
    /// @param p_in Translation vector.
    SE3(const Eigen::Quaternion<Scalar>& q_in, const Vec3& p_in);

    /// @brief Returns the identity element of SE(3): zero rotation, zero translation.
    /// @return Identity SE3 element.
    [[nodiscard]] static SE3 Identity() { return SE3{}; }

    /// @brief Returns the rotation component as a unit quaternion.
    /// @return Unit quaternion @f$ q \in SO(3) @f$.
    [[nodiscard]] Eigen::Quaternion<Scalar> rotation()    const { return q; }

    /// @brief Returns the translation component.
    /// @return Translation vector @f$ \mathbf{p} \in \mathbb{R}^3 @f$.
    [[nodiscard]] Vec3                      translation() const { return p; }

    /// @brief Group composition operator: @f$ (R_a, p_a) \cdot (R_b, p_b) = (R_a R_b,\; p_a + R_a p_b) @f$.
    /// @param a Left operand.
    /// @param b Right operand.
    /// @return Composed SE3 element.
    friend SE3 operator*(const SE3& a, const SE3& b) {
        SE3 out;
        out.R = a.R * b.R;
        out.q = Eigen::Quaternion<Scalar>(out.R);
        out.q.normalize();
        out.p = a.p + a.R * b.p;
        return out;
    }

    /// @brief Returns the group inverse: @f$ (R, p)^{-1} = (R^\top, -R^\top p) @f$.
    /// @return Inverse SE3 element.
    [[nodiscard]] SE3 inverse() const;

    /// @brief Computes the adjoint representation matrix @f$ \text{ad}(\xi) @f$ of a Lie algebra element.
    /// @param xi Tangent vector @f$ \xi \in \mathfrak{se}(3) @f$ as [omega; v].
    /// @return 6×6 adjoint matrix.
    [[nodiscard]] static Mat6 ad_matrix(const Tangent& xi);

    /// @brief Exponential map: maps a Lie algebra element to a group element.
    ///
    /// Computes @f$ \exp(\hat{\xi}) \in SE(3) @f$ using the closed-form Rodrigues-like formula.
    /// @param xi Tangent vector @f$ \xi \in \mathfrak{se}(3) @f$ as [omega; v].
    /// @return Corresponding SE3 group element.
    [[nodiscard]] static SE3 Exp(const Tangent& xi);

    /// @brief Alias for Exp(); provided for API compatibility.
    /// @param xi Tangent vector @f$ \xi \in \mathfrak{se}(3) @f$.
    /// @return Corresponding SE3 group element.
    [[nodiscard]] static SE3 exp(const Tangent& xi) { return Exp(xi); }

    /// @brief Computes the inverse of the right-trivialized tangent map @f$ d\exp^{-1}(x) @f$.
    /// @param x Tangent vector at which to evaluate the map.
    /// @return 6×6 matrix @f$ d\exp^{-1}(x) @f$.
    [[nodiscard]] static Mat6 dexp_inv(const Tangent& x);

    /// @brief Applies @f$ d\exp^{-1}(x) @f$ to a tangent vector @f$ y @f$.
    /// @param x Tangent vector defining the evaluation point.
    /// @param y Tangent vector to transform.
    /// @return Result of @f$ d\exp^{-1}(x)\,y @f$.
    [[nodiscard]] static Tangent dexp_inv(const Tangent& x, const Tangent& y) {
        return dexp_inv(x) * y;
    }
};

} // namespace Aetherion::ODE::RKMK::Lie

#include <Aetherion/ODE/RKMK/Lie/SE3.inl>

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// AerodynamicAngles.h
//
// CppAD-friendly aerodynamic angle utilities.
//
// Conventions (body frame, typical aerospace):
// - Body axes: +x forward, +y right, +z down (NED-style body).
// - v_body = [u, v, w] is the air-relative velocity expressed in body axes.
// - Angle of attack: alpha = atan2(w, u)
// - Sideslip angle:  beta  = atan2(v, sqrt(u^2 + w^2))
//
// Notes:
// - No `if` branches; "safety" uses smooth eps terms.
// - Works for double, float, CppAD::AD<double>, etc.
//

#pragma once

#include <array>
#include <cppad/cppad.hpp>

#include "Aetherion/Aerodynamics/Math.h"

namespace Aetherion::Aerodynamics {

/// @brief Precomputed aerodynamic angles and speed derived from a body-frame velocity vector.
///
/// All fields are populated by AnglesFromVelocityBody() and are provided as a
/// convenience bundle so downstream computations (force, moment) can reuse them
/// without re-deriving each quantity independently.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
template <class Scalar>
struct AeroAngles {
    Scalar alpha_rad; ///< Angle of attack [rad]: atan2(w, u).
    Scalar beta_rad;  ///< Sideslip angle [rad]: atan2(v, sqrt(u^2 + w^2)).
    Scalar speed_m_s; ///< Airspeed magnitude ||v_body|| [m/s].
    Scalar u;         ///< Body-axis x component of air-relative velocity (forward) [m/s].
    Scalar v;         ///< Body-axis y component of air-relative velocity (right) [m/s].
    Scalar w;         ///< Body-axis z component of air-relative velocity (down) [m/s].
};

/// @brief Computes angle of attack, sideslip, and airspeed from the body-frame air-relative velocity.
///
/// Uses smooth (branch-free) approximations of atan2 and sqrt so the result is
/// differentiable everywhere — required for CppAD tape recording.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param v_body_m_s Air-relative velocity vector expressed in body axes [m/s].
/// @param eps Smoothing parameter for the speed regularisation; default 1e-12.
/// @return AeroAngles struct containing alpha, beta, speed, and the raw u/v/w components.
template <class Scalar>
inline AeroAngles<Scalar> AnglesFromVelocityBody(
    const Vec3<Scalar>& v_body_m_s,
    const Scalar& eps = Scalar(1e-12))
{
    const Scalar u = v_body_m_s[0];
    const Scalar v = v_body_m_s[1];
    const Scalar w = v_body_m_s[2];

    const Scalar speed = SpeedFromVelocity(v_body_m_s, eps);

    // beta = atan2(v, sqrt(u^2 + w^2))
    const Scalar uw = CppAD::sqrt(u * u + w * w + eps * eps);
    const Scalar beta = CppAD::atan2(v, uw);

    // alpha = atan2(w, u)
    const Scalar alpha = CppAD::atan2(w, u);

    return AeroAngles<Scalar>{ alpha, beta, speed, u, v, w };
}

/// @brief Cosine of the angle of attack.
/// @tparam Scalar Numeric scalar type.
/// @param alpha_rad Angle of attack [rad].
/// @return cos(alpha_rad).
template <class Scalar>
inline Scalar CosAlpha(const Scalar& alpha_rad) { return CppAD::cos(alpha_rad); }

/// @brief Sine of the angle of attack.
/// @tparam Scalar Numeric scalar type.
/// @param alpha_rad Angle of attack [rad].
/// @return sin(alpha_rad).
template <class Scalar>
inline Scalar SinAlpha(const Scalar& alpha_rad) { return CppAD::sin(alpha_rad); }

/// @brief Cosine of the sideslip angle.
/// @tparam Scalar Numeric scalar type.
/// @param beta_rad Sideslip angle [rad].
/// @return cos(beta_rad).
template <class Scalar>
inline Scalar CosBeta(const Scalar& beta_rad) { return CppAD::cos(beta_rad); }

/// @brief Sine of the sideslip angle.
/// @tparam Scalar Numeric scalar type.
/// @param beta_rad Sideslip angle [rad].
/// @return sin(beta_rad).
template <class Scalar>
inline Scalar SinBeta(const Scalar& beta_rad) { return CppAD::sin(beta_rad); }

} // namespace Aetherion::Aerodynamics

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
// - No `if` branches; “safety” uses smooth eps terms.
// - Works for double, float, CppAD::AD<double>, etc.
//

#pragma once

#include <array>
#include <cppad/cppad.hpp>

#include "Aetherion/Aerodynamics/Math.h"

namespace Aetherion::Aerodynamics {

    template <class Scalar>
    struct AeroAngles {
        Scalar alpha_rad; // angle of attack
        Scalar beta_rad;  // sideslip
        Scalar speed_m_s; // |v|
        Scalar u;         // convenience: v_body[0]
        Scalar v;         // convenience: v_body[1]
        Scalar w;         // convenience: v_body[2]
    };

    // Compute alpha/beta/speed from body-frame air-relative velocity.
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

    template <class Scalar>
    inline Scalar CosAlpha(const Scalar& alpha_rad) { return CppAD::cos(alpha_rad); }

    template <class Scalar>
    inline Scalar SinAlpha(const Scalar& alpha_rad) { return CppAD::sin(alpha_rad); }

    template <class Scalar>
    inline Scalar CosBeta(const Scalar& beta_rad) { return CppAD::cos(beta_rad); }

    template <class Scalar>
    inline Scalar SinBeta(const Scalar& beta_rad) { return CppAD::sin(beta_rad); }

} // namespace Aetherion::Aerodynamics

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// AerodynamicForce.h
//
// CppAD-friendly aerodynamic force utilities.
//
// Conventions (typical aerospace, body frame):
// - Body axes: +x forward, +y right, +z down.
// - v_body is air-relative velocity expressed in body axes.
// - alpha, beta from AerodynamicAngles.h:
//     alpha = atan2(w, u)
//     beta  = atan2(v, sqrt(u^2 + w^2))
//
// Two common coefficient conventions are supported:
//
// 1) Wind-axis coefficients (CL, CD, CY):
//      D = q S CD,  L = q S CL,  Y = q S CY
//    Wind-axis force vector (magnitudes D,L,Y positive):
//      F_w = [ -D, +Y, -L ]
//    Output is body-frame force: F_b = C_b_w * F_w
//
// 2) Body-axis coefficients (Cx, Cy, Cz):
//      F_b = q S [ Cx, Cy, Cz ]
//
// Notes:
// - No `if` branches; intended for use with CppAD::AD<...>.
// - Uses CppAD math explicitly to avoid MSVC overload ambiguity.
//

#pragma once

#include <array>
#include <cppad/cppad.hpp>

#include <Aetherion/Aerodynamics/AerodynamicAngles.h>

namespace Aetherion::Aerodynamics {

    template <class Scalar>
    using Mat3 = std::array<std::array<Scalar, 3>, 3>;

    template <class Scalar>
    inline Vec3<Scalar> MatVecMul(const Mat3<Scalar>& A, const Vec3<Scalar>& x)
    {
        return Vec3<Scalar>{
            A[0][0] * x[0] + A[0][1] * x[1] + A[0][2] * x[2],
                A[1][0] * x[0] + A[1][1] * x[1] + A[1][2] * x[2],
                A[2][0] * x[0] + A[2][1] * x[1] + A[2][2] * x[2],
        };
    }

    // Dynamic pressure: q = 0.5 * rho * V^2
    template <class Scalar>
    inline Scalar DynamicPressure(const Scalar& density_kg_m3, const Scalar& speed_m_s)
    {
        return Scalar(0.5) * density_kg_m3 * speed_m_s * speed_m_s;
    }

    // Wind->Body DCM (C_b_w) consistent with alpha,beta definitions in AerodynamicAngles.h.
    // We use: C_b_w = R_y(alpha) * R_z(beta)
    // such that: v_body = C_b_w * v_wind.
    template <class Scalar>
    inline Mat3<Scalar> WindToBodyDCM(const Scalar& alpha_rad, const Scalar& beta_rad)
    {
        const Scalar ca = CppAD::cos(alpha_rad);
        const Scalar sa = CppAD::sin(alpha_rad);
        const Scalar cb = CppAD::cos(beta_rad);
        const Scalar sb = CppAD::sin(beta_rad);

        return Mat3<Scalar>{{
            {{ ca* cb, -ca * sb, -sa }},
            { { sb,       cb,       Scalar(0) } },
            { { sa * cb, -sa * sb,  ca } }
            }};
    }

    // --- Forces from wind-axis coefficients (CL, CD, CY) --------------------------

    // Provide alpha/beta explicitly.
    template <class Scalar>
    inline Vec3<Scalar> AerodynamicForceBodyFromCLCDCY_AlphaBeta(
        const Scalar& alpha_rad,
        const Scalar& beta_rad,
        const Scalar& density_kg_m3,
        const Scalar& speed_m_s,
        const Scalar& S_ref_m2,
        const Scalar& CL,
        const Scalar& CD,
        const Scalar& CY)
    {
        const Scalar q = DynamicPressure(density_kg_m3, speed_m_s);

        const Scalar D = q * S_ref_m2 * CD;
        const Scalar L = q * S_ref_m2 * CL;
        const Scalar Y = q * S_ref_m2 * CY;

        const Vec3<Scalar> F_w{ -D, Y, -L };

        const Mat3<Scalar> C_b_w = WindToBodyDCM(alpha_rad, beta_rad);
        return MatVecMul(C_b_w, F_w);
    }

    // Infer alpha/beta/speed from v_body.
    template <class Scalar>
    inline Vec3<Scalar> AerodynamicForceBodyFromCLCDCY(
        const Vec3<Scalar>& v_body_m_s,
        const Scalar& density_kg_m3,
        const Scalar& S_ref_m2,
        const Scalar& CL,
        const Scalar& CD,
        const Scalar& CY,
        const Scalar& eps = Scalar(1e-12))
    {
        const auto ang = AnglesFromVelocityBody(v_body_m_s, eps);
        return AerodynamicForceBodyFromCLCDCY_AlphaBeta(
            ang.alpha_rad, ang.beta_rad, density_kg_m3, ang.speed_m_s, S_ref_m2, CL, CD, CY);
    }

    // --- Forces from body-axis coefficients (Cx, Cy, Cz) --------------------------
    //
    // F_b = q S [Cx, Cy, Cz]
    template <class Scalar>
    inline Vec3<Scalar> AerodynamicForceBodyFromCxyz(
        const Vec3<Scalar>& v_body_m_s,
        const Scalar& density_kg_m3,
        const Scalar& S_ref_m2,
        const Scalar& Cx,
        const Scalar& Cy,
        const Scalar& Cz,
        const Scalar& eps = Scalar(1e-12))
    {
        const Scalar V = SpeedFromVelocity(v_body_m_s, eps);
        const Scalar q = DynamicPressure(density_kg_m3, V);

        return Vec3<Scalar>{
            q* S_ref_m2* Cx,
                q* S_ref_m2* Cy,
                q* S_ref_m2* Cz
        };
    }

} // namespace Aetherion::Aerodynamics

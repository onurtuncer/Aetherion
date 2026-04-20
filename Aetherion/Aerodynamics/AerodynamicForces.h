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

#include "Aetherion/Aerodynamics/Math.h"
#include <Aetherion/Aerodynamics/AerodynamicAngles.h>

namespace Aetherion::Aerodynamics {

/// @brief Direction-cosine matrix (DCM) rotating from wind axes to body axes: @f$ C_{b/w} @f$.
///
/// Constructed as @f$ C_{b/w} = R_y(\alpha) \cdot R_z(\beta) @f$, consistent with the
/// angle-of-attack and sideslip definitions in AerodynamicAngles.h.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param alpha_rad Angle of attack [rad].
/// @param beta_rad Sideslip angle [rad].
/// @return 3×3 DCM @f$ C_{b/w} @f$ such that @f$ v_\text{body} = C_{b/w}\, v_\text{wind} @f$.
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

/// @brief Aerodynamic force in body axes computed from wind-axis coefficients, given pre-computed alpha/beta.
///
/// Computes the dynamic pressure @f$ q = \tfrac{1}{2}\rho V^2 @f$, forms the wind-axis force
/// vector @f$ F_w = [-q S\,C_D,\; q S\,C_Y,\; -q S\,C_L] @f$, then rotates it to body axes
/// via WindToBodyDCM().
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param alpha_rad Angle of attack [rad].
/// @param beta_rad Sideslip angle [rad].
/// @param density_kg_m3 Air density [kg/m³].
/// @param speed_m_s Airspeed magnitude [m/s].
/// @param S_ref_m2 Aerodynamic reference area [m²].
/// @param CL Lift coefficient (dimensionless).
/// @param CD Drag coefficient (dimensionless).
/// @param CY Side-force coefficient (dimensionless).
/// @return Aerodynamic force vector expressed in body axes [N].
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

/// @brief Aerodynamic force in body axes computed from wind-axis coefficients, inferring alpha/beta from the velocity vector.
///
/// Calls AnglesFromVelocityBody() to derive alpha, beta, and speed from @p v_body_m_s, then
/// delegates to AerodynamicForceBodyFromCLCDCY_AlphaBeta().
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param v_body_m_s Air-relative velocity vector in body axes [m/s].
/// @param density_kg_m3 Air density [kg/m³].
/// @param S_ref_m2 Aerodynamic reference area [m²].
/// @param CL Lift coefficient (dimensionless).
/// @param CD Drag coefficient (dimensionless).
/// @param CY Side-force coefficient (dimensionless).
/// @param eps Smoothing parameter for angle and speed computations; default 1e-12.
/// @return Aerodynamic force vector expressed in body axes [N].
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

/// @brief Aerodynamic force in body axes computed directly from body-axis force coefficients.
///
/// Computes @f$ F_b = q\,S\,[C_x,\,C_y,\,C_z]^\top @f$ where @f$ q = \tfrac{1}{2}\rho V^2 @f$.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param v_body_m_s Air-relative velocity vector in body axes [m/s]; used to compute airspeed.
/// @param density_kg_m3 Air density [kg/m³].
/// @param S_ref_m2 Aerodynamic reference area [m²].
/// @param Cx Body-axis x force coefficient (dimensionless).
/// @param Cy Body-axis y force coefficient (dimensionless).
/// @param Cz Body-axis z force coefficient (dimensionless).
/// @param eps Smoothing parameter for speed computation; default 1e-12.
/// @return Aerodynamic force vector expressed in body axes [N].
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

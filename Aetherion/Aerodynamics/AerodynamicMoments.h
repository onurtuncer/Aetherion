// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// AerodynamicMoment.h
//
// CppAD-friendly aerodynamic moment utilities.
//
// Conventions (body frame):
// - Body axes: +x forward, +y right, +z down.
// - Moments are returned in body axes: [L, M, N] = [roll, pitch, yaw] in N*m.
//
// Two common ways to compute aerodynamic moments are supported:
//
// (1) From nondimensional moment coefficients (Cl, Cm, Cn):
//     L = q * S * b     * Cl
//     M = q * S * cbar  * Cm
//     N = q * S * b     * Cn
//
// (2) From an aerodynamic force applied at a center-of-pressure (CP) offset:
//
//     M = r_cp_cg × F
//
// where r_cp_cg is CP position relative to CG in body axes [m],
// and F is the aerodynamic force in body axes [N].
//
// Notes:
// - No `if` branches; intended for use with CppAD::AD<...>.
// - Uses the same Vec3 and SpeedFromVelocity helpers as AerodynamicAngles.h.
// - Uses explicit CppAD math to avoid MSVC overload ambiguity.
//

#pragma once

#include <array>
#include <cppad/cppad.hpp>

#include "Aetherion/Aerodynamics/Math.h"
#include "Aetherion/Aerodynamics/AerodynamicAngles.h"

namespace Aetherion::Aerodynamics {

/// @brief Aerodynamic moment in body axes from nondimensional moment coefficients, inferring speed from velocity.
///
/// Computes @f$ [L, M, N] = q S [b\,C_l,\; \bar{c}\,C_m,\; b\,C_n] @f$ where
/// @f$ q = \tfrac{1}{2}\rho V^2 @f$ and @f$ V = \|v_\text{body}\| @f$.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param v_body_m_s Air-relative velocity vector in body axes [m/s]; used to compute dynamic pressure.
/// @param density_kg_m3 Air density [kg/m³].
/// @param S_ref_m2 Aerodynamic reference area [m²].
/// @param b_ref_m Reference span used for roll and yaw moments [m].
/// @param cbar_ref_m Mean aerodynamic chord used for pitch moment [m].
/// @param Cl Nondimensional roll moment coefficient.
/// @param Cm Nondimensional pitch moment coefficient.
/// @param Cn Nondimensional yaw moment coefficient.
/// @param eps Smoothing parameter for the speed norm; default 1e-12.
/// @return Aerodynamic moment vector [L, M, N] in body axes [N·m].
template <class Scalar>
inline Vec3<Scalar> AerodynamicMomentBodyFromClCmCn(
    const Vec3<Scalar>& v_body_m_s,
    const Scalar& density_kg_m3,
    const Scalar& S_ref_m2,
    const Scalar& b_ref_m,
    const Scalar& cbar_ref_m,
    const Scalar& Cl,
    const Scalar& Cm,
    const Scalar& Cn,
    const Scalar& eps = Scalar(1e-12))
{
    const Scalar V = SpeedFromVelocity(v_body_m_s, eps);
    const Scalar q = DynamicPressure(density_kg_m3, V);

    const Scalar L = q * S_ref_m2 * b_ref_m * Cl;
    const Scalar M = q * S_ref_m2 * cbar_ref_m * Cm;
    const Scalar N = q * S_ref_m2 * b_ref_m * Cn;

    return Vec3<Scalar>{ L, M, N };
}

/// @brief Aerodynamic moment in body axes from nondimensional moment coefficients, using a pre-computed airspeed.
///
/// Avoids recomputing the speed norm when it has already been derived (e.g. from AnglesFromVelocityBody()).
/// Otherwise equivalent to AerodynamicMomentBodyFromClCmCn().
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param speed_m_s Pre-computed airspeed magnitude [m/s].
/// @param density_kg_m3 Air density [kg/m³].
/// @param S_ref_m2 Aerodynamic reference area [m²].
/// @param b_ref_m Reference span [m].
/// @param cbar_ref_m Mean aerodynamic chord [m].
/// @param Cl Nondimensional roll moment coefficient.
/// @param Cm Nondimensional pitch moment coefficient.
/// @param Cn Nondimensional yaw moment coefficient.
/// @return Aerodynamic moment vector [L, M, N] in body axes [N·m].
template <class Scalar>
inline Vec3<Scalar> AerodynamicMomentBodyFromClCmCn_Speed(
    const Scalar& speed_m_s,
    const Scalar& density_kg_m3,
    const Scalar& S_ref_m2,
    const Scalar& b_ref_m,
    const Scalar& cbar_ref_m,
    const Scalar& Cl,
    const Scalar& Cm,
    const Scalar& Cn)
{
    const Scalar q = DynamicPressure(density_kg_m3, speed_m_s);

    const Scalar L = q * S_ref_m2 * b_ref_m * Cl;
    const Scalar M = q * S_ref_m2 * cbar_ref_m * Cm;
    const Scalar N = q * S_ref_m2 * b_ref_m * Cn;

    return Vec3<Scalar>{ L, M, N };
}

/// @brief Aerodynamic moment in body axes from a center-of-pressure offset and a body-frame force.
///
/// Computes the moment arm cross product: @f$ M = r_\text{CP/CG} \times F_\text{body} @f$.
/// @tparam Scalar Numeric scalar type (e.g. double, CppAD::AD<double>).
/// @param r_cp_minus_cg_m Position of the center of pressure relative to the center of gravity, in body axes [m].
/// @param F_body_N Aerodynamic force vector in body axes [N].
/// @return Aerodynamic moment vector in body axes [N·m].
template <class Scalar>
inline Vec3<Scalar> AerodynamicMomentBodyFromCPForce(
    const Vec3<Scalar>& r_cp_minus_cg_m,
    const Vec3<Scalar>& F_body_N)
{
    return Cross(r_cp_minus_cg_m, F_body_N);
}

} // namespace Aetherion::Aerodynamics

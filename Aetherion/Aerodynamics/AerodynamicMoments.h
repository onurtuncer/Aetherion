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

#include <Aetherion/Aerodynamics/AerodynamicAngles.h>

namespace Aetherion::Aerodynamics {

    template <class Scalar>
    inline Vec3<Scalar> Cross(const Vec3<Scalar>& a, const Vec3<Scalar>& b)
    {
        return Vec3<Scalar>{
            a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
        };
    }

    // Dynamic pressure: q = 0.5 * rho * V^2
    template <class Scalar>
    inline Scalar DynamicPressure(const Scalar& density_kg_m3, const Scalar& speed_m_s)
    {
        return Scalar(0.5) * density_kg_m3 * speed_m_s * speed_m_s;
    }

    // --- Moments from nondimensional moment coefficients --------------------------
    //
    // Inputs:
    // - density_kg_m3: air density [kg/m^3]
    // - v_body_m_s: air-relative velocity in body axes [m/s]
    // - S_ref_m2: reference area [m^2]
    // - b_ref_m: reference span [m]
    // - cbar_ref_m: reference mean aerodynamic chord [m]
    // - Cl, Cm, Cn: nondimensional roll/pitch/yaw moment coefficients
    //
    // Output:
    // - M_body_Nm = [L, M, N] [N*m]
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

    // If you already have speed computed (e.g., from AnglesFromVelocityBody), use this:
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

    // --- Moments from CP offset and body force -----------------------------------
    //
    // M = r_cp_cg × F_body
    template <class Scalar>
    inline Vec3<Scalar> AerodynamicMomentBodyFromCPForce(
        const Vec3<Scalar>& r_cp_minus_cg_m,
        const Vec3<Scalar>& F_body_N)
    {
        return Cross(r_cp_minus_cg_m, F_body_N);
    }

} // namespace Aetherion::Aerodynamics

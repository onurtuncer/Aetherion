// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// AerodynamicWrench.h
//
// CppAD-friendly aerodynamic wrench utilities.
//
// A "wrench" is a force + moment pair, both expressed in body axes.
//
// Conventions (body frame):
// - Body axes: +x forward, +y right, +z down.
// - Force:  F_body_N  [N]
// - Moment: M_body_Nm [N*m] = [L, M, N] = [roll, pitch, yaw]
//
// This header provides convenience functions to build an aerodynamic wrench from:
//
// (1) Coefficient-based force (CL, CD, CY) + coefficient-based moment (Cl, Cm, Cn)
// (2) Coefficient-based force (CL, CD, CY) + CP offset moment (r_cp_minus_cg × F)
//
// Notes:
// - No `if` branches; suitable for CppAD::AD<...>.
// - Depends on AerodynamicForce.h and AerodynamicMoment.h.
//

#pragma once

#include "Aetherion/Aerodynamics/AerodynamicForces.h"
#include "Aetherion/Aerodynamics/AerodynamicMoments.h"

namespace Aetherion::Aerodynamics {

    template <class Scalar>
    struct AeroWrench
    {
        Vec3<Scalar> F_body_N{};
        Vec3<Scalar> M_body_Nm{};
    };

    // (1) Wrench from force coeffs (CL,CD,CY) and moment coeffs (Cl,Cm,Cn).
    template <class Scalar>
    inline AeroWrench<Scalar> AerodynamicWrenchBodyFromCoefficients(
        const Vec3<Scalar>& v_body_m_s,
        const Scalar& density_kg_m3,
        const Scalar& S_ref_m2,
        const Scalar& b_ref_m,
        const Scalar& cbar_ref_m,
        const Scalar& CL,
        const Scalar& CD,
        const Scalar& CY,
        const Scalar& Cl,
        const Scalar& Cm,
        const Scalar& Cn,
        const Scalar& eps = Scalar(1e-12))
    {
        AeroWrench<Scalar> w{};

        w.F_body_N = AerodynamicForceBodyFromCLCDCY(
            v_body_m_s, density_kg_m3, S_ref_m2, CL, CD, CY, eps);

        w.M_body_Nm = AerodynamicMomentBodyFromClCmCn(
            v_body_m_s, density_kg_m3, S_ref_m2, b_ref_m, cbar_ref_m, Cl, Cm, Cn, eps);

        return w;
    }

    // (2) Wrench from force coeffs (CL,CD,CY) and CP offset moment (r x F).
    template <class Scalar>
    inline AeroWrench<Scalar> AerodynamicWrenchBodyFromCoefficientsAndCP(
        const Vec3<Scalar>& v_body_m_s,
        const Scalar& density_kg_m3,
        const Scalar& S_ref_m2,
        const Scalar& CL,
        const Scalar& CD,
        const Scalar& CY,
        const Vec3<Scalar>& r_cp_minus_cg_m,
        const Scalar& eps = Scalar(1e-12))
    {
        AeroWrench<Scalar> w{};

        w.F_body_N = AerodynamicForceBodyFromCLCDCY(
            v_body_m_s, density_kg_m3, S_ref_m2, CL, CD, CY, eps);

        w.M_body_Nm = AerodynamicMomentBodyFromCPForce(r_cp_minus_cg_m, w.F_body_N);

        return w;
    }

} // namespace Aetherion::Aerodynamics

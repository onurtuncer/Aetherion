// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// RocketGravityPolicy.h
//
// J₂ gravity policy extended with the moment-transfer correction that accounts
// for the CG–MRC offset of the two-stage rocket.
//
// The NASA TM aero model computes moments about the Moment Reference Centre
// (MRC), but gravity acts at the Centre of Gravity (CG), which is DXCG metres
// forward of the MRC along the body +x axis.  The Newton-Euler equations are
// integrated about the MRC, so gravity must contribute an additional moment:
//
//   M_gravity_MRC = r_CG × F_grav
//
// where r_CG = [xcg_m, 0, 0]^T (body frame) gives:
//   Mx = 0                    (axisymmetric: no roll moment)
//   My = −xcg_m × Fz_body
//   Mz = +xcg_m × Fy_body
//
// xcg_m (= DXCG) is a mutable public member updated each step from the inertia
// DML output (zero-order hold, consistent with other ZOH state updates).
//
// Satisfies GravityPolicy for both S = double and S = CppAD::AD<double>.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>
#include <Aetherion/Environment/Gravity.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Spatial/Wrench.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Dense>

namespace Aetherion::Examples::TwoStageRocket {

/// @brief J₂ gravity policy with CG–MRC moment-transfer correction.
///
/// Identical to @c J2GravityPolicy for the force component, but also
/// produces a pitching/yawing moment proportional to the CG offset from
/// the moment reference centre (MRC):
///   @f$ M_y = -x_{CG}\,F_z,\quad M_z = +x_{CG}\,F_y @f$
///
/// @c xcg_m must be updated to @c DXCG before every integration step (ZOH).
struct RocketGravityPolicy {
    double mu   { Environment::WGS84::kGM_m3_s2 };        ///< Gravitational parameter [m³/s²].
    double Re   { Environment::WGS84::kSemiMajorAxis_m };  ///< Earth equatorial radius [m].
    double J2   { Environment::WGS84::kJ2 };               ///< J₂ zonal harmonic [-].
    double xcg_m{ 0.0 };  ///< CG forward of MRC in body +x [m]; updated per step (ZOH).

    template<class S>
    Spatial::Wrench<S>
    operator()(const ODE::RKMK::Lie::SE3<S>& g, S mass) const
    {
        // J2 gravitational acceleration in ECI frame
        const Environment::Vec3<S> r{ g.p(0), g.p(1), g.p(2) };
        const auto g_arr = Environment::J2(r, S(mu), S(Re), S(J2));

        // Rotate ECI gravity force to body frame
        const Eigen::Matrix<S, 3, 1> F_body =
            g.R.transpose().template cast<S>() *
            Eigen::Matrix<S, 3, 1>{ g_arr[0] * mass,
                                    g_arr[1] * mass,
                                    g_arr[2] * mass };

        Spatial::Wrench<S> w{};
        w.f.setZero();
        w.f.template tail<3>() = F_body;

        // CG moment transfer to MRC:
        //   r_CG = [xcg_m, 0, 0]^T  →  M_MRC = r_CG × F
        //   Mx = 0 (axisymmetric), My = -xcg*Fz, Mz = +xcg*Fy
        w.f(1) -= S(xcg_m) * F_body(2);   // My = -xcg · Fz
        w.f(2) += S(xcg_m) * F_body(1);   // Mz = +xcg · Fy

        return w;
    }
};

static_assert(FlightDynamics::GravityPolicy<RocketGravityPolicy>);

} // namespace Aetherion::Examples::TwoStageRocket

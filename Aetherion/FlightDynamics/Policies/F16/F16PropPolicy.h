// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16PropPolicy.h
//
// Propulsion wrench policy backed by the F-16 DAVE-ML propulsion model.
//
// At each call:
//   1. Derives atmosphere-relative TAS in the body frame.
//   2. Computes geometric altitude and Mach number from the US 1976 atmosphere.
//   3. Queries DAVEMLPropModel::evaluate<S> at the stored throttle setting.
//   4. Returns a body-frame wrench with thrust forces (N) and moments (N·m).
//
// The throttle (pwr_pct [0–100]) is stored as a member and may be updated
// between integration steps by a controller.
//
// Satisfies PropulsionPolicy (alias of AeroPolicy) for S = double and
// S = CppAD::AD<double>.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Serialization/DAVEML/DAVEMLPropModel.h>
#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Environment/GeometricAltitude.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Environment/detail/MathWrappers.h>
#include <Aetherion/Spatial/Wrench.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Dense>
#include <memory>

namespace Aetherion::FlightDynamics {

/// @brief Propulsion policy for the F-16, driven by a DAVE-ML prop model.
class F16PropPolicy
{
public:
    //TODO [Onur]: Move this constant to a more general unit conversion header.
    static constexpr double kFt_m           = 0.3048;

    double pwr_pct{ 0.0 };  ///< Throttle [0–100 %]

    // ── Construction ──────────────────────────────────────────────────────────

    explicit F16PropPolicy(std::shared_ptr<const Serialization::DAVEMLPropModel> model,
                           double throttle = 0.0)
        : m_model(std::move(model)), pwr_pct(throttle) {}

    // ── PropulsionPolicy interface ────────────────────────────────────────────

    template<class S>
    Spatial::Wrench<S>
    operator()(const ODE::RKMK::Lie::SE3<S>& g,
               const Eigen::Matrix<S, 6, 1>& nu_B,
               S /*mass*/, S /*t*/) const
    {
        using Environment::detail::SquareRoot;

        // ── Atmosphere-relative TAS ───────────────────────────────────────────
        constexpr double kOmegaE = Environment::WGS84::kRotationRate_rad_s;
        const Eigen::Matrix<S, 3, 1> omega_E(S(0), S(0), S(kOmegaE));
        const Eigen::Matrix<S, 3, 1> v_surface = g.R.transpose() * omega_E.cross(g.p);
        const Eigen::Matrix<S, 3, 1> v_rel = nu_B.template tail<3>() - v_surface;
        const S vt_mps = SquareRoot(v_rel.squaredNorm() + S(1.0e-30));

        // ── Geometric altitude (ft) and Mach ─────────────────────────────────
        const S alt_m  = Environment::GeometricAltitude_m(g.p);
        const S alt_ft = alt_m / S(kFt_m);
        const S a_mps  = Environment::US1976Atmosphere(alt_m).a;
        const S mach   = vt_mps / a_mps;

        // ── Evaluate propulsion model ─────────────────────────────────────────
        Serialization::DAVEMLPropModel::Inputs<S> in{};
        in.pwr_pct = S(pwr_pct);
        in.alt_ft  = alt_ft;
        in.mach    = mach;
        const auto out = m_model->evaluate<S>(in);

        // ── Build wrench [Mx, My, Mz, Fx, Fy, Fz] (N, N·m) ──────────────────
        Spatial::Wrench<S> wrench{};
        wrench.f.setZero();
        wrench.f(0) = out.mx_Nm;  // rolling  thrust moment [N·m]
        wrench.f(1) = out.my_Nm;  // pitching thrust moment [N·m]
        wrench.f(2) = out.mz_Nm;  // yawing   thrust moment [N·m]
        wrench.f(3) = out.fx_N;   // body X (forward) thrust [N]
        wrench.f(4) = out.fy_N;   // body Y thrust [N]
        wrench.f(5) = out.fz_N;   // body Z thrust [N]
        return wrench;
    }

private:
    std::shared_ptr<const Serialization::DAVEMLPropModel> m_model;
};

static_assert(PropulsionPolicy<F16PropPolicy>);

} // namespace Aetherion::FlightDynamics

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// RocketAeroPolicy.h
//
// Aerodynamic wrench policy backed by the two-stage rocket DAVE-ML aero model
// (twostage_aero.dml).
//
// The DML model expects:
//   Inputs : alpha [deg], beta [deg]
//   Outputs: totalCoefficientOfLift (CL), totalCoefficientOfDrag (CD),
//            aeroBodyForceCoefficient_Y (CY),
//            aeroBodyMomentCoefficient_Pitch (Cm),
//            aeroBodyMomentCoefficient_Yaw  (Cn)
//            aeroBodyMomentCoefficient_Roll  = 0 (axisymmetric)
//
// Reference geometry (from DML constants): Sref = 7 m², cbar = 3 m, bspan = 3 m.
//
// Force transform — wind to body (body +x = nose/thrust axis, +z = down):
//   R_BW = Ry(alpha) * Rz(beta)
//   F_wind = [−CD, CY, −CL]^T × qS
//   F_body = R_BW * F_wind
//
// Moment outputs (about MRC, already correct — no transfer needed here):
//   Mx = 0
//   My = Cm × qS × cbar
//   Mz = Cn × qS × bspan
//
// Satisfies AeroPolicy for both S = double and S = CppAD::AD<double>.
// The DAVE-ML evaluator (evaluateRaw<S>) handles both scalar types.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>
#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Environment/detail/MathWrappers.h>
#include <Aetherion/Spatial/Wrench.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Dense>
#include <memory>
#include <numbers>
#include <unordered_map>
#include <string>

namespace Aetherion::Examples::TwoStageRocket {

/// @brief Aerodynamic wrench policy for the NASA two-stage rocket DML model.
///
/// Computes CL, CD, CY, Cm, Cn from the DAVE-ML lookup tables at the current
/// alpha/beta angles and applies the wind-to-body rotation to yield body-frame
/// forces and moments about the MRC.
///
/// All quantities in SI (N, N·m). Dynamic pressure uses US 1976 density at
/// geocentric altitude.
class RocketAeroPolicy
{
public:
    // Reference geometry (from twostage_aero.dml constants)
    static constexpr double kSref_m2  = 7.0;  ///< Reference area [m²].
    static constexpr double kCbar_m   = 3.0;  ///< Longitudinal reference length [m].
    static constexpr double kBspan_m  = 3.0;  ///< Lateral reference length [m].

    // ── Construction ──────────────────────────────────────────────────────────

    RocketAeroPolicy() = default;

    explicit RocketAeroPolicy(
        std::shared_ptr<const Serialization::DAVEMLAeroModel> model)
        : m_model(std::move(model))
    {}

    // ── AeroPolicy interface ──────────────────────────────────────────────────

    template<class S>
    Spatial::Wrench<S>
    operator()(const ODE::RKMK::Lie::SE3<S>& g,
               const Eigen::Matrix<S, 6, 1>& nu_B,
               S /*mass*/, S /*t*/) const
    {
        using Environment::detail::SquareRoot;
        using Environment::detail::ArcTangent;
        using Environment::detail::ArcSine;
        using Environment::detail::Sine;
        using Environment::detail::Cosine;

        constexpr double kOmegaE = Environment::WGS84::kRotationRate_rad_s;
        constexpr double kPi     = std::numbers::pi;

        // ── Atmosphere-relative velocity in body frame ────────────────────────
        const Eigen::Matrix<S, 3, 1> omega_E(S(0), S(0), S(kOmegaE));
        const Eigen::Matrix<S, 3, 1> v_surface = g.R.transpose() * omega_E.cross(g.p);
        const Eigen::Matrix<S, 3, 1> v_B   = nu_B.template tail<3>();
        const Eigen::Matrix<S, 3, 1> v_rel = v_B - v_surface;

        const S u   = v_rel(0);
        const S v   = v_rel(1);
        const S w   = v_rel(2);
        const S vt  = SquareRoot(v_rel.squaredNorm() + S(1.0e-30));

        // ── Aerodynamic angles [deg] ──────────────────────────────────────────
        const S alpha_deg = ArcTangent(w / (u + S(1.0e-12)))
                          * S(180.0 / kPi);
        const S beta_deg  = ArcSine(v / vt)
                          * S(180.0 / kPi);

        // ── Geometric altitude above WGS-84 ellipsoid ────────────────────────
        // Approximate: use geocentric latitude to get the ellipsoid surface
        // radius at the current position, then subtract from the ECI radius.
        // Correct at all latitudes (equatorial approx.: r - a gives ~21 km
        // error at the poles which would badly misplace the atmosphere layer).
        constexpr double a   = Environment::WGS84::kSemiMajorAxis_m;
        constexpr double b   = a * (1.0 - Environment::WGS84::kFlattening);
        constexpr double a2  = a * a;
        constexpr double b2  = b * b;
        const S r        = SquareRoot(g.p.squaredNorm() + S(1.0e-30));
        const S sin_gc   = g.p(2) / r;                   // sin(geocentric lat)
        const S cos2_gc  = S(1) - sin_gc * sin_gc;       // cos²(geocentric lat)
        // Ellipsoid surface radius: r_s = a·b / sqrt(b²cos²φ + a²sin²φ)
        const S r_surf   = SquareRoot(S(a2 * b2) /
            (S(b2) * cos2_gc + S(a2) * sin_gc * sin_gc + S(1.0e-30)));
        const S alt      = r - r_surf;
        const S rho      = Environment::US1976Atmosphere(alt).rho;
        const S qbar = S(0.5) * rho * vt * vt;
        const S qS   = qbar * S(kSref_m2);

        // ── Evaluate DAVE-ML aero tables ──────────────────────────────────────
        if (!m_model) { return Spatial::Wrench<S>{}; }

        std::unordered_map<std::string, S> vars;
        vars["alpha"] = alpha_deg;
        vars["beta"]  = beta_deg;
        const auto c = m_model->evaluateRaw<S>(std::move(vars));

        auto get = [&](const char* id) -> S {
            const auto it = c.find(id);
            return (it != c.end()) ? it->second : S(0);
        };

        const S CL = get("totalCoefficientOfLift");
        const S CD = get("totalCoefficientOfDrag");
        const S CY = get("aeroBodyForceCoefficient_Y");
        const S Cm = get("aeroBodyMomentCoefficient_Pitch");
        const S Cn = get("aeroBodyMomentCoefficient_Yaw");

        // ── Wind-to-body force transform: R_BW = Ry(alpha) * Rz(beta) ─────────
        // F_wind = [−CD, CY, −CL]^T × qS
        // F_body = R_BW * F_wind:
        //   Fx = (−CD·ca·cb − CY·ca·sb + CL·sa) × qS
        //   Fy = (−CD·sb    + CY·cb            ) × qS
        //   Fz = (−CD·sa·cb − CY·sa·sb − CL·ca ) × qS
        const S alpha_rad = alpha_deg * S(kPi / 180.0);
        const S beta_rad  = beta_deg  * S(kPi / 180.0);
        const S sa = Sine(alpha_rad),  ca = Cosine(alpha_rad);
        const S sb = Sine(beta_rad),   cb = Cosine(beta_rad);

        const S Fx = (-CD*ca*cb - CY*ca*sb + CL*sa) * qS;
        const S Fy = (-CD*sb    + CY*cb            ) * qS;
        const S Fz = (-CD*sa*cb - CY*sa*sb - CL*ca ) * qS;

        // ── Assemble wrench [Mx, My, Mz, Fx, Fy, Fz] ─────────────────────────
        Spatial::Wrench<S> wrench{};
        wrench.f.setZero();
        wrench.f(0) = S(0);                          // Mx = 0 (axisymmetric body)
        wrench.f(1) = Cm * qS * S(kCbar_m);          // My = Cm × qS × c̄
        wrench.f(2) = Cn * qS * S(kBspan_m);         // Mz = Cn × qS × b
        wrench.f(3) = Fx;
        wrench.f(4) = Fy;
        wrench.f(5) = Fz;
        return wrench;
    }

private:
    std::shared_ptr<const Serialization::DAVEMLAeroModel> m_model;
};

static_assert(FlightDynamics::AeroPolicy<RocketAeroPolicy>);

} // namespace Aetherion::Examples::TwoStageRocket

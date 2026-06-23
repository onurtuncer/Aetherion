// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16AeroPolicy.h
//
// Aerodynamic wrench policy backed by the F-16 DAVE-ML aero model.
//
// Computes alpha, beta and TAS from the body-frame atmosphere-relative velocity,
// then queries DAVEMLAeroModel for the six non-dimensional coefficients
// (CX, CY, CZ, Cl, Cm, Cn).  Dimensional forces and moments are returned in
// SI (N, N·m) consistent with the rest of the Aetherion wrench framework.
//
// Control-surface deflections (el, ail, rdr) are fixed at construction time
// (trim or open-loop use).  A flight-control layer should update them between
// integration steps for closed-loop simulations.
//
// Satisfies AeroPolicy for both S = double and S = CppAD::AD<double>.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>
#include <Aetherion/FlightDynamics/Policies/PolicyConcepts.h>
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Environment/GeometricAltitude.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Environment/detail/MathWrappers.h>
#include <Aetherion/Spatial/Wrench.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Dense>
#include <memory>
#include <cmath>
#include <numbers>

namespace Aetherion::FlightDynamics {

/// @brief Aerodynamic policy for the F-16, driven by a DAVE-ML aero model.
///
/// At each call:
///  1. Derives atmosphere-relative airspeed in the body frame (ECI velocity
///     minus the Earth surface velocity R^T(ω_E × r_ECI)).
///  2. Converts to angle of attack (α), sideslip (β), and TAS in ft/s.
///  3. Calls DAVEMLAeroModel::evaluate\<S\> with the current body rates and
///     fixed control-surface deflections.
///  4. Scales the dimensionless coefficients by qbar × Sref (and span/chord)
///     and converts lbf / ft·lbf → N / N·m.
class F16AeroPolicy
{
public:
    // ── Conversion constants ──────────────────────────────────────────────────
    static constexpr double kFt_m           = 0.3048;
    static constexpr double kSlugFt3_kg_m3  = 515.3788184;   ///< 1 slug/ft³ in kg/m³
    static constexpr double kLbf_N          = 4.448221615260751;
    static constexpr double kFtLbf_Nm       = 1.355817948329279;

    // ── Construction ──────────────────────────────────────────────────────────

    /// @param model        Shared, parsed DAVE-ML aero model.
    /// @param el           Elevator deflection [deg].
    /// @param ail          Aileron  deflection [deg].
    /// @param rdr          Rudder   deflection [deg].
    /// @param xcg_ac_m     CG aft of AC [m].  Must match TrimSolver xcg_from_ac_ft/0.3048.
    explicit F16AeroPolicy(std::shared_ptr<const Serialization::DAVEMLAeroModel> model,
                           double el = 0.0, double ail = 0.0, double rdr = 0.0,
                           double xcg_ac_m = 0.0)
        : m_model(std::move(model))
        , m_elDeg(el), m_ailDeg(ail), m_rdrDeg(rdr)
        , m_xcgFromAcM(xcg_ac_m)
    {}

    // ── Control-surface update (closed-loop flight-control layer) ─────────────

    /// @brief Update the fixed control-surface deflections between integration steps.
    void setControls(double el_deg, double ail_deg, double rdr_deg) noexcept
    {
        m_elDeg  = el_deg;
        m_ailDeg = ail_deg;
        m_rdrDeg = rdr_deg;
    }

    /// @brief CG-aft-of-AC offset [m] used for the AC→CG moment transfer.
    [[nodiscard]] double xcgFromAcM() const noexcept { return m_xcgFromAcM; }

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

        // ── Atmosphere-relative velocity in body frame ────────────────────────
        constexpr double kOmegaE = Environment::WGS84::kRotationRate_rad_s;
        const Eigen::Matrix<S, 3, 1> omega_E(S(0), S(0), S(kOmegaE));
        const Eigen::Matrix<S, 3, 1> v_surface = g.R.transpose() * omega_E.cross(g.p);
        const Eigen::Matrix<S, 3, 1> v_B   = nu_B.template tail<3>();
        const Eigen::Matrix<S, 3, 1> v_rel = v_B - v_surface;

        // ── Aerodynamic angles and TAS ────────────────────────────────────────
        const S& u      = v_rel(0);
        const S& v      = v_rel(1);
        const S& w      = v_rel(2);
        const S vt_mps  = SquareRoot(v_rel.squaredNorm() + S(1.0e-30));
        const S vt_fps  = vt_mps / S(kFt_m);

        // α = atan(w/u)  — valid for forward flight (u > 0)
        const S alpha_deg = ArcTangent(w / (u + S(1.0e-12))) * S(180.0 / std::numbers::pi);
        // β = asin(v/V)
        const S beta_deg  = ArcSine(v / vt_mps) * S(180.0 / std::numbers::pi);

        // ── Body angular rates (ECI-relative ≈ aerodynamic rates) ─────────────
        const Eigen::Matrix<S, 3, 1> omega_B = nu_B.template head<3>();

        // ── Geometric altitude and atmospheric density ────────────────────────
        const S alt_m    = Environment::GeometricAltitude_m(g.p);
        const S rho_slug = Environment::US1976Atmosphere(alt_m).rho / S(kSlugFt3_kg_m3);

        // ── Dynamic pressure [lbf/ft²] ────────────────────────────────────────
        const S qbar_psf = S(0.5) * rho_slug * vt_fps * vt_fps;

        // ── Evaluate DAVE-ML aero model ───────────────────────────────────────
        Serialization::DAVEMLAeroModel::Inputs<S> in{};
        in.vt_fps    = vt_fps;
        in.alpha_deg = alpha_deg;
        in.beta_deg  = beta_deg;
        in.p_rps     = omega_B(0);
        in.q_rps     = omega_B(1);
        in.r_rps     = omega_B(2);
        in.el_deg    = S(m_elDeg);
        in.ail_deg   = S(m_ailDeg);
        in.rdr_deg   = S(m_rdrDeg);
        const auto c = m_model->evaluate<S>(in);

        // ── Reference geometry ────────────────────────────────────────────────
        const double Sref  = m_model->srefFt2();
        const double cbar  = m_model->cbarFt();
        const double bspan = m_model->bspanFt();

        const S qS = qbar_psf * S(Sref);  // [lbf]

        // ── Build wrench: [Mx, My, Mz, Fx, Fy, Fz]  (N, N·m) ─────────────────
        Spatial::Wrench<S> wrench{};
        wrench.f(0) = c.cl * qS * S(bspan) * S(kFtLbf_Nm);  // roll  moment [N·m]
        wrench.f(2) = c.cn * qS * S(bspan) * S(kFtLbf_Nm);  // yaw   moment [N·m]
        wrench.f(3) = c.cx * qS            * S(kLbf_N);      // body X force [N]
        wrench.f(4) = c.cy * qS            * S(kLbf_N);      // body Y force [N]
        wrench.f(5) = c.cz * qS            * S(kLbf_N);      // body Z force [N]
        // Pitch moment about the CG:  My_CG = CM×qS×c̄  +  x_cg_ac × FZ
        wrench.f(1) = c.cm * qS * S(cbar)  * S(kFtLbf_Nm)   // CM×qS×c̄  [N·m]
                    + S(m_xcgFromAcM) * wrench.f(5);          // x_cg_ac × FZ [N·m]
        return wrench;
    }

private:
    std::shared_ptr<const Serialization::DAVEMLAeroModel> m_model;
    double m_elDeg;        ///< Elevator deflection [deg]
    double m_ailDeg;       ///< Aileron  deflection [deg]
    double m_rdrDeg;       ///< Rudder   deflection [deg]

    /// Distance from the aerodynamic moment reference centre (AC) to the CG,
    /// positive aft [m].  The DAVE-ML aero model computes CM about the AC;
    /// this offset transfers that moment to the CG:
    ///   My_CG = CM × qS × c̄ + xcg_from_ac × FZ_aero
    /// For the F-16 standard loading: (35%−25%) × 11.32 ft × 0.3048 = 0.345 m.
    double m_xcgFromAcM;
};

static_assert(AeroPolicy<F16AeroPolicy>);

} // namespace Aetherion::FlightDynamics

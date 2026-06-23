// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TrimSolver.h
//
// Two-stage Newton-Raphson trim solver for straight-and-level fixed-wing flight.
//
// Background — aerodynamic moment reference centre (MRC)
// ───────────────────────────────────────────────────────
// DAVE-ML aero models compute CM about the aerodynamic reference centre (AC),
// typically the wing quarter-chord (25 % MAC).  The equations of motion are
// written about the centre of gravity (CG).  Transferring moments from AC to
// CG adds a correction term:
//
//   My_CG = CM × q_bar × S × c̄   +   x_cg_from_ac × FZ_aero
//
// where x_cg_from_ac = x_CG − x_AC (positive = CG aft of AC).
//
// For the F-16 standard loading (CG at 35 % MAC, AC at 25 % MAC):
//   x_cg_from_ac = (0.35 − 0.25) × 11.32 ft = 1.132 ft  (0.345 m)
//
// Without this correction, the trim solver zero-crosses CM at the wrong alpha
// (~2.35° instead of the correct ~2.64°).  The NASA NESC reference data
// confirms My_total ≈ 23 120 ft·lbf at trim — this is entirely from the
// moment transfer (thrust line passes through the CG for the F-16).
//
// Trim algorithm (two-stage sequential)
// ──────────────────────────────────────
// Stage 1 — find (α, δe) from the longitudinal force and moment balance
//   (no throttle dependence; the aero model only is taped):
//     r_long[0] = CZ × q × S + W·cos(α)·cos(φ)                       = 0  [lbf]
//     r_long[1] = CM × q × S × c̄ + x_cg_ac × CZ × q × S             = 0  [ft·lbf]
//
// Stage 2 — derive required thrust from the X-force balance, then invert
//   the prop model to find the throttle setting:
//     FEX_required = W·sin(α) − CX × q × S                            [lbf]
//     thrustX_lbf(pwr, alt, Mach) = FEX_required   (bisection in pwr)
//
// Jacobian for Stage 1 is computed exactly via CppAD (2×2 tape).
//
//TODO [Onur] use std::numbers for pi; unify unit-conversion constants in a Units namespace.
//TODO [Onur] do not hard-code physical limits; read them from the DAVE-ML file.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLPropModel.h>
#include <Aetherion/Environment/Atmosphere.h>

#include <cppad/cppad.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <vector>

namespace Aetherion::FlightDynamics {

/// @brief Fixed flight condition for the trim problem.
struct TrimInputs {
    double vt_fps    {};   ///< True airspeed [ft/s]
    double alt_ft    {};   ///< Geometric altitude [ft]
    double weight_lbf{};   ///< Aircraft weight at trim [lbf]
    double beta_deg  {0.0};///< Sideslip angle [deg]  (0 for coordinated flight)
    double phi_deg   {0.0};///< Bank angle [deg]       (0 for wings-level)
};

/// @brief Trim solution returned by TrimSolver::solve().
struct TrimPoint {
    double alpha_deg      {};  ///< Angle of attack at trim [deg]
    double el_deg         {};  ///< Elevator deflection at trim [deg]
    double pwr_pct        {};  ///< Throttle at trim [0–100 %]
    double residual_norm  {};  ///< 2-norm of final residual [lbf]
    bool   converged      {false};
};

/// @brief Two-stage trim solver for straight-and-level fixed-wing flight.
class TrimSolver
{
public:
    // ── Physical / unit constants ─────────────────────────────────────────────
    static constexpr double kFt_m          = 0.3048;
    static constexpr double kSlugFt3_kg_m3 = 515.3788184;
    static constexpr double kLbf_N         = 4.448221615260751;
    static constexpr double kFtLbf_Nm      = 1.355817948329279;
    static constexpr double kDegRad        = std::numbers::pi / 180.0;

    // ── Solver parameters ─────────────────────────────────────────────────────
    static constexpr int    kMaxIter    = 50;
    static constexpr double kTol        = 1.0e-5;  ///< Stage-1 convergence [lbf]
    static constexpr double kTolThrust  = 1.0e-4;  ///< Stage-2 convergence [lbf]

    // ── Construction ──────────────────────────────────────────────────────────

    /// @param aero           Parsed aerodynamic model.
    /// @param prop           Parsed propulsion model.
    /// @param xcg_from_ac_ft CG distance aft of the aerodynamic moment reference
    ///                       centre [ft].  Zero ⇒ CM is already about the CG.
    ///                       For the F-16 standard loading: 1.132 ft (35 %−25 % MAC).
    TrimSolver(const Serialization::DAVEMLAeroModel& aero,
               const Serialization::DAVEMLPropModel& prop,
               double xcg_from_ac_ft = 0.0)
        : m_aero(&aero), m_prop(&prop), m_xcg_ft(xcg_from_ac_ft) {}

    // ── Main interface ────────────────────────────────────────────────────────

    /// @brief Find the two-stage trim for straight-and-level flight.
    TrimPoint solve(const TrimInputs& in,
                    double alpha0 = 2.5,
                    double el0    = -1.0) const;

    /// @brief Evaluate the full 3-component residual for diagnosis.
    ///
    /// Returns [r_Fx, r_Fz, r_My_cg] in [lbf, lbf, ft·lbf].
    Eigen::Vector3d residual(const TrimInputs& in,
                             double alpha_deg,
                             double el_deg,
                             double pwr_pct) const;

private:
    // ── Stage-1 longitudinal residual (no throttle, taped for AD) ─────────────
    template<class S>
    std::array<S, 2> longi_residual(const TrimInputs& in,
                                    S alpha_deg, S el_deg,
                                    double qbar_psf) const;

    // ── Stage-2: required thrust from force balance ───────────────────────────
    double requiredThrust_lbf(const TrimInputs& in,
                              double alpha_deg, double el_deg,
                              double qbar_psf) const;

    // ── Stage-2: invert prop model to find throttle ───────────────────────────
    double solveThrottle(double FEX_required_lbf,
                         double alt_ft, double mach) const;

    const Serialization::DAVEMLAeroModel* m_aero;
    const Serialization::DAVEMLPropModel* m_prop;
    double                                m_xcg_ft;  ///< CG aft of AC [ft]
};

// ── Stage-1 longitudinal residual ─────────────────────────────────────────────

template<class S>
std::array<S, 2>
TrimSolver::longi_residual(const TrimInputs& in,
                           S alpha_deg, S el_deg,
                           double qbar_psf) const
{
    using std::cos;

    const double Sref = m_aero->sref_ft2;
    const double cbar = m_aero->cbar_ft;

    Serialization::DAVEMLAeroModel::Inputs<S> ai{};
    ai.vt_fps    = S(in.vt_fps);
    ai.alpha_deg = alpha_deg;
    ai.beta_deg  = S(in.beta_deg);
    ai.p_rps = ai.q_rps = ai.r_rps = S(0.0);
    ai.el_deg  = el_deg;
    ai.ail_deg = ai.rdr_deg = S(0.0);
    const auto c = m_aero->evaluate<S>(ai);

    const S qS      = S(qbar_psf * Sref);
    const S FZ_aero = c.cz * qS;

    // Body-Z force balance (no thrust in Z for aligned engine)
    const S alpha_rad = alpha_deg * S(kDegRad);
    const S phi_rad   = S(in.phi_deg * kDegRad);
    const S rFZ = FZ_aero + S(in.weight_lbf) * cos(alpha_rad) * cos(phi_rad);

    // Pitching moment about the CG:  My_CG = CM×qS×cbar + x_cg_ac × FZ_aero
    const S rMY = c.cm * qS * S(cbar) + S(m_xcg_ft) * FZ_aero;

    return { rFZ, rMY };
}

// ── Stage-2 helpers ───────────────────────────────────────────────────────────

inline double
TrimSolver::requiredThrust_lbf(const TrimInputs& in,
                                double alpha_deg, double el_deg,
                                double qbar_psf) const
{
    const double Sref = m_aero->sref_ft2;

    Serialization::DAVEMLAeroModel::Inputs<double> ai{};
    ai.vt_fps    = in.vt_fps;
    ai.alpha_deg = alpha_deg;
    ai.beta_deg  = in.beta_deg;
    ai.p_rps = ai.q_rps = ai.r_rps = 0.0;
    ai.el_deg = el_deg;
    ai.ail_deg = ai.rdr_deg = 0.0;
    const auto c = m_aero->evaluate<double>(ai);

    const double FX_aero = c.cx * qbar_psf * Sref;
    const double alpha_rad = alpha_deg * kDegRad;
    return in.weight_lbf * std::sin(alpha_rad) - FX_aero;
}

inline double
TrimSolver::solveThrottle(double FEX_required_lbf, double alt_ft, double mach) const
{
    // Bisection on pwr_pct ∈ [0, 100] — thrust is monotone in power.
    double lo = 0.0, hi = 100.0;

    const double thrust_lo = m_prop->thrustX_lbf(lo, alt_ft, mach);
    const double thrust_hi = m_prop->thrustX_lbf(hi, alt_ft, mach);

    if (FEX_required_lbf <= thrust_lo) return lo;
    if (FEX_required_lbf >= thrust_hi) return hi;

    for (int i = 0; i < 60; ++i) {
        const double mid   = 0.5 * (lo + hi);
        const double t_mid = m_prop->thrustX_lbf(mid, alt_ft, mach);
        if (std::abs(t_mid - FEX_required_lbf) < kTolThrust) return mid;
        (t_mid < FEX_required_lbf ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

// ── Main solve ────────────────────────────────────────────────────────────────

inline TrimPoint
TrimSolver::solve(const TrimInputs& in, double alpha0, double el0) const
{
    using AD = CppAD::AD<double>;

    // ── Atmosphere (double, not taped) ────────────────────────────────────────
    const double alt_m    = in.alt_ft * kFt_m;
    const auto   atm      = Environment::US1976Atmosphere(alt_m);
    const double rho_slug = atm.rho / kSlugFt3_kg_m3;
    const double a_fps    = atm.a   / kFt_m;
    const double qbar_psf = 0.5 * rho_slug * in.vt_fps * in.vt_fps;
    const double mach     = in.vt_fps / a_fps;

    // ── Stage 1: 2D Newton for (alpha, el) ───────────────────────────────────
    std::vector<double> x = { alpha0, el0 };
    bool converged = false;

    for (int iter = 0; iter < kMaxIter; ++iter) {
        const auto   r_arr = longi_residual<double>(in, x[0], x[1], qbar_psf);
        const Eigen::Vector2d r{ r_arr[0], r_arr[1] };

        if (r.norm() < kTol) { converged = true; break; }

        // CppAD 2×2 Jacobian
        std::vector<AD> xa = { AD(x[0]), AD(x[1]) };
        CppAD::Independent(xa);
        const auto r_ad = longi_residual<AD>(in, xa[0], xa[1], qbar_psf);
        CppAD::ADFun<double> f(xa, std::vector<AD>{ r_ad[0], r_ad[1] });
        f.optimize();

        const auto jvec = f.Jacobian(x);  // row-major 2×2
        Eigen::Matrix2d J;
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                J(i, j) = jvec[static_cast<std::size_t>(i) * 2 + static_cast<std::size_t>(j)];

        const Eigen::Vector2d dx = J.fullPivLu().solve(-r);
        x[0] = std::clamp(x[0] + dx(0), -10.0, 30.0);  // alpha
        x[1] = std::clamp(x[1] + dx(1), -25.0, 25.0);  // el
    }

    const double alpha = x[0];
    const double el    = x[1];

    // ── Stage 2: required thrust → throttle ───────────────────────────────────
    const double FEX_req = requiredThrust_lbf(in, alpha, el, qbar_psf);
    const double pwr     = solveThrottle(FEX_req, in.alt_ft, mach);

    // ── Final residual ────────────────────────────────────────────────────────
    const auto   r_arr = longi_residual<double>(in, alpha, el, qbar_psf);
    const double r_norm = Eigen::Vector2d(r_arr[0], r_arr[1]).norm();

    return { alpha, el, pwr, r_norm, converged };
}

// ── Full 3-component residual for diagnosis ───────────────────────────────────

inline Eigen::Vector3d
TrimSolver::residual(const TrimInputs& in,
                     double alpha_deg, double el_deg, double pwr_pct) const
{
    const double alt_m    = in.alt_ft * kFt_m;
    const auto   atm      = Environment::US1976Atmosphere(alt_m);
    const double rho_slug = atm.rho / kSlugFt3_kg_m3;
    const double a_fps    = atm.a   / kFt_m;
    const double qbar_psf = 0.5 * rho_slug * in.vt_fps * in.vt_fps;
    const double mach     = in.vt_fps / a_fps;
    const double Sref     = m_aero->sref_ft2;
    const double cbar     = m_aero->cbar_ft;

    Serialization::DAVEMLAeroModel::Inputs<double> ai{};
    ai.vt_fps    = in.vt_fps;
    ai.alpha_deg = alpha_deg;
    ai.beta_deg  = in.beta_deg;
    ai.p_rps = ai.q_rps = ai.r_rps = 0.0;
    ai.el_deg = el_deg;
    ai.ail_deg = ai.rdr_deg = 0.0;
    const auto c = m_aero->evaluate<double>(ai);

    const double qS      = qbar_psf * Sref;
    const double FX_aero = c.cx * qS;
    const double FZ_aero = c.cz * qS;
    const double MY_cg   = c.cm * qS * cbar + m_xcg_ft * FZ_aero;
    const double FEX_lbf = m_prop->thrustX_lbf(pwr_pct, in.alt_ft, mach);

    const double alpha_rad = alpha_deg * kDegRad;
    const double phi_rad   = in.phi_deg * kDegRad;
    const double W         = in.weight_lbf;

    return Eigen::Vector3d{
        FX_aero + FEX_lbf - W * std::sin(alpha_rad),
        FZ_aero + W * std::cos(alpha_rad) * std::cos(phi_rad),
        MY_cg
    };
}

} // namespace Aetherion::FlightDynamics

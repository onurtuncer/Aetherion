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
// Newton-Raphson trim solver for straight-and-level fixed-wing flight.
//
// The trim problem:  find (α, δe, PWR) such that the body-axis force and
// pitching-moment residuals are simultaneously zero at the given flight
// condition (V, h, W).
//
// All internal computations use English units (ft, slug, lbf, ft·lbf) to
// match the F-16 DAVE-ML model directly.  The Jacobian is approximated by
// forward finite differences.  Convergence criterion: 2-norm of the residual
// vector below kTol (lbf).
//
// Residuals (3 equations, 3 unknowns):
//   r[0] = FX_aero + FEX_prop − W·sin(α)   [lbf]  (body-X force balance)
//   r[1] = FZ_aero + W·cos(α)·cos(φ)       [lbf]  (body-Z force balance)
//   r[2] = MY_aero                          [ft·lbf]  (pitch moment balance)
//
// Unknowns:  x = [ α_deg,  δe_deg,  PWR_pct ]
// ------------------------------------------------------------------------------

//TODO [Onur] use AD to compute the Jacobian instead of finite differences for better accuracy and performance.
//TODO [Onur] use std::numbers for pi also I want all unit conversion constants in a single place, maybe a Units namespace or something. I have some in DAVEMLPropModel and some in TrimSolver, should be unified.
//TODO [Onur] do not hard code physical limits in the solver, instead read them from the DAVE-ML file or define them as static constants in the class. This would
//TODO [Onur] make the solver more flexible and reusable for different aircraft models.

#pragma once

#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLPropModel.h>
#include <Aetherion/Environment/Atmosphere.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

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

/// @brief Newton-Raphson trim solver.
///
/// Instantiate once with references to the loaded aerodynamic and propulsion
/// models, then call solve() for any flight condition.
class TrimSolver
{
public:
    // ── Physical / unit constants ─────────────────────────────────────────────
    static constexpr double kFt_m           = 0.3048;
    static constexpr double kSlugFt3_kg_m3  = 515.3788184;
    static constexpr double kLbf_N          = 4.448221615260751;
    static constexpr double kDegRad         = std::numbers::pi / 180.0;

    // ── Solver parameters ─────────────────────────────────────────────────────
    static constexpr int    kMaxIter = 50;
    static constexpr double kTol     = 1.0e-5;  ///< Convergence threshold [lbf]

    // ── Construction ──────────────────────────────────────────────────────────

    TrimSolver(const Serialization::DAVEMLAeroModel& aero,
               const Serialization::DAVEMLPropModel& prop)
        : m_aero(aero), m_prop(prop) {}

    // ── Main interface ────────────────────────────────────────────────────────

    /// @brief Find the trim state for straight-and-level flight.
    ///
    /// @param in      Fixed flight condition (see TrimInputs).
    /// @param alpha0  Initial alpha guess [deg].  Default: 2.0.
    /// @param el0     Initial elevator guess [deg].  Default: −1.0.
    /// @param pwr0    Initial throttle guess [%].  Default: 15.0.
    /// @return        Converged trim point (check @c converged flag).
    TrimPoint solve(const TrimInputs& in,
                    double alpha0 = 2.0,
                    double el0    = -1.0,
                    double pwr0   = 15.0) const;

    /// @brief Evaluate the trim residual at a specific operating point.
    ///
    /// Useful for diagnosis after convergence.
    Eigen::Vector3d residual(const TrimInputs& in,
                             double alpha_deg,
                             double el_deg,
                             double pwr_pct) const;

private:
    const Serialization::DAVEMLAeroModel& m_aero;
    const Serialization::DAVEMLPropModel& m_prop;
};

// ── Inline implementation ─────────────────────────────────────────────────────

inline Eigen::Vector3d
TrimSolver::residual(const TrimInputs& in,
                     double alpha_deg,
                     double el_deg,
                     double pwr_pct) const
{
    // ── Atmosphere ────────────────────────────────────────────────────────────
    const double alt_m     = in.alt_ft * kFt_m;
    const auto   atm       = Environment::US1976Atmosphere(alt_m);
    const double rho_slug  = atm.rho / kSlugFt3_kg_m3;  // [slug/ft³]
    const double a_fps     = atm.a   / kFt_m;            // [ft/s]

    // ── Dynamic pressure and Mach ─────────────────────────────────────────────
    const double qbar_psf = 0.5 * rho_slug * in.vt_fps * in.vt_fps;
    const double mach     = in.vt_fps / a_fps;

    // ── Reference geometry ────────────────────────────────────────────────────
    const double Sref  = m_aero.sref_ft2;
    const double cbar  = m_aero.cbar_ft;

    // ── Aerodynamic coefficients ──────────────────────────────────────────────
    Serialization::DAVEMLAeroModel::Inputs<double> ai{};
    ai.vt_fps    = in.vt_fps;
    ai.alpha_deg = alpha_deg;
    ai.beta_deg  = in.beta_deg;
    ai.p_rps     = 0.0;
    ai.q_rps     = 0.0;
    ai.r_rps     = 0.0;
    ai.el_deg    = el_deg;
    ai.ail_deg   = 0.0;
    ai.rdr_deg   = 0.0;
    const auto c = m_aero.evaluate<double>(ai);

    // ── Dimensional aero forces and pitch moment [lbf, ft·lbf] ───────────────
    const double qS      = qbar_psf * Sref;
    const double FX_aero = c.cx * qS;
    const double FZ_aero = c.cz * qS;
    const double MY_aero = c.cm * qS * cbar;

    // ── Thrust [lbf] ─────────────────────────────────────────────────────────
    const double FEX_lbf = m_prop.thrustX_lbf(pwr_pct, in.alt_ft, mach);

    // ── Body-frame gravity (pitch = alpha for straight-and-level, phi = bank) ─
    const double alpha_rad = alpha_deg * kDegRad;
    const double phi_rad   = in.phi_deg  * kDegRad;
    const double W         = in.weight_lbf;
    const double grav_x    = -W * std::sin(alpha_rad);
    const double grav_z    =  W * std::cos(alpha_rad) * std::cos(phi_rad);

    // ── Residuals ─────────────────────────────────────────────────────────────
    Eigen::Vector3d r;
    r(0) = FX_aero + FEX_lbf + grav_x;  // body-X force [lbf]
    r(1) = FZ_aero + grav_z;             // body-Z force [lbf]
    r(2) = MY_aero;                      // pitch moment [ft·lbf]
    return r;
}

inline TrimPoint
TrimSolver::solve(const TrimInputs& in,
                  double alpha0, double el0, double pwr0) const
{
    // Finite-difference step sizes
    constexpr double h_alpha = 1.0e-4;  // [deg]
    constexpr double h_el    = 1.0e-4;  // [deg]
    constexpr double h_pwr   = 1.0e-3;  // [%]

    double alpha = alpha0;
    double el    = el0;
    double pwr   = pwr0;

    bool converged = false;

    for (int iter = 0; iter < kMaxIter; ++iter) {
        const Eigen::Vector3d r = residual(in, alpha, el, pwr);

        if (r.norm() < kTol) {
            converged = true;
            break;
        }

        // ── Forward-difference Jacobian ───────────────────────────────────────
        const Eigen::Vector3d ra = residual(in, alpha + h_alpha, el, pwr);
        const Eigen::Vector3d re = residual(in, alpha, el + h_el, pwr);
        const Eigen::Vector3d rp = residual(in, alpha, el, pwr + h_pwr);

        Eigen::Matrix3d J;
        J.col(0) = (ra - r) / h_alpha;
        J.col(1) = (re - r) / h_el;
        J.col(2) = (rp - r) / h_pwr;

        // ── Newton step ───────────────────────────────────────────────────────
        const Eigen::Vector3d dx = J.fullPivLu().solve(-r);

        alpha += dx(0);
        el    += dx(1);
        pwr   += dx(2);

        // Clamp to physical limits
        alpha = std::clamp(alpha, -10.0,  30.0);
        el    = std::clamp(el,    -25.0,  25.0);
        pwr   = std::clamp(pwr,     0.0, 100.0);
    }

    const Eigen::Vector3d r_final = residual(in, alpha, el, pwr);
    return { alpha, el, pwr, r_final.norm(), converged };
}

} // namespace Aetherion::FlightDynamics

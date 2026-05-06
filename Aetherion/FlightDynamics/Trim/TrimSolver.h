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
// match the F-16 DAVE-ML model directly.
//
// Residuals (3 equations, 3 unknowns):
//   r[0] = FX_aero + FEX_prop − W·sin(α)   [lbf]  (body-X force balance)
//   r[1] = FZ_aero + W·cos(α)·cos(φ)       [lbf]  (body-Z force balance)
//   r[2] = MY_aero                          [ft·lbf]  (pitch moment balance)
//
// Unknowns:  x = [ α_deg,  δe_deg,  PWR_pct ]
//
// The Jacobian is computed exactly using CppAD automatic differentiation.
// This gives correct first-order derivatives everywhere except at breakpoint
// boundaries of the DAVE-ML lookup tables (where the index selection is
// treated as a constant via CppAD::Var2Par).
//
//TODO [Onur] use std::numbers for pi also I want all unit conversion constants in a single place, maybe a Units namespace or something. I have some in DAVEMLPropModel and some in TrimSolver, should be unified.
//TODO [Onur] do not hard code physical limits in the solver, instead read them from the DAVE-ML file or define them as static constants in the class. This would make the solver more flexible and reusable for different aircraft models.
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
    double vt_fps      {};   ///< True airspeed [ft/s]
    double alt_ft      {};   ///< Geometric altitude [ft]
    double weight_lbf  {};   ///< Aircraft weight at trim [lbf]
    double beta_deg    {0.0};///< Sideslip angle [deg]  (0 for coordinated flight)
    double phi_deg     {0.0};///< Bank angle [deg]       (0 for wings-level)
};

/// @brief Trim solution returned by TrimSolver::solve().
struct TrimPoint {
    double alpha_deg      {};  ///< Angle of attack at trim [deg]
    double el_deg         {};  ///< Elevator deflection at trim [deg]
    double pwr_pct        {};  ///< Throttle at trim [0–100 %]
    double residual_norm  {};  ///< 2-norm of final residual [lbf]
    bool   converged      {false};
};

/// @brief Newton-Raphson trim solver with CppAD Jacobian.
///
/// Instantiate once with references to the loaded aerodynamic and propulsion
/// models, then call solve() for any flight condition.
class TrimSolver
{
public:
    // ── Physical / unit constants ─────────────────────────────────────────────
    static constexpr double kFt_m          = 0.3048;
    static constexpr double kSlugFt3_kg_m3 = 515.3788184;
    static constexpr double kLbf_N         = 4.448221615260751;
    static constexpr double kDegRad        = std::numbers::pi / 180.0;

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
    /// The Jacobian at each Newton iteration is computed exactly via CppAD
    /// automatic differentiation through the DAVE-ML lookup tables.
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

    /// @brief Evaluate the double-precision trim residual at an operating point.
    ///
    /// Returns [r_Fx, r_Fz, r_My] in [lbf, lbf, ft·lbf].  Useful for
    /// post-convergence diagnosis.
    Eigen::Vector3d residual(const TrimInputs& in,
                             double alpha_deg,
                             double el_deg,
                             double pwr_pct) const;

private:
    // ── Templated residual — called for both double and CppAD::AD<double> ─────

    /// @brief Compute the 3-component trim residual for scalar type S.
    ///
    /// Altitude, dynamic pressure, and Mach are evaluated at the fixed
    /// (double) flight condition; only (alpha, el, pwr) are of type S.
    /// This allows a CppAD tape to record derivatives w.r.t. the three
    /// trim unknowns without taping through the atmosphere model.
    template<class S>
    std::array<S, 3> residual_impl(const TrimInputs& in,
                                   S alpha_deg,
                                   S el_deg,
                                   S pwr_pct) const;

    const Serialization::DAVEMLAeroModel& m_aero;
    const Serialization::DAVEMLPropModel& m_prop;
};

// ── Template implementation ───────────────────────────────────────────────────

template<class S>
std::array<S, 3>
TrimSolver::residual_impl(const TrimInputs& in,
                          S alpha_deg, S el_deg, S pwr_pct) const
{
    using std::sin;
    using std::cos;

    // ── Atmosphere at the fixed trim altitude (double — not taped) ────────────
    const double alt_m    = in.alt_ft * kFt_m;
    const auto   atm      = Environment::US1976Atmosphere(alt_m);
    const double rho_slug = atm.rho / kSlugFt3_kg_m3;
    const double a_fps    = atm.a   / kFt_m;

    const double qbar_psf = 0.5 * rho_slug * in.vt_fps * in.vt_fps;
    const double mach     = in.vt_fps / a_fps;

    const double Sref = m_aero.sref_ft2;
    const double cbar = m_aero.cbar_ft;

    // ── Aerodynamic coefficients — taped w.r.t. alpha and el ─────────────────
    Serialization::DAVEMLAeroModel::Inputs<S> ai{};
    ai.vt_fps    = S(in.vt_fps);
    ai.alpha_deg = alpha_deg;
    ai.beta_deg  = S(in.beta_deg);
    ai.p_rps     = S(0.0);
    ai.q_rps     = S(0.0);
    ai.r_rps     = S(0.0);
    ai.el_deg    = el_deg;
    ai.ail_deg   = S(0.0);
    ai.rdr_deg   = S(0.0);
    const auto c = m_aero.evaluate<S>(ai);

    const S qS      = S(qbar_psf * Sref);
    const S FX_aero = c.cx * qS;
    const S FZ_aero = c.cz * qS;
    const S MY_aero = c.cm * qS * S(cbar);

    // ── Thrust — taped w.r.t. pwr ────────────────────────────────────────────
    Serialization::DAVEMLPropModel::Inputs<S> pi{};
    pi.pwr_pct = pwr_pct;
    pi.alt_ft  = S(in.alt_ft);
    pi.mach    = S(mach);
    const auto prop     = m_prop.evaluate<S>(pi);
    const S FEX_lbf     = prop.fx_N / S(kLbf_N);
    // TEM from the DAVE-ML propulsion file (= 0 for the simplified S&L model).
    // The geometry-driven engine moment arm (My = z_engine × Fx) lives in
    // F16PropPolicy::z_engine_m and is not visible here; it is intentionally
    // excluded because it is an aircraft property, not a flight condition.
    const S MY_prop_ftlbf = prop.my_Nm / S(1.355817948329279);

    // ── Body-frame gravity ────────────────────────────────────────────────────
    const S alpha_rad = alpha_deg * S(kDegRad);
    const S W         = S(in.weight_lbf);
    const S grav_x    = -W * sin(alpha_rad);
    const S grav_z    =  W * cos(alpha_rad) * S(std::cos(in.phi_deg * kDegRad));

    return { FX_aero + FEX_lbf + grav_x,
             FZ_aero + grav_z,
             MY_aero + MY_prop_ftlbf };
}

// ── Inline double wrapper ─────────────────────────────────────────────────────

inline Eigen::Vector3d
TrimSolver::residual(const TrimInputs& in,
                     double alpha_deg, double el_deg, double pwr_pct) const
{
    const auto r = residual_impl<double>(in, alpha_deg, el_deg, pwr_pct);
    return Eigen::Vector3d{ r[0], r[1], r[2] };
}

// ── Newton-Raphson solve with CppAD Jacobian ──────────────────────────────────

inline TrimPoint
TrimSolver::solve(const TrimInputs& in,
                  double alpha0, double el0, double pwr0) const
{
    using AD = CppAD::AD<double>;

    std::vector<double> x = { alpha0, el0, pwr0 };
    bool converged = false;

    for (int iter = 0; iter < kMaxIter; ++iter) {
        // ── Double residual for convergence check ────────────────────────────
        const auto   r_arr  = residual_impl<double>(in, x[0], x[1], x[2]);
        const Eigen::Vector3d r{ r_arr[0], r_arr[1], r_arr[2] };

        if (r.norm() < kTol) {
            converged = true;
            break;
        }

        // ── Record CppAD tape for exact Jacobian ──────────────────────────────
        std::vector<AD> xa(x.begin(), x.end());
        CppAD::Independent(xa);

        const auto r_ad = residual_impl<AD>(in, xa[0], xa[1], xa[2]);
        const std::vector<AD> ya{ r_ad[0], r_ad[1], r_ad[2] };

        CppAD::ADFun<double> f(xa, ya);
        f.optimize();

        // ── Jacobian evaluated at current x ───────────────────────────────────
        const std::vector<double> jvec = f.Jacobian(x);  // row-major, 3×3

        Eigen::Matrix3d J;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                J(i, j) = jvec[static_cast<std::size_t>(i * 3 + j)];

        // ── Newton step ───────────────────────────────────────────────────────
        const Eigen::Vector3d dx = J.fullPivLu().solve(-r);

        x[0] = std::clamp(x[0] + dx(0), -10.0,  30.0);  // alpha [deg]
        x[1] = std::clamp(x[1] + dx(1), -25.0,  25.0);  // el    [deg]
        x[2] = std::clamp(x[2] + dx(2),   0.0, 100.0);  // pwr   [%]
    }

    const auto r_f = residual_impl<double>(in, x[0], x[1], x[2]);
    const double norm = Eigen::Vector3d(r_f[0], r_f[1], r_f[2]).norm();
    return { x[0], x[1], x[2], norm, converged };
}

} // namespace Aetherion::FlightDynamics

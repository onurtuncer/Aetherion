// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// File: Aetherion/ODE/RKMK/Core/NewtonOptions.h
//
// Options for damped Newton iterations used in implicit RK stage solves.
// (We keep tolerances as double: convergence decisions should be numeric.)
//
#pragma once

#include <cstdint>

namespace Aetherion::ODE::RKMK::Core {

/// @brief Tuning parameters for the damped Newton solver used in implicit Runge-Kutta stage equations.
///
/// Convergence is declared when either the residual criterion or the step-stagnation criterion
/// (if enabled) is satisfied:
/// - **Residual criterion**: @f$ \|r\| \leq \texttt{abs\_tol} + \texttt{rel\_tol} \cdot \|r_0\| @f$
/// - **Stagnation criterion**: @f$ \|\Delta x\| \leq \texttt{step\_tol}(1 + \|x\|) @f$
///   AND the residual is within a factor of 10 of the residual threshold.
///
/// All convergence thresholds are stored as `double` because convergence decisions must be
/// made in full precision regardless of the integrator's scalar type.
struct NewtonOptions final {

    /// @name Iteration limits
    ///@{
    int max_iters = 20; ///< Maximum number of Newton iterations before declaring failure.
    ///@}

    /// @name Convergence thresholds
    ///@{
    double abs_tol  = 1e-12; ///< Absolute residual tolerance; accept if @f$ \|r\| \leq \texttt{abs\_tol} + \texttt{rel\_tol}\|r_0\| @f$.
    double rel_tol  = 1e-10; ///< Relative residual tolerance (fraction of the initial residual).
    double step_tol = 1e-12; ///< Step-stagnation tolerance; set to 0 to disable the stagnation check.
    ///@}

    /// @name Damping / line search
    ///@{
    bool   use_damping          = true; ///< Enable the backtracking line search: @f$ x_{k+1} = x_k + \lambda\,\Delta x @f$.
    double damping_init         = 1.0;  ///< Initial step length @f$ \lambda @f$.
    double damping_min          = 1e-4; ///< Smallest allowed @f$ \lambda @f$; iteration fails if @f$ \lambda @f$ would go below this.
    double damping_shrink       = 0.5;  ///< Multiplicative reduction applied to @f$ \lambda @f$ on each backtrack: @f$ \lambda \mathrel{*}= \texttt{damping\_shrink} @f$.
    int    max_line_search_iters= 10;   ///< Maximum number of backtracking steps per Newton iteration.
    double sufficient_decrease  = 0.9;  ///< Accept a damped step if @f$ \|r_\text{new}\| \leq \texttt{sufficient\_decrease} \cdot \|r_\text{old}\| @f$.
    ///@}

    /// @name Failure behaviour
    ///@{
    bool throw_on_fail = false; ///< If `true`, throw `std::runtime_error` when the solver does not converge within `max_iters`; if `false`, return silently with the last iterate.
    ///@}
};

/// @brief Checks whether the Newton iteration has converged given the current residual and step norms.
///
/// Evaluates the residual criterion and, if `opt.step_tol > 0`, the stagnation criterion.
/// @param r_norm  Current residual norm @f$ \|r_k\| @f$.
/// @param r0_norm Initial residual norm @f$ \|r_0\| @f$ (used for the relative threshold).
/// @param dx_norm Current Newton step norm @f$ \|\Delta x_k\| @f$.
/// @param x_norm  Current iterate norm @f$ \|x_k\| @f$ (used for the stagnation threshold).
/// @param opt     NewtonOptions containing all tolerance and flag settings.
/// @return `true` if the residual or stagnation criterion is satisfied; `false` otherwise.
[[nodiscard]] inline bool newton_converged(double r_norm,
    double r0_norm,
    double dx_norm,
    double x_norm,
    const NewtonOptions& opt) noexcept {
    const double r_thresh = opt.abs_tol + opt.rel_tol * r0_norm;
    if (r_norm <= r_thresh) return true;

    // Stagnation stop: only if residual is already "close-ish"
    if (opt.step_tol > 0.0) {
        const double dx_thresh = opt.step_tol * (1.0 + x_norm);
        if (dx_norm <= dx_thresh && r_norm <= 10.0 * r_thresh) return true;
    }
    return false;
}


} // namespace Aetherion::ODE::RKMK::Core

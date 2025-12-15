// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
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

    struct NewtonOptions final {
        // --- iteration limits ---
        int max_iters = 20;

        // --- convergence thresholds ---
        // Accept if ||r|| <= abs_tol + rel_tol * ||r0||
        double abs_tol = 1e-12;
        double rel_tol = 1e-10;

        // Optional secondary stop: accept if ||dx|| <= step_tol * (1 + ||x||)
        double step_tol = 1e-12;

        // --- damping / line search ---
        // We typically do: x_{k+1} = x_k + lambda * dx
        bool   use_damping = true;
        double damping_init = 1.0;     // initial lambda
        double damping_min = 1e-4;    // smallest allowed lambda
        double damping_shrink = 0.5;   // lambda *= damping_shrink until residual decreases

        int max_line_search_iters = 10;

        // Require sufficient decrease (simple heuristic)
        double sufficient_decrease = 0.9; // accept if ||r_new|| <= sufficient_decrease * ||r_old||

        // --- failure behavior ---
        bool throw_on_fail = false;
    };

    // Tiny helper used by Newton loops (header-only, no Eigen dependency here).
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

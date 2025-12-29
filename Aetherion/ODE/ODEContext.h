// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::ODE
{
    struct ODEContext {
        // pointers/refs to environment, params, etc.
        void* user = nullptr;
    };

    using RHS = std::function<void(double t,
        const Eigen::VectorXd& x,
        Eigen::VectorXd& dxdt,
        const ODEContext& ctx)>;

    struct StepResult {
        double t_next{};
        Eigen::VectorXd x_next;
        bool ok{ true };
    };

}
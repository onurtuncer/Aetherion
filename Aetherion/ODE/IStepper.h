// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::ODE {

    using JacobianFn = std::function<void(double t,
        const Eigen::VectorXd& x,
        Eigen::MatrixXd& J,
        const ODEContext& ctx)>;

    class IStepper {
    public:
        virtual ~IStepper() = default;

        virtual StepResult step(double t,
            const Eigen::VectorXd& x,
            double h,
            const RHS& f,
            const ODEContext& ctx,
            const JacobianFn* dfdx = nullptr) = 0;
    };


} // namespace Aetherion::ODE
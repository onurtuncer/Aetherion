// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// RigidBody6DoFStepper.h
//
// Thin facade combining KinematicsXiField + RigidBodyVectorField into a single
// step() call on SE(3) x R^7.
//
// Usage:
//   using VF      = RigidBodyVectorField<CentralGravityPolicy>;
//   using Stepper = RigidBody6DoFStepper<VF>;
//
//   Stepper stepper(ip);    // ip = InertialParameters
//   auto res = stepper.step(t0, state, h);
//   if (res.converged) { /* use res.g1, res.x1 */ }
//
#pragma once

#include <Aetherion/FlightDynamics/RigidBodyState.h>
#include <Aetherion/FlightDynamics/RigidBodyVectorField.h>
#include <Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_ProductSE3.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>

namespace Aetherion::FlightDynamics {

    template<class VectorField>
    class RigidBody6DoFStepper {
    public:
        static constexpr int EuclidDim = 7;

        using Integrator = ODE::RKMK::Integrators::RadauIIA_RKMK_ProductSE3<
            KinematicsXiField, VectorField, EuclidDim>;

        using StepResult = typename Integrator::StepResult;
        using SE3d = ODE::RKMK::Lie::SE3<double>;
        using VecE = Eigen::Matrix<double, EuclidDim, 1>;

        // Construct from InertialParameters (and any policy arguments
        // forwarded through to VectorField).
        explicit RigidBody6DoFStepper(VectorField vf,
            ODE::RKMK::Core::NewtonOptions opt = {})
            : integrator_(KinematicsXiField{}, std::move(vf))
            , opt_(opt)
        {
        }

        // Convenience constructor: build VF from InertialParameters directly.
        // Only compiles when VectorField is default-constructible from InertialParameters.
        explicit RigidBody6DoFStepper(const InertialParameters& ip,
            ODE::RKMK::Core::NewtonOptions opt = {})
            : integrator_(KinematicsXiField{}, VectorField(ip))
            , opt_(opt)
        {
        }

        // step(t0, state, h) -> StepResult{g1, x1, converged, ...}
        [[nodiscard]]
        StepResult step(double t0, const RigidBodyStateD& s, double h) const
        {
            VecE x0;
            x0.template head<6>() = s.nu_B;
            x0(6) = s.m;
            return integrator_.step(t0, s.g, x0, h, opt_);
        }

        // Unpack StepResult back into RigidBodyStateD.
        [[nodiscard]] static RigidBodyStateD unpack(const StepResult& r)
        {
            RigidBodyStateD s;
            s.g = r.g1;
            s.nu_B = r.x1.template head<6>();
            s.m = r.x1(6);
            return s;
        }

    private:
        Integrator  integrator_;
        ODE::RKMK::Core::NewtonOptions opt_;
    };

} // namespace Aetherion::FlightDynamics
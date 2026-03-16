// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// SixDoFStepper.h
//
// Thin facade combining KinematicsXiField + RigidBodyVectorField into a single
// step() call on SE(3) x R^7.
//
// All template parameters are fully constrained via C++20 concepts defined in
// Aetherion/ODE/RKMK/Concepts.h -- compiler errors surface at the instantiation
// site rather than deep inside the integrator internals.
//
// Usage:
//   using VF      = RigidBodyVectorField<CentralGravityPolicy>;
//   using Stepper = SixDoFStepper<VF>;
//
//   Stepper stepper(ip);
//   auto res = stepper.step(t0, s, h);
//   if (res.converged) {
//       auto s1 = SixDoFStepper<VF>::unpack(res);
//   }
//
#pragma once

#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/FlightDynamics/KinematicsXiField.h>
#include <Aetherion/ODE/RKMK/Concepts.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_ProductSE3.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Core>

namespace Aetherion::RigidBody {

    // --------------------------------------------------------------------------
    // EuclidDim -- layout of x in R^7:
    //   x[0..5] = nu_B   body-frame twist [omega(3); v(3)]
    //   x[6]    = m      current mass
    // --------------------------------------------------------------------------
    inline constexpr int RigidBody6DoFEuclidDim = 7;

    // --------------------------------------------------------------------------
    // SixDoFStepper<VectorField>
    //
    // VectorField must model VectorFieldOnProductSE3<VF, 7, double>.
    // KinematicsXiField<double> is the fixed kinematic model -- not a template
    // parameter. For a different kinematic model, derive a new stepper.
    // --------------------------------------------------------------------------
    template<class VectorField>
        requires
    ODE::RKMK::KinematicsFieldOnSE3
        <FlightDynamics::KinematicsXiField, double > &&
        ODE::RKMK::VectorFieldOnProductSE3<VectorField, 7, double>
        class SixDoFStepper
    {
    public:
        // ------------------------------------------------------------------
        // Public type aliases
        // ------------------------------------------------------------------
        static constexpr int EuclidDim = RigidBody6DoFEuclidDim;

        using KinematicsField = FlightDynamics::KinematicsXiField;
        using SE3d = ODE::RKMK::Lie::SE3<double>;
        using VecE = Eigen::Matrix<double, EuclidDim, 1>;

        using Integrator = ODE::RKMK::Integrators::RadauIIA_RKMK_ProductSE3
            <KinematicsField, VectorField, EuclidDim> ;

        using StepResult = typename Integrator::StepResult;

        // Fires at instantiation time -- before any method is called.
        static_assert(
            ODE::RKMK::RKMKIntegratorOnProductSE3<Integrator, EuclidDim>,
            "RadauIIA_RKMK_ProductSE3 does not satisfy RKMKIntegratorOnProductSE3 "
            "-- check that StepResult exposes { g1, x1, converged } and that "
            "VecE::RowsAtCompileTime == EuclidDim.");

        // ------------------------------------------------------------------
        // Constructors
        // ------------------------------------------------------------------

        // Primary: accepts a fully-constructed VectorField.
        explicit SixDoFStepper(
            VectorField                    vf,
            ODE::RKMK::Core::NewtonOptions opt = {})
            : integrator_(KinematicsField{}, std::move(vf))
            , opt_(opt)
        {
        }

        // Convenience: builds VectorField from InertialParameters directly.
        // Only participates in overload resolution when VF models
        // ConstructibleFromInertialParameters.
        explicit SixDoFStepper(
            const InertialParameters& ip,
            ODE::RKMK::Core::NewtonOptions opt = {})
            requires ODE::RKMK::ConstructibleFromInertialParameters<VectorField>
        : integrator_(KinematicsField{}, VectorField(ip))
            , opt_(opt)
        {
        }

        // ------------------------------------------------------------------
        // step()
        // ------------------------------------------------------------------
        [[nodiscard]]
        StepResult step(double t0, const StateD& s, double h) const
        {
            return integrator_.step(t0, s.g, pack(s), h, opt_);
        }

        // ------------------------------------------------------------------
        // pack() / unpack()
        // ------------------------------------------------------------------
        [[nodiscard]]
        static VecE pack(const StateD& s) noexcept
        {
            VecE x;
            x.template head<6>() = s.nu_B;
            x(6) = s.m;
            return x;
        }

        [[nodiscard]]
        static StateD unpack(const SE3d& g, const VecE& x) noexcept
        {
            StateD s;
            s.g = g;
            s.nu_B = x.template head<6>();
            s.m = x(6);
            return s;
        }

        [[nodiscard]]
        static StateD unpack(const StepResult& r) noexcept
        {
            return unpack(r.g1, r.x1);
        }

        // ------------------------------------------------------------------
        // Newton options -- readable and writable for per-phase tuning
        // ------------------------------------------------------------------
        [[nodiscard]]
        const ODE::RKMK::Core::NewtonOptions& options() const noexcept
        {
            return opt_;
        }

        ODE::RKMK::Core::NewtonOptions& options() noexcept
        {
            return opt_;
        }

    private:
        Integrator                     integrator_;
        ODE::RKMK::Core::NewtonOptions opt_;
    };

} // namespace Aetherion::RigidBody
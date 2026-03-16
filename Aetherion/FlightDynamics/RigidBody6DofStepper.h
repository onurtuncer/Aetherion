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
// All template parameters are fully constrained via C++20 concepts defined in
// Aetherion/ODE/RKMK/Concepts.h — compiler errors surface at the instantiation
// site rather than deep inside the integrator internals.
//
// Usage:
//   using VF      = RigidBodyVectorField<CentralGravityPolicy>;
//   using Stepper = RigidBody6DoFStepper<VF>;
//
//   Stepper stepper(ip);               // ip = InertialParameters
//   auto res = stepper.step(t0, s, h);
//   if (res.converged) {
//       auto s1 = Stepper::unpack(res);
//   }
//
#pragma once

#include <Aetherion/FlightDynamics/InertialParameters.h>
#include <Aetherion/FlightDynamics/RigidBodyState.h>
#include <Aetherion/FlightDynamics/RigidBodyVectorField.h>
#include <Aetherion/FlightDynamics/KinematicsXiField.h>
#include <Aetherion/ODE/RKMK/Concepts.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_ProductSE3.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Core>

namespace Aetherion::FlightDynamics {

    // ------------------------------------------------------------------------------
    // EuclidDim
    //
    // Layout of the Euclidean state vector x in R^7:
    //
    //   x[0..5]  =  nu_B   body-frame twist  [omega_x, omega_y, omega_z, v_x, v_y, v_z]
    //   x[6]     =  m      current mass
    //
    // Changing this constant (e.g. to 13 for fuel-slosh states) is the only edit
    // needed to extend the Euclidean state — the concepts and integrator propagate
    // the dimension automatically.
    // ------------------------------------------------------------------------------
    inline constexpr int RigidBody6DoFEuclidDim = 7;

    // ------------------------------------------------------------------------------
    // RigidBody6DoFStepper<VF>
    //
    // Template parameters
    // -------------------
    //   VectorField   models VectorFieldOnProductSE3<VF, RigidBody6DoFEuclidDim>
    //                 i.e. callable as: vf(t, g, x) -> Matrix<double, 7, 1>
    //
    // The KinematicsXiField is not a template parameter — it is the canonical
    // body-frame twist kinematic model for this stepper.  If you need a different
    // kinematic model (dual quaternion, quaternion+R3) derive a new stepper rather
    // than patching this one.
    // ------------------------------------------------------------------------------
    template<class VectorField>
        requires
        ODE::RKMK::KinematicsFieldOnSE3<KinematicsXiField<Scalar>, Scalar>&&
        ODE::RKMK::VectorFieldOnProductSE3<VectorField, 7, Scalar>
        class RigidBody6DoFStepper
    {
    public:
        // ------------------------------------------------------------------
        // Public type aliases
        // ------------------------------------------------------------------
        static constexpr int EuclidDim = RigidBody6DoFEuclidDim;

        using KinematicsField = KinematicsXiField;
        using SE3d = ODE::RKMK::Lie::SE3<double>;
        using VecE = Eigen::Matrix<double, EuclidDim, 1>;

        using Integrator = ODE::RKMK::Integrators::RadauIIA_RKMK_ProductSE3
            KinematicsField, VectorField, EuclidDim > ;

        using StepResult = typename Integrator::StepResult;

        // Confirm the assembled integrator satisfies the integrator concept.
        // This fires at class instantiation time — before any method is called.
        static_assert(
            ODE::RKMK::RKMKIntegratorOnProductSE3<Integrator, EuclidDim>,
            "RadauIIA_RKMK_ProductSE3 does not satisfy RKMKIntegratorOnProductSE3 "
            "— check that StepResult exposes { g1, x1, converged } and that "
            "VecE::RowsAtCompileTime == EuclidDim.");

        // ------------------------------------------------------------------
        // Constructors
        // ------------------------------------------------------------------

        // Primary constructor: accepts a fully-constructed VectorField.
        // Use this when the policy requires arguments beyond InertialParameters
        // (e.g. atmosphere tables, thrust curves, aerodynamic databases).
        explicit RigidBody6DoFStepper(
            VectorField                    vf,
            ODE::RKMK::Core::NewtonOptions opt = {})
            : integrator_(KinematicsField{}, std::move(vf))
            , opt_(opt)
        {
        }

        // Convenience constructor: builds VectorField directly from InertialParameters.
        // Only participates in overload resolution when VF models
        // ConstructibleFromInertialParameters — SFINAE-free thanks to the concept.
        explicit RigidBody6DoFStepper(
            const InertialParameters& ip,
            ODE::RKMK::Core::NewtonOptions opt = {})
            requires ODE::RKMK::ConstructibleFromInertialParameters<VectorField>
        : integrator_(KinematicsField{}, VectorField(ip))
            , opt_(opt)
        {
        }

        // ------------------------------------------------------------------
        // step()
        //
        // Advances the rigid-body state by one time step h using Radau IIA.
        //
        // Parameters
        //   t0  -- current time (seconds)
        //   s   -- current state { g ∈ SE(3), nu_B ∈ R^6, m ∈ R }
        //   h   -- step size (seconds); caller owns adaptation / rejection
        //
        // Returns
        //   StepResult { g1, x1, converged, [iterations, residual] }
        //   g1 is guaranteed to remain on the SE(3) manifold.
        //   If !converged the Newton solve diverged — caller should reduce h.
        // ------------------------------------------------------------------
        [[nodiscard]]
        StepResult step(double t0, const RigidBodyStateD& s, double h) const
        {
            return integrator_.step(t0, s.g, pack(s), h, opt_);
        }

        // ------------------------------------------------------------------
        // pack() / unpack()
        //
        // Convert between RigidBodyStateD and the flat Euclidean vector x
        // used internally by the integrator.
        //
        // pack   : RigidBodyStateD  -->  VecE   (called inside step())
        // unpack : StepResult       -->  RigidBodyStateD
        //
        // Both are static so call-sites can use them without a stepper instance.
        // ------------------------------------------------------------------
        [[nodiscard]]
        static VecE pack(const RigidBodyStateD& s) noexcept
        {
            VecE x;
            x.template head<6>() = s.nu_B;
            x(6) = s.m;
            return x;
        }

        [[nodiscard]]
        static RigidBodyStateD unpack(const SE3d& g, const VecE& x) noexcept
        {
            RigidBodyStateD s;
            s.g = g;
            s.nu_B = x.template head<6>();
            s.m = x(6);
            return s;
        }

        // Convenience overload: unpack directly from a StepResult.
        [[nodiscard]]
        static RigidBodyStateD unpack(const StepResult& r) noexcept
        {
            return unpack(r.g1, r.x1);
        }

        // ------------------------------------------------------------------
        // Accessors
        // ------------------------------------------------------------------

        // Expose Newton options for inspection or per-phase tuning.
        [[nodiscard]] const ODE::RKMK::Core::NewtonOptions& options() const noexcept
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

} // namespace Aetherion::FlightDynamics

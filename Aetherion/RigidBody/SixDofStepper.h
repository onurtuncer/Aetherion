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
/// @brief High-level 6-DoF integrator facade for rigid-body flight dynamics.
///
/// Combines the @c KinematicsXiField (SE(3) kinematic ODE) with a user-supplied
/// @c VectorField (Newton-Euler ODE) into a single @c step() call that advances
/// the full state @f$ (g, \nu_B, m) \in SE(3) \times \mathbb{R}^7 @f$.
///
/// Internally uses a 3-stage Radau IIA RKMK integrator with implicit Newton solve.
/// The Newton convergence options can be tuned via @c options().
///
/// @tparam VectorField Must satisfy @c VectorFieldOnProductSE3<VF, 7, double>.
///
/// **Typical usage:**
/// @code
/// using VF      = RigidBody::VectorField<CentralGravityPolicy>;
/// using Stepper = RigidBody::SixDoFStepper<VF>;
///
/// Stepper stepper(ip);
/// auto res = stepper.step(t0, state, h);
/// if (res.converged)
///     state = SixDoFStepper<VF>::unpack(res);
/// @endcode
    template<class VectorField>
        requires
    ODE::RKMK::KinematicsFieldOnSE3
        <FlightDynamics::KinematicsXiField, double >&&
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
            <KinematicsField, VectorField, EuclidDim>;

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

        /// @brief Construct from a fully-built @c VectorField.
        /// @param vf   Vector-field instance (moved in).
        /// @param opt  Newton solver options (tolerance, max iterations).
        explicit SixDoFStepper(
            VectorField                    vf,
            ODE::RKMK::Core::NewtonOptions opt = {})
            : vf_(std::move(vf))
            , integrator_(KinematicsField{}, vf_)
            , opt_(opt)
        {
        }

        /// @brief Convenience constructor — builds @c VectorField from @c InertialParameters.
        ///
        /// Only participates in overload resolution when @c VectorField is
        /// constructible from @c InertialParameters (concept-constrained).
        /// @param ip   Inertial parameters.
        /// @param opt  Newton solver options.
        explicit SixDoFStepper(
            const InertialParameters& ip,
            ODE::RKMK::Core::NewtonOptions opt = {})
            requires ODE::RKMK::ConstructibleFromInertialParameters<VectorField>
        : vf_(ip)
            , integrator_(KinematicsField{}, vf_)
            , opt_(opt)
        {
        }

        // ------------------------------------------------------------------
        // step()
        // ------------------------------------------------------------------
        /// @brief Advance the state by one time step using Radau IIA RKMK.
        ///
        /// @param t0  Start of the step [s].
        /// @param s   Current state @f$(g, \nu_B, m)@f$.
        /// @param h   Step size [s] (must be positive).
        /// @return    @c StepResult containing @c g1, @c x1, and @c converged.
        ///            If @c converged is @c false the Newton iteration did not
        ///            reach the requested tolerance; the returned state is the
        ///            last iterate and should be treated with caution.
        [[nodiscard]]
        StepResult step(double t0, const StateD& s, double h) const
        {
            return integrator_.step(t0, s.g, pack(s), h, opt_);
        }

        // ------------------------------------------------------------------
        // pack() / unpack()
        // ------------------------------------------------------------------

        /// @brief Pack a @c StateD into the flat @f$\mathbb{R}^7@f$ Euclidean vector.
        /// @param s  State to pack.
        /// @return   Vector @f$[\nu_B(6);\, m(1)]@f$.
        [[nodiscard]]
        static VecE pack(const StateD& s) noexcept
        {
            VecE x;
            x.template head<6>() = s.nu_B;
            x(6) = s.m;
            return x;
        }

        /// @brief Unpack an SE(3) pose and Euclidean vector into a @c StateD.
        /// @param g  SE(3) pose.
        /// @param x  Euclidean vector @f$[\nu_B(6);\, m(1)]@f$.
        /// @return   Assembled @c StateD.
        [[nodiscard]]
        static StateD unpack(const SE3d& g, const VecE& x) noexcept
        {
            StateD s;
            s.g = g;
            s.nu_B = x.template head<6>();
            s.m = x(6);
            return s;
        }

        /// @brief Unpack a @c StepResult into a @c StateD.
        /// @param r  Result from @c step().
        /// @return   Assembled @c StateD.
        [[nodiscard]]
        static StateD unpack(const StepResult& r) noexcept
        {
            return unpack(r.g1, r.x1);
        }

        // ------------------------------------------------------------------
        // vectorField() -- exposes the VectorField instance so that callers
        // (e.g. ISimulator::vectorField() -> MakeSnapshot1) can access the
        // gravity policy and other sub-policies without duplicating constants.
        // ------------------------------------------------------------------
        [[nodiscard]] const VectorField& vectorField() const noexcept { return vf_; }
        [[nodiscard]] VectorField& vectorField()       noexcept { return vf_; }

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
        // vf_ is stored here (not only inside integrator_) so that
        // vectorField() can return a stable reference without requiring an
        // accessor on the deeper integrator internals.
        VectorField                    vf_;
        Integrator                     integrator_;
        ODE::RKMK::Core::NewtonOptions opt_;
    };

} // namespace Aetherion::RigidBody

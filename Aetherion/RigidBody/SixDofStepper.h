// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// SixDoFStepper.h
//
// Thin facade combining KinematicsXiField + RigidBodyVectorField into a single
// step() call on SE(3) x R^EuclidDim.
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
#include <Aetherion/RigidBody/KinematicsXiField.h>
#include <Aetherion/ODE/RKMK/Concepts.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_ProductSE3.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Core>

namespace Aetherion::RigidBody {

    // --------------------------------------------------------------------------
    // RigidBody6DoFEuclidDim -- default Euclidean state dimension:
    //   x[0..5] = nu_B   body-frame twist [omega(3); v(3)]
    //   x[6]    = m      current mass
    // EuclidDim may be extended (e.g. >= 8) for Kalman-filter augmented states;
    // the base layout [nu_B | m] always occupies the first 7 slots.
    // --------------------------------------------------------------------------
    inline constexpr int RigidBody6DoFEuclidDim = 7;

    // --------------------------------------------------------------------------
    // SixDoFStepper<VectorField, EuclidDim, IntegratorPolicy>
    //
    // VectorField must model VectorFieldOnProductSE3<VF, EuclidDim, double>.
    // KinematicsXiField<double> is the fixed kinematic model -- not a template
    // parameter. For a different kinematic model, derive a new stepper.
    // --------------------------------------------------------------------------
/// @brief High-level 6-DoF integrator facade for rigid-body flight dynamics.
///
/// Combines the @c KinematicsXiField (SE(3) kinematic ODE) with a user-supplied
/// @c VectorField (Newton-Euler ODE) into a single @c step() call that advances
/// the full state @f$ (g, \nu_B, m, \ldots) \in SE(3) \times \mathbb{R}^{\text{EuclidDim}} @f$.
///
/// Internally uses a 3-stage Radau IIA RKMK integrator with implicit Newton solve.
/// The Newton convergence options can be tuned via @c options().
///
/// @tparam VectorField    Must satisfy @c VectorFieldOnProductSE3\<VF, EuclidDim, double\>.
/// @tparam EuclidDim      Euclidean state dimension (default 7: 6 twist + 1 mass).
///                        Must be ≥ 7. Extra components beyond 7 are zero-initialised
///                        by @c pack() and ignored by @c unpack() — reserved for
///                        future augmented-state use (e.g. Kalman-filter bias states).
/// @tparam IntegratorPolicy  Must satisfy @c IntegratorFor\<IP, KinematicsXiField, VF, EuclidDim\>.
///
/// **Typical usage:**
/// @code
/// using VF      = RigidBody::VectorField<CentralGravityPolicy>;
/// using Stepper = RigidBody::SixDoFStepper<VF>;          // EuclidDim = 7
///
/// Stepper stepper(ip);
/// auto res = stepper.step(t0, state, h);
/// if (res.converged)
///     state = SixDoFStepper<VF>::unpack(res);
/// @endcode
    template<class VectorField,
             int EuclidDim = RigidBody6DoFEuclidDim,
             class IntegratorPolicy = ODE::RKMK::Integrators::RadauIIA_RKMK_ProductSE3<
                 KinematicsXiField, VectorField, EuclidDim>>
        requires
            ODE::RKMK::KinematicsFieldOnSE3<KinematicsXiField, double> &&
            ODE::RKMK::VectorFieldOnProductSE3<VectorField, EuclidDim, double> &&
            ODE::RKMK::IntegratorFor<IntegratorPolicy, KinematicsXiField, VectorField, EuclidDim>
    class SixDoFStepper
    {
    public:
        static_assert(EuclidDim >= RigidBody6DoFEuclidDim,
            "EuclidDim must be >= 7 (6 se(3) twist components + 1 scalar mass)");

        // ------------------------------------------------------------------
        // Public type aliases and constants
        // ------------------------------------------------------------------

        // dim(se(3)) == 6; this is a fixed physical constant, not a template parameter.
        static constexpr int kTwistDim = 6;

        using KinematicsField = KinematicsXiField;
        using SE3d            = ODE::RKMK::Lie::SE3<double>;
        using VecE            = Eigen::Matrix<double, EuclidDim, 1>;
        using Integrator      = IntegratorPolicy;
        using StepResult      = typename Integrator::StepResult;

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
            // The Integrator stores f_field_ as a value-copy of vf_ taken at
            // construction time.  If vf_ was mutated since then (closed-loop
            // simulators call mutableVectorField() between steps to update
            // el_deg / pwr_pct), the stored copy is stale.
            // Building a fresh Integrator from the current vf_ ensures every
            // step evaluates the up-to-date policies.  For open-loop
            // simulations vf_ is never mutated, so this is a no-op copy with
            // negligible overhead relative to the Newton solve.
            Integrator current(KinematicsField{}, vf_);
            return current.step(t0, s.g, pack(s), h, opt_);
        }

        // ------------------------------------------------------------------
        // pack() / unpack()
        // ------------------------------------------------------------------

        /// @brief Pack a @c StateD into the flat Euclidean vector.
        ///
        /// The base 7 components are @f$[\nu_B(6);\, m(1)]@f$.
        /// Extra components (indices 7…EuclidDim−1) are zero-initialised and
        /// reserved for augmented-state use.
        /// @param s  State to pack.
        [[nodiscard]]
        static VecE pack(const StateD& s) noexcept
        {
            VecE x = VecE::Zero();
            x.template head<kTwistDim>() = s.nu_B;
            x(kTwistDim) = s.m;
            return x;
        }

        /// @brief Unpack an SE(3) pose and Euclidean vector into a @c StateD.
        ///
        /// Only the base 7 components are read; extra augmented-state components
        /// (indices 7…EuclidDim−1) are ignored.
        /// @param g  SE(3) pose.
        /// @param x  Euclidean vector.
        /// @return   Assembled @c StateD.
        [[nodiscard]]
        static StateD unpack(const SE3d& g, const VecE& x) noexcept
        {
            StateD s;
            s.g = g;
            s.nu_B = x.template head<kTwistDim>();
            s.m = x(kTwistDim);
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

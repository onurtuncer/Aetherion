// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace Aetherion::Simulation {

    class ISimulator
    {
    public:

        // -----------------------------------------------------------------------
        // StepResult — returned by step() and advance_to()
        // -----------------------------------------------------------------------
        struct StepResult
        {
            bool   converged{ false };
            double t{ 0.0 };    // simulation time after the step [s]
        };

        // -----------------------------------------------------------------------
        // Construction / assignment
        // -----------------------------------------------------------------------
        explicit ISimulator(const RigidBody::Config& cfg,
            RigidBody::SixDofStepper  stepper);

        // Non-copyable, movable
        ISimulator(const ISimulator&) = delete;
        ISimulator& operator=(const ISimulator&) = delete;  // FIX 1: was DraglessSphereSimulator
        ISimulator(ISimulator&&) = default;
        ISimulator& operator=(ISimulator&&) = default;  // FIX 1: was DraglessSphereSimulator

        virtual ~ISimulator() = default;

        // -----------------------------------------------------------------------
        // step() — advance by exactly m_StepSize seconds
        // -----------------------------------------------------------------------
        [[nodiscard]] StepResult step();

        // -----------------------------------------------------------------------
        // step_by(dt) — advance by an arbitrary dt > 0
        //
        // If dt > m_StepSize, the interval is sub-stepped at m_StepSize
        // internally so that the integrator never exceeds its configured step size.
        // -----------------------------------------------------------------------
        [[nodiscard]] StepResult step_by(double dt);

        // -----------------------------------------------------------------------
        // advance_to(t_target) — integrate forward until simulation time reaches
        // t_target.  Throws std::invalid_argument if t_target <= current time.
        // -----------------------------------------------------------------------
        [[nodiscard]] StepResult advance_to(double t_target);

        // -----------------------------------------------------------------------
        // reset() — restore to the initial state supplied at construction
        // -----------------------------------------------------------------------
        void reset();

        // -----------------------------------------------------------------------
        // Accessors
        // -----------------------------------------------------------------------
        [[nodiscard]] double                    time()   const noexcept;
        [[nodiscard]] const RigidBody::StateD& state()  const noexcept;
        [[nodiscard]] const RigidBody::Config& config() const noexcept; 

        [[nodiscard]] virtual StepResult snapshot() const noexcept = 0;

    protected:

        // -----------------------------------------------------------------------
        // Validate — called by the constructor; derived classes may override
        // -----------------------------------------------------------------------
        virtual void validate() const; 

        // -----------------------------------------------------------------------
        // build_initial_state — implemented by derived classes that know their
        // own Config layout (ISimulator itself has no mu / mass_kg / radius_m)
        // -----------------------------------------------------------------------
        virtual RigidBody::StateD build_initial_state(const RigidBody::Config& cfg) = 0;

    private:
        RigidBody::Config        m_Config;
        double                   m_Time;      
        RigidBody::StateD        m_State;     
        RigidBody::SixDofStepper m_Stepper;
    };

} // namespace Aetherion::Simulation
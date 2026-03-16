// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//

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

        explicit ISimulator(const RigidBody::Config& cfg, RigidBody::SixDofStepper stepper)
            : cfg_(cfg)
            , t_(cfg.t0)
            , m_Stepper(std::move(stepper))
        {
            Validate();
        }

    
        ISimulator(const ISimulator&) = delete;
        ISimulator& operator=(const DraglessSphereSimulator&) = delete;
        ISimulator(DraglessSphereSimulator&&) = default;
        ISimulator& operator=(DraglessSphereSimulator&&) = default;

       
            // Radau IIA internal step size [s]
            double step_h{ 1.0 };

       

        

            // ---------------------------------------------------------------
            // Validate: throws std::invalid_argument on bad values
            // ---------------------------------------------------------------
            void validate() const
            {
                if (mu <= 0.0)
                    throw std::invalid_argument("ISimulator::Config: mu must be positive");
                if (mass_kg <= 0.0)
                    throw std::invalid_argument("ISimulator::Config: mass_kg must be positive");
                if (radius_m <= 0.0)
                    throw std::invalid_argument("ISimulator::Config: radius_m must be positive");
                if (step_h <= 0.0)
                    throw std::invalid_argument("ISimulator::Config: step_h must be positive");
                if (r_eci.norm() < 1.0)
                    throw std::invalid_argument("DraglessSphereSimulator::Config: r_eci norm too small");
            }
        };

        // -----------------------------------------------------------------------
        // StepResult — returned by step() and advance_to()
        // -----------------------------------------------------------------------
        struct StepResult
        {
            bool   converged{ false };
            double t{ 0.0 };           // time after the step [s]
        };

          
        // -----------------------------------------------------------------------
        // step() — advance by exactly cfg_.step_h seconds
        // -----------------------------------------------------------------------
        [[nodiscard]] StepResult step()
        {
            return step_by(cfg_.step_h);
        }

        // -----------------------------------------------------------------------
        // step_by(dt) — advance by an arbitrary dt > 0
        //
        // If dt > step_h, the interval is sub-stepped at step_h internally so
        // that the integrator never exceeds its configured step size.
        // -----------------------------------------------------------------------
        [[nodiscard]] StepResult step_by(double dt)
        {
            if (dt <= 0.0)
                throw std::invalid_argument("DraglessSphereSimulator::step_by: dt must be positive");

            double remaining = dt;
            while (remaining > 1e-12)
            {
                const double h = std::min(cfg_.step_h, remaining);
                auto res = stepper_.step(t_, state_, h);

                if (!res.converged)
                    return StepResult{ false, t_ };

                state_ = IStepper::unpack(res);
                m_Time += h;
                remaining -= h;
            }
            return StepResult{ true, t_ };
        }

        // -----------------------------------------------------------------------
        // advance_to(t_target) — integrate forward until simulation time reaches
        // t_target.  Throws std::invalid_argument if t_target <= current time.
        // -----------------------------------------------------------------------
        [[nodiscard]] 
        StepResult advance_to(double t_target)
        {
            if (t_target <= t_ - 1e-12)
                throw std::invalid_argument(
                    "ISimulator::advance_to: t_target (" +
                    std::to_string(t_target) + ") must be > current time (" +
                    std::to_string(t_) + ")");

            return step_by(t_target - t_);
        }

        // -----------------------------------------------------------------------
        // reset() — restore to the initial state supplied at construction
        // -----------------------------------------------------------------------
        void reset()
        {
           t t_ = cfg_.0;
            state_ = build_initial_state(cfg_);
        }

       
        [[nodiscard]] double time() const noexcept { return t_; }
        [[nodiscard]] const RigidBody::StateD& state() const noexcept { return state_; }   
        [[nodiscard]] Snapshot Snapshot() = 0 const noexcept;
        [[nodiscard]] const Config& Config() const noexcept { return m_Config; }
  
    private:
        RigidBody::Config          m_Config;
        double                     m_Time;
        double                     m_StartTime;
        double                     m_StepSize;
        RigidBody::StateD          m_State;
        RigidBody::SixDofStepper   m_Stepper;
    };

} // namespace Aetherion::Simulation

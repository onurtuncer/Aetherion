// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <Aetherion/Simulation/ISimulator.h>

#include <stdexcept>
#include <string>

namespace Aetherion::Simulation {

    ISimulator::ISimulator(const RigidBody::Config& cfg,
        RigidBody::SixDofStepper  stepper)
        : m_Config{ cfg }
        , m_Time{ cfg.t0 }
        , m_State{ build_initial_state(cfg) }
        , m_Stepper{ std::move(stepper) }
    {
        validate();  
    }

    void ISimulator::validate() const
    {
        if (m_Config.t0 < 0.0)
            throw std::invalid_argument(
                "ISimulator::validate: t0 must be non-negative");

        if (m_Config.step_h <= 0.0)
            throw std::invalid_argument(
                "ISimulator::validate: step_h must be positive");
    }

    // ---------------------------------------------------------------------------
    // step — advance by exactly m_Config.step_h seconds
    // ---------------------------------------------------------------------------
    ISimulator::StepResult ISimulator::step()
    {
        return step_by(m_Config.step_h);
    }

    // ---------------------------------------------------------------------------
    // step_by — advance by an arbitrary dt > 0, sub-stepping at step_h
    // ---------------------------------------------------------------------------
    ISimulator::StepResult ISimulator::step_by(double dt)
    {
        if (dt <= 0.0)
            throw std::invalid_argument(
                "ISimulator::step_by: dt must be positive");

        double remaining = dt;

        while (remaining > 1e-12)
        {
            const double h = std::min(m_Config.step_h, remaining);

            auto res = m_Stepper.step(m_Time, m_State, h);

            if (!res.converged)
                return StepResult{ false, m_Time };

            m_State = m_Stepper.unpack(res); 
            m_Time += h;                    
            remaining -= h;
        }

        return StepResult{ true, m_Time };
    }

    // ---------------------------------------------------------------------------
    // advance_to — integrate until simulation time reaches t_target
    // ---------------------------------------------------------------------------
    ISimulator::StepResult ISimulator::advance_to(double t_target)
    {
        if (t_target <= m_Time - 1e-12)
            throw std::invalid_argument(
                "ISimulator::advance_to: t_target (" +
                std::to_string(t_target) +
                ") must be > current time (" +
                std::to_string(m_Time) + ")");

        return step_by(t_target - m_Time);
    }

    void ISimulator::reset()
    {
        m_Time = m_Config.t0;
        m_State = build_initial_state(m_Config);
    }

    // ---------------------------------------------------------------------------
    // Accessors
    // ---------------------------------------------------------------------------
    double ISimulator::time() const noexcept
    {
        return m_Time;  
    }

    const RigidBody::StateD& ISimulator::state() const noexcept
    {
        return m_State; 
    }

    const RigidBody::Config& ISimulator::config() const noexcept
    {
        return m_Config; 
    }

} // namespace Aetherion::Simulation
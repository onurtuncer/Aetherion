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
#include <stdexcept>
#include <string>

namespace Aetherion::Simulation {

    template<class VectorField>
    class ISimulator
    {
    public:
        using Stepper = RigidBody::SixDoFStepper<VectorField>;
        using StepResult = typename Stepper::StepResult;

        // -----------------------------------------------------------------------
        // Construction
        // -----------------------------------------------------------------------
        explicit ISimulator(const RigidBody::InertialParameters& ip,
            RigidBody::StateD                    initialState,
            ODE::RKMK::Core::NewtonOptions       opt = {})
            : m_Stepper(ip, opt)
            , m_State(initialState)
            , m_Time(0.0)
        {
        }

        // Non-copyable, movable
        ISimulator(const ISimulator&) = delete;
        ISimulator& operator=(const ISimulator&) = delete;
        ISimulator(ISimulator&&) = default;
        ISimulator& operator=(ISimulator&&) = default;
        virtual ~ISimulator() = default;

        // -----------------------------------------------------------------------
        // step() — advance by exactly m_StepSize seconds
        // -----------------------------------------------------------------------
        [[nodiscard]] StepResult step(double h)
        {
            auto res = m_Stepper.step(m_Time, m_State, h);
            if (res.converged) {
                m_State = Stepper::unpack(res);
                m_Time += h;
            }
            return res;
        }

        // -----------------------------------------------------------------------
        // advance_to(t_target)
        // -----------------------------------------------------------------------
        [[nodiscard]] StepResult advance_to(double t_target, double h)
        {
            if (t_target <= m_Time)
                throw std::invalid_argument("t_target must be greater than current time.");

            StepResult res{};
            while (m_Time < t_target) {
                double dt = std::min(h, t_target - m_Time);
                res = step(dt);
                if (!res.converged) break;
            }
            return res;
        }

        // -----------------------------------------------------------------------
        // Accessors
        // -----------------------------------------------------------------------
        [[nodiscard]] double                     time()  const noexcept { return m_Time; }
        [[nodiscard]] const RigidBody::StateD& state() const noexcept { return m_State; }

        [[nodiscard]] virtual StepResult snapshot() const noexcept = 0;

    protected:
        virtual void validate() const {}

    private:
        Stepper           m_Stepper;
        double            m_Time;
        RigidBody::StateD m_State;
    };

} // namespace Aetherion::Simulation
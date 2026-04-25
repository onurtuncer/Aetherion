// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TumblingBrickNoDampingSimulator.h
//
// Concrete simulator for NASA TM-2015-218675 Atmospheric Scenario 2:
//   Tumbling brick with no aerodynamic damping, J2 gravity.
//
// The rotational state is driven entirely by the Euler torque-free equations:
//   I dω/dt + ω × (I ω) = 0  (no external moments)
// coupled to the translational dynamics under J2 gravity.
//
// Earth Rotation Angle propagated linearly:
//   θ(t) = θ₀ + ω_E × t
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Examples/TumblingBrickNoDamping/TumblingBrickNoDampingTypes.h>
#include <Aetherion/Simulation/ISimulator.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>

namespace Aetherion::Examples::TumblingBrickNoDamping {

class TumblingBrickNoDampingSimulator
    : public Simulation::ISimulator<TumblingBrickNoDampingVF, Simulation::Snapshot1>
{
public:
    static constexpr double kOmegaEarth_rad_s = 7.2921150e-5;

    explicit TumblingBrickNoDampingSimulator(
        const RigidBody::InertialParameters& ip,
        RigidBody::StateD                    initialState,
        double                               theta0_rad = 0.0,
        ODE::RKMK::Core::NewtonOptions       opt = {})
        : ISimulator<TumblingBrickNoDampingVF, Simulation::Snapshot1>(
            TumblingBrickNoDampingVF(
                ip,
                FlightDynamics::J2GravityPolicy{},
                FlightDynamics::ZeroAeroPolicy{}),
            std::move(initialState), opt)
        , m_Theta0(theta0_rad)
    {
    }

    [[nodiscard]]
    Simulation::Snapshot1 snapshot() const noexcept override
    {
        return Simulation::MakeSnapshot1(
            time(),
            state(),
            currentTheta(),
            vectorField().gravity);
    }

    [[nodiscard]]
    double currentTheta() const noexcept
    {
        return m_Theta0 + kOmegaEarth_rad_s * time();
    }

protected:
    void validate() const override {}

private:
    double m_Theta0;
};

} // namespace Aetherion::Examples::TumblingBrickNoDamping

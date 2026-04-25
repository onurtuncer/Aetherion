// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TumblingBrickWithDampingSimulator.h
//
// Concrete simulator for NASA TM-2015-218675 Atmospheric Scenario 3:
//   Tumbling brick with aerodynamic drag and rotary damping, J2 gravity.
//
// The aerodynamic moments damp the angular rates as the brick falls and
// gains airspeed; eventually the rotation damps out.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Examples/TumblingBrickWithDamping/TumblingBrickWithDampingTypes.h>
#include <Aetherion/Simulation/ISimulator.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/AerodynamicParameters.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>

namespace Aetherion::Examples::TumblingBrickWithDamping {

class TumblingBrickWithDampingSimulator
    : public Simulation::ISimulator<TumblingBrickWithDampingVF, Simulation::Snapshot1>
{
public:
    static constexpr double kOmegaEarth_rad_s = 7.2921150e-5;

    explicit TumblingBrickWithDampingSimulator(
        const RigidBody::InertialParameters&    ip,
        const RigidBody::AerodynamicParameters& aero,
        RigidBody::StateD                       initialState,
        double                                  theta0_rad = 0.0,
        ODE::RKMK::Core::NewtonOptions          opt = {})
        : ISimulator<TumblingBrickWithDampingVF, Simulation::Snapshot1>(
            TumblingBrickWithDampingVF(
                ip,
                FlightDynamics::J2GravityPolicy{},
                FlightDynamics::BrickDampingAeroPolicy(
                    aero.CD, aero.S, aero.b, aero.cbar,
                    aero.Clp, aero.Cmq, aero.Cnr)),
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

} // namespace Aetherion::Examples::TumblingBrickWithDamping

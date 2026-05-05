// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16SteadyFlightSimulator.h
//
// Six-DoF simulator for NASA TM-2015-218675 Check-Case 11.
//
// Wraps F16VF (J2 gravity + DAVE-ML aero + DAVE-ML prop) and produces
// Snapshot1 telemetry at each step.  Snapshot2 (31-column NASA format) is
// available via the base-class snapshot2() conversion.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Examples/F16SteadyFlight/F16Types.h>
#include <Aetherion/Simulation/ISimulator.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/FlightDynamics/Trim/TrimSolver.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>

#include <memory>

namespace Aetherion::Examples::F16SteadyFlight {

class F16SteadyFlightSimulator
    : public Simulation::ISimulator<F16VF, Simulation::Snapshot1>
{
public:
    static constexpr double kOmegaEarth_rad_s = 7.2921150e-5;

    /// @param ip          Inertial parameters (mass, MOI) read from F16_inertia.dml.
    /// @param trim        Trim solution (alpha, elevator, throttle).
    /// @param aero_model  Parsed F16_aero.dml model.
    /// @param prop_model  Parsed F16_prop.dml model.
    /// @param x0          Initial state (ECI frame).
    /// @param theta0      Earth Rotation Angle at t = 0 [rad].
    /// @param opt         Newton solver options (tolerances, max iterations).
    explicit F16SteadyFlightSimulator(
        const RigidBody::InertialParameters&                      ip,
        const FlightDynamics::TrimPoint&                          trim,
        std::shared_ptr<const Serialization::DAVEMLAeroModel>     aero_model,
        std::shared_ptr<const Serialization::DAVEMLPropModel>     prop_model,
        RigidBody::StateD                                         x0,
        double                                                    theta0,
        ODE::RKMK::Core::NewtonOptions                            opt = {})
        : ISimulator<F16VF, Simulation::Snapshot1>(
            F16VF(
                ip,
                FlightDynamics::J2GravityPolicy{},
                FlightDynamics::F16AeroPolicy(aero_model, trim.el_deg, 0.0, 0.0),
                FlightDynamics::F16PropPolicy(prop_model, trim.pwr_pct)),
            std::move(x0), opt)
        , m_theta0(theta0)
    {}

    [[nodiscard]]
    Simulation::Snapshot1 snapshot() const noexcept override
    {
        return Simulation::MakeSnapshot1(
            time(), state(), currentTheta(),
            vectorField().gravity, vectorField().aero);
    }

    [[nodiscard]]
    double currentTheta() const noexcept
    {
        return m_theta0 + kOmegaEarth_rad_s * time();
    }

protected:
    void validate() const override {}

private:
    double m_theta0;
};

} // namespace Aetherion::Examples::F16SteadyFlight

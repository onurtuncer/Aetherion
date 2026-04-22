// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Examples/SphereWithAtmosphericDrag/SphereWithAtmosphereicDragTypes.h>
#include <Aetherion/Simulation/ISimulator.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/AerodynamicParameters.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>

namespace Aetherion::Examples::SphereWithAtmosphericDrag {

    // -------------------------------------------------------------------------
    // SphereWithAtmosphericDragSimulator
    //
    // Concrete simulator for NASA TM-2015-218675 Atmospheric Scenario 6.
    // Inherits ISimulator<SphereWithAtmosphericDragVF, Simulation::Snapshot1>:
    //   - VectorField  = SphereWithAtmosphericDragVF  (J2 gravity, drag-only aero)
    //   - SnapshotType = Simulation::Snapshot1
    //
    // The DragOnlyAeroPolicy is constructed from the aerodynamic parameters
    // (CD, S_ref) read from the JSON config and passed to the VectorField before
    // the stepper is built.  The ISimulator VectorField constructor is used so
    // that the policy instance is preserved inside the integrator.
    //
    // Earth Rotation Angle propagated linearly:
    //   theta(t) = theta_0 + omega_E * t
    // -------------------------------------------------------------------------
    class SphereWithAtmosphericDragSimulator
        : public Simulation::ISimulator<SphereWithAtmosphericDragVF, Simulation::Snapshot1>
    {
    public:
        static constexpr double kOmegaEarth_rad_s = 7.2921150e-5;

        explicit SphereWithAtmosphericDragSimulator(
            const RigidBody::InertialParameters&    ip,
            const RigidBody::AerodynamicParameters& aero_params,
            RigidBody::StateD                       initialState,
            double                                  theta0_rad = 0.0,
            ODE::RKMK::Core::NewtonOptions          opt = {})
            : ISimulator<SphereWithAtmosphericDragVF, Simulation::Snapshot1>(
                SphereWithAtmosphericDragVF(
                    ip,
                    FlightDynamics::J2GravityPolicy{},
                    FlightDynamics::DragOnlyAeroPolicy(aero_params.CD, aero_params.S)),
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

} // namespace Aetherion::Examples::SphereWithAtmosphericDrag
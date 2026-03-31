// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Examples/DraglessSphere/DraglessSphereTypes.h>
#include <Aetherion/Simulation/ISimulator.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>

namespace Aetherion::Examples::DraglessSphere {

    // -------------------------------------------------------------------------
    // DraglessSphereSimulator
    //
    // Concrete simulator for NASA TM-2015-218675 Atmospheric Scenario 1.
    // Inherits ISimulator<DraglessSphereVF, Simulation::Snapshot1>:
    //   - VectorField  = DraglessSphereVF  (J2 gravity, no aero, constant mass)
    //   - SnapshotType = Simulation::Snapshot1
    //
    // The Earth Rotation Angle is propagated linearly:
    //   theta(t) = theta_0 + omega_E * t
    // -------------------------------------------------------------------------
    class DraglessSphereSimulator
        : public Simulation::ISimulator<DraglessSphereVF, Simulation::Snapshot1>
    {
    public:
        static constexpr double kOmegaEarth_rad_s = 7.2921150e-5;

        explicit DraglessSphereSimulator(
            const RigidBody::InertialParameters& ip,
            RigidBody::StateD                       initialState,
            double                                  theta0_rad = 0.0,
            ODE::RKMK::Core::NewtonOptions          opt = {})
            : ISimulator<DraglessSphereVF, Simulation::Snapshot1>(
                ip, std::move(initialState), opt)
            , m_Theta0(theta0_rad)
        {
        }


        [[nodiscard]]
        Simulation::Snapshot1 snapshot() const noexcept override
        {
            return Simulation::MakeSnapshot1(time(), state(), currentTheta());
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

} // namespace Aetherion::Examples::DraglessSphere
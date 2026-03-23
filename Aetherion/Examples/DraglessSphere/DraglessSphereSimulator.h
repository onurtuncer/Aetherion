// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// DraglessSphereSimulator.h
//
// Concrete ISimulator for the NASA TM-2015-218675 Atmospheric Scenario 1
// (dragless sphere with J2 gravitation).
//
// Inherits from ISimulator<DraglessSphereVF> and implements:
//   - snapshot()  — converts current StateD to Snapshot1 via MakeSnapshot1
//   - validate()  — checks NASA check-case tolerances (optional)
//
// The Earth Rotation Angle (ERA) is propagated linearly from the initial
// value supplied at construction:
//   theta(t) = theta_0 + omega_E * t
// where omega_E = 7.2921150e-5 rad/s (WGS-84).
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
    // Wraps ISimulator<DraglessSphereVF> and provides snapshot() which
    // delivers a Snapshot1 aligned to the NASA Atmos-01 CSV columns.
    // -------------------------------------------------------------------------
    class DraglessSphereSimulator
        : public Simulation::ISimulator<DraglessSphereVF>
    {
    public:
        // -----------------------------------------------------------------------
        // omega_E — WGS-84 Earth rotation rate [rad/s]
        // -----------------------------------------------------------------------
        static constexpr double kOmegaEarth_rad_s = 7.2921150e-5;

        // -----------------------------------------------------------------------
        // Construction
        //
        // @param ip           Vehicle inertial parameters (mass, inertia)
        // @param initialState Initial SE(3) x R^7 state from BuildInitialStateVector
        // @param theta0_rad   Earth Rotation Angle at t=startTime [rad]
        //                     Typically: theta0 = kOmegaEarth * startTime
        // @param opt          Newton solver options (default tolerances are fine)
        // -----------------------------------------------------------------------
        explicit DraglessSphereSimulator(
            const RigidBody::InertialParameters& ip,
            RigidBody::StateD                       initialState,
            double                                  theta0_rad = 0.0,
            ODE::RKMK::Core::NewtonOptions          opt = {})
            : ISimulator<DraglessSphereVF>(ip, std::move(initialState), opt)
            , m_Theta0(theta0_rad)
        {
        }

        // Non-copyable, movable (inherited from ISimulator)
        DraglessSphereSimulator(const DraglessSphereSimulator&) = delete;
        DraglessSphereSimulator& operator=(const DraglessSphereSimulator&) = delete;
        DraglessSphereSimulator(DraglessSphereSimulator&&) = default;
        DraglessSphereSimulator& operator=(DraglessSphereSimulator&&) = default;

        // -----------------------------------------------------------------------
        // snapshot()
        //
        // Returns a fully populated Snapshot1 from the current state, including
        // geodetic position, atmosphere, NED velocity, ZYX Euler angles, etc.
        // Aerodynamic force/moment fields are left at zero (no aero).
        // -----------------------------------------------------------------------
        [[nodiscard]]
        Simulation::Snapshot1 snapshot() const noexcept override
        {
            const double theta = currentTheta();
            return Simulation::MakeSnapshot1(time(), state(), theta);
        }

        // -----------------------------------------------------------------------
        // currentTheta()
        //
        // Earth Rotation Angle at the current simulation time.
        // -----------------------------------------------------------------------
        [[nodiscard]]
        double currentTheta() const noexcept
        {
            return m_Theta0 + kOmegaEarth_rad_s * time();
        }

    protected:
        // -----------------------------------------------------------------------
        // validate() — invoked by step() in ISimulator (currently a no-op hook).
        // Can be extended to check NASA tolerances here if desired.
        // -----------------------------------------------------------------------
        void validate() const override {}

    private:
        double m_Theta0;  // ERA at t = startTime [rad]
    };

} // namespace Aetherion::Examples::DraglessSphere
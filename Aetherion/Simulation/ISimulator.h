// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// ISimulator.h
//
// Abstract base class for all Aetherion simulators.
//
// Template parameters:
//   VectorField  -- Euclidean ODE right-hand side (must satisfy
//                   VectorFieldOnProductSE3 concept)
//   SnapshotType -- The concrete snapshot/telemetry struct produced by
//                   snapshot().  Each simulator family defines its own:
//                     DraglessSphereSimulator -> Snapshot1
//                     (future)  AeroSimulator  -> Snapshot2
//                     (future)  GNCSimulator    -> Snapshot3
//                   This keeps the base class independent of any concrete
//                   snapshot schema while still enforcing the contract that
//                   every simulator can produce a snapshot.
//
// Usage:
//   class MySimulator
//       : public ISimulator<MyVF, MySnapshot>
//   {
//       [[nodiscard]] MySnapshot snapshot() const noexcept override { ... }
//   };
//
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/Simulation/SnapshotTraits.h>
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

    template<class VectorField, class SnapshotType>
    class ISimulator
    {
    public:
        using Stepper = RigidBody::SixDoFStepper<VectorField>;
        using StepResult = typename Stepper::StepResult;
        using Snapshot = SnapshotType;

        // -----------------------------------------------------------------------
        // Construction
        // -----------------------------------------------------------------------
        explicit ISimulator(const RigidBody::InertialParameters& ip,
            RigidBody::StateD                    initialState,
            ODE::RKMK::Core::NewtonOptions       opt = {})
            : m_Stepper(ip, opt)
            , m_State(std::move(initialState))
            , m_Time(0.0)
        {
        }

        /// @brief Construct from a pre-built @c VectorField.
        ///
        /// Use this overload when the aerodynamic, propulsion, or other policies
        /// require non-default construction (e.g. @c DragOnlyAeroPolicy with CD
        /// and S_ref parameters).
        explicit ISimulator(VectorField                    vf,
            RigidBody::StateD              initialState,
            ODE::RKMK::Core::NewtonOptions opt = {})
            : m_Stepper(std::move(vf), opt)
            , m_State(std::move(initialState))
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
        // step() -- advance by h seconds
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
        // advance_to(t_target, h)
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

        // Expose the vector field so derived simulators can forward the gravity
        // policy instance to MakeSnapshot1, ensuring the reported
        // localGravity_m_s2 always matches the model used by the integrator.
        [[nodiscard]] const VectorField& vectorField() const noexcept
        {
            return m_Stepper.vectorField();
        }

        // -----------------------------------------------------------------------
        // snapshot() -- pure virtual
        //
        // Returns a SnapshotType populated from the current state.
        // Each concrete simulator overrides this to produce its own telemetry
        // schema (Snapshot1, Snapshot2, ...) without any coupling in this base.
        // -----------------------------------------------------------------------
        [[nodiscard]] virtual SnapshotType snapshot() const noexcept = 0;

        // -----------------------------------------------------------------------
        // snapshot2() -- NASA 31-column format
        //
        // Default: converts snapshot() (Snapshot1) → Snapshot2 by copying the
        // 31 shared fields.  Simulators with active aero policies should override
        // this to call MakeSnapshot2 directly so that aero_bodyForce/Moment are
        // populated from the policy rather than from the already-computed (and
        // populated) Snapshot1 fields.
        // -----------------------------------------------------------------------
        [[nodiscard]] virtual Snapshot2 snapshot2() const noexcept
        {
            const auto& s1 = snapshot();
            Snapshot2 s2;
            s2.time                              = s1.time;
            s2.gePosition_m                      = s1.gePosition_m;
            s2.feVelocity_m_s                    = s1.feVelocity_m_s;
            s2.altitudeMsl_m                     = s1.altitudeMsl_m;
            s2.longitude_rad                     = s1.longitude_rad;
            s2.latitude_rad                      = s1.latitude_rad;
            s2.localGravity_m_s2                 = s1.localGravity_m_s2;
            s2.eulerAngle_rad_Yaw                = s1.eulerAngle_rad_Yaw;
            s2.eulerAngle_rad_Pitch              = s1.eulerAngle_rad_Pitch;
            s2.eulerAngle_rad_Roll               = s1.eulerAngle_rad_Roll;
            s2.bodyAngularRateWrtEi_rad_s_Roll   = s1.bodyAngularRateWrtEi_rad_s_Roll;
            s2.bodyAngularRateWrtEi_rad_s_Pitch  = s1.bodyAngularRateWrtEi_rad_s_Pitch;
            s2.bodyAngularRateWrtEi_rad_s_Yaw    = s1.bodyAngularRateWrtEi_rad_s_Yaw;
            s2.altitudeRateWrtMsl_m_s            = s1.altitudeRateWrtMsl_m_s;
            s2.speedOfSound_m_s                  = s1.speedOfSound_m_s;
            s2.airDensity_kg_m3                  = s1.airDensity_kg_m3;
            s2.ambientPressure_Pa                = s1.ambientPressure_Pa;
            s2.ambientTemperature_K              = s1.ambientTemperature_K;
            s2.aero_bodyForce_N_X                = s1.aero_bodyForce_N_X;
            s2.aero_bodyForce_N_Y                = s1.aero_bodyForce_N_Y;
            s2.aero_bodyForce_N_Z                = s1.aero_bodyForce_N_Z;
            s2.aero_bodyMoment_Nm_L              = s1.aero_bodyMoment_Nm_L;
            s2.aero_bodyMoment_Nm_M              = s1.aero_bodyMoment_Nm_M;
            s2.aero_bodyMoment_Nm_N              = s1.aero_bodyMoment_Nm_N;
            s2.mach                              = s1.mach;
            s2.dynamicPressure_Pa                = s1.dynamicPressure_Pa;
            s2.trueAirspeed_m_s                  = s1.trueAirspeed_m_s;
            return s2;
        }

    protected:
        virtual void validate() const {}

    private:
        Stepper            m_Stepper;
        double             m_Time;
        RigidBody::StateD  m_State;
    };

} // namespace Aetherion::Simulation

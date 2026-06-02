// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TwoStageRocketSimulator.h
//
// Six-DoF open-loop simulator for the NASA TM-2015-218675 Atmospheric
// Scenario 17 two-stage rocket.
//
// Architecture
// ────────────
// This simulator intentionally does NOT inherit ISimulator.  ISimulator stores
// m_State as private, making it impossible to apply the discontinuous mass drop
// (Δm = 35 000 kg) that occurs when Stage 1 separates.  Owning the StateD
// directly removes this restriction.
//
// VectorField type
// ────────────────
//   TwoStageRocketVF = VectorField<
//       RocketGravityPolicy, – J₂ gravity + CG–MRC moment transfer (My, Mz)
//       RocketAeroPolicy,    – DAVE-ML CL/CD/CY/Cm/Cn with wind-to-body transform
//       AxialThrustPolicy,   – body +x thrust (bodyThrustForce_X from DML)
//       LinearBurnPolicy     – variable mdot updated each step (ZOH)
//   >
//
// Per-step loop (ZOH)
// ───────────────────
//   1. query RocketStageModel.propulsion()        → update thrust_N, mdot_kgs in VF
//   2. query RocketStageModel.inertialParameters() → rebuild M / M_inv + xcg_m in VF
//   3. integrate one step with SixDoFStepper
//   4. advance RocketStageModel fuel state
//   5. on staging event: subtract kStg1DryMass_kg from m_state.m, rebuild M
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Examples/TwoStageRocket/RocketStageModel.h>
#include <Aetherion/Examples/TwoStageRocket/RocketGravityPolicy.h>
#include <Aetherion/Examples/TwoStageRocket/RocketAeroPolicy.h>

#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/FlightDynamics/Policies/MassPolicies.h>
#include <Aetherion/Simulation/Snapshot2.h>
#include <Aetherion/Simulation/MakeSnapshot2.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>

#include <memory>

namespace Aetherion::Examples::TwoStageRocket {

class TwoStageRocketSimulator {
public:
    // ── VectorField type ──────────────────────────────────────────────────────
    using VF = RigidBody::VectorField<
        RocketGravityPolicy,
        RocketAeroPolicy,
        FlightDynamics::AxialThrustPolicy,
        FlightDynamics::LinearBurnPolicy
    >;
    using Stepper    = RigidBody::SixDoFStepper<VF>;
    using StepResult = typename Stepper::StepResult;

    // ── Step result ───────────────────────────────────────────────────────────
    struct StepObservation {
        bool   converged     = true;
        double residual_norm = 0.0;
    };

    // ── Construction ──────────────────────────────────────────────────────────

    /// @brief Construct the simulator.
    ///
    /// @param ip         Initial inertial parameters (liftoff mass / inertia).
    /// @param x0         Initial 6-DoF state (ECI position, attitude, body rates, mass).
    /// @param theta0     Earth Rotation Angle at t=0 [rad].
    /// @param inertiaDml Parsed @c twostage_inertia.dml engine.
    /// @param propDml    Parsed @c twostage_prop.dml engine.
    /// @param aeroDml            Parsed @c twostage_aero.dml engine.
    /// @param stg2IgnitionTime_s Absolute sim time at which S2 may ignite [s].
    ///                           Pass (endTime − S2_burn_duration) to match the
    ///                           NASA TM reference coast-then-fire sequencing.
    explicit TwoStageRocketSimulator(
        const RigidBody::InertialParameters&                  ip,
        const RigidBody::StateD&                              x0,
        double                                                theta0,
        std::shared_ptr<Serialization::DAVEMLAeroModel>       inertiaDml,
        std::shared_ptr<Serialization::DAVEMLAeroModel>       propDml,
        std::shared_ptr<const Serialization::DAVEMLAeroModel> aeroDml,
        double                                                stg2IgnitionTime_s = 0.0);

    // ── Step ──────────────────────────────────────────────────────────────────

    /// @brief Advance the simulation by @p h seconds.
    ///
    /// Performs the ZOH pre-step update (inertia, thrust, mdot), integrates,
    /// advances the fuel tracker, and applies any staging event.
    StepObservation step(double h);

    // ── Snapshot ──────────────────────────────────────────────────────────────

    /// @brief Build a 31-column NASA-compatible snapshot of the current state.
    [[nodiscard]] Simulation::Snapshot2 snapshot2() const;

    // ── Accessors ─────────────────────────────────────────────────────────────
    [[nodiscard]] double                     time()  const noexcept { return m_time; }
    [[nodiscard]] const RigidBody::StateD&   state() const noexcept { return m_state; }
    [[nodiscard]] const RocketStageModel&    stageModel() const noexcept { return m_stageModel; }

private:
    // Synchronise VF policies with the current RocketStageModel state.
    // Called before every integration step (ZOH).
    void updateVectorField();

    // Rebuild VF.M and VF.M_inv from inertial parameters in-place.
    void rebuildSpatialInertia(const RigidBody::InertialParameters& ip);

    Stepper           m_stepper;
    RigidBody::StateD m_state;
    double            m_time   = 0.0;
    double            m_theta0 = 0.0;  ///< ERA at t=0 [rad].

    RocketStageModel  m_stageModel;
};

} // namespace Aetherion::Examples::TwoStageRocket

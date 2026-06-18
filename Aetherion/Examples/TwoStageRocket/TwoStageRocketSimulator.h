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
//
// Integrator template parameter
// ──────────────────────────────
// The IntegratorPolicy parameter follows the same pattern as ISimulator: it
// defaults to RadauIIA_RKMK_ProductSE3 and must satisfy IntegratorFor<>.
// A custom integrator (e.g. an explicit RKMK4 for the non-stiff coast phase)
// can be substituted without any other changes to this class.
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
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/ODE/RKMK/Concepts.h>

#include <memory>

namespace Aetherion::Examples::TwoStageRocket {

// ── Concrete VectorField for this simulator ───────────────────────────────────
//
// Defined at namespace scope so it can be referenced in the default template
// argument for IntegratorPolicy below.
using TwoStageRocketVF = RigidBody::VectorField<
    RocketGravityPolicy,
    RocketAeroPolicy,
    FlightDynamics::AxialThrustPolicy,
    FlightDynamics::LinearBurnPolicy
>;

// ── TwoStageRocketSimulator<IntegratorPolicy> ─────────────────────────────────

template<int EuclidDim = RigidBody::RigidBody6DoFEuclidDim,
         class IntegratorPolicy =
             ODE::RKMK::Integrators::RadauIIA_RKMK_ProductSE3<
                 RigidBody::KinematicsXiField,
                 TwoStageRocketVF,
                 EuclidDim>>
    requires ODE::RKMK::IntegratorFor<IntegratorPolicy,
                                       RigidBody::KinematicsXiField,
                                       TwoStageRocketVF,
                                       EuclidDim>
class TwoStageRocketSimulator {
public:
    // ── Type aliases ──────────────────────────────────────────────────────────
    using VF         = TwoStageRocketVF;
    using Integrator = IntegratorPolicy;
    using Stepper    = RigidBody::SixDoFStepper<VF, EuclidDim, Integrator>;
    using StepResult = typename Stepper::StepResult;

    // ── Step observation ──────────────────────────────────────────────────────
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
        const std::shared_ptr<const Serialization::DAVEMLAeroModel>& aeroDml,
        double                                                stg2IgnitionTime_s = 0.0)
        : m_stepper   (VF{ ip, RocketGravityPolicy{}, RocketAeroPolicy{std::move(aeroDml)} })
        , m_state     (x0)
        , m_time      (0.0)
        , m_theta0    (theta0)
        , m_stageModel(std::move(inertiaDml), std::move(propDml))
    {
        m_stageModel.stg2IgnitionTime_s = stg2IgnitionTime_s;
        updateVectorField();
    }

    // ── Step ──────────────────────────────────────────────────────────────────

    /// @brief Advance the simulation by @p h seconds.
    ///
    /// Performs the ZOH pre-step update (inertia, thrust, mdot), integrates,
    /// advances the fuel tracker, and applies any staging event.
    StepObservation step(double h)
    {
        updateVectorField();
        const double mdot_kgs = m_stageModel.propulsion(m_time).mdot_kgs;

        auto res = m_stepper.step(m_time, m_state, h);

        if (!res.converged)
            return { false, res.residual_norm };

        m_state  = Stepper::unpack(res);
        m_time  += h;

        const bool staged = m_stageModel.advance(h, mdot_kgs);

        if (staged) {
            m_state.m -= RocketStageModel::kStg1DryMass_kg;
            updateVectorField();
        }

        return { true, res.residual_norm };
    }

    // ── Snapshot ──────────────────────────────────────────────────────────────

    /// @brief Build a 31-column NASA-compatible snapshot of the current state.
    [[nodiscard]] Simulation::Snapshot2 snapshot2() const
    {
        constexpr double kOmegaE = Environment::WGS84::kRotationRate_rad_s;
        const double theta_gst   = m_theta0 + m_time * kOmegaE;

        const auto& vf = m_stepper.vectorField();
        return Simulation::MakeSnapshot2(m_time, m_state, theta_gst,
                                         vf.gravity, vf.aero);
    }

    // ── Accessors ─────────────────────────────────────────────────────────────
    [[nodiscard]] double                     time()       const noexcept { return m_time; }
    [[nodiscard]] const RigidBody::StateD&   state()      const noexcept { return m_state; }
    [[nodiscard]] const RocketStageModel&    stageModel() const noexcept { return m_stageModel; }

private:
    void updateVectorField()
    {
        auto& vf = m_stepper.vectorField();

        const auto prop = m_stageModel.propulsion(m_time);
        vf.thrust.thrust_N     =  prop.thrust_N;
        vf.mass_model.mdot_kgs = -prop.mdot_kgs;

        const auto ip = m_stageModel.inertialParameters();
        rebuildSpatialInertia(ip);

        vf.gravity.xcg_m = ip.xbar_m;
    }

    void rebuildSpatialInertia(const RigidBody::InertialParameters& ip)
    {
        auto& vf = m_stepper.vectorField();

        const double m   = ip.mass_kg;
        const double Ixx = ip.Ixx, Iyy = ip.Iyy, Izz = ip.Izz;
        const double Ixy = ip.Ixy, Iyz = ip.Iyz, Ixz = ip.Ixz;
        const double rx  = ip.xbar_m;
        const double ry  = ip.ybar_m;
        const double rz  = ip.zbar_m;

        vf.M.setZero();

        vf.M(0, 0) =  Ixx;   vf.M(0, 1) = -Ixy;   vf.M(0, 2) = -Ixz;
        vf.M(1, 0) = -Ixy;   vf.M(1, 1) =  Iyy;   vf.M(1, 2) = -Iyz;
        vf.M(2, 0) = -Ixz;   vf.M(2, 1) = -Iyz;   vf.M(2, 2) =  Izz;

        vf.M(3, 3) = m;   vf.M(4, 4) = m;   vf.M(5, 5) = m;

        vf.M(0, 4) = -m * rz;   vf.M(0, 5) =  m * ry;
        vf.M(1, 3) =  m * rz;   vf.M(1, 5) = -m * rx;
        vf.M(2, 3) = -m * ry;   vf.M(2, 4) =  m * rx;
        vf.M(3, 1) =  m * rz;   vf.M(3, 2) = -m * ry;
        vf.M(4, 0) = -m * rz;   vf.M(4, 2) =  m * rx;
        vf.M(5, 0) =  m * ry;   vf.M(5, 1) = -m * rx;

        vf.M_inv = vf.M.inverse();
    }

    Stepper           m_stepper;
    RigidBody::StateD m_state;
    double            m_time   = 0.0;
    double            m_theta0 = 0.0;  ///< ERA at t=0 [rad].

    RocketStageModel  m_stageModel;
};

} // namespace Aetherion::Examples::TwoStageRocket

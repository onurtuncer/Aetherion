// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <Aetherion/Examples/TwoStageRocket/TwoStageRocketSimulator.h>
#include <Aetherion/Examples/TwoStageRocket/RocketGravityPolicy.h>
#include <Aetherion/Examples/TwoStageRocket/RocketAeroPolicy.h>
#include <Aetherion/Environment/WGS84.h>

namespace Aetherion::Examples::TwoStageRocket {

// ── Construction ──────────────────────────────────────────────────────────────

TwoStageRocketSimulator::TwoStageRocketSimulator(
    const RigidBody::InertialParameters&                  ip,
    const RigidBody::StateD&                              x0,
    double                                                theta0,
    std::shared_ptr<Serialization::DAVEMLAeroModel>       inertiaDml,
    std::shared_ptr<Serialization::DAVEMLAeroModel>       propDml,
    std::shared_ptr<const Serialization::DAVEMLAeroModel> aeroDml,
    double                                                stg2IgnitionTime_s)
    : m_stepper   (VF{ ip, RocketGravityPolicy{}, RocketAeroPolicy{std::move(aeroDml)} })
    , m_state     (x0)
    , m_time      (0.0)
    , m_theta0    (theta0)
    , m_stageModel(std::move(inertiaDml), std::move(propDml))
{
    m_stageModel.stg2IgnitionTime_s = stg2IgnitionTime_s;
    // Prime the VF with the initial propulsion and inertia state so that
    // snapshot2() at t=0 shows the correct mass properties.
    updateVectorField();
}

// ── rebuildSpatialInertia ─────────────────────────────────────────────────────

// Mirror of the VectorField constructor's M-building block, written directly
// to the public M / M_inv members so we avoid reconstructing the whole VF
// (which would reset the policy instances).
void TwoStageRocketSimulator::rebuildSpatialInertia(
    const RigidBody::InertialParameters& ip)
{
    auto& vf = m_stepper.vectorField();

    const double m   = ip.mass_kg;
    const double Ixx = ip.Ixx, Iyy = ip.Iyy, Izz = ip.Izz;
    const double Ixy = ip.Ixy, Iyz = ip.Iyz, Ixz = ip.Ixz;
    const double rx  = ip.xbar_m;   // DXCG: CG forward of MRC (+x)
    const double ry  = ip.ybar_m;   // 0 for this axisymmetric vehicle
    const double rz  = ip.zbar_m;   // 0

    vf.M.setZero();

    // Rotational inertia block (top-left 3×3) — about MRC, not CG.
    // For the rocket DML, XIXX/XIYY/XIZZ are already about the MRC.
    vf.M(0, 0) =  Ixx;   vf.M(0, 1) = -Ixy;   vf.M(0, 2) = -Ixz;
    vf.M(1, 0) = -Ixy;   vf.M(1, 1) =  Iyy;   vf.M(1, 2) = -Iyz;
    vf.M(2, 0) = -Ixz;   vf.M(2, 1) = -Iyz;   vf.M(2, 2) =  Izz;

    // Translational inertia block (bottom-right 3×3)
    vf.M(3, 3) = m;   vf.M(4, 4) = m;   vf.M(5, 5) = m;

    // CG-offset cross-coupling (Featherstone: h× = m·[c×], [c×]^T = -[c×])
    vf.M(0, 4) = -m * rz;   vf.M(0, 5) =  m * ry;
    vf.M(1, 3) =  m * rz;   vf.M(1, 5) = -m * rx;
    vf.M(2, 3) = -m * ry;   vf.M(2, 4) =  m * rx;
    vf.M(3, 1) =  m * rz;   vf.M(3, 2) = -m * ry;
    vf.M(4, 0) = -m * rz;   vf.M(4, 2) =  m * rx;
    vf.M(5, 0) =  m * ry;   vf.M(5, 1) = -m * rx;

    vf.M_inv = vf.M.inverse();
}

// ── updateVectorField ─────────────────────────────────────────────────────────

void TwoStageRocketSimulator::updateVectorField()
{
    auto& vf = m_stepper.vectorField();

    // 1. Propulsion — update thrust and mass-depletion rate
    const auto prop = m_stageModel.propulsion(m_time);
    vf.thrust.thrust_N     =  prop.thrust_N;
    vf.mass_model.mdot_kgs = -prop.mdot_kgs;   // negative → mass decreases

    // 2. Inertia — rebuild spatial inertia matrix for current fuel state
    const auto ip = m_stageModel.inertialParameters();
    rebuildSpatialInertia(ip);

    // 3. CG offset — update RocketGravityPolicy so moment transfer uses
    //    the latest DXCG value (ZOH; same granularity as inertia rebuild)
    vf.gravity.xcg_m = ip.xbar_m;
}

// ── step ──────────────────────────────────────────────────────────────────────

TwoStageRocketSimulator::StepObservation
TwoStageRocketSimulator::step(double h)
{
    // ── Pre-step: synchronise VF with current stage/fuel state (ZOH) ─────────
    updateVectorField();
    const double mdot_kgs = m_stageModel.propulsion(m_time).mdot_kgs;

    // ── Integrate ─────────────────────────────────────────────────────────────
    auto res = m_stepper.step(m_time, m_state, h);

    if (!res.converged)
        return { false, res.residual_norm };

    m_state  = Stepper::unpack(res);
    m_time  += h;

    // ── Post-step: advance fuel tracker ──────────────────────────────────────
    const bool staged = m_stageModel.advance(h, mdot_kgs);

    if (staged) {
        // Staging event: drop the dry S1 hardware mass from the ODE state.
        // The integrator already consumed S1 propellant via mdot; this handles
        // the discrete jettison of the empty stage-1 structure.
        m_state.m -= RocketStageModel::kStg1DryMass_kg;

        // Immediately refresh M/M_inv for the post-staging configuration
        // so the next step uses the correct S2-only inertia.
        updateVectorField();
    }

    return { true, res.residual_norm };
}

// ── snapshot2 ─────────────────────────────────────────────────────────────────

Simulation::Snapshot2 TwoStageRocketSimulator::snapshot2() const
{
    constexpr double kOmegaE = Environment::WGS84::kRotationRate_rad_s;
    const double theta_gst   = m_theta0 + m_time * kOmegaE;

    const auto& vf = m_stepper.vectorField();
    return Simulation::MakeSnapshot2(m_time, m_state, theta_gst,
                                     vf.gravity, vf.aero);
}

} // namespace Aetherion::Examples::TwoStageRocket

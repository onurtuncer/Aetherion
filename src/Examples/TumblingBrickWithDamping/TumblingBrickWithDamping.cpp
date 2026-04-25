// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TumblingBrickWithDamping.cpp
//
// Entry point for the NASA TM-2015-218675 Atmospheric Scenario 3 executable.
//
// Usage:
//   TumblingBrickWithDamping.exe \
//       --inputFileName  nasa_2015_scenario3_tumbling_brick_with_damping.json \
//       --outputFileName atmos_03_output.csv                                  \
//       --startTime      0.0                                                  \
//       --endTime        30.0                                                 \
//       --timeStep       0.002                                                \
//       --writeInterval  50
//

#include <Aetherion/Examples/TumblingBrickWithDamping/TumblingBrickWithDampingApplication.h>

#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Serialization/LoadConfig.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/RigidBody/StateLayout.h>

#include <cmath>
#include <stdexcept>

namespace Aetherion::Examples::TumblingBrickWithDamping {

    void TumblingBrickWithDampingApplication::logStartupBanner() const
    {
        AE_CORE_INFO("=======================================================");
        AE_CORE_INFO("Aetherion - NASA TM-2015-218675 Atmospheric Scenario 3");
        AE_CORE_INFO("Tumbling brick with aero damping - J2 gravitation");
        AE_CORE_INFO("  Ixx={:.6f}  Iyy={:.6f}  Izz={:.6f} kg·m²",
            0.002568217, 0.008421012, 0.009754656);
        AE_CORE_INFO("  CD={:.4f}  Clp={:.1f}  Cmq={:.1f}  Cnr={:.1f}",
            0.01, -1.0, -1.0, -1.0);
        AE_CORE_INFO("=======================================================");
    }

    void TumblingBrickWithDampingApplication::prepareSimulation() const
    {
        const auto& cfg = getConfig();

        const RigidBody::Config rb_cfg = loadVehicleConfig();
        const double            theta0 = computeInitialERA(cfg.startTime);
        const RigidBody::StateD x0     = buildInitialState(rb_cfg, theta0);

        constexpr double kToDeg = 180.0 / 3.14159265358979323846;
        AE_CORE_INFO("Initial ECI position  = [{:.3f}, {:.3f}, {:.3f}] m",
            x0.g.p.x(), x0.g.p.y(), x0.g.p.z());
        AE_CORE_INFO("Initial body angular rates (ECI) = [{:.4f}, {:.4f}, {:.4f}] deg/s",
            x0.nu_B(0) * kToDeg, x0.nu_B(1) * kToDeg, x0.nu_B(2) * kToDeg);
        AE_CORE_INFO("CD={:.4f}  S={:.6f} m²  b={:.6f} m  cbar={:.6f} m",
            rb_cfg.aerodynamicParameters.CD,
            rb_cfg.aerodynamicParameters.S,
            rb_cfg.aerodynamicParameters.b,
            rb_cfg.aerodynamicParameters.cbar);
        AE_CORE_INFO("Clp={:.2f}  Cmq={:.2f}  Cnr={:.2f}",
            rb_cfg.aerodynamicParameters.Clp,
            rb_cfg.aerodynamicParameters.Cmq,
            rb_cfg.aerodynamicParameters.Cnr);

        m_Simulator = constructSimulator(
            rb_cfg.inertialParameters,
            rb_cfg.aerodynamicParameters,
            x0, theta0);
    }

    void TumblingBrickWithDampingApplication::writeInitialSnapshot(std::ofstream& csv) const
    {
        const auto snap = m_Simulator->snapshot();
        Simulation::Snapshot1_WriteCsvRow(csv, snap);
        constexpr double kToDeg = 180.0 / 3.14159265358979323846;
        AE_CORE_INFO("t={:.6f} s  alt={:.3f} m  |v_NED|={:.6f} m/s  "
                     "ω=[{:.4f},{:.4f},{:.4f}] deg/s",
            snap.time, snap.altitudeMsl_m, snap.feVelocity_m_s.norm(),
            snap.bodyAngularRateWrtEi_rad_s_Roll  * kToDeg,
            snap.bodyAngularRateWrtEi_rad_s_Pitch * kToDeg,
            snap.bodyAngularRateWrtEi_rad_s_Yaw   * kToDeg);
    }

    auto TumblingBrickWithDampingApplication::stepAndRecord(
        std::ofstream& csv, double h, bool doWrite) const -> StepObservation
    {
        const auto res  = m_Simulator->step(h);
        const auto snap = m_Simulator->snapshot();
        if (doWrite)
            Simulation::Snapshot1_WriteCsvRow(csv, snap);

        constexpr double kToDeg = 180.0 / 3.14159265358979323846;
        AE_CORE_TRACE("  t={:.6f} s  alt={:.4f} m  v={:.6f} m/s  "
                      "ω=[{:.4f},{:.4f},{:.4f}] deg/s",
            snap.time, snap.altitudeMsl_m, snap.feVelocity_m_s.norm(),
            snap.bodyAngularRateWrtEi_rad_s_Roll  * kToDeg,
            snap.bodyAngularRateWrtEi_rad_s_Pitch * kToDeg,
            snap.bodyAngularRateWrtEi_rad_s_Yaw   * kToDeg);

        return StepObservation{
            .time_s        = snap.time,
            .altitude_m    = snap.altitudeMsl_m,
            .speed_mps     = snap.feVelocity_m_s.norm(),
            .latitude_rad  = snap.latitude_rad,
            .longitude_rad = snap.longitude_rad,
            .converged     = res.converged,
            .residual      = res.residual_norm,
        };
    }

    void TumblingBrickWithDampingApplication::logFinalSummary() const
    {
        const auto snap = m_Simulator->snapshot();
        constexpr double kToDeg = 180.0 / 3.14159265358979323846;

        AE_CORE_INFO("=======================================================");
        AE_CORE_INFO("Simulation complete.");
        AE_CORE_INFO("  Final time      : {:.6f} s",  snap.time);
        AE_CORE_INFO("  Final altitude  : {:.4f} m",  snap.altitudeMsl_m);
        AE_CORE_INFO("  Final |v_NED|   : {:.6f} m/s", snap.feVelocity_m_s.norm());
        AE_CORE_INFO("  Final latitude  : {:.8f} deg", snap.latitude_rad  * kToDeg);
        AE_CORE_INFO("  Final longitude : {:.8f} deg", snap.longitude_rad * kToDeg);
        AE_CORE_INFO("  Final ω (ECI)   : [{:.6f}, {:.6f}, {:.6f}] deg/s",
            snap.bodyAngularRateWrtEi_rad_s_Roll  * kToDeg,
            snap.bodyAngularRateWrtEi_rad_s_Pitch * kToDeg,
            snap.bodyAngularRateWrtEi_rad_s_Yaw   * kToDeg);
        AE_CORE_INFO("  Final q (body→ECI) : [{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
            snap.q_body_to_eci.w(), snap.q_body_to_eci.x(),
            snap.q_body_to_eci.y(), snap.q_body_to_eci.z());
        AE_CORE_INFO("Output written to '{}'", getConfig().outputFileName);
        AE_CORE_INFO("=======================================================");
    }

    // =========================================================================
    // Private helpers
    // =========================================================================

    RigidBody::Config TumblingBrickWithDampingApplication::loadVehicleConfig() const
    {
        const auto& fname = getConfig().inputFileName;
        if (fname.empty())
            throw std::runtime_error(
                "No --inputFileName specified. "
                "Provide the path to the vehicle JSON config.");

        AE_CORE_INFO("Loading vehicle config from '{}'", fname);
        const RigidBody::Config cfg = Serialization::LoadConfig(fname);

        constexpr double kToDeg = 180.0 / 3.14159265358979323846;
        AE_CORE_TRACE("Vehicle config loaded:");
        AE_CORE_TRACE("  lat={:.4f} deg  lon={:.4f} deg  alt={:.2f} m",
            cfg.pose.lat_deg, cfg.pose.lon_deg, cfg.pose.alt_m);
        AE_CORE_TRACE("  mass={:.6f} kg  Ixx={:.6f}  Iyy={:.6f}  Izz={:.6f} kg·m²",
            cfg.inertialParameters.mass_kg,
            cfg.inertialParameters.Ixx,
            cfg.inertialParameters.Iyy,
            cfg.inertialParameters.Izz);
        AE_CORE_TRACE("  bodyRates = [{:.6f}, {:.6f}, {:.6f}] rad/s (ECEF-relative)",
            cfg.bodyRates.roll_rad_s, cfg.bodyRates.pitch_rad_s, cfg.bodyRates.yaw_rad_s);
        return cfg;
    }

    double TumblingBrickWithDampingApplication::computeInitialERA(double t0) noexcept
    {
        const double theta0 = TumblingBrickWithDampingSimulator::kOmegaEarth_rad_s * t0;
        AE_CORE_TRACE("Initial ERA theta0 = {:.10f} rad", theta0);
        return theta0;
    }

    RigidBody::StateD TumblingBrickWithDampingApplication::buildInitialState(
        const RigidBody::Config& rb_cfg, double theta0)
    {
        AE_CORE_INFO("Building initial state vector (ECI frame)...");
        const auto x0 = FlightDynamics::BuildInitialStateVector(rb_cfg, theta0);

        RigidBody::StateD state;
        using Layout = RigidBody::StateLayout;

        state.g.p = Eigen::Vector3d(
            x0[Layout::IDX_P], x0[Layout::IDX_P + 1], x0[Layout::IDX_P + 2]);

        const Eigen::Quaterniond q(
            x0[Layout::IDX_Q],
            x0[Layout::IDX_Q + 1],
            x0[Layout::IDX_Q + 2],
            x0[Layout::IDX_Q + 3]);
        state.g.R = q.normalized().toRotationMatrix();

        state.nu_B <<
            x0[Layout::IDX_W], x0[Layout::IDX_W + 1], x0[Layout::IDX_W + 2],
            x0[Layout::IDX_V], x0[Layout::IDX_V + 1], x0[Layout::IDX_V + 2];

        state.m = x0[Layout::IDX_M];
        return state;
    }

    std::unique_ptr<TumblingBrickWithDampingSimulator>
        TumblingBrickWithDampingApplication::constructSimulator(
            const RigidBody::InertialParameters&    ip,
            const RigidBody::AerodynamicParameters& aero,
            const RigidBody::StateD&                x0,
            double                                  theta0)
    {
        AE_CORE_INFO("Constructing TumblingBrickWithDampingSimulator "
                     "(J2 gravity, CD={:.4f}, Clp={:.1f}, Cmq={:.1f}, Cnr={:.1f})...",
            aero.CD, aero.Clp, aero.Cmq, aero.Cnr);

        // Brick forces (~22 N) are ~6× smaller than the sphere; loosen abs_tol
        // so the Newton solver doesn't cycle at the floating-point noise floor.
        ODE::RKMK::Core::NewtonOptions opt{};
        opt.abs_tol = 1.0e-10;

        return std::make_unique<TumblingBrickWithDampingSimulator>(
            ip, aero, x0, theta0, opt);
    }

} // namespace Aetherion::Examples::TumblingBrickWithDamping

// -----------------------------------------------------------------------------
// Client factory — called by the EntryPoint-generated main().
// -----------------------------------------------------------------------------
namespace Aetherion::Simulation {

    Application* CreateApplication(int argc, char* argv[])
    {
        return new Examples::TumblingBrickWithDamping::TumblingBrickWithDampingApplication(argc, argv);
    }

} // namespace Aetherion::Simulation

#include <Aetherion/Simulation/EntryPoint.h>

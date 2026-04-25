// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TumblingBrickNoDamping.cpp
//
// Entry point for the NASA TM-2015-218675 Atmospheric Scenario 2 executable.
//
// Usage:
//   TumblingBrickNoDamping.exe \
//       --inputFileName  nasa_2015_scenario2_tumbling_brick_no_damping.json \
//       --outputFileName atmos_02_output.csv                                \
//       --startTime      0.0                                                \
//       --endTime        30.0                                               \
//       --timeStep       0.01
//

#include <Aetherion/Examples/TumblingBrickNoDamping/TumblingBrickNoDampingApplication.h>

#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Serialization/LoadConfig.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/RigidBody/StateLayout.h>

#include <cmath>
#include <stdexcept>

namespace Aetherion::Examples::TumblingBrickNoDamping {

    void TumblingBrickNoDampingApplication::logStartupBanner() const
    {
        AE_CORE_INFO("=======================================================");
        AE_CORE_INFO("Aetherion - NASA TM-2015-218675 Atmospheric Scenario 2");
        AE_CORE_INFO("Tumbling brick, no damping - J2 gravitation");
        AE_CORE_INFO("  Ixx={:.6f}  Iyy={:.6f}  Izz={:.6f} kg·m²  (1:2:3 slug·ft²)",
            1.355817948329279, 2.711635896658559, 4.067453844987838);
        AE_CORE_INFO("=======================================================");
    }

    void TumblingBrickNoDampingApplication::prepareSimulation() const
    {
        const auto& cfg = getConfig();

        const RigidBody::Config rb_cfg = loadVehicleConfig();
        const double            theta0 = computeInitialERA(cfg.startTime);
        const RigidBody::StateD x0     = buildInitialState(rb_cfg, theta0);

        constexpr double kToDeg = 180.0 / 3.14159265358979323846;
        AE_CORE_INFO("Initial ECI position  = [{:.3f}, {:.3f}, {:.3f}] m",
            x0.g.p.x(), x0.g.p.y(), x0.g.p.z());
        AE_CORE_INFO("Initial ECI position magnitude = {:.3f} m", x0.g.p.norm());
        AE_CORE_INFO("Initial body angular rates (ECI) = [{:.4f}, {:.4f}, {:.4f}] rad/s",
            x0.nu_B(0), x0.nu_B(1), x0.nu_B(2));
        AE_CORE_INFO("  = [{:.4f}, {:.4f}, {:.4f}] deg/s",
            x0.nu_B(0) * kToDeg, x0.nu_B(1) * kToDeg, x0.nu_B(2) * kToDeg);
        AE_CORE_INFO("Initial body linear velocity (ECI) = [{:.6e}, {:.6e}, {:.6e}] m/s",
            x0.nu_B(3), x0.nu_B(4), x0.nu_B(5));

        m_Simulator = constructSimulator(rb_cfg.inertialParameters, x0, theta0);
    }

    void TumblingBrickNoDampingApplication::writeInitialSnapshot(std::ofstream& csv) const
    {
        const auto snap = m_Simulator->snapshot();
        Simulation::Snapshot1_WriteCsvRow(csv, snap);
        constexpr double kToDeg = 180.0 / 3.14159265358979323846;
        AE_CORE_INFO("t={:.6f} s  alt={:.3f} m  |v_NED|={:.6f} m/s  "
                     "ω=[{:.4f},{:.4f},{:.4f}] deg/s",
            snap.time,
            snap.altitudeMsl_m,
            snap.feVelocity_m_s.norm(),
            snap.bodyAngularRateWrtEi_rad_s_Roll  * kToDeg,
            snap.bodyAngularRateWrtEi_rad_s_Pitch * kToDeg,
            snap.bodyAngularRateWrtEi_rad_s_Yaw   * kToDeg);
    }

    auto TumblingBrickNoDampingApplication::stepAndRecord(
        std::ofstream& csv, double h, bool doWrite) const -> StepObservation
    {
        const auto res  = m_Simulator->step(h);
        const auto snap = m_Simulator->snapshot();
        if (doWrite)
            Simulation::Snapshot1_WriteCsvRow(csv, snap);

        constexpr double kToDeg = 180.0 / 3.14159265358979323846;
        AE_CORE_TRACE("  t={:.6f} s  alt={:.4f} m  v={:.6f} m/s  "
                      "ω=[{:.4f},{:.4f},{:.4f}] deg/s",
            snap.time, snap.altitudeMsl_m,
            snap.feVelocity_m_s.norm(),
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

    void TumblingBrickNoDampingApplication::logFinalSummary() const
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

    RigidBody::Config TumblingBrickNoDampingApplication::loadVehicleConfig() const
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
        AE_CORE_TRACE("  azimuth={:.2f} deg  zenith={:.2f} deg  roll={:.2f} deg",
            cfg.pose.azimuth_deg, cfg.pose.zenith_deg, cfg.pose.roll_deg);
        AE_CORE_TRACE("  v_NED  = [{:.4f}, {:.4f}, {:.4f}] m/s",
            cfg.velocityNED.north_mps, cfg.velocityNED.east_mps, cfg.velocityNED.down_mps);
        AE_CORE_TRACE("  mass   = {:.6f} kg", cfg.inertialParameters.mass_kg);
        AE_CORE_TRACE("  Ixx={:.6f}  Iyy={:.6f}  Izz={:.6f} kg·m²",
            cfg.inertialParameters.Ixx,
            cfg.inertialParameters.Iyy,
            cfg.inertialParameters.Izz);
        AE_CORE_TRACE("  bodyRates = [{:.6f}, {:.6f}, {:.6f}] rad/s  (ECEF-relative)",
            cfg.bodyRates.roll_rad_s, cfg.bodyRates.pitch_rad_s, cfg.bodyRates.yaw_rad_s);

        return cfg;
    }

    double TumblingBrickNoDampingApplication::computeInitialERA(double t0) noexcept
    {
        const double theta0 = TumblingBrickNoDampingSimulator::kOmegaEarth_rad_s * t0;
        AE_CORE_TRACE("Initial ERA theta0 = {:.10f} rad  ({:.6f} deg)",
            theta0, theta0 * (180.0 / 3.14159265358979323846));
        return theta0;
    }

    RigidBody::StateD TumblingBrickNoDampingApplication::buildInitialState(
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

    std::unique_ptr<TumblingBrickNoDampingSimulator>
        TumblingBrickNoDampingApplication::constructSimulator(
            const RigidBody::InertialParameters& ip,
            const RigidBody::StateD&             x0,
            double                               theta0)
    {
        AE_CORE_INFO("Constructing TumblingBrickNoDampingSimulator "
                     "(J2 gravity, zero aero, Ixx={:.6f} Iyy={:.6f} Izz={:.6f} kg·m²)...",
            ip.Ixx, ip.Iyy, ip.Izz);

        // The brick's forces (~22 N) are ~6× smaller than the sphere's (~143 N),
        // so residuals saturate near 1e-11 rather than 1e-13.  Loosen abs_tol
        // to 1e-10 so the solver declares convergence instead of cycling.
        ODE::RKMK::Core::NewtonOptions opt{};
        opt.abs_tol = 1.0e-10;

        return std::make_unique<TumblingBrickNoDampingSimulator>(ip, x0, theta0, opt);
    }

} // namespace Aetherion::Examples::TumblingBrickNoDamping

// -----------------------------------------------------------------------------
// Client factory — called by the EntryPoint-generated main().
// -----------------------------------------------------------------------------
namespace Aetherion::Simulation {

    Application* CreateApplication(int argc, char* argv[])
    {
        return new Examples::TumblingBrickNoDamping::TumblingBrickNoDampingApplication(argc, argv);
    }

} // namespace Aetherion::Simulation

#include <Aetherion/Simulation/EntryPoint.h>

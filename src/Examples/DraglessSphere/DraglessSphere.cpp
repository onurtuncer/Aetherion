// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// DraglessSphere.cpp
//
// Entry point for the NASA TM-2015-218675 Atmospheric Scenario 1 executable.
//
// Usage:
//   DraglessSphere.exe \
//       --inputFileName  nasa_atmos_01_dragless_sphere.json \
//       --outputFileName atmos_01_output.csv                \
//       --startTime      0.0                                 \
//       --endTime        30.0                                \
//       --timeStep       0.01
//
// All flags have sensible defaults (see Simulation::Application / Config.h).
//

#include <Aetherion/Examples/DraglessSphere/DraglessSphereApplication.h>

#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Serialization/LoadConfig.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/RigidBody/StateLayout.h>

#include <cmath>
#include <stdexcept>

namespace Aetherion::Examples::DraglessSphere {

    void DraglessSphereApplication::logStartupBanner() const
    {
        AE_CORE_INFO("=======================================================");
        AE_CORE_INFO("Aetherion — NASA TM-2015-218675 Atmospheric Scenario 1");
        AE_CORE_INFO("Dragless sphere - J2 gravitation - US 1976 atmosphere");
        AE_CORE_INFO("=======================================================");
    }

    void DraglessSphereApplication::prepareSimulation() const
    {
        const auto& cfg = getConfig();

        const RigidBody::Config rb_cfg = loadVehicleConfig();          // step 1
        const double            theta0 = computeInitialERA(cfg.startTime); // step 2
        const RigidBody::StateD x0 = buildInitialState(rb_cfg, theta0);// step 3

        AE_CORE_INFO("Initial ECI position  = [{:.3f}, {:.3f}, {:.3f}] m",
            x0.g.p.x(), x0.g.p.y(), x0.g.p.z());
        AE_CORE_INFO("Initial ECI position magnitude = {:.3f} m", x0.g.p.norm());
        AE_CORE_INFO("Initial body twist nu_B = [{:.6e}, {:.6e}, {:.6e}, {:.6e}, {:.6e}, {:.6e}]",
            x0.nu_B(0), x0.nu_B(1), x0.nu_B(2),
            x0.nu_B(3), x0.nu_B(4), x0.nu_B(5));

        m_Simulator = constructSimulator(rb_cfg.inertialParameters, x0, theta0); // step 4
    }

    void DraglessSphereApplication::writeInitialSnapshot(std::ofstream& csv) const
    {
        const auto snap = m_Simulator->snapshot();
        Simulation::Snapshot1_WriteCsvRow(csv, snap);
        AE_CORE_INFO("t={:.6f} s  alt={:.3f} m  |v_NED|={:.6f} m/s",
            snap.time,
            snap.altitudeMsl_m,
            snap.feVelocity_m_s.norm());
    }

    auto DraglessSphereApplication::stepAndRecord(std::ofstream& csv, double h) const
        -> StepObservation
    {
        const auto res = m_Simulator->step(h);
        const auto snap = m_Simulator->snapshot();
        Simulation::Snapshot1_WriteCsvRow(csv, snap);

        constexpr double kToDeg = 180.0 / 3.14159265358979323846;

        AE_CORE_TRACE("  t={:.6f} s  alt={:.4f} m  v={:.6f} m/s  Mach={:.6f}",
            snap.time, snap.altitudeMsl_m,
            snap.feVelocity_m_s.norm(), snap.mach);

        return StepObservation{
            .time_s = snap.time,
            .altitude_m = snap.altitudeMsl_m,
            .speed_mps = snap.feVelocity_m_s.norm(),
            .latitude_rad = snap.latitude_rad,
            .longitude_rad = snap.longitude_rad,
            .converged = res.converged,
            .residual = res.residual_norm,
        };
    }

    void DraglessSphereApplication::logFinalSummary() const
    {
        const auto snap = m_Simulator->snapshot();
        constexpr double kToDeg = 180.0 / 3.14159265358979323846;

        AE_CORE_INFO("=======================================================");
        AE_CORE_INFO("Simulation complete.");
        AE_CORE_INFO("  Final time      : {:.6f} s", snap.time);
        AE_CORE_INFO("  Final altitude  : {:.4f} m", snap.altitudeMsl_m);
        AE_CORE_INFO("  Final |v_NED|   : {:.6f} m/s", snap.feVelocity_m_s.norm());
        AE_CORE_INFO("  Final latitude  : {:.8f} deg", snap.latitude_rad * kToDeg);
        AE_CORE_INFO("  Final longitude : {:.8f} deg", snap.longitude_rad * kToDeg);
        AE_CORE_INFO("  Final Mach      : {:.6f}", snap.mach);
        AE_CORE_INFO("  Final q (body→ECI) : [{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
            snap.q_body_to_eci.w(), snap.q_body_to_eci.x(),
            snap.q_body_to_eci.y(), snap.q_body_to_eci.z());
        AE_CORE_INFO("Output written to '{}'", getConfig().outputFileName);
        AE_CORE_INFO("=======================================================");
    }

    // =========================================================================
    // Private helpers
    // =========================================================================

    RigidBody::Config DraglessSphereApplication::loadVehicleConfig() const
    {
        const auto& fname = getConfig().inputFileName;
        if (fname.empty())
            throw std::runtime_error(
                "No --inputFileName specified. "
                "Provide the path to the vehicle JSON config.");

        AE_CORE_INFO("Loading vehicle config from '{}'", fname);
        const RigidBody::Config cfg = Serialization::LoadConfig(fname);

        AE_CORE_TRACE("Vehicle config loaded:");
        AE_CORE_TRACE("  lat={:.4f} deg  lon={:.4f} deg  alt={:.2f} m",
            cfg.pose.lat_deg, cfg.pose.lon_deg, cfg.pose.alt_m);
        AE_CORE_TRACE("  azimuth={:.2f} deg  zenith={:.2f} deg  roll={:.2f} deg",
            cfg.pose.azimuth_deg, cfg.pose.zenith_deg, cfg.pose.roll_deg);
        AE_CORE_TRACE("  v_NED  = [{:.4f}, {:.4f}, {:.4f}] m/s",
            cfg.velocityNED.north_mps, cfg.velocityNED.east_mps, cfg.velocityNED.down_mps);
        AE_CORE_TRACE("  mass   = {:.4f} kg", cfg.inertialParameters.mass_kg);

        return cfg;
    }

    double DraglessSphereApplication::computeInitialERA(double t0) noexcept
    {
        const double theta0 = DraglessSphereSimulator::kOmegaEarth_rad_s * t0;
        AE_CORE_TRACE("Initial ERA theta0 = {:.10f} rad  ({:.6f} deg)",
            theta0, theta0 * (180.0 / 3.14159265358979323846));
        return theta0;
    }

    RigidBody::StateD DraglessSphereApplication::buildInitialState(
        const RigidBody::Config& rb_cfg, double theta0)
    {
        AE_CORE_INFO("Building initial state vector (ECI frame)...");
        const auto x0 = FlightDynamics::BuildInitialStateVector(rb_cfg, theta0);

        RigidBody::StateD state;
        using Layout = RigidBody::StateLayout;

        state.g.p = Eigen::Vector3d(
            x0[Layout::IDX_P], x0[Layout::IDX_P + 1], x0[Layout::IDX_P + 2]);

        const Eigen::Quaterniond q(
            x0[Layout::IDX_Q],      // w
            x0[Layout::IDX_Q + 1],  // x
            x0[Layout::IDX_Q + 2],  // y
            x0[Layout::IDX_Q + 3]); // z
        state.g.R = q.normalized().toRotationMatrix();

        state.nu_B <<
            x0[Layout::IDX_W], x0[Layout::IDX_W + 1], x0[Layout::IDX_W + 2],
            x0[Layout::IDX_V], x0[Layout::IDX_V + 1], x0[Layout::IDX_V + 2];

        state.m = x0[Layout::IDX_M];
        return state;
    }

    // Step 4 --------------------------------------------------------------
    std::unique_ptr<DraglessSphereSimulator>
        DraglessSphereApplication::constructSimulator(
            const RigidBody::InertialParameters& ip,
            const RigidBody::StateD& x0,
            double                               theta0)
    {
        AE_CORE_INFO("Constructing DraglessSphereSimulator (J2 gravity)...");
        return std::make_unique<DraglessSphereSimulator>(ip, x0, theta0);
    }

} // namespace Aetherion::Examples::DraglessSphere

// -----------------------------------------------------------------------------
// Client factory — called by the EntryPoint-generated main().
// -----------------------------------------------------------------------------
namespace Aetherion::Simulation {

    Application* CreateApplication(int argc, char* argv[])
    {
        return new Examples::DraglessSphere::DraglessSphereApplication(argc, argv);
    }

} // namespace Aetherion::Simulation

#include <Aetherion/Simulation/EntryPoint.h>  
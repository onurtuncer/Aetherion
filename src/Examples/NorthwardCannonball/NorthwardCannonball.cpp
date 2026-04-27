// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// NorthwardCannonball.cpp
//
// Entry point for the NASA TM-2015-218675 Atmospheric Scenario 10 executable.
//
// Usage:
//   NorthwardCannonball.exe \
//       --inputFileName  nasa_2015_scenario10_northward_cannonball.json \
//       --outputFileName atmos_10_output.csv                            \
//       --startTime      0.0                                            \
//       --endTime        30.0                                           \
//       --timeStep       0.002                                          \
//       --writeInterval  50
//

#include <Aetherion/Examples/NorthwardCannonball/NorthwardCannonballApplication.h>

#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Serialization/LoadConfig.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/RigidBody/StateLayout.h>

#include <cmath>
#include <stdexcept>

namespace Aetherion::Examples::NorthwardCannonball {

    void NorthwardCannonballApplication::logStartupBanner() const
    {
        AE_CORE_INFO("========================================================");
        AE_CORE_INFO("Aetherion - NASA TM-2015-218675 Atmospheric Scenario 10");
        AE_CORE_INFO("Northward cannonball - J2 gravity - sphere with drag");
        AE_CORE_INFO("  alt=0 m  v_N=304.8 m/s  v_U=304.8 m/s  CD=0.1");
        AE_CORE_INFO("========================================================");
    }

    void NorthwardCannonballApplication::prepareSimulation() const
    {
        const auto& cfg = getConfig();

        const RigidBody::Config rb_cfg = loadVehicleConfig();
        const double            theta0 = computeInitialERA(cfg.startTime);
        const RigidBody::StateD x0     = buildInitialState(rb_cfg, theta0);

        AE_CORE_INFO("Initial ECI position  = [{:.3f}, {:.3f}, {:.3f}] m",
            x0.g.p.x(), x0.g.p.y(), x0.g.p.z());
        AE_CORE_INFO("Initial body twist nu_B = [{:.6e}, {:.6e}, {:.6e}, {:.6e}, {:.6e}, {:.6e}]",
            x0.nu_B(0), x0.nu_B(1), x0.nu_B(2),
            x0.nu_B(3), x0.nu_B(4), x0.nu_B(5));
        AE_CORE_INFO("Drag: CD={:.4f}  S_ref={:.6f} m²",
            rb_cfg.aerodynamicParameters.CD, rb_cfg.aerodynamicParameters.S);

        m_Simulator = constructSimulator(
            rb_cfg.inertialParameters, rb_cfg.aerodynamicParameters, x0, theta0);
    }

    void NorthwardCannonballApplication::writeInitialSnapshot(std::ofstream& csv) const
    {
        const auto snap = m_Simulator->snapshot();
        Simulation::Snapshot1_WriteCsvRow(csv, snap);
        AE_CORE_INFO("t={:.6f} s  alt={:.3f} m  |v_NED|={:.6f} m/s",
            snap.time, snap.altitudeMsl_m, snap.feVelocity_m_s.norm());
    }

    auto NorthwardCannonballApplication::stepAndRecord(
        std::ofstream& csv, double h, bool doWrite) const -> StepObservation
    {
        const auto res  = m_Simulator->step(h);
        const auto snap = m_Simulator->snapshot();
        if (doWrite)
            Simulation::Snapshot1_WriteCsvRow(csv, snap);

        AE_CORE_TRACE("  t={:.6f} s  alt={:.4f} m  v={:.6f} m/s  Mach={:.6f}",
            snap.time, snap.altitudeMsl_m,
            snap.feVelocity_m_s.norm(), snap.mach);

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

    void NorthwardCannonballApplication::logFinalSummary() const
    {
        const auto snap = m_Simulator->snapshot();
        constexpr double kToDeg = 180.0 / 3.14159265358979323846;

        AE_CORE_INFO("========================================================");
        AE_CORE_INFO("Simulation complete.");
        AE_CORE_INFO("  Final time      : {:.6f} s",  snap.time);
        AE_CORE_INFO("  Final altitude  : {:.4f} m",  snap.altitudeMsl_m);
        AE_CORE_INFO("  Final |v_NED|   : {:.6f} m/s", snap.feVelocity_m_s.norm());
        AE_CORE_INFO("  Final latitude  : {:.8f} deg", snap.latitude_rad  * kToDeg);
        AE_CORE_INFO("  Final longitude : {:.8f} deg", snap.longitude_rad * kToDeg);
        AE_CORE_INFO("  Final Mach      : {:.6f}",    snap.mach);
        AE_CORE_INFO("Output written to '{}'", getConfig().outputFileName);
        AE_CORE_INFO("========================================================");
    }

    // =========================================================================
    // Private helpers
    // =========================================================================

    RigidBody::Config NorthwardCannonballApplication::loadVehicleConfig() const
    {
        const auto& fname = getConfig().inputFileName;
        if (fname.empty())
            throw std::runtime_error("No --inputFileName specified.");

        AE_CORE_INFO("Loading vehicle config from '{}'", fname);
        const RigidBody::Config cfg = Serialization::LoadConfig(fname);

        AE_CORE_TRACE("  lat={:.4f} deg  lon={:.4f} deg  alt={:.2f} m",
            cfg.pose.lat_deg, cfg.pose.lon_deg, cfg.pose.alt_m);
        AE_CORE_TRACE("  v_NED = [{:.2f}, {:.2f}, {:.2f}] m/s",
            cfg.velocityNED.north_mps, cfg.velocityNED.east_mps, cfg.velocityNED.down_mps);
        return cfg;
    }

    double NorthwardCannonballApplication::computeInitialERA(double t0) noexcept
    {
        return SWADSim::kOmegaEarth_rad_s * t0;
    }

    RigidBody::StateD NorthwardCannonballApplication::buildInitialState(
        const RigidBody::Config& rb_cfg, double theta0)
    {
        AE_CORE_INFO("Building initial state vector (ECI frame)...");
        const auto x0 = FlightDynamics::BuildInitialStateVector(rb_cfg, theta0);

        RigidBody::StateD state;
        using Layout = RigidBody::StateLayout;

        state.g.p = Eigen::Vector3d(
            x0[Layout::IDX_P], x0[Layout::IDX_P + 1], x0[Layout::IDX_P + 2]);

        const Eigen::Quaterniond q(
            x0[Layout::IDX_Q], x0[Layout::IDX_Q + 1],
            x0[Layout::IDX_Q + 2], x0[Layout::IDX_Q + 3]);
        state.g.R = q.normalized().toRotationMatrix();

        state.nu_B <<
            x0[Layout::IDX_W], x0[Layout::IDX_W + 1], x0[Layout::IDX_W + 2],
            x0[Layout::IDX_V], x0[Layout::IDX_V + 1], x0[Layout::IDX_V + 2];

        state.m = x0[Layout::IDX_M];
        return state;
    }

    std::unique_ptr<SWADSim>
        NorthwardCannonballApplication::constructSimulator(
            const RigidBody::InertialParameters&    ip,
            const RigidBody::AerodynamicParameters& aero,
            const RigidBody::StateD&                x0,
            double                                  theta0)
    {
        AE_CORE_INFO("Constructing SphereWithAtmosphericDragSimulator "
                     "(J2 gravity, CD={:.4f}, S_ref={:.6f} m²)...",
            aero.CD, aero.S);
        return std::make_unique<SWADSim>(ip, aero, x0, theta0);
    }

} // namespace Aetherion::Examples::NorthwardCannonball

namespace Aetherion::Simulation {
    Application* CreateApplication(int argc, char* argv[])
    {
        return new Examples::NorthwardCannonball::NorthwardCannonballApplication(argc, argv);
    }
} // namespace Aetherion::Simulation

#include <Aetherion/Simulation/EntryPoint.h>

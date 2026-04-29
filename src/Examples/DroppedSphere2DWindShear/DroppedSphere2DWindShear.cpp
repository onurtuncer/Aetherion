// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// DroppedSphere2DWindShear.cpp  —  NASA TM-2015-218675 Atmospheric Scenario 8
//

#include <Aetherion/Examples/DroppedSphere2DWindShear/DroppedSphere2DWindShearApplication.h>

#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Serialization/LoadConfig.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/RigidBody/StateLayout.h>

#include <cmath>
#include <stdexcept>

namespace Aetherion::Examples::DroppedSphere2DWindShear {

    void DroppedSphere2DWindShearApplication::logStartupBanner() const
    {
        AE_CORE_INFO("=======================================================");
        AE_CORE_INFO("Aetherion - NASA TM-2015-218675 Atmospheric Scenario 8");
        AE_CORE_INFO("Dropped sphere - 2D wind shear - J2 gravity");
        AE_CORE_INFO("  v_E(h) = 21.336*(h/9144)^(4/3) m/s  [70 ft/s at 30 000 ft]");
        AE_CORE_INFO("=======================================================");
    }

    void DroppedSphere2DWindShearApplication::prepareSimulation() const
    {
        const auto& cfg = getConfig();
        const RigidBody::Config rb_cfg = loadVehicleConfig();
        const double            theta0 = computeInitialERA(cfg.startTime);
        const RigidBody::StateD x0     = buildInitialState(rb_cfg, theta0);

        AE_CORE_INFO("Initial ECI position  = [{:.3f}, {:.3f}, {:.3f}] m",
            x0.g.p.x(), x0.g.p.y(), x0.g.p.z());
        AE_CORE_INFO("Wind shear: E_ref={:.4f} m/s  h_ref={:.1f} m  n={:.4f}",
            rb_cfg.windShear.east_ref_mps,
            rb_cfg.windShear.h_ref_m,
            rb_cfg.windShear.shear_exp);

        m_Simulator = constructSimulator(rb_cfg, x0, theta0);
    }

    void DroppedSphere2DWindShearApplication::writeInitialSnapshot(std::ofstream& csv) const
    {
        const auto snap = m_Simulator->snapshot();
        Simulation::Snapshot1_WriteCsvRow(csv, snap);
        AE_CORE_INFO("t={:.6f} s  alt={:.3f} m  TAS={:.4f} m/s",
            snap.time, snap.altitudeMsl_m, snap.trueAirspeed_m_s);
    }

    auto DroppedSphere2DWindShearApplication::stepAndRecord(
        std::ofstream& csv, double h, bool doWrite) const -> StepObservation
    {
        const auto res  = m_Simulator->step(h);
        const auto snap = m_Simulator->snapshot();
        if (doWrite)
            Simulation::Snapshot1_WriteCsvRow(csv, snap);

        AE_CORE_TRACE("  t={:.6f} s  alt={:.4f} m  v={:.6f} m/s  TAS={:.4f} m/s",
            snap.time, snap.altitudeMsl_m,
            snap.feVelocity_m_s.norm(), snap.trueAirspeed_m_s);

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

    void DroppedSphere2DWindShearApplication::logFinalSummary() const
    {
        const auto snap = m_Simulator->snapshot();
        constexpr double kToDeg = 180.0 / 3.14159265358979323846;

        AE_CORE_INFO("=======================================================");
        AE_CORE_INFO("Simulation complete.");
        AE_CORE_INFO("  Final time      : {:.6f} s",  snap.time);
        AE_CORE_INFO("  Final altitude  : {:.4f} m",  snap.altitudeMsl_m);
        AE_CORE_INFO("  Final |v_NED|   : {:.6f} m/s", snap.feVelocity_m_s.norm());
        AE_CORE_INFO("  Final TAS       : {:.6f} m/s", snap.trueAirspeed_m_s);
        AE_CORE_INFO("  Final latitude  : {:.8f} deg", snap.latitude_rad  * kToDeg);
        AE_CORE_INFO("  Final longitude : {:.8f} deg", snap.longitude_rad * kToDeg);
        AE_CORE_INFO("Output written to '{}'", getConfig().outputFileName);
        AE_CORE_INFO("=======================================================");
    }

    // =========================================================================

    RigidBody::Config DroppedSphere2DWindShearApplication::loadVehicleConfig() const
    {
        const auto& fname = getConfig().inputFileName;
        if (fname.empty())
            throw std::runtime_error("No --inputFileName specified.");
        AE_CORE_INFO("Loading vehicle config from '{}'", fname);
        return Serialization::LoadConfig(fname);
    }

    double DroppedSphere2DWindShearApplication::computeInitialERA(double t0) noexcept
    {
        return DroppedSphere2DWindShearSimulator::kOmegaEarth_rad_s * t0;
    }

    RigidBody::StateD DroppedSphere2DWindShearApplication::buildInitialState(
        const RigidBody::Config& rb_cfg, double theta0)
    {
        AE_CORE_INFO("Building initial state vector (ECI frame)...");
        const auto x0 = FlightDynamics::BuildInitialStateVector(rb_cfg, theta0);

        RigidBody::StateD state;
        using Layout = RigidBody::StateLayout;

        state.g.p = Eigen::Vector3d(
            x0[Layout::IDX_P], x0[Layout::IDX_P+1], x0[Layout::IDX_P+2]);

        const Eigen::Quaterniond q(
            x0[Layout::IDX_Q], x0[Layout::IDX_Q+1],
            x0[Layout::IDX_Q+2], x0[Layout::IDX_Q+3]);
        state.g.R = q.normalized().toRotationMatrix();

        state.nu_B <<
            x0[Layout::IDX_W], x0[Layout::IDX_W+1], x0[Layout::IDX_W+2],
            x0[Layout::IDX_V], x0[Layout::IDX_V+1], x0[Layout::IDX_V+2];

        state.m = x0[Layout::IDX_M];
        return state;
    }

    std::unique_ptr<DroppedSphere2DWindShearSimulator>
        DroppedSphere2DWindShearApplication::constructSimulator(
            const RigidBody::Config& rb_cfg,
            const RigidBody::StateD& x0,
            double                   theta0)
    {
        constexpr double kDeg = 3.14159265358979323846 / 180.0;
        const double lat0 = rb_cfg.pose.lat_deg * kDeg;
        const double lon0 = rb_cfg.pose.lon_deg * kDeg;

        AE_CORE_INFO("Constructing DroppedSphere2DWindShearSimulator...");
        return std::make_unique<DroppedSphere2DWindShearSimulator>(
            rb_cfg.inertialParameters,
            rb_cfg.aerodynamicParameters,
            rb_cfg.windShear,
            lat0, lon0,
            x0, theta0);
    }

} // namespace Aetherion::Examples::DroppedSphere2DWindShear

namespace Aetherion::Simulation {
    Application* CreateApplication(int argc, char* argv[])
    {
        return new Examples::DroppedSphere2DWindShear::DroppedSphere2DWindShearApplication(argc, argv);
    }
}

#include <Aetherion/Simulation/EntryPoint.h>

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TwoStageRocket.cpp  —  NASA TM-2015-218675 Atmospheric Scenario 17
//
// Two-stage rocket to orbit — gravity-turn ascent from the equator.
//
// Initial conditions (from Atmos_17_sim_06.csv, t = 0):
//   Location  : lat = 0°, lon = 0°, alt = 0 m  (equatorial launch site)
//   Heading   : 90° (due East) — maximise orbital inclination reduction
//   Pitch     : 55.22° from horizontal (= zenith 34.78° from vertical)
//   Roll      : 0°
//   Velocity  : stationary relative to Earth (ECEF v = 0)
//   Body rates: ECEF-relative = 0 (Earth rotation adds ~7.29e-5 rad/s to pitch)
//   Mass      : liftoff XMASS from inertia DML at stg1fuelUsed = 0
// ------------------------------------------------------------------------------

#include <Aetherion/Examples/TwoStageRocket/TwoStageRocketApplication.h>
#include <Aetherion/Examples/TwoStageRocket/TwoStageRocketSimulator.h>
#include <Aetherion/Examples/TwoStageRocket/RocketStageModel.h>

#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/SnapshotTraits.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>
#include <Aetherion/RigidBody/Config.h>
#include <Aetherion/RigidBody/StateLayout.h>
#include <Aetherion/Environment/WGS84.h>

#include <numbers>
#include <cmath>
#include <stdexcept>

// ── DML file paths — injected by CMake at configure time ─────────────────────
#ifndef TWOSTAGE_INERTIA_FILE
#  define TWOSTAGE_INERTIA_FILE ""
#endif
#ifndef TWOSTAGE_PROP_FILE
#  define TWOSTAGE_PROP_FILE ""
#endif
#ifndef TWOSTAGE_AERO_FILE
#  define TWOSTAGE_AERO_FILE ""
#endif

// ── Scenario 17 initial conditions ───────────────────────────────────────────
namespace {
    // Launch from the equator at the prime meridian
    constexpr double kLat_deg      =   0.0;
    constexpr double kLon_deg      =   0.0;
    constexpr double kAlt_m        =   0.0;   // sea level

    // Initial attitude: yaw East (90°), pitch 55.22° up from horizontal
    constexpr double kAzimuth_deg  =  90.0;   // pointing East
    constexpr double kPitch_deg    =  55.22;  // nose up from horizontal
    constexpr double kZenith_deg   =  90.0 - kPitch_deg;  // = 34.78° from vertical
    constexpr double kRoll_deg     =   0.0;

    // Rocket is stationary on the launch pad (ECEF velocity = 0)
    constexpr double kVnorth_mps   =   0.0;
    constexpr double kVeast_mps    =   0.0;
    constexpr double kVdown_mps    =   0.0;

    // Body angular rates relative to ECEF = 0 (Earth rotation adds ~ωE to body)
    constexpr double kRollRate_rps  =  0.0;
    constexpr double kPitchRate_rps =  0.0;
    constexpr double kYawRate_rps   =  0.0;

    constexpr double kOmegaE = Aetherion::Environment::WGS84::kRotationRate_rad_s;
}

namespace Aetherion::Examples::TwoStageRocket {

// ── logStartupBanner ──────────────────────────────────────────────────────────

void TwoStageRocketApplication::logStartupBanner() const
{
    AE_CORE_INFO("=======================================================");
    AE_CORE_INFO("Aetherion - NASA TM-2015-218675 Atmospheric Scenario 17");
    AE_CORE_INFO("Two-stage rocket to orbit — equatorial gravity-turn ascent");
    AE_CORE_INFO("  Launch site : {:.1f}° N, {:.1f}° E  (equatorial)", kLat_deg, kLon_deg);
    AE_CORE_INFO("  Altitude    : {:.0f} m  (sea level)", kAlt_m);
    AE_CORE_INFO("  Azimuth     : {:.1f}°  (due East)", kAzimuth_deg);
    AE_CORE_INFO("  Pitch       : {:.3f}° from horizontal", kPitch_deg);
    AE_CORE_INFO("=======================================================");
}

// ── writeCsvHeader — Snapshot2 (31-column NASA format) ───────────────────────

void TwoStageRocketApplication::writeCsvHeader(std::ofstream& csv) const
{
    Simulation::Snapshot2Traits::write_header(csv);
}

// ── prepareSimulation ─────────────────────────────────────────────────────────

void TwoStageRocketApplication::prepareSimulation() const
{
    const auto& simCfg = getConfig();

    // ── 1. Check DML paths ────────────────────────────────────────────────────
    const std::string inertiaPath = TWOSTAGE_INERTIA_FILE;
    const std::string propPath    = TWOSTAGE_PROP_FILE;
    const std::string aeroPath    = TWOSTAGE_AERO_FILE;

    if (inertiaPath.empty()) throw std::runtime_error("TWOSTAGE_INERTIA_FILE not configured.");
    if (propPath.empty())    throw std::runtime_error("TWOSTAGE_PROP_FILE not configured.");
    if (aeroPath.empty())    throw std::runtime_error("TWOSTAGE_AERO_FILE not configured.");

    // ── 2. Load DML engines ───────────────────────────────────────────────────
    AE_CORE_INFO("Loading inertia DML from '{}'", inertiaPath);
    auto inertiaDml = std::make_shared<Serialization::DAVEMLAeroModel>(inertiaPath);

    AE_CORE_INFO("Loading propulsion DML from '{}'", propPath);
    auto propDml    = std::make_shared<Serialization::DAVEMLAeroModel>(propPath);

    AE_CORE_INFO("Loading aero DML from '{}'", aeroPath);
    auto aeroDml    = std::make_shared<const Serialization::DAVEMLAeroModel>(aeroPath);

    // ── 3. Query liftoff inertial parameters from inertia DML ─────────────────
    //    Use a temporary RocketStageModel at initial (zero fuel) state.
    RocketStageModel stageModel0(inertiaDml, propDml);
    const RigidBody::InertialParameters ip = stageModel0.inertialParameters();

    AE_CORE_INFO("Liftoff inertia:");
    AE_CORE_INFO("  XMASS = {:.1f} kg", ip.mass_kg);
    AE_CORE_INFO("  Ixx   = {:.3e} kg·m²", ip.Ixx);
    AE_CORE_INFO("  Iyy   = {:.3e} kg·m²  (= Izz)", ip.Iyy);
    AE_CORE_INFO("  DXCG  = {:.4f} m  (CG forward of MRC)", ip.xbar_m);

    // ── 4. Build initial 6-DoF state ──────────────────────────────────────────
    const double theta0 = kOmegaE * simCfg.startTime;

    RigidBody::Config cfg{};
    cfg.pose.lat_deg      = kLat_deg;
    cfg.pose.lon_deg      = kLon_deg;
    cfg.pose.alt_m        = kAlt_m;
    cfg.pose.azimuth_deg  = kAzimuth_deg;
    cfg.pose.zenith_deg   = kZenith_deg;
    cfg.pose.roll_deg     = kRoll_deg;
    cfg.velocityNED.north_mps = kVnorth_mps;
    cfg.velocityNED.east_mps  = kVeast_mps;
    cfg.velocityNED.down_mps  = kVdown_mps;
    cfg.bodyRates.roll_rad_s  = kRollRate_rps;
    cfg.bodyRates.pitch_rad_s = kPitchRate_rps;
    cfg.bodyRates.yaw_rad_s   = kYawRate_rps;
    cfg.inertialParameters    = ip;

    const auto x0_flat = FlightDynamics::BuildInitialStateVector(cfg, theta0);

    RigidBody::StateD x0{};
    using Layout = RigidBody::StateLayout;
    x0.g.p = Eigen::Vector3d(
        x0_flat[Layout::IDX_P],
        x0_flat[Layout::IDX_P + 1],
        x0_flat[Layout::IDX_P + 2]);

    const Eigen::Quaterniond q(
        x0_flat[Layout::IDX_Q],
        x0_flat[Layout::IDX_Q + 1],
        x0_flat[Layout::IDX_Q + 2],
        x0_flat[Layout::IDX_Q + 3]);
    x0.g.q = q.normalized();
    x0.g.R = x0.g.q.toRotationMatrix();

    x0.nu_B <<
        x0_flat[Layout::IDX_W],
        x0_flat[Layout::IDX_W + 1],
        x0_flat[Layout::IDX_W + 2],
        x0_flat[Layout::IDX_V],
        x0_flat[Layout::IDX_V + 1],
        x0_flat[Layout::IDX_V + 2];

    x0.m = x0_flat[Layout::IDX_M];

    AE_CORE_INFO("Initial ECI position = [{:.3f}, {:.3f}, {:.3f}] m",
                 x0.g.p.x(), x0.g.p.y(), x0.g.p.z());
    AE_CORE_INFO("Initial mass         = {:.1f} kg", x0.m);

    // ── 5. Construct simulator ────────────────────────────────────────────────
    AE_CORE_INFO("Constructing TwoStageRocketSimulator...");
    m_Simulator = std::make_unique<TwoStageRocketSimulator>(
        ip, x0, theta0,
        std::move(inertiaDml),
        std::move(propDml),
        std::move(aeroDml));
}

// ── writeInitialSnapshot ──────────────────────────────────────────────────────

void TwoStageRocketApplication::writeInitialSnapshot(std::ofstream& csv) const
{
    const auto s2 = m_Simulator->snapshot2();
    Simulation::Snapshot2Traits::write_row(csv, s2);

    const auto& fuel = m_Simulator->stageModel().fuelState();
    AE_CORE_INFO("t={:.3f} s  alt={:.1f} m  TAS={:.2f} m/s  "
                 "mass={:.1f} kg  staged={}",
        s2.time, s2.altitudeMsl_m, s2.trueAirspeed_m_s,
        m_Simulator->state().m,
        fuel.staged ? "YES" : "no");
}

// ── stepAndRecord ─────────────────────────────────────────────────────────────

auto TwoStageRocketApplication::stepAndRecord(
    std::ofstream& csv, double h, bool doWrite) const -> StepObservation
{
    const auto res = m_Simulator->step(h);
    const auto s2  = m_Simulator->snapshot2();

    if (doWrite)
        Simulation::Snapshot2Traits::write_row(csv, s2);

    AE_CORE_TRACE("  t={:.3f} s  alt={:.0f} m  TAS={:.1f} m/s  mass={:.0f} kg",
        s2.time, s2.altitudeMsl_m, s2.trueAirspeed_m_s,
        m_Simulator->state().m);

    return StepObservation{
        .time_s        = s2.time,
        .altitude_m    = s2.altitudeMsl_m,
        .speed_mps     = s2.feVelocity_m_s.norm(),
        .latitude_rad  = s2.latitude_rad,
        .longitude_rad = s2.longitude_rad,
        .converged     = res.converged,
        .residual      = res.residual_norm,
    };
}

// ── logFinalSummary ───────────────────────────────────────────────────────────

void TwoStageRocketApplication::logFinalSummary() const
{
    const auto s2  = m_Simulator->snapshot2();
    const auto& fuel = m_Simulator->stageModel().fuelState();
    constexpr double kToDeg = 180.0 / std::numbers::pi;

    AE_CORE_INFO("=======================================================");
    AE_CORE_INFO("Simulation complete.");
    AE_CORE_INFO("  Final time        : {:.3f} s",   s2.time);
    AE_CORE_INFO("  Final altitude    : {:.1f} m",   s2.altitudeMsl_m);
    AE_CORE_INFO("  Final TAS         : {:.2f} m/s", s2.trueAirspeed_m_s);
    AE_CORE_INFO("  Final ECEF speed  : {:.2f} m/s", s2.feVelocity_m_s.norm());
    AE_CORE_INFO("  Final mass        : {:.1f} kg",  m_Simulator->state().m);
    AE_CORE_INFO("  Stage 1 fuel used : {:.1f} kg",  fuel.stg1FuelUsed_kg);
    AE_CORE_INFO("  Stage 2 fuel used : {:.1f} kg",  fuel.stg2FuelUsed_kg);
    AE_CORE_INFO("  Staged            : {}",          fuel.staged ? "YES" : "no");
    AE_CORE_INFO("  Final latitude    : {:.4f} deg",  s2.latitude_rad  * kToDeg);
    AE_CORE_INFO("  Final longitude   : {:.4f} deg",  s2.longitude_rad * kToDeg);
    AE_CORE_INFO("Output written to '{}'", getConfig().outputFileName);
    AE_CORE_INFO("=======================================================");
}

} // namespace Aetherion::Examples::TwoStageRocket

// ── Entry point ───────────────────────────────────────────────────────────────

namespace Aetherion::Simulation {
    Application* CreateApplication(int argc, char* argv[])
    {
        return new Examples::TwoStageRocket::TwoStageRocketApplication(argc, argv);
    }
}

#include <Aetherion/Simulation/EntryPoint.h>

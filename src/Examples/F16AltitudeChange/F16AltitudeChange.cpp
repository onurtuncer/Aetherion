// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16AltitudeChange.cpp  —  NASA TM-2015-218675 Atmospheric Scenario 13.1
//
// F-16 subsonic altitude change (+100 ft step from Case-11 trim).
// Closed-loop with LQR SAS + autopilot (F16_control.dml).
//

#include <Aetherion/Examples/F16AltitudeChange/F16AltitudeChangeApplication.h>

#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/SnapshotTraits.h>
#include <Aetherion/RigidBody/BuildInitialState.h>
#include <Aetherion/FlightDynamics/Trim/TrimSolver.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLPropModel.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLControlModel.h>
#include <Aetherion/Serialization/DAVEML/LoadInertiaFromDAVEML.h>
#include <Aetherion/RigidBody/Config.h>
#include <Aetherion/RigidBody/GeodeticPoseNED.h>
#include <Aetherion/RigidBody/StateLayout.h>
#include <Aetherion/Environment/Atmosphere.h>

#include <numbers>
#include <cmath>
#include <stdexcept>

// ── DML file paths — injected by CMake at configure time ─────────────────────
#ifndef F16_INERTIA_FILE
#  define F16_INERTIA_FILE ""
#endif
#ifndef F16_AERO_FILE
#  define F16_AERO_FILE ""
#endif
#ifndef F16_PROP_FILE
#  define F16_PROP_FILE ""
#endif
#ifndef F16_CONTROL_FILE
#  define F16_CONTROL_FILE ""
#endif

// ── Scenario 13.1 constants (identical to Case 11 initial conditions) ─────────
namespace {
    constexpr double kLat_deg      =  36.01917;
    constexpr double kLon_deg      = -75.67444;
    constexpr double kAlt_ft       =  10013.0;
    constexpr double kAlt_m        =  kAlt_ft * 0.3048;
    constexpr double kHeading_deg  =  45.0;
    constexpr double kRoll_deg     =  -0.172;
    constexpr double kVelocity_fps =  400.0;
    constexpr double kVelocity_mps =  kVelocity_fps * 0.3048;
    constexpr double kTAS_fps      =  565.685;
    constexpr double kG_mps2       =  9.80665;
    constexpr double kLbf_N        =  4.448221615260751;
    constexpr double kFtLbf_Nm     =  1.355817948329279;

    // CG offset: (35%-25%)/100 * 11.32 ft * 0.3048 m/ft
    constexpr double kXcgFromAC_m =  (35.0 - 25.0) / 100.0 * 11.32 * 0.3048;

    // Autopilot commands for Scenario 13.1.
    // keasCmd is computed at runtime from our US1976 atmosphere at the trim
    // altitude so that deltaVequiv ≈ 0 at t=0 (avoids spurious throttle
    // correction from the mismatch between our rho and the DML constant).
    constexpr double kAltCmd_ft      = 10113.0;  // +100 ft altitude step
    constexpr double kAltStepTime_s  =    5.0;  // NASA applies the step at t=5 s
    constexpr double kBaseChiCmd_deg =  45.0;   // hold NE heading
}

namespace Aetherion::Examples::F16AltitudeChange {

// ── logStartupBanner ──────────────────────────────────────────────────────────

void F16AltitudeChangeApplication::logStartupBanner() const
{
    AE_CORE_INFO("=======================================================");
    AE_CORE_INFO("Aetherion - NASA TM-2015-218675 Atmospheric Scenario 13.1");
    AE_CORE_INFO("F-16 subsonic altitude change (closed-loop, +100 ft step)");
    AE_CORE_INFO("  Location : {:.5f} N, {:.5f} E  (Kitty Hawk, NC)", kLat_deg, kLon_deg);
    AE_CORE_INFO("  Altitude : {:.0f} ft ({:.2f} m)", kAlt_ft, kAlt_m);
    AE_CORE_INFO("  Heading  : {:.1f} deg NE", kHeading_deg);
    AE_CORE_INFO("  TAS      : {:.2f} ft/s (335.15 KTAS)", kTAS_fps);
    AE_CORE_INFO("  AltCmd   : {:.0f} ft  (+100 ft step at t={:.0f} s)", kAltCmd_ft, kAltStepTime_s);
    AE_CORE_INFO("=======================================================");
}

// ── writeCsvHeader ────────────────────────────────────────────────────────────

void F16AltitudeChangeApplication::writeCsvHeader(std::ofstream& csv) const
{
    Simulation::Snapshot2Traits::write_header(csv);
}

// ── prepareSimulation ─────────────────────────────────────────────────────────

void F16AltitudeChangeApplication::prepareSimulation() const
{
    const auto& simCfg = getConfig();

    // ── 1. Load inertia ───────────────────────────────────────────────────────
    const std::string inertiaPath = F16_INERTIA_FILE;
    if (inertiaPath.empty())
        throw std::runtime_error("F16_INERTIA_FILE not configured.");
    AE_CORE_INFO("Loading inertia from '{}'", inertiaPath);
    const RigidBody::InertialParameters ip = Serialization::LoadInertiaFromDAVEML(inertiaPath);
    AE_CORE_INFO("  mass = {:.4f} kg", ip.mass_kg);

    // ── 2. Load aero, prop, and control models ────────────────────────────────
    const std::string aeroPath    = F16_AERO_FILE;
    const std::string propPath    = F16_PROP_FILE;
    const std::string controlPath = F16_CONTROL_FILE;
    if (aeroPath.empty() || propPath.empty())
        throw std::runtime_error("F16_AERO_FILE or F16_PROP_FILE not configured.");
    if (controlPath.empty())
        throw std::runtime_error("F16_CONTROL_FILE not configured.");

    AE_CORE_INFO("Loading aero    from '{}'", aeroPath);
    auto aero_model = std::make_shared<const Serialization::DAVEMLAeroModel>(aeroPath);
    AE_CORE_INFO("Loading prop    from '{}'", propPath);
    auto prop_model = std::make_shared<const Serialization::DAVEMLPropModel>(propPath);
    AE_CORE_INFO("Loading control from '{}'", controlPath);
    auto ctrl_model = std::make_shared<const Serialization::DAVEMLControlModel>(controlPath);

    // ── 3. Run trim solver (same as Case 11) ──────────────────────────────────
    const double weight_lbf = ip.mass_kg * kG_mps2 / kLbf_N;
    FlightDynamics::TrimInputs tin{};
    tin.vt_fps     = kTAS_fps;
    tin.alt_ft     = kAlt_ft;
    tin.weight_lbf = weight_lbf;

    AE_CORE_INFO("Running trim solver ...");
    const double xcg_ft = kXcgFromAC_m / FlightDynamics::TrimSolver::kFt_m;
    FlightDynamics::TrimSolver solver(*aero_model, *prop_model, xcg_ft);
    const FlightDynamics::TrimPoint trim = solver.solve(tin);

    if (!trim.converged)
        throw std::runtime_error("Trim solver did not converge.");

    AE_CORE_INFO("Trim converged:");
    AE_CORE_INFO("  alpha = {:.4f} deg", trim.alpha_deg);
    AE_CORE_INFO("  el    = {:.4f} deg", trim.el_deg);
    AE_CORE_INFO("  pwr   = {:.4f} %%",  trim.pwr_pct);

    // ── 4. Build initial state (same as Case 11) ──────────────────────────────
    RigidBody::Config cfg{};
    cfg.pose.lat_deg      = kLat_deg;
    cfg.pose.lon_deg      = kLon_deg;
    cfg.pose.alt_m        = kAlt_m;
    cfg.pose.azimuth_deg  = kHeading_deg;
    cfg.pose.zenith_deg   = 90.0 - trim.alpha_deg;
    cfg.pose.roll_deg     = kRoll_deg;
    cfg.velocityNED.north_mps = kVelocity_mps;
    cfg.velocityNED.east_mps  = kVelocity_mps;
    cfg.velocityNED.down_mps  = 0.0;
    cfg.bodyRates.roll_rad_s  = 0.0;
    cfg.bodyRates.pitch_rad_s = 0.0;
    cfg.bodyRates.yaw_rad_s   = 0.0;
    cfg.inertialParameters    = ip;

    const double theta0 = computeInitialERA(simCfg.startTime);
    const RigidBody::StateD x0 = buildInitialState(cfg, theta0);

    // ── 5. Autopilot commands ─────────────────────────────────────────────────
    // Compute KEAS from our US1976 atmosphere so deltaVequiv ≈ 0 at t=0.
    // Using the same formula as F16AltitudeChangeSimulator::extractFeedback():
    //   KEAS [kt] = TAS [kt] × sqrt(rho / rho_SL)
    // TAS from the initial NED velocity magnitude: sqrt(vN² + vE²).
    const auto atm_trim = Environment::US1976Atmosphere(kAlt_m);
    constexpr double kRhoSL_kg_m3 = 1.225;
    constexpr double kKt_mps      = 0.5144444;
    const double tas_mps = kVelocity_mps * std::numbers::sqrt2;  // |[vN, vE, 0]|
    const double keas_kt = (tas_mps / kKt_mps) * std::sqrt(atm_trim.rho / kRhoSL_kg_m3);

    F16AltitudeChangeSimulator::AutopilotCmds cmds;
    cmds.altCmd_ft      = kAltCmd_ft;
    cmds.altStepTime_s  = kAltStepTime_s;
    cmds.initialAltCmd_ft = kAlt_ft;       // hold trim altitude before the step
    cmds.keasCmd_kt     = keas_kt;
    cmds.baseChiCmd_deg = kBaseChiCmd_deg;
    cmds.latOffset_ft   = 0.0;

    AE_CORE_INFO("Autopilot commands:");
    AE_CORE_INFO("  altCmd   = {:.0f} ft  (step at t={:.0f} s, holding {:.0f} ft before)",
                 cmds.altCmd_ft, cmds.altStepTime_s, cmds.initialAltCmd_ft);
    AE_CORE_INFO("  keasCmd  = {:.4f} kt  (computed from US1976 atm at {:.0f} ft)",
                 keas_kt, kAlt_ft);
    AE_CORE_INFO("  chiCmd   = {:.1f} deg", cmds.baseChiCmd_deg);

    m_Simulator = constructSimulator(ip, trim, aero_model, prop_model, ctrl_model,
                                     x0, theta0, cmds, kXcgFromAC_m);
}

// ── writeInitialSnapshot ──────────────────────────────────────────────────────

void F16AltitudeChangeApplication::writeInitialSnapshot(std::ofstream& csv) const
{
    const auto s2 = m_Simulator->snapshot2();
    Simulation::Snapshot2Traits::write_row(csv, s2);
    AE_CORE_INFO("t={:.6f} s  alt={:.3f} m  TAS={:.4f} m/s  Mach={:.4f}",
        s2.time, s2.altitudeMsl_m, s2.trueAirspeed_m_s, s2.mach);
}

// ── stepAndRecord ─────────────────────────────────────────────────────────────

auto F16AltitudeChangeApplication::stepAndRecord(
    std::ofstream& csv, double h, bool doWrite) const -> StepObservation
{
    // Closed-loop step: controller evaluates → surfaces update → ODE advances
    const auto res = m_Simulator->step(h);
    const auto s2  = m_Simulator->snapshot2();

    if (doWrite)
        Simulation::Snapshot2Traits::write_row(csv, s2);

    AE_CORE_TRACE("  t={:.4f} s  alt={:.2f} m  TAS={:.4f} m/s",
        s2.time, s2.altitudeMsl_m, s2.trueAirspeed_m_s);

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

void F16AltitudeChangeApplication::logFinalSummary() const
{
    const auto s2 = m_Simulator->snapshot2();
    constexpr double kToDeg = 180.0 / std::numbers::pi;
    constexpr double kToFt  = 1.0 / 0.3048;

    AE_CORE_INFO("=======================================================");
    AE_CORE_INFO("Simulation complete.");
    AE_CORE_INFO("  Final time      : {:.4f} s",   s2.time);
    AE_CORE_INFO("  Final altitude  : {:.2f} m  ({:.1f} ft)",
                 s2.altitudeMsl_m, s2.altitudeMsl_m * kToFt);
    AE_CORE_INFO("  Altitude change : {:.2f} m  ({:.1f} ft)",
                 s2.altitudeMsl_m - kAlt_m,
                 (s2.altitudeMsl_m - kAlt_m) * kToFt);
    AE_CORE_INFO("  Final TAS       : {:.4f} m/s", s2.trueAirspeed_m_s);
    AE_CORE_INFO("  Final Mach      : {:.6f}",     s2.mach);
    AE_CORE_INFO("  Final pitch     : {:.4f} deg", s2.eulerAngle_rad_Pitch * kToDeg);
    AE_CORE_INFO("  Final roll      : {:.4f} deg", s2.eulerAngle_rad_Roll  * kToDeg);
    AE_CORE_INFO("Output written to '{}'", getConfig().outputFileName);
    AE_CORE_INFO("=======================================================");
}

// ── Private helpers ───────────────────────────────────────────────────────────

double F16AltitudeChangeApplication::computeInitialERA(double t0) noexcept
{
    constexpr double kOmegaE = 7.2921150e-5;
    return kOmegaE * t0;
}

RigidBody::StateD F16AltitudeChangeApplication::buildInitialState(
    const RigidBody::Config& cfg, double theta0)
{
    const auto x0 = RigidBody::BuildInitialStateVector(cfg, theta0);

    RigidBody::StateD state;
    using Layout = RigidBody::StateLayout;

    state.g.p = Eigen::Vector3d(
        x0[Layout::IDX_P], x0[Layout::IDX_P + 1], x0[Layout::IDX_P + 2]);

    const Eigen::Quaterniond q(
        x0[Layout::IDX_Q], x0[Layout::IDX_Q + 1],
        x0[Layout::IDX_Q + 2], x0[Layout::IDX_Q + 3]);
    state.g.q = q.normalized();
    state.g.R = state.g.q.toRotationMatrix();

    state.nu_B <<
        x0[Layout::IDX_W],     x0[Layout::IDX_W + 1], x0[Layout::IDX_W + 2],
        x0[Layout::IDX_V],     x0[Layout::IDX_V + 1], x0[Layout::IDX_V + 2];

    state.m = x0[Layout::IDX_M];
    return state;
}

std::unique_ptr<F16AltitudeChangeSimulator>
F16AltitudeChangeApplication::constructSimulator(
    const RigidBody::InertialParameters&                      ip,
    const FlightDynamics::TrimPoint&                          trim,
    const std::shared_ptr<const Serialization::DAVEMLAeroModel>&     aero,
    const std::shared_ptr<const Serialization::DAVEMLPropModel>&     prop,
    const std::shared_ptr<const Serialization::DAVEMLControlModel>&  ctrl,
    const RigidBody::StateD& x0,
    double theta0,
    const F16AltitudeChangeSimulator::AutopilotCmds& cmds,
    double xcg_from_ac_m)
{
    AE_CORE_INFO("Constructing F16AltitudeChangeSimulator (closed-loop)...");
    AE_CORE_INFO("  xcg_from_ac = {:.4f} m", xcg_from_ac_m);
    return std::make_unique<F16AltitudeChangeSimulator>(
        ip, trim, aero, prop, ctrl, x0, theta0, cmds, xcg_from_ac_m);
}

} // namespace Aetherion::Examples::F16AltitudeChange

// ── Entry point ───────────────────────────────────────────────────────────────

namespace Aetherion::Simulation {
    Application* CreateApplication(int argc, char** argv)
    {
        return new Examples::F16AltitudeChange::F16AltitudeChangeApplication(argc, argv);
    }
}

#include <Aetherion/Simulation/EntryPoint.h>

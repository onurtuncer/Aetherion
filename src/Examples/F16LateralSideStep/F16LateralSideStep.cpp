// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16LateralSideStep.cpp  —  NASA TM-2015-218675 Atmospheric Scenario 13.4
//
// F-16 subsonic lateral side step: closed-loop LQR SAS + autopilot commands a
// 2 000 ft right-of-course offset from the Case-11 trim condition at t = 20 s.
//
// The lateral offset error (latOffset) is computed dynamically at each step
// from the aircraft's position relative to the original 45° NE courseline;
// the commanded offset (2 000 ft) is subtracted after t = 20 s.  As the
// aircraft crosses to the new parallel track, latOffset → 0 and the heading
// autopilot returns the aircraft to ψ ≈ 45°.
//
// Recommended step size: --timeStep 0.02
//

#include <Aetherion/Examples/F16LateralSideStep/F16LateralSideStepApplication.h>
#include <Aetherion/Examples/F16AltitudeChange/F16AltitudeChangeSimulator.h>

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

// ── DML file paths — injected by CMake ───────────────────────────────────────
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

// ── Scenario 13.4 constants ───────────────────────────────────────────────────
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
    constexpr double kXcgFromAC_m  =  (35.0 - 25.0) / 100.0 * 11.32 * 0.3048;

    // ── Scenario 13.4 autopilot commands ─────────────────────────────────────
    // Hold altitude and airspeed at trim; command a 2 000 ft lateral side step
    // to the right of the 45° NE courseline at t = 20 s.
    // NASA TM-2015-218675 §B-13.4 specifies the step is applied at t = 20 s.
    constexpr double kAltCmd_ft       = 10013.0;   // hold initial altitude [ft]
    constexpr double kBaseChiCmd_deg  =   45.0;    // hold NE course [deg]
    constexpr double kLatStep_ft      = 2000.0;    // lateral step right of course [ft]
    constexpr double kLatStepTime_s   =   20.0;    // step applied at t = 20 s
}

namespace Aetherion::Examples::F16LateralSideStep {

using Sim = F16AltitudeChange::F16AltitudeChangeSimulator;

// ── logStartupBanner ──────────────────────────────────────────────────────────

void F16LateralSideStepApplication::logStartupBanner() const
{
    AE_CORE_INFO("=======================================================");
    AE_CORE_INFO("Aetherion - NASA TM-2015-218675 Atmospheric Scenario 13.4");
    AE_CORE_INFO("F-16 subsonic lateral side step  ({:.0f} ft right at t={:.0f} s)",
                 kLatStep_ft, kLatStepTime_s);
    AE_CORE_INFO("  Location : {:.5f} N, {:.5f} E  (Kitty Hawk, NC)", kLat_deg, kLon_deg);
    AE_CORE_INFO("  Altitude : {:.0f} ft ({:.2f} m)", kAlt_ft, kAlt_m);
    AE_CORE_INFO("  Course   : {:.1f} deg NE  (held throughout)", kHeading_deg);
    AE_CORE_INFO("  TAS init : {:.2f} ft/s (335.15 KTAS)", kTAS_fps);
    AE_CORE_INFO("  LatStep  : {:.0f} ft right of course at t = {:.0f} s",
                 kLatStep_ft, kLatStepTime_s);
    AE_CORE_INFO("=======================================================");
}

// ── writeCsvHeader ────────────────────────────────────────────────────────────

void F16LateralSideStepApplication::writeCsvHeader(std::ofstream& csv) const
{
    Simulation::Snapshot2Traits::write_header(csv);
}

// ── prepareSimulation ─────────────────────────────────────────────────────────

void F16LateralSideStepApplication::prepareSimulation() const
{
    const auto& simCfg = getConfig();

    // ── 1. Load models ────────────────────────────────────────────────────────
    const std::string inertiaPath = F16_INERTIA_FILE;
    const std::string aeroPath    = F16_AERO_FILE;
    const std::string propPath    = F16_PROP_FILE;
    const std::string controlPath = F16_CONTROL_FILE;
    if (inertiaPath.empty()) throw std::runtime_error("F16_INERTIA_FILE not configured.");
    if (aeroPath.empty() || propPath.empty())
        throw std::runtime_error("F16_AERO_FILE or F16_PROP_FILE not configured.");
    if (controlPath.empty()) throw std::runtime_error("F16_CONTROL_FILE not configured.");

    AE_CORE_INFO("Loading inertia from '{}'", inertiaPath);
    const RigidBody::InertialParameters ip = Serialization::LoadInertiaFromDAVEML(inertiaPath);
    AE_CORE_INFO("  mass = {:.4f} kg", ip.mass_kg);

    AE_CORE_INFO("Loading aero    from '{}'", aeroPath);
    auto aero_model = std::make_shared<const Serialization::DAVEMLAeroModel>(aeroPath);
    AE_CORE_INFO("Loading prop    from '{}'", propPath);
    auto prop_model = std::make_shared<const Serialization::DAVEMLPropModel>(propPath);
    AE_CORE_INFO("Loading control from '{}'", controlPath);
    auto ctrl_model = std::make_shared<const Serialization::DAVEMLControlModel>(controlPath);

    // ── 2. Trim solver ────────────────────────────────────────────────────────
    const double weight_lbf = ip.mass_kg * kG_mps2 / kLbf_N;
    FlightDynamics::TrimInputs tin{};
    tin.vt_fps     = kTAS_fps;
    tin.alt_ft     = kAlt_ft;
    tin.weight_lbf = weight_lbf;

    AE_CORE_INFO("Running trim solver ...");
    const double xcg_ft = kXcgFromAC_m / FlightDynamics::TrimSolver::kFt_m;
    FlightDynamics::TrimSolver solver(*aero_model, *prop_model, xcg_ft);
    const FlightDynamics::TrimPoint trim = solver.solve(tin);
    if (!trim.converged) throw std::runtime_error("Trim solver did not converge.");

    AE_CORE_INFO("Trim converged:");
    AE_CORE_INFO("  alpha = {:.4f} deg", trim.alpha_deg);
    AE_CORE_INFO("  el    = {:.4f} deg", trim.el_deg);
    AE_CORE_INFO("  pwr   = {:.4f} %%",  trim.pwr_pct);

    // ── 3. Initial state ──────────────────────────────────────────────────────
    RigidBody::Config cfg{};
    cfg.pose.lat_deg       = kLat_deg;
    cfg.pose.lon_deg       = kLon_deg;
    cfg.pose.alt_m         = kAlt_m;
    cfg.pose.azimuth_deg   = kHeading_deg;
    cfg.pose.zenith_deg    = 90.0 - trim.alpha_deg;
    cfg.pose.roll_deg      = kRoll_deg;
    cfg.velocityNED.north_mps  = kVelocity_mps;
    cfg.velocityNED.east_mps   = kVelocity_mps;
    cfg.velocityNED.down_mps   = 0.0;
    cfg.bodyRates.roll_rad_s   = 0.0;
    cfg.bodyRates.pitch_rad_s  = 0.0;
    cfg.bodyRates.yaw_rad_s    = 0.0;
    cfg.inertialParameters     = ip;

    const double theta0 = computeInitialERA(simCfg.startTime);
    const RigidBody::StateD x0 = buildInitialState(cfg, theta0);

    // ── 4. Autopilot commands ─────────────────────────────────────────────────
    // keasCmd computed from US1976 atmosphere so deltaVequiv ≈ 0 at t=0.
    const auto atm_trim = Environment::US1976Atmosphere(kAlt_m);
    constexpr double kRhoSL_kg_m3 = 1.225;
    constexpr double kKt_mps      = 0.5144444;
    const double tas_mps = kVelocity_mps * std::numbers::sqrt2;
    const double keas_kt = (tas_mps / kKt_mps) * std::sqrt(atm_trim.rho / kRhoSL_kg_m3);

    Sim::AutopilotCmds cmds;
    cmds.altCmd_ft        = kAltCmd_ft;
    cmds.keasCmd_kt       = keas_kt;
    cmds.baseChiCmd_deg   = kBaseChiCmd_deg;
    cmds.latOffset_ft     = 0.0;          // dynamic; overridden by latStepOffset_ft
    cmds.latStepTime_s    = kLatStepTime_s;
    cmds.latStepOffset_ft = kLatStep_ft;
    cmds.coursePsi_deg    = kHeading_deg;

    AE_CORE_INFO("Autopilot commands:");
    AE_CORE_INFO("  altCmd      = {:.0f} ft  (hold)", cmds.altCmd_ft);
    AE_CORE_INFO("  keasCmd     = {:.4f} kt  (computed from US1976 atm at {:.0f} ft)",
                 keas_kt, kAlt_ft);
    AE_CORE_INFO("  chiCmd      = {:.1f} deg  (hold NE course)", cmds.baseChiCmd_deg);
    AE_CORE_INFO("  latStep     = {:.0f} ft right of course at t = {:.0f} s",
                 cmds.latStepOffset_ft, cmds.latStepTime_s);

    m_Simulator = constructSimulator(ip, trim, aero_model, prop_model, ctrl_model,
                                     x0, theta0, cmds, kXcgFromAC_m);
}

// ── writeInitialSnapshot ──────────────────────────────────────────────────────

void F16LateralSideStepApplication::writeInitialSnapshot(std::ofstream& csv) const
{
    const auto s2 = m_Simulator->snapshot2();
    Simulation::Snapshot2Traits::write_row(csv, s2);
    AE_CORE_INFO("t={:.6f} s  alt={:.3f} m  TAS={:.4f} m/s  Mach={:.4f}",
        s2.time, s2.altitudeMsl_m, s2.trueAirspeed_m_s, s2.mach);
}

// ── stepAndRecord ─────────────────────────────────────────────────────────────

auto F16LateralSideStepApplication::stepAndRecord(
    std::ofstream& csv, double h, bool doWrite) const -> StepObservation
{
    const auto res = m_Simulator->step(h);
    const auto s2  = m_Simulator->snapshot2();

    if (doWrite)
        Simulation::Snapshot2Traits::write_row(csv, s2);

    AE_CORE_TRACE("  t={:.4f} s  alt={:.2f} m  yaw={:.4f} deg",
        s2.time, s2.altitudeMsl_m,
        s2.eulerAngle_rad_Yaw * (180.0 / std::numbers::pi));

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

void F16LateralSideStepApplication::logFinalSummary() const
{
    const auto s2 = m_Simulator->snapshot2();
    constexpr double kToDeg = 180.0 / std::numbers::pi;
    constexpr double kFt    = 1.0 / 0.3048;

    // Compute lateral displacement from initial position (for reporting)
    constexpr double kRE_m  = 6'371'000.0;
    constexpr double kLat0  = kLat_deg * (std::numbers::pi / 180.0);
    constexpr double kLon0  = kLon_deg * (std::numbers::pi / 180.0);
    constexpr double kPsi0  = kHeading_deg * (std::numbers::pi / 180.0);
    const double dp_N_m = (s2.latitude_rad  - kLat0) * kRE_m;
    const double dp_E_m = (s2.longitude_rad - kLon0) * kRE_m * std::cos(kLat0);
    const double lat_dev_ft = (-dp_N_m * std::sin(kPsi0) + dp_E_m * std::cos(kPsi0)) * kFt;

    AE_CORE_INFO("=======================================================");
    AE_CORE_INFO("Simulation complete.");
    AE_CORE_INFO("  Final time      : {:.4f} s",   s2.time);
    AE_CORE_INFO("  Final altitude  : {:.2f} m  ({:.1f} ft)",
                 s2.altitudeMsl_m, s2.altitudeMsl_m * kFt);
    AE_CORE_INFO("  Final TAS       : {:.4f} m/s  ({:.2f} kt)",
                 s2.trueAirspeed_m_s, s2.trueAirspeed_m_s / 0.5144444);
    AE_CORE_INFO("  Final Mach      : {:.6f}",     s2.mach);
    AE_CORE_INFO("  Final yaw       : {:.4f} deg", s2.eulerAngle_rad_Yaw * kToDeg);
    AE_CORE_INFO("  Final roll      : {:.4f} deg", s2.eulerAngle_rad_Roll * kToDeg);
    AE_CORE_INFO("  Lateral dev     : {:.1f} ft  (cmd = {:.0f} ft)",
                 lat_dev_ft, kLatStep_ft);
    AE_CORE_INFO("Output written to '{}'", getConfig().outputFileName);
    AE_CORE_INFO("=======================================================");
}

// ── Private helpers ───────────────────────────────────────────────────────────

double F16LateralSideStepApplication::computeInitialERA(double t0) noexcept
{
    constexpr double kOmegaE = 7.2921150e-5;
    return kOmegaE * t0;
}

RigidBody::StateD F16LateralSideStepApplication::buildInitialState(
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

std::unique_ptr<Sim>
F16LateralSideStepApplication::constructSimulator(
    const RigidBody::InertialParameters&                      ip,
    const FlightDynamics::TrimPoint&                          trim,
    const std::shared_ptr<const Serialization::DAVEMLAeroModel>&     aero,
    const std::shared_ptr<const Serialization::DAVEMLPropModel>&     prop,
    const std::shared_ptr<const Serialization::DAVEMLControlModel>&  ctrl,
    const RigidBody::StateD& x0,
    double theta0,
    const Sim::AutopilotCmds& cmds,
    double xcg_from_ac_m)
{
    AE_CORE_INFO("Constructing F16LateralSideStep simulator (reuses F16AltitudeChangeSimulator)...");
    AE_CORE_INFO("  xcg_from_ac = {:.4f} m", xcg_from_ac_m);
    return std::make_unique<Sim>(ip, trim, aero, prop, ctrl, x0, theta0,
                                 cmds, xcg_from_ac_m);
}

} // namespace Aetherion::Examples::F16LateralSideStep

// ── Entry point ───────────────────────────────────────────────────────────────

namespace Aetherion::Simulation {
    Application* CreateApplication(int argc, char** argv)
    {
        return new Examples::F16LateralSideStep::F16LateralSideStepApplication(argc, argv);
    }
}

#include <Aetherion/Simulation/EntryPoint.h>

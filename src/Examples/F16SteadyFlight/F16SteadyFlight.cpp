// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16SteadyFlight.cpp  —  NASA TM-2015-218675 Atmospheric Scenario 11
//

#include <Aetherion/Examples/F16SteadyFlight/F16SteadyFlightApplication.h>

#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/SnapshotTraits.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/FlightDynamics/Trim/TrimSolver.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLPropModel.h>
#include <Aetherion/Serialization/DAVEML/LoadInertiaFromDAVEML.h>
#include <Aetherion/RigidBody/Config.h>
#include <Aetherion/RigidBody/GeodeticPoseNED.h>
#include <Aetherion/RigidBody/StateLayout.h>

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

// ── NASA Scenario 11 initial conditions ──────────────────────────────────────
namespace {
    constexpr double kLat_deg     =  36.01917;    // Kitty Hawk, NC
    constexpr double kLon_deg     = -75.67444;
    constexpr double kAlt_ft      =  10013.0;
    constexpr double kAlt_m       =  kAlt_ft * 0.3048;   // 3051.9624 m

    constexpr double kHeading_deg =  45.0;         // NE
    constexpr double kRoll_deg    =  -0.172;        // small bank (NASA ref)
    constexpr double kVelocity_fps=  400.0;         // NE components in ft/s
    constexpr double kVelocity_mps=  kVelocity_fps * 0.3048;  // 121.92 m/s

    constexpr double kTAS_fps     =  565.685;       // 335.15 KTAS
    constexpr double kG_mps2      =  9.80665;
    constexpr double kLbf_N       =  4.448221615260751;
    constexpr double kDeg         =  std::numbers::pi / 180.0;
}

namespace Aetherion::Examples::F16SteadyFlight {

// ── logStartupBanner ──────────────────────────────────────────────────────────

void F16SteadyFlightApplication::logStartupBanner() const
{
    AE_CORE_INFO("=======================================================");
    AE_CORE_INFO("Aetherion - NASA TM-2015-218675 Atmospheric Scenario 11");
    AE_CORE_INFO("F-16 steady straight-and-level flight");
    AE_CORE_INFO("  Location : {:.5f} N, {:.5f} E  (Kitty Hawk, NC)", kLat_deg, kLon_deg);
    AE_CORE_INFO("  Altitude : {:.0f} ft ({:.2f} m)", kAlt_ft, kAlt_m);
    AE_CORE_INFO("  Heading  : {:.1f} deg NE", kHeading_deg);
    AE_CORE_INFO("  TAS      : {:.2f} ft/s (335.15 KTAS)", kTAS_fps);
    AE_CORE_INFO("=======================================================");
}

// ── writeCsvHeader — Snapshot2 (31-column NASA format) ───────────────────────

void F16SteadyFlightApplication::writeCsvHeader(std::ofstream& csv) const
{
    Simulation::Snapshot2Traits::write_header(csv);
}

// ── prepareSimulation ─────────────────────────────────────────────────────────

void F16SteadyFlightApplication::prepareSimulation() const
{
    const auto& simCfg = getConfig();

    // ── 1. Load inertia from DAVE-ML ─────────────────────────────────────────
    const std::string inertiaPath = F16_INERTIA_FILE;
    if (inertiaPath.empty())
        throw std::runtime_error("F16_INERTIA_FILE not configured.");
    AE_CORE_INFO("Loading inertia from '{}'", inertiaPath);
    const RigidBody::InertialParameters ip = Serialization::LoadInertiaFromDAVEML(inertiaPath);
    AE_CORE_INFO("  mass = {:.4f} kg  ({:.4f} slug)", ip.mass_kg, ip.mass_kg / 14.5939);
    AE_CORE_INFO("  Ixx={:.1f}  Iyy={:.1f}  Izz={:.1f}  Ixz={:.1f}  [kg·m²]",
                 ip.Ixx, ip.Iyy, ip.Izz, ip.Ixz);

    // ── 2. Load aero and prop models ─────────────────────────────────────────
    const std::string aeroPath = F16_AERO_FILE;
    const std::string propPath = F16_PROP_FILE;
    if (aeroPath.empty() || propPath.empty())
        throw std::runtime_error("F16_AERO_FILE or F16_PROP_FILE not configured.");
    AE_CORE_INFO("Loading aero model from '{}'", aeroPath);
    auto aero_model = std::make_shared<const Serialization::DAVEMLAeroModel>(aeroPath);
    AE_CORE_INFO("Loading prop model from '{}'", propPath);
    auto prop_model = std::make_shared<const Serialization::DAVEMLPropModel>(propPath);

    // ── 3. Run trim solver ────────────────────────────────────────────────────
    const double weight_lbf = ip.mass_kg * kG_mps2 / kLbf_N;
    FlightDynamics::TrimInputs tin{};
    tin.vt_fps     = kTAS_fps;
    tin.alt_ft     = kAlt_ft;
    tin.weight_lbf = weight_lbf;

    AE_CORE_INFO("Running trim solver  (W = {:.1f} lbf, V = {:.2f} ft/s, h = {:.0f} ft) ...",
                 weight_lbf, kTAS_fps, kAlt_ft);
    FlightDynamics::TrimSolver solver(*aero_model, *prop_model);
    const FlightDynamics::TrimPoint trim = solver.solve(tin);

    if (!trim.converged)
        throw std::runtime_error("Trim solver did not converge.");

    AE_CORE_INFO("Trim converged in {} iterations:", FlightDynamics::TrimSolver::kMaxIter);
    AE_CORE_INFO("  alpha = {:.4f} deg", trim.alpha_deg);
    AE_CORE_INFO("  el    = {:.4f} deg", trim.el_deg);
    AE_CORE_INFO("  pwr   = {:.4f} %%",  trim.pwr_pct);
    AE_CORE_INFO("  |r|   = {:.3e} lbf", trim.residual_norm);

    // ── 4. Build initial state ────────────────────────────────────────────────
    RigidBody::Config cfg{};
    cfg.pose.lat_deg      = kLat_deg;
    cfg.pose.lon_deg      = kLon_deg;
    cfg.pose.alt_m        = kAlt_m;
    cfg.pose.azimuth_deg  = kHeading_deg;
    cfg.pose.zenith_deg   = 90.0 - trim.alpha_deg;  // nearly horizontal, slight nose-up
    // MakeLaunchStateECI places body-z UP at roll=0; +180° flips it to z-DOWN
    // (standard aviation convention).  The NASA initial bank φ = kRoll_deg is
    // then added on top of that base orientation.
    cfg.pose.roll_deg     = 180.0 + kRoll_deg;
    cfg.velocityNED.north_mps = kVelocity_mps;
    cfg.velocityNED.east_mps  = kVelocity_mps;
    cfg.velocityNED.down_mps  = 0.0;
    cfg.bodyRates.roll_rad_s  = 0.0;
    cfg.bodyRates.pitch_rad_s = 0.0;
    cfg.bodyRates.yaw_rad_s   = 0.0;
    cfg.inertialParameters    = ip;

    const double theta0 = computeInitialERA(simCfg.startTime);
    const RigidBody::StateD x0 = buildInitialState(cfg, theta0);
    AE_CORE_INFO("Initial ECI position = [{:.3f}, {:.3f}, {:.3f}] m",
                 x0.g.p.x(), x0.g.p.y(), x0.g.p.z());

    m_Simulator = constructSimulator(ip, trim, aero_model, prop_model, x0, theta0);
}

// ── writeInitialSnapshot ──────────────────────────────────────────────────────

void F16SteadyFlightApplication::writeInitialSnapshot(std::ofstream& csv) const
{
    const auto s2 = m_Simulator->snapshot2();
    Simulation::Snapshot2Traits::write_row(csv, s2);
    AE_CORE_INFO("t={:.6f} s  alt={:.3f} m  TAS={:.4f} m/s  Mach={:.4f}",
        s2.time, s2.altitudeMsl_m, s2.trueAirspeed_m_s, s2.mach);
}

// ── stepAndRecord ─────────────────────────────────────────────────────────────

auto F16SteadyFlightApplication::stepAndRecord(
    std::ofstream& csv, double h, bool doWrite) const -> StepObservation
{
    const auto res = m_Simulator->step(h);
    const auto s2  = m_Simulator->snapshot2();

    if (doWrite)
        Simulation::Snapshot2Traits::write_row(csv, s2);

    AE_CORE_TRACE("  t={:.4f} s  alt={:.2f} m  TAS={:.4f} m/s  Mach={:.4f}",
        s2.time, s2.altitudeMsl_m, s2.trueAirspeed_m_s, s2.mach);

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

void F16SteadyFlightApplication::logFinalSummary() const
{
    const auto s2 = m_Simulator->snapshot2();
    constexpr double kToDeg = 180.0 / std::numbers::pi;

    AE_CORE_INFO("=======================================================");
    AE_CORE_INFO("Simulation complete.");
    AE_CORE_INFO("  Final time      : {:.4f} s",   s2.time);
    AE_CORE_INFO("  Final altitude  : {:.4f} m",   s2.altitudeMsl_m);
    AE_CORE_INFO("  Final TAS       : {:.6f} m/s", s2.trueAirspeed_m_s);
    AE_CORE_INFO("  Final Mach      : {:.6f}",     s2.mach);
    AE_CORE_INFO("  Final pitch     : {:.4f} deg", s2.eulerAngle_rad_Pitch * kToDeg);
    AE_CORE_INFO("  Final roll      : {:.4f} deg", s2.eulerAngle_rad_Roll  * kToDeg);
    AE_CORE_INFO("  Final latitude  : {:.8f} deg", s2.latitude_rad  * kToDeg);
    AE_CORE_INFO("  Final longitude : {:.8f} deg", s2.longitude_rad * kToDeg);
    AE_CORE_INFO("Output written to '{}'", getConfig().outputFileName);
    AE_CORE_INFO("=======================================================");
}

// ── Private helpers ───────────────────────────────────────────────────────────

double F16SteadyFlightApplication::computeInitialERA(double t0) noexcept
{
    constexpr double kOmegaE = 7.2921150e-5;
    return kOmegaE * t0;
}

RigidBody::StateD F16SteadyFlightApplication::buildInitialState(
    const RigidBody::Config& cfg, double theta0)
{
    const auto x0 = FlightDynamics::BuildInitialStateVector(cfg, theta0);

    RigidBody::StateD state;
    using Layout = RigidBody::StateLayout;

    state.g.p = Eigen::Vector3d(
        x0[Layout::IDX_P], x0[Layout::IDX_P + 1], x0[Layout::IDX_P + 2]);

    const Eigen::Quaterniond q(
        x0[Layout::IDX_Q], x0[Layout::IDX_Q + 1],
        x0[Layout::IDX_Q + 2], x0[Layout::IDX_Q + 3]);
    state.g.R = q.normalized().toRotationMatrix();

    state.nu_B <<
        x0[Layout::IDX_W],     x0[Layout::IDX_W + 1], x0[Layout::IDX_W + 2],
        x0[Layout::IDX_V],     x0[Layout::IDX_V + 1], x0[Layout::IDX_V + 2];

    state.m = x0[Layout::IDX_M];
    return state;
}

std::unique_ptr<F16SteadyFlightSimulator>
F16SteadyFlightApplication::constructSimulator(
    const RigidBody::InertialParameters&                  ip,
    const FlightDynamics::TrimPoint&                      trim,
    std::shared_ptr<const Serialization::DAVEMLAeroModel> aero,
    std::shared_ptr<const Serialization::DAVEMLPropModel> prop,
    const RigidBody::StateD& x0,
    double theta0)
{
    AE_CORE_INFO("Constructing F16SteadyFlightSimulator...");
    return std::make_unique<F16SteadyFlightSimulator>(ip, trim, aero, prop, x0, theta0);
}

} // namespace Aetherion::Examples::F16SteadyFlight

// ── Entry point ───────────────────────────────────────────────────────────────

namespace Aetherion::Simulation {
    Application* CreateApplication(int argc, char* argv[])
    {
        return new Examples::F16SteadyFlight::F16SteadyFlightApplication(argc, argv);
    }
}

#include <Aetherion/Simulation/EntryPoint.h>

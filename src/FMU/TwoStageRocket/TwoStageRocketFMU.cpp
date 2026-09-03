// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TwoStageRocketFMU.cpp
//
// FMI 2.0 Co-Simulation FMU wrapping the Aetherion two-stage rocket 6-DoF
// plant (NASA TM-2015-218675 Atmospheric Scenario 17).
//
// Model identity : TwoStageRocket
// FMI standard   : FMI 2.0 (Co-Simulation)
// Integrator     : Radau IIA RKMK (3-stage implicit, Lie-group preserving)
//
// Unlike F16PlantFMU (which bypasses ISimulator and owns the stepper
// directly), this FMU composes Examples::TwoStageRocket::TwoStageRocketSimulator
// as-is: that class already owns the ZOH per-step sequence (thrust/mdot lookup,
// inertia rebuild, integration, fuel advance, staging mass drop) and reusing it
// avoids duplicating that coupling logic. Two small mutable accessors were
// added to TwoStageRocketSimulator/RocketStageModel (stageModel(), setState())
// purely so this FMU can save/restore a full checkpoint.
//
// Variable map
// ─────────────────────────────────────────────────────────────────────────────
//  PARAMETERS  (fixed before fmi2ExitInitializationMode)
//    lat0_deg          Launch geodetic latitude            [deg]  def   0.0
//    lon0_deg          Launch geodetic longitude            [deg]  def   0.0
//    alt0_m            Launch altitude above MSL            [m]    def   0.0
//    azimuth0_deg      Initial heading (from North)         [deg]  def  90.0
//    pitch0_deg        Initial pitch, nose-up from horizon  [deg]  def  55.22
//    roll0_deg         Initial roll angle                   [deg]  def   0.0
//    vNorth0_mps       Initial NED north velocity            [m/s]  def   0.0
//    vEast0_mps        Initial NED east  velocity            [m/s]  def   0.0
//    vDown0_mps        Initial NED down  velocity            [m/s]  def   0.0
//    solver.max_step_s Max internal integrator step          [s]    def   0.0 (= comm. step)
//
//  PARAMETER (TUNABLE — may also be changed between fmi2DoStep calls)
//    stg2.ignition_time_s   Absolute sim time gating S2 ignition [s]  def 0.0
//                           (0 = ignite immediately once S1 has separated;
//                           forwarded every step to RocketStageModel, which
//                           checks it against elapsed simulation time)
//
//  OUTPUTS  (valid after fmi2ExitInitializationMode and each fmi2DoStep)
//    out.alt_m, out.lat_deg, out.lon_deg, out.g_m_s2
//    out.yaw_rad, out.pitch_rad, out.roll_rad
//    out.p_rad_s, out.q_rad_s, out.r_rad_s, out.altRate_m_s
//    out.v_north_m_s, out.v_east_m_s, out.v_down_m_s
//    out.a_m_s, out.rho_kg_m3, out.P_Pa, out.T_K
//    out.aero_Fx_N, out.aero_Fy_N, out.aero_Fz_N
//    out.aero_Mx_Nm, out.aero_My_Nm, out.aero_Mz_Nm
//    out.mach, out.qbar_Pa, out.vt_m_s
//    out.thrust_N, out.mdot_kgs, out.mass_kg
//    out.stg1_fuel_used_kg, out.stg2_fuel_used_kg
//    out.staged          Stage-1 separation flag (bool)
//
// FMU lifecycle
// ─────────────────────────────────────────────────────────────────────────────
//  fmi2Instantiate          → constructor: register variables
//  fmi2EnterInitializationMode
//  fmi2SetReal (params)     → write parameters
//  fmi2ExitInitializationMode → exit_initialisation_mode():
//                               load twostage_{inertia,prop,aero}.dml from
//                               resources/, query liftoff inertia, build the
//                               initial 6-DoF state, construct the simulator
//  fmi2DoStep(t, dt)        → do_step(dt):
//                               forward stg2.ignition_time_s, call
//                               TwoStageRocketSimulator::step() (subdivided by
//                               solver.max_step_s if set), refresh outputs
//  fmi2GetFMUstate /
//  fmi2SetFMUstate          → save/restore TwoStageRocketState (POD struct):
//                               integration state, elapsed sim time, and
//                               fuel/staging state
//  fmi2Reset                → reset(): tear down simulator, restore defaults
// ------------------------------------------------------------------------------

#include <fmu4cpp/fmu_base.hpp>

// Standard library
#include <algorithm>      // std::min
#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>

// Aetherion — two-stage rocket simulator (includes RocketStageModel,
// RocketGravityPolicy, RocketAeroPolicy, Snapshot2/MakeSnapshot2)
#include <Aetherion/Examples/TwoStageRocket/TwoStageRocketSimulator.h>

// Initial state construction
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/BuildInitialState.h>
#include <Aetherion/RigidBody/Config.h>
#include <Aetherion/RigidBody/StateLayout.h>

// Serialization (DAVE-ML)
#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>

// Earth rotation rate (Earth Rotation Angle at t=0)
#include <Aetherion/Environment/WGS84.h>

using namespace fmu4cpp;

// ── Namespace aliases ─────────────────────────────────────────────────────────
namespace AE_RB  = Aetherion::RigidBody;
namespace AE_SR  = Aetherion::Serialization;
namespace AE_RKT = Aetherion::Examples::TwoStageRocket;

// ── Type aliases ──────────────────────────────────────────────────────────────
using RocketSimulator = AE_RKT::TwoStageRocketSimulator<>;

// ── Module-level constants ────────────────────────────────────────────────────
namespace {
    constexpr double kOmegaEarth_rad_s = Aetherion::Environment::WGS84::kRotationRate_rad_s;
    constexpr double kRadToDeg         = 180.0 / std::numbers::pi;

    // Default initial conditions — NASA TM-2015-218675 Scenario 17 (equatorial
    // gravity-turn ascent). Settable via FMI PARAMETER before
    // fmi2ExitInitializationMode.
    constexpr double kDefault_lat0_deg     =  0.0;
    constexpr double kDefault_lon0_deg     =  0.0;
    constexpr double kDefault_alt0_m       =  0.0;
    constexpr double kDefault_azimuth0_deg = 90.0;
    constexpr double kDefault_pitch0_deg   = 55.22;
    constexpr double kDefault_roll0_deg    =  0.0;
    constexpr double kDefault_vNorth0_mps  =  0.0;
    constexpr double kDefault_vEast0_mps   =  0.0;
    constexpr double kDefault_vDown0_mps   =  0.0;

    constexpr double kDefault_stg2_ignition_time_s = 0.0;  // ignite immediately after staging
    constexpr double kDefault_max_step_s           = 0.0;  // 0 → use the communication step directly
}

// ── FMU state (trivially copyable POD) ───────────────────────────────────────
// Everything captured by getFMUState / setFMUState lives in this struct.
// On setFMUState, the base class copies this back, then unpackState()
// re-derives the simulator's internal state (RigidBody::StateD + fuel state).
struct TwoStageRocketState {
    // ── Integration state ─────────────────────────────────────────────────────
    std::array<double, 9> R    {};   // SO(3) rotation matrix, row-major (body → ECI)
    std::array<double, 3> r_I  {};   // ECI position [m]
    std::array<double, 6> nu_B {};   // body twist: [wx, wy, wz (rad/s), vx, vy, vz (m/s)]
    double mass_kg     {};   // vehicle mass [kg]
    double sim_time_s  {};   // elapsed simulation time inside the simulator [s]

    // ── Fuel / staging state (preserved across save/restore) ──────────────────
    double stg1_fuel_used_kg {};
    double stg2_fuel_used_kg {};
    bool   staged             {};

    // ── Output cache (updated in do_step, registered by pointer) ─────────────
    double alt_m       {};  // altitude above MSL [m]
    double lat_deg     {};  // geodetic latitude [deg]
    double lon_deg     {};  // geodetic longitude [deg]
    double g_m_s2      {};  // local gravity magnitude [m/s²]
    double yaw_rad     {};  // ZYX Euler yaw   (body → NED) [rad]
    double pitch_rad   {};  // ZYX Euler pitch (body → NED) [rad]
    double roll_rad    {};  // ZYX Euler roll  (body → NED) [rad]
    double p_rad_s      {};  // body roll  rate wrt ECI [rad/s]
    double q_rad_s      {};  // body pitch rate wrt ECI [rad/s]
    double r_rad_s      {};  // body yaw   rate wrt ECI [rad/s]
    double altRate_m_s  {};  // dh/dt [m/s]
    double v_north_m_s {};  // NED north velocity [m/s]
    double v_east_m_s  {};  // NED east  velocity [m/s]
    double v_down_m_s  {};  // NED down  velocity [m/s]
    double a_m_s       {};  // speed of sound [m/s]
    double rho_kg_m3   {};  // air density [kg/m³]
    double P_Pa        {};  // ambient pressure [Pa]
    double T_K         {};  // ambient temperature [K]
    double aero_Fx_N   {};  // aero body-X force  [N]
    double aero_Fy_N   {};  // aero body-Y force  [N]
    double aero_Fz_N   {};  // aero body-Z force  [N]
    double aero_Mx_Nm  {};  // aero roll  moment  [N·m]
    double aero_My_Nm  {};  // aero pitch moment  [N·m]
    double aero_Mz_Nm  {};  // aero yaw   moment  [N·m]
    double mach        {};  // Mach number [-]
    double qbar_Pa     {};  // dynamic pressure [Pa]
    double vt_m_s      {};  // true airspeed [m/s]
    double thrust_N    {};  // total axial thrust, body +x [N]
    double mdot_kgs    {};  // propellant consumption rate [kg/s]
};
static_assert(std::is_trivially_copyable_v<TwoStageRocketState>,
    "TwoStageRocketState must be trivially copyable for fmu4cpp state save/restore.");

// ─────────────────────────────────────────────────────────────────────────────
// TwoStageRocketFMU
// ─────────────────────────────────────────────────────────────────────────────
class TwoStageRocketFMU : public fmu_base {
public:

    // ── Constructor: register all FMI variables ───────────────────────────────
    FMU4CPP_CTOR(TwoStageRocketFMU)
    {
        // ── Parameters ───────────────────────────────────────────────────────
        register_real("lat0_deg", &p_lat0_deg_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Launch geodetic latitude [deg]");

        register_real("lon0_deg", &p_lon0_deg_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Launch geodetic longitude [deg]");

        register_real("alt0_m", &p_alt0_m_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Launch altitude above MSL [m]");

        register_real("azimuth0_deg", &p_azimuth0_deg_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Initial heading, azimuth from North [deg]");

        register_real("pitch0_deg", &p_pitch0_deg_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Initial pitch angle, nose-up from horizontal [deg]");

        register_real("roll0_deg", &p_roll0_deg_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Initial roll angle [deg]");

        register_real("vNorth0_mps", &p_vNorth0_mps_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Initial NED north velocity [m/s]");

        register_real("vEast0_mps", &p_vEast0_mps_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Initial NED east velocity [m/s]");

        register_real("vDown0_mps", &p_vDown0_mps_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Initial NED down velocity [m/s]");

        register_real("solver.max_step_s", &p_max_step_s_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Maximum internal integrator sub-step size [s] (0 = use communication step)");

        register_real("stg2.ignition_time_s", &p_stg2_ignition_time_s_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::TUNABLE)
            .setInitial(initial_t::EXACT)
            .setDescription("Absolute sim time at which Stage 2 may ignite [s] "
                            "(0 = ignite immediately once Stage 1 has separated)");

        // ── Outputs ───────────────────────────────────────────────────────────
        register_real("out.alt_m",       &state_.alt_m)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Altitude above MSL [m]");

        register_real("out.lat_deg",     &state_.lat_deg)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Geodetic latitude [deg]");

        register_real("out.lon_deg",     &state_.lon_deg)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Geodetic longitude [deg]");

        register_real("out.g_m_s2",      &state_.g_m_s2)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Local gravitational acceleration [m/s²]");

        register_real("out.yaw_rad",     &state_.yaw_rad)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("ZYX Euler yaw angle (body → NED) [rad]");

        register_real("out.pitch_rad",   &state_.pitch_rad)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("ZYX Euler pitch angle (body → NED) [rad]");

        register_real("out.roll_rad",    &state_.roll_rad)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("ZYX Euler roll angle (body → NED) [rad]");

        register_real("out.p_rad_s",     &state_.p_rad_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Body roll rate wrt ECI [rad/s]");

        register_real("out.q_rad_s",     &state_.q_rad_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Body pitch rate wrt ECI [rad/s]");

        register_real("out.r_rad_s",     &state_.r_rad_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Body yaw rate wrt ECI [rad/s]");

        register_real("out.altRate_m_s", &state_.altRate_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Altitude rate, dh/dt [m/s]");

        register_real("out.v_north_m_s", &state_.v_north_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("NED north velocity [m/s]");

        register_real("out.v_east_m_s",  &state_.v_east_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("NED east velocity [m/s]");

        register_real("out.v_down_m_s",  &state_.v_down_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("NED down velocity [m/s]");

        register_real("out.a_m_s",       &state_.a_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Speed of sound [m/s]");

        register_real("out.rho_kg_m3",   &state_.rho_kg_m3)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Air density [kg/m³]");

        register_real("out.P_Pa",        &state_.P_Pa)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Ambient static pressure [Pa]");

        register_real("out.T_K",         &state_.T_K)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Ambient static temperature [K]");

        register_real("out.aero_Fx_N",   &state_.aero_Fx_N)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Aerodynamic body-X force [N]");

        register_real("out.aero_Fy_N",   &state_.aero_Fy_N)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Aerodynamic body-Y force [N]");

        register_real("out.aero_Fz_N",   &state_.aero_Fz_N)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Aerodynamic body-Z force [N]");

        register_real("out.aero_Mx_Nm",  &state_.aero_Mx_Nm)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Aerodynamic roll moment [N·m]");

        register_real("out.aero_My_Nm",  &state_.aero_My_Nm)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Aerodynamic pitch moment (about MRC) [N·m]");

        register_real("out.aero_Mz_Nm",  &state_.aero_Mz_Nm)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Aerodynamic yaw moment [N·m]");

        register_real("out.mach",        &state_.mach)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Mach number [-]");

        register_real("out.qbar_Pa",     &state_.qbar_Pa)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Dynamic pressure [Pa]");

        register_real("out.vt_m_s",      &state_.vt_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("True airspeed [m/s]");

        register_real("out.thrust_N",    &state_.thrust_N)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Total axial thrust, body +x [N]");

        register_real("out.mdot_kgs",    &state_.mdot_kgs)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Propellant consumption rate [kg/s]");

        register_real("out.mass_kg",     &state_.mass_kg)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Vehicle mass [kg]");

        register_real("out.stg1_fuel_used_kg", &state_.stg1_fuel_used_kg)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Stage-1 propellant consumed so far [kg]");

        register_real("out.stg2_fuel_used_kg", &state_.stg2_fuel_used_kg)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Stage-2 propellant consumed so far [kg]");

        register_boolean("out.staged", &state_.staged)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::DISCRETE)
            .setInitial(initial_t::CALCULATED).setDescription("True once Stage 1 has separated");

        // ── State registration (enables getFMUState / setFMUState) ────────────
        register_state(&TwoStageRocketFMU::state_);
    }

    // ── exit_initialisation_mode ──────────────────────────────────────────────
    // Called after fmi2ExitInitializationMode: all parameters are set.
    // Loads DAVE-ML models, queries liftoff inertia, builds the initial ECI
    // state, and constructs the simulator.
    void exit_initialisation_mode() override
    {
        // 1. Build DML paths from the FMU resources/ folder
        const std::string res         = resourceLocation().string();
        const std::string inertiaPath = res + "/twostage_inertia.dml";
        const std::string propPath    = res + "/twostage_prop.dml";
        const std::string aeroPath    = res + "/twostage_aero.dml";

        // 2. Load inertia, propulsion, and aero engines
        auto inertiaDml = std::make_shared<AE_SR::DAVEMLAeroModel>(inertiaPath);
        auto propDml    = std::make_shared<AE_SR::DAVEMLAeroModel>(propPath);
        auto aeroDml    = std::make_shared<const AE_SR::DAVEMLAeroModel>(aeroPath);

        // 3. Earth Rotation Angle at fmi2EnterInitializationMode time.
        m_theta0 = kOmegaEarth_rad_s * currentTime();

        // 4. Query liftoff inertial parameters from a temporary stage model at
        //    zero fuel usage (mirrors src/Examples/TwoStageRocket/TwoStageRocket.cpp).
        AE_RKT::RocketStageModel stageModel0(inertiaDml, propDml);
        const AE_RB::InertialParameters ip = stageModel0.inertialParameters();

        // 5. Build initial 6-DoF state from FMI parameters.
        AE_RB::Config cfg{};
        cfg.pose.lat_deg          = p_lat0_deg_;
        cfg.pose.lon_deg          = p_lon0_deg_;
        cfg.pose.alt_m            = p_alt0_m_;
        cfg.pose.azimuth_deg      = p_azimuth0_deg_;
        cfg.pose.zenith_deg       = 90.0 - p_pitch0_deg_;
        cfg.pose.roll_deg         = p_roll0_deg_;
        cfg.velocityNED.north_mps = p_vNorth0_mps_;
        cfg.velocityNED.east_mps  = p_vEast0_mps_;
        cfg.velocityNED.down_mps  = p_vDown0_mps_;
        cfg.bodyRates.roll_rad_s  = 0.0;
        cfg.bodyRates.pitch_rad_s = 0.0;
        cfg.bodyRates.yaw_rad_s   = 0.0;
        cfg.inertialParameters    = ip;

        const auto x0vec = AE_RB::BuildInitialStateVector(cfg, m_theta0);
        using L = AE_RB::StateLayout;

        AE_RB::StateD x0{};
        x0.g.p = Eigen::Vector3d(x0vec[L::IDX_P], x0vec[L::IDX_P+1], x0vec[L::IDX_P+2]);
        const Eigen::Quaterniond q0(x0vec[L::IDX_Q], x0vec[L::IDX_Q+1],
                                    x0vec[L::IDX_Q+2], x0vec[L::IDX_Q+3]);
        x0.g.q = q0.normalized();
        x0.g.R = x0.g.q.toRotationMatrix();
        x0.nu_B << x0vec[L::IDX_W], x0vec[L::IDX_W+1], x0vec[L::IDX_W+2],
                   x0vec[L::IDX_V], x0vec[L::IDX_V+1], x0vec[L::IDX_V+2];
        x0.m = x0vec[L::IDX_M];

        // 6. Construct the simulator (owns its own RocketStageModel built from
        //    inertiaDml/propDml; aeroDml feeds the RocketAeroPolicy).
        m_sim.emplace(ip, x0, m_theta0,
                      std::move(inertiaDml), std::move(propDml), aeroDml,
                      p_stg2_ignition_time_s_);

        // 7. Sync POD integration state and populate initial outputs.
        const auto prop0 = m_sim->stageModel().propulsion(m_sim->time());
        packState();
        populateOutputCache(prop0);
    }

    // ── do_step ───────────────────────────────────────────────────────────────
    // Advances the simulation by dt seconds, optionally subdividing into
    // sub-steps of at most solver.max_step_s (0 = single step = communication dt).
    bool do_step(double dt) override
    {
        if (!m_sim.has_value())
            return false;

        const double h = (p_max_step_s_ > 0.0 && p_max_step_s_ < dt)
                          ? p_max_step_s_ : dt;

        double remaining = dt;
        AE_RKT::RocketPropulsionResult lastProp{};

        while (remaining > 1.0e-15) {
            const double step = std::min(remaining, h);

            // Forward the (possibly tuned) S2 ignition gate before each sub-step.
            m_sim->stageModel().stg2IgnitionTime_s = p_stg2_ignition_time_s_;
            lastProp = m_sim->stageModel().propulsion(m_sim->time());

            const auto obs = m_sim->step(step);
            if (!obs.converged) {
                debugLog(fmiWarning, "TwoStageRocketFMU: Radau IIA Newton did not converge.");
                return false;
            }
            remaining -= step;
        }

        packState();
        populateOutputCache(lastProp);
        return true;
    }

    // ── setFmuState ───────────────────────────────────────────────────────────
    // Overridden to keep the simulator's internal state consistent with the
    // restored state_ (POD).
    void setFmuState(void* fmuState) override
    {
        fmu_base::setFmuState(fmuState);   // copies saved TwoStageRocketState → state_
        if (m_sim.has_value())
            unpackState();
    }

    // ── reset ─────────────────────────────────────────────────────────────────
    void reset() override
    {
        m_sim.reset();
        m_theta0 = 0.0;
        state_   = TwoStageRocketState{};

        p_lat0_deg_     = kDefault_lat0_deg;
        p_lon0_deg_     = kDefault_lon0_deg;
        p_alt0_m_       = kDefault_alt0_m;
        p_azimuth0_deg_ = kDefault_azimuth0_deg;
        p_pitch0_deg_   = kDefault_pitch0_deg;
        p_roll0_deg_    = kDefault_roll0_deg;
        p_vNorth0_mps_  = kDefault_vNorth0_mps;
        p_vEast0_mps_   = kDefault_vEast0_mps;
        p_vDown0_mps_   = kDefault_vDown0_mps;

        p_stg2_ignition_time_s_ = kDefault_stg2_ignition_time_s;
        p_max_step_s_           = kDefault_max_step_s;
    }

private:

    // ── packState ─────────────────────────────────────────────────────────────
    // Sync the simulator's integration + fuel state → POD fields in state_.
    void packState()
    {
        if (!m_sim.has_value()) return;

        const auto& x    = m_sim->state();
        const auto& fuel = m_sim->stageModel().fuelState();

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                state_.R[i * 3 + j] = x.g.R(i, j);
        state_.r_I[0] = x.g.p.x();
        state_.r_I[1] = x.g.p.y();
        state_.r_I[2] = x.g.p.z();
        for (int i = 0; i < 6; ++i)
            state_.nu_B[i] = x.nu_B(i);
        state_.mass_kg    = x.m;
        state_.sim_time_s = m_sim->time();

        state_.stg1_fuel_used_kg = fuel.stg1FuelUsed_kg;
        state_.stg2_fuel_used_kg = fuel.stg2FuelUsed_kg;
        state_.staged            = fuel.staged;
    }

    // ── unpackState ───────────────────────────────────────────────────────────
    // Restore the simulator's integration + fuel state from POD fields in
    // state_. Called by setFmuState after the base class has copied the saved
    // state back.
    void unpackState()
    {
        if (!m_sim.has_value()) return;

        AE_RB::StateD x{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                x.g.R(i, j) = state_.R[i * 3 + j];
        x.g.q = Eigen::Quaterniond(x.g.R).normalized();
        x.g.p = Eigen::Vector3d(state_.r_I[0], state_.r_I[1], state_.r_I[2]);
        for (int i = 0; i < 6; ++i)
            x.nu_B(i) = state_.nu_B[i];
        x.m = state_.mass_kg;

        m_sim->setState(x, state_.sim_time_s);

        auto& fuel = m_sim->stageModel().fuelState();
        fuel.stg1FuelUsed_kg = state_.stg1_fuel_used_kg;
        fuel.stg2FuelUsed_kg = state_.stg2_fuel_used_kg;
        fuel.staged          = state_.staged;
    }

    // ── populateOutputCache ───────────────────────────────────────────────────
    // Recomputes all output state_ fields from the simulator's Snapshot2 plus
    // the propulsion/fuel state.
    // @param prop  Propulsion result (thrust/mdot) applied over the step just
    //              taken (ZOH), captured before the sim's internal state advanced.
    void populateOutputCache(const AE_RKT::RocketPropulsionResult& prop)
    {
        if (!m_sim.has_value()) return;

        const auto snap  = m_sim->snapshot2();
        const auto& fuel = m_sim->stageModel().fuelState();

        state_.alt_m       = snap.altitudeMsl_m;
        state_.lat_deg     = snap.latitude_rad  * kRadToDeg;
        state_.lon_deg     = snap.longitude_rad * kRadToDeg;
        state_.g_m_s2      = snap.localGravity_m_s2;

        state_.yaw_rad     = snap.eulerAngle_rad_Yaw;
        state_.pitch_rad   = snap.eulerAngle_rad_Pitch;
        state_.roll_rad    = snap.eulerAngle_rad_Roll;

        state_.p_rad_s     = snap.bodyAngularRateWrtEi_rad_s_Roll;
        state_.q_rad_s     = snap.bodyAngularRateWrtEi_rad_s_Pitch;
        state_.r_rad_s     = snap.bodyAngularRateWrtEi_rad_s_Yaw;
        state_.altRate_m_s = snap.altitudeRateWrtMsl_m_s;

        state_.v_north_m_s = snap.feVelocity_m_s.x();
        state_.v_east_m_s  = snap.feVelocity_m_s.y();
        state_.v_down_m_s  = snap.feVelocity_m_s.z();

        state_.a_m_s       = snap.speedOfSound_m_s;
        state_.rho_kg_m3   = snap.airDensity_kg_m3;
        state_.P_Pa        = snap.ambientPressure_Pa;
        state_.T_K         = snap.ambientTemperature_K;

        state_.aero_Fx_N   = snap.aero_bodyForce_N_X;
        state_.aero_Fy_N   = snap.aero_bodyForce_N_Y;
        state_.aero_Fz_N   = snap.aero_bodyForce_N_Z;
        state_.aero_Mx_Nm  = snap.aero_bodyMoment_Nm_L;
        state_.aero_My_Nm  = snap.aero_bodyMoment_Nm_M;
        state_.aero_Mz_Nm  = snap.aero_bodyMoment_Nm_N;

        state_.mach        = snap.mach;
        state_.qbar_Pa     = snap.dynamicPressure_Pa;
        state_.vt_m_s      = snap.trueAirspeed_m_s;

        state_.thrust_N    = prop.thrust_N;
        state_.mdot_kgs    = prop.mdot_kgs;
        state_.mass_kg     = m_sim->state().m;

        state_.stg1_fuel_used_kg = fuel.stg1FuelUsed_kg;
        state_.stg2_fuel_used_kg = fuel.stg2FuelUsed_kg;
        state_.staged            = fuel.staged;
    }

    // ── Parameters ────────────────────────────────────────────────────────────
    double p_lat0_deg_     { kDefault_lat0_deg     };
    double p_lon0_deg_     { kDefault_lon0_deg     };
    double p_alt0_m_       { kDefault_alt0_m       };
    double p_azimuth0_deg_ { kDefault_azimuth0_deg };
    double p_pitch0_deg_   { kDefault_pitch0_deg   };
    double p_roll0_deg_    { kDefault_roll0_deg    };
    double p_vNorth0_mps_  { kDefault_vNorth0_mps  };
    double p_vEast0_mps_   { kDefault_vEast0_mps   };
    double p_vDown0_mps_   { kDefault_vDown0_mps   };

    double p_stg2_ignition_time_s_ { kDefault_stg2_ignition_time_s };
    double p_max_step_s_           { kDefault_max_step_s           };

    // ── Runtime objects ───────────────────────────────────────────────────────
    double m_theta0 { 0.0 };   ///< Earth Rotation Angle at t=0 [rad].

    // Simulator is empty until exit_initialisation_mode; cleared on reset.
    std::optional<RocketSimulator> m_sim;

    // ── FMU state (POD) ───────────────────────────────────────────────────────
    TwoStageRocketState state_{};
};

// ── Model metadata ────────────────────────────────────────────────────────────
model_info fmu4cpp::get_model_info()
{
    model_info info;
    info.modelName            = "TwoStageRocket";
    info.description          = "Aetherion two-stage rocket 6-DoF plant "
                                "(Radau IIA RKMK on SE(3), DAVE-ML aero/prop/inertia, "
                                "J2 gravity, stage separation) — NASA TM-2015-218675 Scenario 17";
    info.canGetAndSetFMUstate = true;
    info.canSerializeFMUstate = true;
    return info;
}

FMU4CPP_INSTANTIATE(TwoStageRocketFMU);

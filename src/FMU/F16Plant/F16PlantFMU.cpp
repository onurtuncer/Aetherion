// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16PlantFMU.cpp
//
// FMI 2.0 Co-Simulation FMU wrapping the Aetherion F-16 6-DoF plant.
//
// Model identity : F16Plant
// FMI standard   : FMI 2.0 (Co-Simulation)
// Integrator     : Radau IIA RKMK (3-stage implicit, Lie-group preserving)
//
// The FMU owns a SixDoFStepper<F16VF> directly (bypassing ISimulator) so that
// the integration state (m_state : RigidBody::StateD) and the FMU POD state
// (state_ : F16PlantState) can be kept in sync for getFMUState/setFMUState.
//
// Variable map
// ─────────────────────────────────────────────────────────────────────────────
//  PARAMETERS  (fixed before fmi2ExitInitializationMode)
//    vt0_fps          True airspeed at trim              [ft/s]   def 565.685
//    alt0_ft          Altitude at trim                   [ft]     def 10013.0
//    lat0_deg         Geodetic latitude                  [deg]    def  36.019
//    lon0_deg         Geodetic longitude                 [deg]    def -75.674
//    heading0_deg     Heading (azimuth from North, NED)  [deg]    def  45.0
//    roll0_deg        Initial pose roll angle            [deg]    def  -0.172
//    xcg_from_ac_ft   CG offset aft of AC (10%×c̄)      [ft]     def   1.132
//
//  INPUTS  (may be set between fmi2DoStep calls)
//    ctrl.el_deg      Elevator deflection                [deg]
//    ctrl.ail_deg     Aileron  deflection                [deg]
//    ctrl.rdr_deg     Rudder   deflection                [deg]
//    ctrl.pwr_pct     Throttle                           [0–100]
//
//  OUTPUTS  (valid after fmi2ExitInitializationMode and each fmi2DoStep)
//    out.alt_m        Altitude above MSL                 [m]
//    out.lat_rad      Geodetic latitude                  [rad]
//    out.lon_rad      Geodetic longitude                 [rad]
//    out.yaw_rad      ZYX Euler yaw   (body → NED)       [rad]
//    out.pitch_rad    ZYX Euler pitch (body → NED)       [rad]
//    out.roll_rad     ZYX Euler roll  (body → NED)       [rad]
//    out.p_rad_s      Body roll  rate wrt ECI            [rad/s]
//    out.q_rad_s      Body pitch rate wrt ECI            [rad/s]
//    out.r_rad_s      Body yaw   rate wrt ECI            [rad/s]
//    out.v_north_m_s  NED north velocity                 [m/s]
//    out.v_east_m_s   NED east  velocity                 [m/s]
//    out.v_down_m_s   NED down  velocity                 [m/s]
//    out.alpha_deg    Angle of attack                    [deg]
//    out.beta_deg     Sideslip angle                     [deg]
//    out.mach         Mach number                        [-]
//    out.qbar_Pa      Dynamic pressure                   [Pa]
//    out.vt_m_s       True airspeed                      [m/s]
//    out.aero_Fx_N    Aero body-X force                  [N]
//    out.aero_Fy_N    Aero body-Y force                  [N]
//    out.aero_Fz_N    Aero body-Z force                  [N]
//    out.aero_Mx_Nm   Aero roll  moment (about AC)       [N·m]
//    out.aero_My_Nm   Aero pitch moment (about AC)       [N·m]
//    out.aero_Mz_Nm   Aero yaw   moment                  [N·m]
//    out.rho_kg_m3    Air density                        [kg/m³]
//    out.T_K          Ambient static temperature         [K]
//    out.P_Pa         Ambient static pressure            [Pa]
//    out.a_m_s        Speed of sound                     [m/s]
//    out.g_m_s2       Local gravity magnitude            [m/s²]
//
// FMU lifecycle
// ─────────────────────────────────────────────────────────────────────────────
//  fmi2Instantiate          → constructor: register variables
//  fmi2EnterInitializationMode
//  fmi2SetReal (params)     → write parameters
//  fmi2ExitInitializationMode → exit_initialisation_mode():
//                               load DML from resources/
//                               run TrimSolver (CppAD-backed Newton, 2-stage)
//                               build initial ECI state via BuildInitialStateVector
//                               emplace SixDoFStepper<F16VF>
//                               seed state_ and populate output cache
//  fmi2DoStep(t, dt)        → do_step(dt):
//                               push ctrl.* into VectorField policies
//                               call m_stepper->step(currentTime(), m_state, dt)
//                               sync m_state → state_, update output cache
//  fmi2GetFMUstate /
//  fmi2SetFMUstate          → save/restore F16PlantState (POD struct)
//                               setFmuState also calls unpackState() to keep
//                               m_state (Eigen) in sync with restored state_ (POD)
//  fmi2Reset                → reset(): tear down stepper, zero all state
// ------------------------------------------------------------------------------

#include <fmu4cpp/fmu_base.hpp>

// Standard library
#include <algorithm>      // std::clamp
#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>

// Aetherion — F16 type aliases (includes VectorField, SixDoFStepper, policies)
#include <Aetherion/Examples/F16SteadyFlight/F16Types.h>

// Snapshot + coordinate output
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>

// Initial state construction
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/BuildInitialState.h>
#include <Aetherion/RigidBody/Config.h>
#include <Aetherion/RigidBody/StateLayout.h>

// Trim solver
#include <Aetherion/FlightDynamics/Trim/TrimSolver.h>

// Serialization (DAVE-ML)
#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLPropModel.h>
#include <Aetherion/Serialization/DAVEML/LoadInertiaFromDAVEML.h>

// Earth rotation rate
#include <Aetherion/Environment/WGS84.h>

using namespace fmu4cpp;

// ── Namespace aliases ─────────────────────────────────────────────────────────
namespace AE_RB  = Aetherion::RigidBody;
namespace AE_FD  = Aetherion::FlightDynamics;
namespace AE_SR  = Aetherion::Serialization;
namespace AE_SIM = Aetherion::Simulation;
namespace AE_EX  = Aetherion::Examples::F16SteadyFlight;

// ── Type aliases ──────────────────────────────────────────────────────────────
using F16VF      = AE_EX::F16VF;       // VectorField<J2Gravity, F16Aero, F16Prop, ConstMass>
using F16Stepper = AE_EX::F16Stepper;  // SixDoFStepper<F16VF>

// ── Module-level constants ────────────────────────────────────────────────────
namespace {
    constexpr double kOmegaEarth_rad_s = Aetherion::Environment::WGS84::kRotationRate_rad_s;
    constexpr double kFt_m             = 0.3048;
    constexpr double kLbf_N            = 4.448221615260751;
    constexpr double kG_mps2           = 9.80665;
    constexpr double kDeg              = std::numbers::pi / 180.0;

    // Default initial conditions — NASA TM-2015-218675 Scenario 11 (Kitty Hawk, NC).
    // Settable via FMI PARAMETER before fmi2ExitInitializationMode.
    constexpr double kDefault_vt0_fps        = 565.685;
    constexpr double kDefault_alt0_ft        = 10013.0;
    constexpr double kDefault_lat0_deg       =   36.01917;
    constexpr double kDefault_lon0_deg       =  -75.67444;
    constexpr double kDefault_heading0_deg   =   45.0;
    constexpr double kDefault_roll0_deg      =   -0.172;
    constexpr double kDefault_xcg_from_ac_ft =    1.132;   // (35%−25%) × 11.32 ft
}

// ── FMU state (trivially copyable POD) ───────────────────────────────────────
// Everything captured by getFMUState / setFMUState lives in this struct.
// On setFMUState, the base class copies this back, then unpackState()
// re-derives the Eigen types needed by the stepper.
struct F16PlantState {
    // ── Integration state ─────────────────────────────────────────────────────
    std::array<double, 9> R   {};   // SO(3) rotation matrix, row-major (body → ECI)
    std::array<double, 3> r_I {};   // ECI position [m]
    std::array<double, 6> nu_B{};   // body twist: [wx, wy, wz (rad/s), vx, vy, vz (m/s)]
    double mass_kg {};   // vehicle mass [kg]

    // ── Control surface state (preserved across save/restore) ─────────────────
    double el_deg  {};   // elevator  [deg]
    double ail_deg {};   // aileron   [deg]
    double rdr_deg {};   // rudder    [deg]
    double pwr_pct {};   // throttle  [0–100]

    // ── Output cache (updated in do_step, registered by pointer) ─────────────
    double alt_m       {};  // altitude above MSL [m]
    double lat_rad     {};  // geodetic latitude [rad]
    double lon_rad     {};  // geodetic longitude [rad]
    double yaw_rad     {};  // ZYX Euler yaw   (body → NED) [rad]
    double pitch_rad   {};  // ZYX Euler pitch (body → NED) [rad]
    double roll_rad    {};  // ZYX Euler roll  (body → NED) [rad]
    double p_rad_s     {};  // body roll  rate wrt ECI [rad/s]
    double q_rad_s     {};  // body pitch rate wrt ECI [rad/s]
    double r_rad_s     {};  // body yaw   rate wrt ECI [rad/s]
    double v_north_m_s {};  // NED north velocity [m/s]
    double v_east_m_s  {};  // NED east  velocity [m/s]
    double v_down_m_s  {};  // NED down  velocity [m/s]
    double alpha_deg   {};  // angle of attack [deg]
    double beta_deg    {};  // sideslip angle  [deg]
    double mach        {};  // Mach number [-]
    double qbar_Pa     {};  // dynamic pressure [Pa]
    double vt_m_s      {};  // true airspeed [m/s]
    double aero_Fx_N   {};  // aero body-X force  [N]
    double aero_Fy_N   {};  // aero body-Y force  [N]
    double aero_Fz_N   {};  // aero body-Z force  [N]
    double aero_Mx_Nm  {};  // aero roll  moment  [N·m]  (about AC)
    double aero_My_Nm  {};  // aero pitch moment  [N·m]  (about AC)
    double aero_Mz_Nm  {};  // aero yaw   moment  [N·m]
    double rho_kg_m3   {};  // air density [kg/m³]
    double T_K         {};  // ambient temperature [K]
    double P_Pa        {};  // ambient pressure [Pa]
    double a_m_s       {};  // speed of sound [m/s]
    double g_m_s2      {};  // local gravity magnitude [m/s²]
};
static_assert(std::is_trivially_copyable_v<F16PlantState>,
    "F16PlantState must be trivially copyable for fmu4cpp state save/restore.");

// ─────────────────────────────────────────────────────────────────────────────
// F16PlantFMU
// ─────────────────────────────────────────────────────────────────────────────
class F16PlantFMU : public fmu_base {
public:

    // ── Constructor: register all FMI variables ───────────────────────────────
    FMU4CPP_CTOR(F16PlantFMU)
    {
        // ── Parameters ───────────────────────────────────────────────────────
        register_real("vt0_fps", &p_vt0_fps_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("True airspeed at trim [ft/s]");

        register_real("alt0_ft", &p_alt0_ft_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Altitude at trim [ft]");

        register_real("lat0_deg", &p_lat0_deg_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Initial geodetic latitude [deg]");

        register_real("lon0_deg", &p_lon0_deg_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Initial geodetic longitude [deg]");

        register_real("heading0_deg", &p_heading0_deg_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Initial heading (azimuth from North) [deg]");

        register_real("roll0_deg", &p_roll0_deg_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Initial pose roll angle [deg]");

        register_real("xcg_from_ac_ft", &p_xcg_from_ac_ft_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("CG offset aft of aerodynamic reference centre [ft]");

        // ── Control inputs ────────────────────────────────────────────────────
        register_real("ctrl.el_deg",  &state_.el_deg)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Elevator deflection [deg]");

        register_real("ctrl.ail_deg", &state_.ail_deg)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Aileron deflection [deg]");

        register_real("ctrl.rdr_deg", &state_.rdr_deg)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Rudder deflection [deg]");

        register_real("ctrl.pwr_pct", &state_.pwr_pct)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Throttle [0–100 %]");

        // ── Outputs ───────────────────────────────────────────────────────────
        register_real("out.alt_m",       &state_.alt_m)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Altitude above MSL [m]");

        register_real("out.lat_rad",     &state_.lat_rad)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Geodetic latitude [rad]");

        register_real("out.lon_rad",     &state_.lon_rad)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Geodetic longitude [rad]");

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

        register_real("out.v_north_m_s", &state_.v_north_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("NED north velocity [m/s]");

        register_real("out.v_east_m_s",  &state_.v_east_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("NED east velocity [m/s]");

        register_real("out.v_down_m_s",  &state_.v_down_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("NED down velocity [m/s]");

        register_real("out.alpha_deg",   &state_.alpha_deg)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Angle of attack [deg]");

        register_real("out.beta_deg",    &state_.beta_deg)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Sideslip angle [deg]");

        register_real("out.mach",        &state_.mach)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Mach number [-]");

        register_real("out.qbar_Pa",     &state_.qbar_Pa)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Dynamic pressure [Pa]");

        register_real("out.vt_m_s",      &state_.vt_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("True airspeed [m/s]");

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
            .setInitial(initial_t::CALCULATED).setDescription("Aerodynamic roll moment (about AC) [N·m]");

        register_real("out.aero_My_Nm",  &state_.aero_My_Nm)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Aerodynamic pitch moment (about AC) [N·m]");

        register_real("out.aero_Mz_Nm",  &state_.aero_Mz_Nm)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Aerodynamic yaw moment [N·m]");

        register_real("out.rho_kg_m3",   &state_.rho_kg_m3)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Air density [kg/m³]");

        register_real("out.T_K",         &state_.T_K)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Ambient static temperature [K]");

        register_real("out.P_Pa",        &state_.P_Pa)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Ambient static pressure [Pa]");

        register_real("out.a_m_s",       &state_.a_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Speed of sound [m/s]");

        register_real("out.g_m_s2",      &state_.g_m_s2)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Local gravitational acceleration [m/s²]");

        // ── State registration (enables getFMUState / setFMUState) ────────────
        register_state(&F16PlantFMU::state_);
    }

    // ── exit_initialisation_mode ──────────────────────────────────────────────
    // Called after fmi2ExitInitializationMode: all parameters are set.
    // Loads DAVE-ML models, runs trim, builds initial ECI state, emplaces stepper.
    void exit_initialisation_mode() override
    {
        // 1. Build DML paths from the FMU resources/ folder
        const std::string res         = resourceLocation().string();
        const std::string inertiaPath = res + "/F16_inertia.dml";
        const std::string aeroPath    = res + "/F16_aero.dml";
        const std::string propPath    = res + "/F16_prop.dml";

        // 2. Load inertia, aero, and propulsion models
        m_ip        = AE_SR::LoadInertiaFromDAVEML(inertiaPath);
        m_aeroModel = std::make_shared<const AE_SR::DAVEMLAeroModel>(aeroPath);
        m_propModel = std::make_shared<const AE_SR::DAVEMLPropModel>(propPath);

        // 3. Solve trim: find (alpha, elevator, throttle) for the given flight condition
        const double weight_lbf = m_ip.mass_kg * kG_mps2 / kLbf_N;
        AE_FD::TrimInputs tin{};
        tin.vt_fps     = p_vt0_fps_;
        tin.alt_ft     = p_alt0_ft_;
        tin.weight_lbf = weight_lbf;

        AE_FD::TrimSolver solver(*m_aeroModel, *m_propModel, p_xcg_from_ac_ft_);
        const AE_FD::TrimPoint trim = solver.solve(tin);
        if (!trim.converged)
            throw std::runtime_error("F16PlantFMU: trim solver did not converge.");

        // 4. Build initial ECI state from the geodetic initial conditions
        // theta0 = Earth Rotation Angle at fmi2EnterInitializationMode time
        m_theta0 = kOmegaEarth_rad_s * currentTime();

        AE_RB::Config cfg{};
        cfg.pose.lat_deg     = p_lat0_deg_;
        cfg.pose.lon_deg     = p_lon0_deg_;
        cfg.pose.alt_m       = p_alt0_ft_ * kFt_m;
        cfg.pose.azimuth_deg = p_heading0_deg_;
        cfg.pose.zenith_deg  = 90.0 - trim.alpha_deg;   // nearly horizontal, nose-up by alpha
        cfg.pose.roll_deg    = p_roll0_deg_;

        // Horizontal TAS decomposed along heading (no downward component at trim)
        const double vt_mps           = p_vt0_fps_ * kFt_m;
        cfg.velocityNED.north_mps     = vt_mps * std::cos(p_heading0_deg_ * kDeg);
        cfg.velocityNED.east_mps      = vt_mps * std::sin(p_heading0_deg_ * kDeg);
        cfg.velocityNED.down_mps      = 0.0;
        cfg.bodyRates.roll_rad_s      = 0.0;
        cfg.bodyRates.pitch_rad_s     = 0.0;
        cfg.bodyRates.yaw_rad_s       = 0.0;
        cfg.inertialParameters        = m_ip;

        const auto x0vec = AE_RB::BuildInitialStateVector(cfg, m_theta0);
        using L = AE_RB::StateLayout;

        m_state.g.p = Eigen::Vector3d(x0vec[L::IDX_P],   x0vec[L::IDX_P+1], x0vec[L::IDX_P+2]);
        const Eigen::Quaterniond q0(x0vec[L::IDX_Q], x0vec[L::IDX_Q+1],
                                    x0vec[L::IDX_Q+2], x0vec[L::IDX_Q+3]);
        m_state.g.q = q0.normalized();
        m_state.g.R = m_state.g.q.toRotationMatrix();
        m_state.nu_B << x0vec[L::IDX_W],   x0vec[L::IDX_W+1], x0vec[L::IDX_W+2],
                        x0vec[L::IDX_V],   x0vec[L::IDX_V+1], x0vec[L::IDX_V+2];
        m_state.m = x0vec[L::IDX_M];

        // 5. Construct the stepper with a VectorField initialised at trim values
        const double xcg_m = p_xcg_from_ac_ft_ * kFt_m;
        m_stepper.emplace(
            F16VF(m_ip,
                  AE_FD::J2GravityPolicy{},
                  AE_FD::F16AeroPolicy(m_aeroModel, trim.el_deg, 0.0, 0.0, xcg_m),
                  AE_FD::F16PropPolicy(m_propModel, trim.pwr_pct))
        );

        // 6. Seed control state from trim (written to state_ for fmu4cpp variable tracking)
        state_.el_deg  = trim.el_deg;
        state_.ail_deg = 0.0;
        state_.rdr_deg = 0.0;
        state_.pwr_pct = trim.pwr_pct;

        // 7. Sync POD integration state and populate initial outputs
        packState();
        populateOutputCache(currentTime());
    }

    // ── do_step ───────────────────────────────────────────────────────────────
    // Advances the simulation by dt seconds.
    // The FMI master has already written any updated ctrl.* inputs before this call.
    bool do_step(double dt) override
    {
        if (!m_stepper.has_value())
            return false;

        // Push the latest control inputs from state_ into the VectorField policies
        auto& vf = m_stepper->vectorField();
        vf.aero.setControls(state_.el_deg, state_.ail_deg, state_.rdr_deg);
        vf.thrust.pwr_pct = state_.pwr_pct;

        // Advance one step with the Radau IIA RKMK integrator
        const double t0 = currentTime();
        const auto res = m_stepper->step(t0, m_state, dt);
        if (!res.converged) {
            debugLog(fmiWarning, "F16PlantFMU: Radau IIA Newton did not converge.");
            return false;
        }
        m_state = F16Stepper::unpack(res);

        // Sync integration state back into POD and update output variables.
        // Pass t0+dt because fmu_base advances time_ only after do_step returns.
        packState();
        populateOutputCache(t0 + dt);
        return true;
    }

    // ── setFmuState ───────────────────────────────────────────────────────────
    // Overridden to keep m_state (Eigen) consistent with the restored state_ (POD).
    void setFmuState(void* fmuState) override
    {
        fmu_base::setFmuState(fmuState);   // copies saved F16PlantState → state_
        if (m_stepper.has_value())
            unpackState();                 // re-derives m_state from state_
    }

    // ── reset ─────────────────────────────────────────────────────────────────
    void reset() override
    {
        m_stepper.reset();
        m_aeroModel.reset();
        m_propModel.reset();
        m_ip     = {};
        m_theta0 = 0.0;
        m_state  = {};
        state_   = F16PlantState{};

        p_vt0_fps_        = kDefault_vt0_fps;
        p_alt0_ft_        = kDefault_alt0_ft;
        p_lat0_deg_       = kDefault_lat0_deg;
        p_lon0_deg_       = kDefault_lon0_deg;
        p_heading0_deg_   = kDefault_heading0_deg;
        p_roll0_deg_      = kDefault_roll0_deg;
        p_xcg_from_ac_ft_ = kDefault_xcg_from_ac_ft;
    }

private:

    // ── packState ─────────────────────────────────────────────────────────────
    // Sync Eigen integration state (m_state) → POD fields in state_.
    // Call after each step and during initialisation.
    // Does NOT touch control inputs (state_.el_deg etc.) — those are managed
    // by the fmu4cpp variable system and written by the FMI master.
    void packState()
    {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                state_.R[i * 3 + j] = m_state.g.R(i, j);
        state_.r_I[0] = m_state.g.p.x();
        state_.r_I[1] = m_state.g.p.y();
        state_.r_I[2] = m_state.g.p.z();
        for (int i = 0; i < 6; ++i)
            state_.nu_B[i] = m_state.nu_B(i);
        state_.mass_kg = m_state.m;
    }

    // ── unpackState ───────────────────────────────────────────────────────────
    // Restore Eigen integration state (m_state) from POD fields in state_.
    // Called by setFmuState after the base class has copied the saved state back.
    // Also syncs control inputs back into the VectorField.
    void unpackState()
    {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                m_state.g.R(i, j) = state_.R[i * 3 + j];
        // Re-derive quaternion from rotation matrix (SE3 keeps both in sync)
        m_state.g.q = Eigen::Quaterniond(m_state.g.R).normalized();
        m_state.g.p = Eigen::Vector3d(state_.r_I[0], state_.r_I[1], state_.r_I[2]);
        for (int i = 0; i < 6; ++i)
            m_state.nu_B(i) = state_.nu_B[i];
        m_state.m = state_.mass_kg;

        // Re-sync control surface state into VF (also restored from POD)
        if (!m_stepper.has_value()) return;
        auto& vf = m_stepper->vectorField();
        vf.aero.setControls(state_.el_deg, state_.ail_deg, state_.rdr_deg);
        vf.thrust.pwr_pct = state_.pwr_pct;
    }

    // ── populateOutputCache ───────────────────────────────────────────────────
    // Recomputes all output state_ fields from m_state.
    // MakeSnapshot1 handles coordinate transforms (ECI→ECEF→NED, Euler angles,
    // gravity, atmosphere). Alpha and beta are computed separately because
    // Snapshot1 does not carry them.
    // @param t  Simulation time at the END of the step being reported.
    //           Callers pass (currentTime()) at init and (t0+dt) inside do_step
    //           because fmu_base only advances time_ after do_step returns.
    void populateOutputCache(double t)
    {
        if (!m_stepper.has_value()) return;

        const double theta_GST = m_theta0 + kOmegaEarth_rad_s * t;
        const auto&  vf        = m_stepper->vectorField();

        // MakeSnapshot1 (two-policy overload) fills kinematics, atmosphere, and
        // aero forces/moments.  aero_bodyMoment_Nm_M is reported about the AC
        // (MakeSnapshot1 subtracts the xcg_from_ac_m transfer term for output).
        const AE_SIM::Snapshot1 snap =
            AE_SIM::MakeSnapshot1(t, m_state, theta_GST, vf.gravity, vf.aero);

        state_.alt_m       = snap.altitudeMsl_m;
        state_.lat_rad     = snap.latitude_rad;
        state_.lon_rad     = snap.longitude_rad;

        state_.yaw_rad     = snap.eulerAngle_rad_Yaw;
        state_.pitch_rad   = snap.eulerAngle_rad_Pitch;
        state_.roll_rad    = snap.eulerAngle_rad_Roll;

        state_.p_rad_s     = snap.bodyAngularRateWrtEi_rad_s_Roll;
        state_.q_rad_s     = snap.bodyAngularRateWrtEi_rad_s_Pitch;
        state_.r_rad_s     = snap.bodyAngularRateWrtEi_rad_s_Yaw;

        state_.v_north_m_s = snap.feVelocity_m_s.x();
        state_.v_east_m_s  = snap.feVelocity_m_s.y();
        state_.v_down_m_s  = snap.feVelocity_m_s.z();

        state_.vt_m_s      = snap.trueAirspeed_m_s;
        state_.mach        = snap.mach;
        state_.qbar_Pa     = snap.dynamicPressure_Pa;

        state_.aero_Fx_N   = snap.aero_bodyForce_N_X;
        state_.aero_Fy_N   = snap.aero_bodyForce_N_Y;
        state_.aero_Fz_N   = snap.aero_bodyForce_N_Z;
        state_.aero_Mx_Nm  = snap.aero_bodyMoment_Nm_L;
        state_.aero_My_Nm  = snap.aero_bodyMoment_Nm_M;
        state_.aero_Mz_Nm  = snap.aero_bodyMoment_Nm_N;

        state_.rho_kg_m3   = snap.airDensity_kg_m3;
        state_.T_K         = snap.ambientTemperature_K;
        state_.P_Pa        = snap.ambientPressure_Pa;
        state_.a_m_s       = snap.speedOfSound_m_s;
        state_.g_m_s2      = snap.localGravity_m_s2;

        // Alpha and beta — same formula as F16AeroPolicy, avoids re-evaluating the
        // full aero model a second time.
        const Eigen::Vector3d omega_E(0.0, 0.0, kOmegaEarth_rad_s);
        const Eigen::Vector3d v_surface = m_state.g.R.transpose() * omega_E.cross(m_state.g.p);
        const Eigen::Vector3d v_rel     = m_state.nu_B.tail<3>() - v_surface;
        const double vt = std::sqrt(v_rel.squaredNorm() + 1.0e-30);
        state_.alpha_deg = std::atan2(v_rel.z(), v_rel.x() + 1.0e-12)
                           * (180.0 / std::numbers::pi);
        state_.beta_deg  = std::asin(std::clamp(v_rel.y() / vt, -1.0, 1.0))
                           * (180.0 / std::numbers::pi);
    }

    // ── Parameters ────────────────────────────────────────────────────────────
    double p_vt0_fps_        { kDefault_vt0_fps        };
    double p_alt0_ft_        { kDefault_alt0_ft        };
    double p_lat0_deg_       { kDefault_lat0_deg       };
    double p_lon0_deg_       { kDefault_lon0_deg       };
    double p_heading0_deg_   { kDefault_heading0_deg   };
    double p_roll0_deg_      { kDefault_roll0_deg      };
    double p_xcg_from_ac_ft_ { kDefault_xcg_from_ac_ft };

    // ── Runtime objects ───────────────────────────────────────────────────────
    // Models kept alive after exit_initialisation_mode so that setFmuState /
    // reset can rebuild the stepper without reloading DAVE-ML from disk.
    std::shared_ptr<const AE_SR::DAVEMLAeroModel> m_aeroModel;
    std::shared_ptr<const AE_SR::DAVEMLPropModel> m_propModel;
    AE_RB::InertialParameters                     m_ip{};
    double                                         m_theta0 { 0.0 };

    // Stepper is empty until exit_initialisation_mode; cleared on reset.
    std::optional<F16Stepper> m_stepper;

    // Authoritative Eigen integration state — the stepper is stateless and takes
    // this by const-ref.  Managed separately from state_ so the two can be
    // independently saved (getFMUState) and restored (setFMUState).
    AE_RB::StateD m_state{};

    // ── FMU state (POD) ───────────────────────────────────────────────────────
    F16PlantState state_{};
};

// ── Model metadata ────────────────────────────────────────────────────────────
model_info fmu4cpp::get_model_info()
{
    model_info info;
    info.modelName            = "F16Plant";
    info.description          = "Aetherion F-16 6-DoF plant "
                                "(Radau IIA RKMK on SE(3), DAVE-ML aero/prop, J2 gravity)";
    info.canGetAndSetFMUstate = true;
    info.canSerializeFMUstate = true;
    return info;
}

FMU4CPP_INSTANTIATE(F16PlantFMU);

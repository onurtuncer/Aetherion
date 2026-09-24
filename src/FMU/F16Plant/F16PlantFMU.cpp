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
//    roll0_deg        Initial pose roll angle            [deg]    def   0.0
//    xcg_from_ac_ft   CG offset aft of AC (10%×c̄)      [ft]     def   1.132
//    solver.abs_tol   Newton absolute residual tolerance [-]      def 1e-12
//    solver.rel_tol   Newton relative residual tolerance [-]      def 1e-10
//    solver.max_step_s Max internal integrator step      [s]      def 0.0 (= comm. step)
//    wind.north_mps   Steady wind, NED north             [m/s]    def   0.0
//    wind.east_mps    Steady wind, NED east              [m/s]    def   0.0
//    wind.down_mps    Steady wind, NED down              [m/s]    def   0.0
//    turb.sigma_u_mps Dryden RMS gust, body x            [m/s]    def   0.0  (all three 0 = off)
//    turb.sigma_v_mps Dryden RMS gust, body y            [m/s]    def   0.0
//    turb.sigma_w_mps Dryden RMS gust, body z            [m/s]    def   0.0
//    turb.L_u_m       Dryden scale length, u             [m]      def 533.4  (1750 ft)
//    turb.L_v_m       Dryden scale length, v             [m]      def 533.4
//    turb.L_w_m       Dryden scale length, w             [m]      def 533.4
//    turb.seed        Dryden noise seed, integer-valued  [-]      def   1
//    atm.deltaT_K     ISA temperature offset             [K]      def   0.0
//    atm.deltaP_sl_Pa Sea-level pressure offset          [Pa]     def   0.0
//
//  INPUTS  (may be set between fmi2DoStep calls)
//    ctrl.el_deg      Elevator deflection                [deg]
//    ctrl.ail_deg     Aileron  deflection                [deg]
//    ctrl.rdr_deg     Rudder   deflection                [deg]
//    ctrl.pwr_pct     Throttle                           [0–100]
//
//  OUTPUTS  (valid after fmi2ExitInitializationMode and each fmi2DoStep)
//    out.alt_m        Altitude above MSL                 [m]
//    out.lat_deg      Geodetic latitude                  [deg]
//    out.lon_deg      Geodetic longitude                 [deg]
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
//    out.thrust_N     Net propulsive force, body +X      [N]
//    out.mass_kg      Vehicle mass                       [kg]
//    out.specificForce_x_m_s2   Body-X specific force at CG  [m/s²]
//    out.specificForce_y_m_s2   Body-Y specific force at CG  [m/s²]
//    out.specificForce_z_m_s2   Body-Z specific force at CG  [m/s²]
//
//    out.wind_north_m_s, out.wind_east_m_s, out.wind_down_m_s
//                     Total wind at the CG (steady + gust), NED         [m/s]
//    out.gust_u_m_s, out.gust_v_m_s, out.gust_w_m_s
//                     Dryden gust velocity, body axes                    [m/s]
//    out.gust_p_rad_s, out.gust_q_rad_s, out.gust_r_rad_s
//                     Angular velocity of the gust field, body axes      [rad/s]
//
//  Environment: the steady wind is fixed in NED at the initial position and
//  converted to ECEF once (ConstantECEFWind::from_ned).  The trim is
//  air-relative and unchanged by wind; the initial ground velocity is the
//  trim airspeed along the heading plus the wind.  Turbulence (Dryden,
//  MIL-F-8785C) is stepped once per integrator sub-step at the current
//  airspeed and held constant over the step; its eight filter states are in
//  the saved FMU state, the noise stream is not rewound by setFMUState.
//  atm.* shifts the US1976 atmosphere for the forces, the trim and out.P_Pa,
//  out.T_K, out.rho_kg_m3, out.a_m_s alike.  All defaults reproduce the
//  calm, standard-day plant bit for bit.
//
//  out.specificForce_* is the non-gravitational acceleration an ideal
//  accelerometer at the CG would sense, (F_aero + F_thrust)/m, in the same body
//  axes as out.aero_F*_N.  Gravitation is excluded entirely, so it reads zero in
//  free fall.  See Aetherion/Simulation/BodySpecificForce.h.
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
#include <Aetherion/Simulation/BodySpecificForce.h>

// Initial state construction
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/BuildInitialState.h>
#include <Aetherion/RigidBody/Config.h>
#include <Aetherion/RigidBody/StateLayout.h>

// Trim solver
#include <Aetherion/FlightDynamics/Trim/TrimSolver.h>
#include <Aetherion/FlightDynamics/Trim/TrimBodyRates.h>
#include <Aetherion/FlightDynamics/Trim/TrimWeight.h>

// Serialization (DAVE-ML)
#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLPropModel.h>
#include <Aetherion/Serialization/DAVEML/LoadInertiaFromDAVEML.h>

// Earth rotation rate
#include <Aetherion/Environment/WGS84.h>

// Environment: atmosphere offsets, steady wind, Dryden turbulence
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Environment/DrydenTurbulence.h>
#include <Aetherion/Environment/WindModels.h>
#include <Aetherion/Coordinate/InertialToLocal.h>
#include <cstdint>

using namespace fmu4cpp;

// ── Namespace aliases ─────────────────────────────────────────────────────────
namespace AE_RB  = Aetherion::RigidBody;
namespace AE_FD  = Aetherion::FlightDynamics;
namespace AE_SR  = Aetherion::Serialization;
namespace AE_SIM = Aetherion::Simulation;
namespace AE_EX  = Aetherion::Examples::F16SteadyFlight;
namespace AE_ENV = Aetherion::Environment;

// ── Type aliases ──────────────────────────────────────────────────────────────
using F16VF      = AE_EX::F16VF;       // VectorField<J2Gravity, F16Aero, F16Prop, ConstMass>
using F16Stepper = AE_EX::F16Stepper;  // SixDoFStepper<F16VF>

// ── Module-level constants ────────────────────────────────────────────────────
namespace {
    constexpr double kOmegaEarth_rad_s = Aetherion::Environment::WGS84::kRotationRate_rad_s;
    constexpr double kFt_m             = 0.3048;
    constexpr double kDeg              = std::numbers::pi / 180.0;
    constexpr double kRadToDeg         = 180.0 / std::numbers::pi;

    // Default initial conditions — NASA TM-2015-218675 Scenario 11 (Kitty Hawk, NC).
    // Settable via FMI PARAMETER before fmi2ExitInitializationMode.
    constexpr double kDefault_vt0_fps        = 565.685;
    constexpr double kDefault_alt0_ft        = 10013.0;
    constexpr double kDefault_lat0_deg       =   36.01917;
    constexpr double kDefault_lon0_deg       =  -75.67444;
    constexpr double kDefault_heading0_deg   =   45.0;
    constexpr double kDefault_roll0_deg      =    0.0;     // wings-level, as NASA sims 04/05
    constexpr double kDefault_xcg_from_ac_ft =    1.132;   // (35%−25%) × 11.32 ft

    // Default solver tuning — match NewtonOptions defaults so behaviour is
    // unchanged unless the FMI master overrides these parameters.
    constexpr double kDefault_newton_abs_tol = 1.0e-12;
    constexpr double kDefault_newton_rel_tol = 1.0e-10;
    constexpr double kDefault_max_step_s     = 0.0;  // 0 → use the communication step directly

    // Environment defaults: calm air on a standard day.
    constexpr double kDefault_wind_mps       = 0.0;
    constexpr double kDefault_turb_sigma_mps = 0.0;    // all three zero = turbulence off
    constexpr double kDefault_turb_L_m       = 533.4;  // 1750 ft, MIL-F-8785C above 2000 ft
    constexpr double kDefault_turb_seed      = 1.0;
    constexpr double kDefault_atm_deltaT_K   = 0.0;
    constexpr double kDefault_atm_deltaP_Pa  = 0.0;
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
    double lat_deg     {};  // geodetic latitude [deg]
    double lon_deg     {};  // geodetic longitude [deg]
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
    double specificForce_x_m_s2{};  // body-X specific force at the CG [m/s²]
    double specificForce_y_m_s2{};  // body-Y specific force at the CG [m/s²]
    double specificForce_z_m_s2{};  // body-Z specific force at the CG [m/s²]
    double thrust_N    {};  // net propulsive force, body +X [N]

    // ── Environment outputs ───────────────────────────────────────────────────
    double wind_north_m_s{};  // total wind at the CG, NED north (steady + gust) [m/s]
    double wind_east_m_s {};  // total wind at the CG, NED east  [m/s]
    double wind_down_m_s {};  // total wind at the CG, NED down  [m/s]
    double gust_u_m_s    {};  // Dryden gust, body x [m/s]
    double gust_v_m_s    {};  // Dryden gust, body y [m/s]
    double gust_w_m_s    {};  // Dryden gust, body z [m/s]
    double gust_p_rad_s  {};  // angular velocity of the gust field, body x [rad/s]
    double gust_q_rad_s  {};  // angular velocity of the gust field, body y [rad/s]
    double gust_r_rad_s  {};  // angular velocity of the gust field, body z [rad/s]

    // ── Turbulence filter states (preserved across save/restore) ─────────────
    std::array<double, 8> turb_x{};
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
        registerParameters();
        registerInputs();
        registerOutputs();
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

        // 3. Earth Rotation Angle at fmi2EnterInitializationMode time — needed
        //    for the ECI initial state below.
        m_theta0 = kOmegaEarth_rad_s * currentTime();

        // 4. Solve trim: find (alpha, elevator, throttle) for the given flight
        //    condition.  The weight is the apparent weight at the trim point: the
        //    J2 attraction the integrator's J2GravityPolicy applies, less the
        //    centripetal relief of level flight over the rotating Earth.  Shared
        //    with the examples — see Aetherion/FlightDynamics/Trim/TrimWeight.h.
        //    Horizontal TAS decomposed along heading (no sideslip, no climb at
        //    trim) is the air-relative velocity.  The ground velocity that the
        //    initial state, the apparent weight and the transport rate need is
        //    that plus the steady wind: the aircraft crabs in a crosswind and
        //    its ground speed drops in a headwind, while the trim, which is
        //    air-relative, is unchanged.
        const double vt_mps     = p_vt0_fps_ * kFt_m;
        const double vNorthAir  = vt_mps * std::cos(p_heading0_deg_ * kDeg);
        const double vEastAir   = vt_mps * std::sin(p_heading0_deg_ * kDeg);
        const double vNorth_mps = vNorthAir + p_wind_north_mps_;
        const double vEast_mps  = vEastAir  + p_wind_east_mps_;
        const double vDown_mps  = p_wind_down_mps_;

        const double weight_lbf = AE_FD::LevelFlightTrimWeight_lbf(
            m_ip.mass_kg, p_lat0_deg_, p_alt0_ft_ * kFt_m, vNorth_mps, vEast_mps);
        AE_FD::TrimInputs tin{};
        tin.vt_fps     = p_vt0_fps_;
        tin.alt_ft     = p_alt0_ft_;
        tin.weight_lbf = weight_lbf;
        // Aero damping at the rate the aero policy will actually see: the
        // transport rate.  Pitch is not known yet and q does not depend on it.
        tin.bodyRates  = AE_FD::LevelFlightBodyRates(
            p_lat0_deg_, p_alt0_ft_ * kFt_m, vNorth_mps, vEast_mps,
            p_heading0_deg_, 0.0, p_roll0_deg_);

        AE_FD::TrimSolver solver(*m_aeroModel, *m_propModel, p_xcg_from_ac_ft_);
        solver.setAtmosphereOffsets(atmosphereOffsets());
        const AE_FD::TrimPoint trim = solver.solve(tin);
        if (!trim.converged)
            throw std::runtime_error("F16PlantFMU: trim solver did not converge.");

        AE_RB::Config cfg{};
        cfg.pose.lat_deg     = p_lat0_deg_;
        cfg.pose.lon_deg     = p_lon0_deg_;
        cfg.pose.alt_m       = p_alt0_ft_ * kFt_m;
        cfg.pose.azimuth_deg = p_heading0_deg_;
        cfg.pose.zenith_deg  = 90.0 - trim.alpha_deg;   // nearly horizontal, nose-up by alpha
        cfg.pose.roll_deg    = p_roll0_deg_;

        cfg.velocityNED.north_mps     = vNorth_mps;
        cfg.velocityNED.east_mps      = vEast_mps;
        cfg.velocityNED.down_mps      = vDown_mps;
        // Attitude held fixed against the local-level frame, which tips forward
        // at the transport rate as the vehicle moves over the curved Earth.
        cfg.bodyRates = AE_FD::LevelFlightBodyRates(
            p_lat0_deg_, p_alt0_ft_ * kFt_m, vNorth_mps, vEast_mps,
            p_heading0_deg_, trim.alpha_deg, p_roll0_deg_);
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

        // 5. Build Newton options from the FMI solver parameters.
        Aetherion::ODE::RKMK::Core::NewtonOptions newton_opts{};
        newton_opts.abs_tol = p_newton_abs_tol_;
        newton_opts.rel_tol = p_newton_rel_tol_;

        // 6. Construct the stepper with a VectorField initialised at trim values
        const double xcg_m = p_xcg_from_ac_ft_ * kFt_m;
        m_stepper.emplace(
            F16VF(m_ip,
                  AE_FD::J2GravityPolicy{},
                  AE_FD::F16AeroPolicy(m_aeroModel, trim.el_deg, 0.0, 0.0, xcg_m),
                  AE_FD::F16PropPolicy(m_propModel, trim.pwr_pct)),
            newton_opts
        );

        // 6b. Environment on the aero policy: steady wind (NED at the start
        //     point, converted to ECEF once), atmosphere offsets, and the Dryden
        //     filter bank if any gust intensity is non-zero.
        {
            auto& vf = m_stepper->vectorField();
            vf.aero.setWindECEF(windECEF());
            vf.aero.setAtmosphereOffsets(atmosphereOffsets());
            vf.aero.setGust(AE_ENV::GustState{});
            // The engine forms its Mach from the same air the airframe flies in.
            vf.thrust.setWindECEF(windECEF());
            vf.thrust.setAtmosphereOffsets(atmosphereOffsets());
            vf.thrust.setGust(AE_ENV::GustState{});

            m_turb.reset();
            m_turbStep_s     = 0.0;
            m_turbStepWarned = false;
            state_.turb_x.fill(0.0);
            const AE_ENV::DrydenParameters tp = turbulenceParameters();
            if (!tp.isOff()) {
                m_turb.emplace(tp, static_cast<std::uint64_t>(std::llround(p_turb_seed_)));
            }
        }

        // 7. Seed control state from trim (written to state_ for fmu4cpp variable tracking)
        state_.el_deg  = trim.el_deg;
        state_.ail_deg = 0.0;
        state_.rdr_deg = 0.0;
        state_.pwr_pct = trim.pwr_pct;

        // 8. Sync POD integration state and populate initial outputs
        packState();
        populateOutputCache(currentTime());
    }

    // ── do_step ───────────────────────────────────────────────────────────────
    // Advances the simulation by dt seconds, optionally subdividing into
    // sub-steps of at most solver.max_step_s (0 = single step = communication dt).
    // The FMI master has already written any updated ctrl.* inputs before this call.
    bool do_step(double dt) override
    {
        if (!m_stepper.has_value())
            return false;

        const double t0 = currentTime();
        const double h  = (p_max_step_s_ > 0.0 && p_max_step_s_ < dt)
                          ? p_max_step_s_ : dt;

        double t_local   = t0;
        double remaining = dt;

        while (remaining > 1.0e-15) {
            const double step = std::min(remaining, h);

            // Push the latest control inputs into the VectorField policies
            auto& vf = m_stepper->vectorField();
            vf.aero.setControls(state_.el_deg, state_.ail_deg, state_.rdr_deg);
            vf.thrust.pwr_pct = state_.pwr_pct;

            // Turbulence: advance the Dryden filters by this sub-step at the
            // current airspeed and hold the gust constant over the step.  The
            // filters are defined for a fixed step; warn once if it varies.
            if (m_turb.has_value()) {
                if (m_turbStep_s == 0.0) {
                    m_turbStep_s = step;
                } else if (std::abs(step - m_turbStep_s) > 1.0e-12 && !m_turbStepWarned) {
                    debugLog(fmiWarning, "F16PlantFMU: turbulence is being stepped at a varying dt; "
                                         "the Dryden statistics are defined for a fixed step.");
                    m_turbStepWarned = true;
                }
                const Aetherion::ODE::RKMK::Lie::SE3<double> g_now(m_state.g.R, m_state.g.p);
                const double V = vf.aero.airRelativeVelocity_B<double>(g_now, m_state.nu_B, t_local).norm();
                const AE_ENV::GustState gust = m_turb->step(step, V);
                vf.aero.setGust(gust);
                vf.thrust.setGust(gust);
            }

            const auto res = m_stepper->step(t_local, m_state, step);
            if (!res.converged) {
                debugLog(fmiWarning, "F16PlantFMU: Radau IIA Newton did not converge.");
                return false;
            }
            m_state   = F16Stepper::unpack(res);
            t_local  += step;
            remaining -= step;
        }

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

        p_newton_abs_tol_ = kDefault_newton_abs_tol;
        p_newton_rel_tol_ = kDefault_newton_rel_tol;
        p_max_step_s_     = kDefault_max_step_s;

        p_wind_north_mps_ = kDefault_wind_mps;
        p_wind_east_mps_  = kDefault_wind_mps;
        p_wind_down_mps_  = kDefault_wind_mps;
        p_turb_sigma_u_   = kDefault_turb_sigma_mps;
        p_turb_sigma_v_   = kDefault_turb_sigma_mps;
        p_turb_sigma_w_   = kDefault_turb_sigma_mps;
        p_turb_L_u_m_     = kDefault_turb_L_m;
        p_turb_L_v_m_     = kDefault_turb_L_m;
        p_turb_L_w_m_     = kDefault_turb_L_m;
        p_turb_seed_      = kDefault_turb_seed;
        p_atm_deltaT_K_   = kDefault_atm_deltaT_K;
        p_atm_deltaP_Pa_  = kDefault_atm_deltaP_Pa;
        m_turb.reset();
        m_turbStep_s     = 0.0;
        m_turbStepWarned = false;
    }

private:

    // ── registerParameters ──────────────────────────────────────────────────────
    // Register the FMI parameters (fixed before exit_initialisation_mode).
    void registerParameters()
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

        register_real("solver.abs_tol", &p_newton_abs_tol_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Radau IIA Newton absolute residual tolerance [-]");

        register_real("solver.rel_tol", &p_newton_rel_tol_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Radau IIA Newton relative residual tolerance [-]");

        register_real("solver.max_step_s", &p_max_step_s_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Maximum internal integrator sub-step size [s] (0 = use communication step)");

        // ── Environment ──────────────────────────────────────────────────────
        register_real("wind.north_mps", &p_wind_north_mps_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Steady wind, NED north component at the initial position [m/s]. Positive = blowing northward. Fixed in NED at the start point and converted to ECEF once.");

        register_real("wind.east_mps", &p_wind_east_mps_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Steady wind, NED east component [m/s]. Positive = blowing eastward.");

        register_real("wind.down_mps", &p_wind_down_mps_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Steady wind, NED down component [m/s]. Positive = downward.");

        register_real("turb.sigma_u_mps", &p_turb_sigma_u_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Dryden turbulence RMS gust along body x [m/s]. All three sigmas zero = turbulence off. MIL-F-8785C spectra; see Aetherion/Environment/DrydenTurbulence.h.");

        register_real("turb.sigma_v_mps", &p_turb_sigma_v_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Dryden turbulence RMS gust along body y [m/s].");

        register_real("turb.sigma_w_mps", &p_turb_sigma_w_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Dryden turbulence RMS gust along body z (down) [m/s].");

        register_real("turb.L_u_m", &p_turb_L_u_m_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Dryden longitudinal scale length L_u [m]; MIL-F-8785C: 1750 ft above 2000 ft.");

        register_real("turb.L_v_m", &p_turb_L_v_m_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Dryden lateral scale length L_v [m] (MIL-F-8785C definition).");

        register_real("turb.L_w_m", &p_turb_L_w_m_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Dryden vertical scale length L_w [m] (MIL-F-8785C definition).");

        register_real("turb.seed", &p_turb_seed_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Seed of the turbulence noise stream (integer-valued). The same seed and parameters reproduce the same gust record.");

        register_real("atm.deltaT_K", &p_atm_deltaT_K_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("ISA temperature deviation [K], added uniformly to the US1976 profile. Shifts density, speed of sound and pressure consistently (hydrostatic re-integration) for the forces, the trim and out.P_Pa/out.T_K/out.rho_kg_m3/out.a_m_s. The propulsion table stays indexed on geometric altitude.");

        register_real("atm.deltaP_sl_Pa", &p_atm_deltaP_Pa_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Sea-level pressure minus 101 325 Pa (QNH offset) [Pa]. A barometer inverting the standard ISA reads about 8.4 m per 100 Pa of offset at low altitude.");
    }

    // ── registerInputs ──────────────────────────────────────────────────────────
    // Register the control-surface inputs.
    void registerInputs()
    {
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
    }

    // ── registerOutputs ─────────────────────────────────────────────────────────
    // Register the output cache variables.
    void registerOutputs()
    {
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

        register_real("out.specificForce_x_m_s2", &state_.specificForce_x_m_s2)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Body-X specific force at the CG: the non-gravitational acceleration an "
                            "ideal accelerometer would sense, (F_aero + F_thrust)/m. "
                            "Excludes gravitation — zero in free fall. Same body axes as out.aero_F*_N "
                            "(x forward). [m/s²]");

        register_real("out.specificForce_y_m_s2", &state_.specificForce_y_m_s2)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Body-Y specific force at the CG: the non-gravitational acceleration an "
                            "ideal accelerometer would sense, (F_aero + F_thrust)/m. "
                            "Excludes gravitation — zero in free fall. Same body axes as out.aero_F*_N "
                            "(y right). [m/s²]");

        register_real("out.specificForce_z_m_s2", &state_.specificForce_z_m_s2)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Body-Z specific force at the CG: the non-gravitational acceleration an "
                            "ideal accelerometer would sense, (F_aero + F_thrust)/m. "
                            "Excludes gravitation — zero in free fall. Same body axes as out.aero_F*_N "
                            "(z down). [m/s²]");

        register_real("out.thrust_N",    &state_.thrust_N)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Net propulsive force along body +X [N] "
                            "(the F-16 engine deck carries no body-Y/Z thrust component)");

        // ── Environment ──────────────────────────────────────────────────────
        register_real("out.wind_north_m_s", &state_.wind_north_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Total wind at the CG, NED north: steady wind plus gust [m/s]");

        register_real("out.wind_east_m_s", &state_.wind_east_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Total wind at the CG, NED east: steady wind plus gust [m/s]");

        register_real("out.wind_down_m_s", &state_.wind_down_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Total wind at the CG, NED down: steady wind plus gust [m/s]");

        register_real("out.gust_u_m_s", &state_.gust_u_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Dryden gust velocity along body x [m/s]");

        register_real("out.gust_v_m_s", &state_.gust_v_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Dryden gust velocity along body y [m/s]");

        register_real("out.gust_w_m_s", &state_.gust_w_m_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Dryden gust velocity along body z (down) [m/s]");

        register_real("out.gust_p_rad_s", &state_.gust_p_rad_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Angular velocity of the gust field about body x [rad/s]; the aero sees p minus this");

        register_real("out.gust_q_rad_s", &state_.gust_q_rad_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Angular velocity of the gust field about body y [rad/s]");

        register_real("out.gust_r_rad_s", &state_.gust_r_rad_s)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Angular velocity of the gust field about body z [rad/s]");

        register_real("out.mass_kg",     &state_.mass_kg)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED).setDescription("Vehicle mass [kg]");
    }

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
        state_.turb_x  = m_turb.has_value() ? m_turb->states() : std::array<double, 8>{};
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
        // Turbulence filter states come back with the checkpoint; the noise
        // stream does not, so a restored run diverges from the original
        // realisation after the first turbulent step.
        if (m_turb.has_value())
            m_turb->setStates(state_.turb_x);

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
        state_.lat_deg     = snap.latitude_rad  * kRadToDeg;
        state_.lon_deg     = snap.longitude_rad * kRadToDeg;

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

        // Body-frame specific force at the CG — what an ideal accelerometer senses.
        // Deliberately built as a sum of the non-gravitational wrench forces, so no
        // gravity term can leak in: snap.localGravity_m_s2 (mass attraction only) is
        // NOT involved.  The thrust policy is evaluated with the same throttle the
        // integrator just used; the aero force is taken from the snapshot rather than
        // re-evaluating the DAVE-ML tables.
        //
        // F/m is the specific force at the CG exactly — Newton's second law about the
        // centre of mass — independent of where the body frame's origin sits.
        const Eigen::Vector3d F_thrust_B = AE_SIM::PolicyBodyForce_N(t, m_state, vf.thrust);
        const Eigen::Vector3d F_aero_B(snap.aero_bodyForce_N_X,
                                       snap.aero_bodyForce_N_Y,
                                       snap.aero_bodyForce_N_Z);
        const Eigen::Vector3d specificForce_B =
            AE_SIM::BodySpecificForce_m_s2(F_aero_B + F_thrust_B, m_state.m);

        state_.specificForce_x_m_s2 = specificForce_B.x();
        state_.specificForce_y_m_s2 = specificForce_B.y();
        state_.specificForce_z_m_s2 = specificForce_B.z();
        state_.thrust_N     = F_thrust_B.x();

        // Alpha and beta from the same air-relative velocity the aero policy
        // formed its forces from (Earth-surface velocity, steady wind and gust
        // all subtracted), so out.alpha_deg agrees with out.aero_F*_N.
        const Aetherion::ODE::RKMK::Lie::SE3<double> g_now(m_state.g.R, m_state.g.p);
        const Eigen::Vector3d v_rel = vf.aero.airRelativeVelocity_B<double>(g_now, m_state.nu_B, t);
        const double vt = std::sqrt(v_rel.squaredNorm() + 1.0e-30);
        state_.alpha_deg = std::atan2(v_rel.z(), v_rel.x() + 1.0e-12)
                           * (180.0 / std::numbers::pi);
        state_.beta_deg  = std::asin(std::clamp(v_rel.y() / vt, -1.0, 1.0))
                           * (180.0 / std::numbers::pi);

        // Environment outputs: the total wind at the CG in NED (steady wind
        // plus the gust rotated body → NED) and the gust itself in body axes.
        {
            namespace Coord = Aetherion::Coordinate;
            const Eigen::Vector3d& w_ecef = vf.aero.windECEF();
            const Coord::Vec3<double> w_ecef_arr{ w_ecef.x(), w_ecef.y(), w_ecef.z() };
            const Coord::Vec3<double> w_ned =
                Coord::ECEFToNED(w_ecef_arr, snap.latitude_rad, snap.longitude_rad);

            const AE_ENV::GustState& gust = vf.aero.gust();
            const Coord::Mat3<double> R_IN_arr =
                Coord::NEDToInertialRotationMatrix(snap.latitude_rad, snap.longitude_rad, theta_GST);
            Eigen::Matrix3d R_IN;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    R_IN(r, c) = R_IN_arr[3 * r + c];
            const Eigen::Vector3d gust_ned = R_IN.transpose() * m_state.g.R * gust.linear();

            state_.wind_north_m_s = w_ned[0] + gust_ned.x();
            state_.wind_east_m_s  = w_ned[1] + gust_ned.y();
            state_.wind_down_m_s  = w_ned[2] + gust_ned.z();
            state_.gust_u_m_s     = gust.u_mps;
            state_.gust_v_m_s     = gust.v_mps;
            state_.gust_w_m_s     = gust.w_mps;
            state_.gust_p_rad_s   = gust.p_rad_s;
            state_.gust_q_rad_s   = gust.q_rad_s;
            state_.gust_r_rad_s   = gust.r_rad_s;
        }
    }

    // ── Environment helpers ───────────────────────────────────────────────────

    /// Steady wind as an ECEF vector: the NED parameters at the initial position.
    [[nodiscard]] Eigen::Vector3d windECEF() const
    {
        const auto w = AE_ENV::ConstantECEFWind::from_ned(
            p_wind_north_mps_, p_wind_east_mps_, p_wind_down_mps_,
            p_lat0_deg_ * kDeg, p_lon0_deg_ * kDeg);
        return { w.vx, w.vy, w.vz };
    }

    [[nodiscard]] AE_ENV::AtmosphereOffsets atmosphereOffsets() const noexcept
    {
        return { p_atm_deltaT_K_, p_atm_deltaP_Pa_ };
    }

    [[nodiscard]] AE_ENV::DrydenParameters turbulenceParameters() const
    {
        AE_ENV::DrydenParameters tp{};
        tp.sigma_u_mps = p_turb_sigma_u_;
        tp.sigma_v_mps = p_turb_sigma_v_;
        tp.sigma_w_mps = p_turb_sigma_w_;
        tp.L_u_m       = p_turb_L_u_m_;
        tp.L_v_m       = p_turb_L_v_m_;
        tp.L_w_m       = p_turb_L_w_m_;
        tp.wingspan_m  = m_aeroModel ? m_aeroModel->bspanFt() * kFt_m : 9.144;
        return tp;
    }

    // ── Parameters ────────────────────────────────────────────────────────────
    double p_vt0_fps_        { kDefault_vt0_fps        };
    double p_alt0_ft_        { kDefault_alt0_ft        };
    double p_lat0_deg_       { kDefault_lat0_deg       };
    double p_lon0_deg_       { kDefault_lon0_deg       };
    double p_heading0_deg_   { kDefault_heading0_deg   };
    double p_roll0_deg_      { kDefault_roll0_deg      };
    double p_xcg_from_ac_ft_ { kDefault_xcg_from_ac_ft };

    double p_newton_abs_tol_ { kDefault_newton_abs_tol };
    double p_newton_rel_tol_ { kDefault_newton_rel_tol };
    double p_max_step_s_     { kDefault_max_step_s     };

    double p_wind_north_mps_ { kDefault_wind_mps       };
    double p_wind_east_mps_  { kDefault_wind_mps       };
    double p_wind_down_mps_  { kDefault_wind_mps       };
    double p_turb_sigma_u_   { kDefault_turb_sigma_mps };
    double p_turb_sigma_v_   { kDefault_turb_sigma_mps };
    double p_turb_sigma_w_   { kDefault_turb_sigma_mps };
    double p_turb_L_u_m_     { kDefault_turb_L_m       };
    double p_turb_L_v_m_     { kDefault_turb_L_m       };
    double p_turb_L_w_m_     { kDefault_turb_L_m       };
    double p_turb_seed_      { kDefault_turb_seed      };
    double p_atm_deltaT_K_   { kDefault_atm_deltaT_K   };
    double p_atm_deltaP_Pa_  { kDefault_atm_deltaP_Pa  };

    // Dryden filter bank; engaged only when a gust intensity is non-zero.
    std::optional<AE_ENV::DrydenTurbulence> m_turb;
    double m_turbStep_s     { 0.0 };
    bool   m_turbStepWarned { false };

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
    // Aetherion release version, injected by CMake from version.txt. Published as the
    // FMI `version` attribute so a consumer can enforce a version floor by reading the
    // shipped modelDescription.xml rather than trusting the build tree it was found in.
    info.version              = AETHERION_VERSION;
    info.description          = "Aetherion F-16 6-DoF plant "
                                "(Radau IIA RKMK on SE(3), DAVE-ML aero/prop, J2 gravity)";
    info.canGetAndSetFMUstate = true;
    info.canSerializeFMUstate = true;
    return info;
}

FMU4CPP_INSTANTIATE(F16PlantFMU);

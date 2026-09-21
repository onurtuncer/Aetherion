// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16AutopilotFMU.cpp
//
// FMI 2.0 Co-Simulation FMU wrapping the NASA LaRC F-16 LQR autopilot defined
// in F16_control.dml, or — with the `circumnavigate` parameter set — the GNC
// variant F16_gnc.dml (Bruce Jackson, NASA TM-2015-218675).
//
// The autopilot is purely algebraic (static gain/lookup): do_step evaluates
// the DAVE-ML computation graph and returns immediately — no ODE integration.
//
// Control law selection
// ─────────────────────────────────────────────────────────────────────────────
// F16_gnc.dml is F16_control.dml with a navigator in front of the heading
// loop: baseChiCmd and latOffset stop being inputs and are computed from the
// ownship position so as to fly a 3 nmi counter-clockwise circle.  The DML has
// no "navigator off" state — circlePoleSW only picks which circle — so the
// choice of control law is a parameter, fixed at initialisation:
//
//   circumnavigate = false   F16_control.dml   Scenarios 13.1–13.4
//                            cmd.baseChiCmd_deg / cmd.latOffset_ft steer;
//                            cmd.circlePoleSW, fb.lat_deg, fb.lon_deg ignored.
//   circumnavigate = true    F16_gnc.dml       Scenarios 15 and 16
//                            cmd.circlePoleSW > 0.5  circle the North Pole (15)
//                            cmd.circlePoleSW ≤ 0.5  circle the equator /
//                                                    date-line crossing (16)
//                            cmd.baseChiCmd_deg / cmd.latOffset_ft ignored.
//
// Model identity : F16Autopilot
// FMI standard   : FMI 2.0 (Co-Simulation)
//
// Variable map
// ─────────────────────────────────────────────────────────────────────────────
//  PARAMETERS  (fixed before fmi2ExitInitializationMode)
//    circumnavigate      Load F16_gnc.dml instead of F16_control.dml    def false
//    (trim stick and throttle are embedded in the DML)
//
//  INPUTS  (sensor feedbacks from plant + guidance commands)
//   — Commands
//    cmd.altCmd_ft       Altitude command                       [ft]    def 10013.0
//    cmd.keasCmd_kt      Equivalent airspeed command            [kt]    def 287.809
//    cmd.baseChiCmd_deg  Desired heading (CW from North)        [deg]   def 45.0
//    cmd.latOffset_ft    Lateral offset from course (+right)    [ft]    def 0.0
//    cmd.circlePoleSW    Navigator select, F16_gnc.dml only     [-]     def 0.0
//   — Sensor feedbacks (SI from F16PlantFMU; converted internally)
//    fb.alt_m            Altitude above MSL                     [m]
//    fb.vt_m_s           True airspeed                          [m/s]
//    fb.rho_kg_m3        Air density                            [kg/m³]
//    fb.alpha_deg        Angle of attack                        [deg]
//    fb.beta_deg         Sideslip angle                         [deg]
//    fb.roll_rad         ZYX Euler roll  (body → NED)           [rad]
//    fb.pitch_rad        ZYX Euler pitch (body → NED)           [rad]
//    fb.yaw_rad          ZYX Euler yaw   (body → NED)           [rad]
//    fb.p_rad_s          Body roll  rate                        [rad/s]
//    fb.q_rad_s          Body pitch rate                        [rad/s]
//    fb.r_rad_s          Body yaw   rate                        [rad/s]
//    fb.lat_deg          Geodetic latitude,  F16_gnc.dml only   [deg]
//    fb.lon_deg          Geodetic longitude, F16_gnc.dml only   [deg]
//
//  Every input starts at the Scenario-11 trim point, so the initial outputs are
//  at trim when the master sets nothing.  Values the master writes during
//  initialisation mode are honoured.
//
//  OUTPUTS
//    ctrl.el_deg         Elevator deflection                    [deg]
//    ctrl.ail_deg        Aileron  deflection                    [deg]
//    ctrl.rdr_deg        Rudder   deflection                    [deg]
//    ctrl.pwr_pct        Throttle (power lever angle)           [%]
//
// Unit conversions performed in do_step:
//   altMsl_ft  = fb.alt_m  / 0.3048
//   Vequiv_kt  = fb.vt_m_s × √(fb.rho_kg_m3 / 1.225) / 0.514444
//   phi_deg    = fb.roll_rad  × 180/π
//   theta_deg  = fb.pitch_rad × 180/π
//   psi_deg    = fb.yaw_rad   × 180/π
// ─────────────────────────────────────────────────────────────────────────────

#include <fmu4cpp/fmu_base.hpp>

#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

#include <Aetherion/Serialization/DAVEML/DAVEMLControlModel.h>

using namespace fmu4cpp;
namespace AE_SR = Aetherion::Serialization;

namespace {
    constexpr double kFt_m         = 0.3048;
    constexpr double kKt_ms        = 0.514444;   // 1 knot in m/s
    constexpr double kRho_SL_kg_m3 = 1.225;      // ISA sea-level air density
    constexpr double kRad2Deg      = 180.0 / std::numbers::pi;
    constexpr double kDeg2Rad      = std::numbers::pi / 180.0;

    // Design-point trim commands matching F16_control.dml initialValues
    constexpr double kTrimAlt_ft  = 10013.0;
    constexpr double kTrimKEAS_kt = 287.809;  // 2.878e2 kt from DML
    constexpr double kTrimHdg_deg =  45.0;

    // Sensor feedbacks at the same trim point — F16PlantFMU defaults
    constexpr double kTrimVt_m_s    = 565.685 * kFt_m;    // trim TAS
    constexpr double kTrimRho_kg_m3 = 0.9042;             // ~10 000 ft ISA density
    constexpr double kTrimAlpha_deg = 2.6538;             // trim AoA from F16_control.dml
    constexpr double kTrimRoll_rad  = 0.0;                // wings-level
    constexpr double kTrimLat_deg   =  36.01917;          // Kitty Hawk, NC
    constexpr double kTrimLon_deg   = -75.67444;
}

// ─────────────────────────────────────────────────────────────────────────────
class F16AutopilotFMU : public fmu_base {
public:
    FMU4CPP_CTOR(F16AutopilotFMU)
    {
        // ── Parameters ────────────────────────────────────────────────────────
        register_boolean("circumnavigate", &p_circumnavigate_)
            .setCausality(causality_t::PARAMETER).setVariability(variability_t::FIXED)
            .setInitial(initial_t::EXACT)
            .setDescription("Load F16_gnc.dml (navigator flies a 3 nmi circle) instead of "
                            "F16_control.dml (course hold)");

        // ── Command inputs ────────────────────────────────────────────────────
        register_real("cmd.altCmd_ft",      &cmd_altCmd_ft_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Altitude command [ft]");

        register_real("cmd.keasCmd_kt",     &cmd_keasCmd_kt_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Equivalent airspeed command [kt]");

        register_real("cmd.baseChiCmd_deg", &cmd_baseChiCmd_deg_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Desired ground track heading, CW from North [deg]");

        register_real("cmd.latOffset_ft",   &cmd_latOffset_ft_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Lateral deviation from desired course, +right [ft]");

        register_real("cmd.circlePoleSW",   &cmd_circlePoleSW_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Navigator select, F16_gnc.dml only: >0.5 circle the North Pole, "
                            "otherwise the equator/date-line crossing [-]");

        // ── Feedback inputs (SI, from F16PlantFMU) ───────────────────────────
        register_real("fb.alt_m",     &fb_alt_m_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Altitude above MSL from plant [m]");

        register_real("fb.vt_m_s",    &fb_vt_m_s_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("True airspeed from plant [m/s]");

        register_real("fb.rho_kg_m3", &fb_rho_kg_m3_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Air density from plant [kg/m³]");

        register_real("fb.alpha_deg", &fb_alpha_deg_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Angle of attack from plant [deg]");

        register_real("fb.beta_deg",  &fb_beta_deg_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Sideslip angle from plant [deg]");

        register_real("fb.roll_rad",  &fb_roll_rad_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("ZYX Euler roll angle from plant [rad]");

        register_real("fb.pitch_rad", &fb_pitch_rad_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("ZYX Euler pitch angle from plant [rad]");

        register_real("fb.yaw_rad",   &fb_yaw_rad_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("ZYX Euler yaw angle from plant [rad]");

        register_real("fb.p_rad_s",   &fb_p_rad_s_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Body roll  rate from plant [rad/s]");

        register_real("fb.q_rad_s",   &fb_q_rad_s_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Body pitch rate from plant [rad/s]");

        register_real("fb.r_rad_s",   &fb_r_rad_s_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Body yaw   rate from plant [rad/s]");

        register_real("fb.lat_deg",   &fb_lat_deg_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Geodetic latitude from plant, F16_gnc.dml only [deg]");

        register_real("fb.lon_deg",   &fb_lon_deg_)
            .setCausality(causality_t::INPUT).setVariability(variability_t::CONTINUOUS)
            .setDescription("Geodetic longitude from plant, F16_gnc.dml only [deg]");

        // ── Control surface outputs ───────────────────────────────────────────
        register_real("ctrl.el_deg",  &ctrl_el_deg_)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Elevator deflection [deg]");

        register_real("ctrl.ail_deg", &ctrl_ail_deg_)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Aileron deflection [deg]");

        register_real("ctrl.rdr_deg", &ctrl_rdr_deg_)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Rudder deflection [deg]");

        register_real("ctrl.pwr_pct", &ctrl_pwr_pct_)
            .setCausality(causality_t::OUTPUT).setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED)
            .setDescription("Throttle (power lever angle) [%]");
    }

    // ── exit_initialisation_mode ──────────────────────────────────────────────
    void exit_initialisation_mode() override
    {
        const std::string dmlPath = resourceLocation().string()
            + (p_circumnavigate_ ? "/F16_gnc.dml" : "/F16_control.dml");
        m_ctrl = std::make_unique<AE_SR::DAVEMLControlModel>(dmlPath);

        // Inputs are left as the master set them: their start values are the
        // trim point, so untouched inputs still give initial outputs at trim.
        evaluate();
    }

    // ── do_step ───────────────────────────────────────────────────────────────
    // Purely algebraic: evaluate the DAVE-ML control law at each step.
    bool do_step(double /*dt*/) override
    {
        if (!m_ctrl) return false;
        evaluate();
        return true;
    }

    // ── reset ─────────────────────────────────────────────────────────────────
    void reset() override
    {
        m_ctrl.reset();

        p_circumnavigate_   = false;

        cmd_altCmd_ft_      = kTrimAlt_ft;
        cmd_keasCmd_kt_     = kTrimKEAS_kt;
        cmd_baseChiCmd_deg_ = kTrimHdg_deg;
        cmd_latOffset_ft_   = 0.0;
        cmd_circlePoleSW_   = 0.0;

        fb_alt_m_     = kTrimAlt_ft * kFt_m;
        fb_vt_m_s_    = kTrimVt_m_s;
        fb_rho_kg_m3_ = kTrimRho_kg_m3;
        fb_alpha_deg_ = kTrimAlpha_deg;
        fb_beta_deg_  = 0.0;
        fb_roll_rad_  = kTrimRoll_rad;
        fb_pitch_rad_ = kTrimAlpha_deg * kDeg2Rad;
        fb_yaw_rad_   = kTrimHdg_deg   * kDeg2Rad;
        fb_p_rad_s_ = fb_q_rad_s_ = fb_r_rad_s_ = 0.0;
        fb_lat_deg_   = kTrimLat_deg;
        fb_lon_deg_   = kTrimLon_deg;

        ctrl_el_deg_ = ctrl_ail_deg_ = ctrl_rdr_deg_ = ctrl_pwr_pct_ = 0.0;
    }

private:
    // ── evaluate ──────────────────────────────────────────────────────────────
    // Convert SI feedbacks → NASA DML units, call evaluate(), store outputs.
    void evaluate()
    {
        AE_SR::DAVEMLControlModel::Inputs in;

        // Autopilot engaged; pilot inputs zeroed (AP mode)
        in.sasOn  = 1.0;
        in.apOn   = 1.0;
        in.throttle_frac = 0.0;
        in.longStk_frac  = 0.0;
        in.latStk_frac   = 0.0;
        in.pedal_frac    = 0.0;

        // Commands (already in DML native units)
        in.altCmd_ft      = cmd_altCmd_ft_;
        in.keasCmd_kt     = cmd_keasCmd_kt_;
        in.baseChiCmd_deg = cmd_baseChiCmd_deg_;
        in.latOffset_ft   = cmd_latOffset_ft_;

        // Sensor feedbacks — unit conversion from SI
        in.altMsl_ft  = fb_alt_m_  / kFt_m;
        in.Vequiv_kt  = fb_vt_m_s_
                        * std::sqrt(fb_rho_kg_m3_ / kRho_SL_kg_m3)
                        / kKt_ms;
        in.alpha_deg  = fb_alpha_deg_;
        in.beta_deg   = fb_beta_deg_;
        in.phi_deg    = fb_roll_rad_  * kRad2Deg;
        in.theta_deg  = fb_pitch_rad_ * kRad2Deg;
        in.psi_deg    = fb_yaw_rad_   * kRad2Deg;
        in.pb_rad_s   = fb_p_rad_s_;
        in.qb_rad_s   = fb_q_rad_s_;
        in.rb_rad_s   = fb_r_rad_s_;

        // Navigator inputs — read by F16_gnc.dml only; F16_control.dml defines
        // none of these varIDs, so they are inert when circumnavigate is false.
        in.ownshipN_deg = fb_lat_deg_;
        in.ownshipE_deg = fb_lon_deg_;
        in.circlePoleSW = cmd_circlePoleSW_;

        const auto out = m_ctrl->evaluate(in);
        ctrl_el_deg_  = out.el_deg;
        ctrl_ail_deg_ = out.ail_deg;
        ctrl_rdr_deg_ = out.rdr_deg;
        ctrl_pwr_pct_ = out.pwr_pct;
    }

    // ── DAVE-ML evaluator ─────────────────────────────────────────────────────
    std::unique_ptr<AE_SR::DAVEMLControlModel> m_ctrl;

    // ── FMI parameter storage ─────────────────────────────────────────────────
    bool p_circumnavigate_ { false };

    // ── FMI input storage (start values = Scenario-11 trim point) ─────────────
    double cmd_altCmd_ft_      { kTrimAlt_ft  };
    double cmd_keasCmd_kt_     { kTrimKEAS_kt };
    double cmd_baseChiCmd_deg_ { kTrimHdg_deg };
    double cmd_latOffset_ft_   { 0.0 };
    double cmd_circlePoleSW_   { 0.0 };

    double fb_alt_m_     { kTrimAlt_ft * kFt_m };
    double fb_vt_m_s_    { kTrimVt_m_s };
    double fb_rho_kg_m3_ { kTrimRho_kg_m3 };
    double fb_alpha_deg_ { kTrimAlpha_deg };
    double fb_beta_deg_  { 0.0 };
    double fb_roll_rad_  { kTrimRoll_rad };
    double fb_pitch_rad_ { kTrimAlpha_deg * kDeg2Rad };
    double fb_yaw_rad_   { kTrimHdg_deg   * kDeg2Rad };
    double fb_p_rad_s_   { 0.0 };
    double fb_q_rad_s_   { 0.0 };
    double fb_r_rad_s_   { 0.0 };
    double fb_lat_deg_   { kTrimLat_deg };
    double fb_lon_deg_   { kTrimLon_deg };

    // ── FMI output storage ────────────────────────────────────────────────────
    double ctrl_el_deg_  { 0.0 };
    double ctrl_ail_deg_ { 0.0 };
    double ctrl_rdr_deg_ { 0.0 };
    double ctrl_pwr_pct_ { 0.0 };
};

// ── Model metadata ────────────────────────────────────────────────────────────
model_info fmu4cpp::get_model_info()
{
    model_info info;
    info.modelName  = "F16Autopilot";
    // Aetherion release version, injected by CMake from version.txt. Published as the
    // FMI `version` attribute so a consumer can enforce a version floor by reading the
    // shipped modelDescription.xml rather than trusting the build tree it was found in.
    info.version              = AETHERION_VERSION;
    info.description =
        "Aetherion F-16 LQR autopilot (NASA LaRC F16_control.dml / F16_gnc.dml) — "
        "altitude hold, airspeed hold, heading hold or circumnavigation, with LQR SAS inner loop";
    info.canGetAndSetFMUstate = false;
    info.canSerializeFMUstate = false;
    return info;
}

FMU4CPP_INSTANTIATE(F16AutopilotFMU);

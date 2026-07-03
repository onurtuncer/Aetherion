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
// in F16_control.dml (Bruce Jackson, NASA TM-2015-218675).
//
// The autopilot is purely algebraic (static gain/lookup): do_step evaluates
// the DAVE-ML computation graph and returns immediately — no ODE integration.
//
// Model identity : F16Autopilot
// FMI standard   : FMI 2.0 (Co-Simulation)
//
// Variable map
// ─────────────────────────────────────────────────────────────────────────────
//  PARAMETERS  (fixed before fmi2ExitInitializationMode)
//    (none beyond defaults; trim values embedded in F16_control.dml)
//
//  INPUTS  (sensor feedbacks from plant + guidance commands)
//   — Commands
//    cmd.altCmd_ft       Altitude command                       [ft]    def 10013.0
//    cmd.keasCmd_kt      Equivalent airspeed command            [kt]    def 287.809
//    cmd.baseChiCmd_deg  Desired heading (CW from North)        [deg]   def 45.0
//    cmd.latOffset_ft    Lateral offset from course (+right)    [ft]    def 0.0
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

    // Design-point trim commands matching F16_control.dml initialValues
    constexpr double kTrimAlt_ft  = 10013.0;
    constexpr double kTrimKEAS_kt = 287.809;  // 2.878e2 kt from DML
    constexpr double kTrimHdg_deg =  45.0;
}

// ─────────────────────────────────────────────────────────────────────────────
class F16AutopilotFMU : public fmu_base {
public:
    FMU4CPP_CTOR(F16AutopilotFMU)
    {
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
        const std::string dmlPath =
            resourceLocation().string() + "/F16_control.dml";
        m_ctrl = std::make_unique<AE_SR::DAVEMLControlModel>(dmlPath);

        // Seed command defaults so initial outputs are at trim
        cmd_altCmd_ft_      = kTrimAlt_ft;
        cmd_keasCmd_kt_     = kTrimKEAS_kt;
        cmd_baseChiCmd_deg_ = kTrimHdg_deg;
        cmd_latOffset_ft_   = 0.0;

        // Initial sensor values matching plant trim defaults
        fb_alt_m_     = kTrimAlt_ft  * kFt_m;
        fb_vt_m_s_    = 565.685      * kFt_m;  // trim TAS from F16PlantFMU default
        fb_rho_kg_m3_ = 0.9042;                 // ~10 000 ft ISA density
        fb_alpha_deg_ = 2.6538;                 // trim AoA from F16_control.dml
        fb_beta_deg_  = 0.0;
        fb_roll_rad_  = -0.172 * (std::numbers::pi / 180.0);
        fb_pitch_rad_ = 2.6538 * (std::numbers::pi / 180.0);
        fb_yaw_rad_   = kTrimHdg_deg * (std::numbers::pi / 180.0);
        fb_p_rad_s_ = fb_q_rad_s_ = fb_r_rad_s_ = 0.0;

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

        cmd_altCmd_ft_      = kTrimAlt_ft;
        cmd_keasCmd_kt_     = kTrimKEAS_kt;
        cmd_baseChiCmd_deg_ = kTrimHdg_deg;
        cmd_latOffset_ft_   = 0.0;

        fb_alt_m_ = fb_vt_m_s_ = fb_rho_kg_m3_ = 0.0;
        fb_alpha_deg_ = fb_beta_deg_ = 0.0;
        fb_roll_rad_ = fb_pitch_rad_ = fb_yaw_rad_ = 0.0;
        fb_p_rad_s_ = fb_q_rad_s_ = fb_r_rad_s_ = 0.0;

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

        // Circumnavigator inputs unused (F16_control.dml, not F16_gnc.dml)
        in.ownshipN_deg = 0.0;
        in.ownshipE_deg = 0.0;
        in.circlePoleSW = 0.0;

        const auto out = m_ctrl->evaluate(in);
        ctrl_el_deg_  = out.el_deg;
        ctrl_ail_deg_ = out.ail_deg;
        ctrl_rdr_deg_ = out.rdr_deg;
        ctrl_pwr_pct_ = out.pwr_pct;
    }

    // ── DAVE-ML evaluator ─────────────────────────────────────────────────────
    std::unique_ptr<AE_SR::DAVEMLControlModel> m_ctrl;

    // ── FMI input storage ─────────────────────────────────────────────────────
    double cmd_altCmd_ft_      { kTrimAlt_ft  };
    double cmd_keasCmd_kt_     { kTrimKEAS_kt };
    double cmd_baseChiCmd_deg_ { kTrimHdg_deg };
    double cmd_latOffset_ft_   { 0.0 };

    double fb_alt_m_     { 0.0 };
    double fb_vt_m_s_    { 0.0 };
    double fb_rho_kg_m3_ { kRho_SL_kg_m3 };
    double fb_alpha_deg_ { 0.0 };
    double fb_beta_deg_  { 0.0 };
    double fb_roll_rad_  { 0.0 };
    double fb_pitch_rad_ { 0.0 };
    double fb_yaw_rad_   { 0.0 };
    double fb_p_rad_s_   { 0.0 };
    double fb_q_rad_s_   { 0.0 };
    double fb_r_rad_s_   { 0.0 };

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
    info.description =
        "Aetherion F-16 LQR autopilot (NASA LaRC F16_control.dml) — "
        "altitude hold, airspeed hold, heading hold with LQR SAS inner loop";
    info.canGetAndSetFMUstate = false;
    info.canSerializeFMUstate = false;
    return info;
}

FMU4CPP_INSTANTIATE(F16AutopilotFMU);

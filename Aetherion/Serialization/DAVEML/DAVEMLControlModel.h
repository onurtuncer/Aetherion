// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// DAVEMLControlModel.h
//
// DAVE-ML flight control / GNC model evaluator — double-only wrapper around
// DAVEMLAeroModel::evaluateRaw().
//
// Designed for NASA TM-2015-218675 F16_control.dml and F16_gnc.dml which
// define an LQR stability-augmentation system and an altitude / airspeed /
// heading autopilot.  The DML uses the same evaluation graph as the aero
// model; the only differences are the input and output variable names and the
// absence of gridded tables (all expressions are pure MathML arithmetic with
// piecewise / abs / trig operators).
//
// Usage:
//   auto ctrl = std::make_shared<DAVEMLControlModel>("path/to/F16_control.dml");
//   DAVEMLControlModel::Inputs in;
//   in.sasOn = 1.0;  in.apOn = 1.0;
//   in.altMsl_ft = 10013.0;  in.altCmd_ft = 10113.0;
//   // ... fill all sensor feedbacks ...
//   auto out = ctrl->evaluate(in);
//   // out.el_deg, out.ail_deg, out.rdr_deg, out.pwr_pct
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace Aetherion::Serialization {

/// @brief DAVE-ML control / GNC model evaluator (double scalars only).
///
/// Parses any DAVE-ML file that conforms to the F16_control / F16_gnc schema
/// and evaluates it at a given flight state, returning control-surface
/// deflection commands.  Internally delegates to DAVEMLAeroModel::evaluateRaw()
/// so all MathML operators, piecewise logic, and minValue/maxValue clamping
/// added to that evaluator are automatically available here.
class DAVEMLControlModel
{
public:
    // ── Input variables (standard AIAA DAVE-ML variable IDs) ─────────────────

    struct Inputs {
        // Pilot stick / pedal / throttle (all zero when autopilot is active)
        double throttle_frac  { 0.0 };  ///< varID="throttle"  [0, 1]
        double longStk_frac   { 0.0 };  ///< varID="longStk"   [-1, +1], +AFT
        double latStk_frac    { 0.0 };  ///< varID="latStk"    [-1, +1], +RWD
        double pedal_frac     { 0.0 };  ///< varID="pedal"     [-1, +1], +ANR

        // Mode flags
        double sasOn           { 1.0 }; ///< varID="sasOn"  1=SAS engaged
        double apOn            { 1.0 }; ///< varID="apOn"   1=AP engaged

        // Autopilot commands
        double altCmd_ft       { 0.0 }; ///< varID="altCmd"      [ft]
        double keasCmd_kt      { 0.0 }; ///< varID="keasCmd"     [nmi/h = kt]
        double latOffset_ft    { 0.0 }; ///< varID="latOffset"   [ft, +RT]
        double baseChiCmd_deg  { 0.0 }; ///< varID="baseChiCmd"  [deg, +CW from N]

        // Sensor feedbacks
        double altMsl_ft       { 0.0 }; ///< varID="altMsl"   [ft]
        double Vequiv_kt       { 0.0 }; ///< varID="Vequiv"   [nmi/h = kt]
        double alpha_deg       { 0.0 }; ///< varID="alpha"    [deg]
        double beta_deg        { 0.0 }; ///< varID="beta"     [deg]
        double phi_deg         { 0.0 }; ///< varID="phi"      [deg]
        double theta_deg       { 0.0 }; ///< varID="theta"    [deg]
        double psi_deg         { 0.0 }; ///< varID="psi"      [deg]
        double pb_rad_s        { 0.0 }; ///< varID="pb"       [rad/s]
        double qb_rad_s        { 0.0 }; ///< varID="qb"       [rad/s]
        double rb_rad_s        { 0.0 }; ///< varID="rb"       [rad/s]

        // Trimmed pilot control values (implementation-supplied; DML defaults
        // are for the NASA Simulink reference at 10013 ft / Mach 0.525)
        double throttleTrim    { 0.1390191130965607 }; ///< varID="throttleTrim"
        double longStkTrim     { 0.1296382327486013 }; ///< varID="longStkTrim"

        // F16_gnc.dml only — circumnavigator inputs (set to 0 / unused for 13.1)
        double ownshipN_deg    { 0.0 }; ///< varID="ownshipN_deg"
        double ownshipE_deg    { 0.0 }; ///< varID="ownshipE_deg"
        double circlePoleSW    { 0.0 }; ///< varID="circlePoleSW"
    };

    // ── Output variables ──────────────────────────────────────────────────────

    struct Outputs {
        double el_deg  { 0.0 };  ///< Elevator deflection [deg]  varID="el"
        double ail_deg { 0.0 };  ///< Aileron deflection  [deg]  varID="ail"
        double rdr_deg { 0.0 };  ///< Rudder deflection   [deg]  varID="rdr"
        double pwr_pct { 0.0 };  ///< Throttle            [%]    varID="PWR"
    };

    // ── Construction ──────────────────────────────────────────────────────────

    /// @brief Parse the DAVE-ML control / GNC file and build the evaluation graph.
    /// @throws std::runtime_error on XML parse failure or missing data.
    explicit DAVEMLControlModel(const std::string& path);

    // ── Evaluation ────────────────────────────────────────────────────────────

    /// @brief Evaluate the control law at a given flight state.
    /// @param in  Current flight state and autopilot commands.
    /// @return    Control surface deflections and throttle.
    [[nodiscard]] Outputs evaluate(const Inputs& in) const;

private:
    DAVEMLAeroModel m_impl;  ///< Shared evaluator (parses DML, runs topo eval)
};

} // namespace Aetherion::Serialization

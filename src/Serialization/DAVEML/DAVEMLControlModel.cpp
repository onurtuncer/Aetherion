// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <Aetherion/Serialization/DAVEML/DAVEMLControlModel.h>

namespace Aetherion::Serialization {

DAVEMLControlModel::DAVEMLControlModel(const std::string& path)
    : m_impl(path)
{}

DAVEMLControlModel::Outputs
DAVEMLControlModel::evaluate(const Inputs& in) const
{
    std::unordered_map<std::string, double> vars;
    vars.reserve(64);

    // ── Pilot controls ────────────────────────────────────────────────────────
    vars["throttle"]  = in.throttle_frac;
    vars["longStk"]   = in.longStk_frac;
    vars["latStk"]    = in.latStk_frac;
    vars["pedal"]     = in.pedal_frac;

    // ── Mode flags ────────────────────────────────────────────────────────────
    vars["sasOn"]     = in.sasOn;
    vars["apOn"]      = in.apOn;

    // ── Autopilot commands ────────────────────────────────────────────────────
    vars["altCmd"]    = in.altCmd_ft;
    vars["keasCmd"]   = in.keasCmd_kt;
    vars["latOffset"] = in.latOffset_ft;
    vars["baseChiCmd"]= in.baseChiCmd_deg;

    // ── Sensor feedbacks ──────────────────────────────────────────────────────
    vars["altMsl"]    = in.altMsl_ft;
    vars["Vequiv"]    = in.Vequiv_kt;
    vars["alpha"]     = in.alpha_deg;
    vars["beta"]      = in.beta_deg;
    vars["phi"]       = in.phi_deg;
    vars["theta"]     = in.theta_deg;
    vars["psi"]       = in.psi_deg;
    vars["pb"]        = in.pb_rad_s;
    vars["qb"]        = in.qb_rad_s;
    vars["rb"]        = in.rb_rad_s;

    // ── Trim values (caller-supplied; override DML initialValue defaults) ─────
    vars["throttleTrim"] = in.throttleTrim;
    vars["longStkTrim"]  = in.longStkTrim;

    // ── F16_gnc.dml circumnavigator inputs (unused for Scenario 13.x) ─────────
    vars["ownshipN_deg"]  = in.ownshipN_deg;
    vars["ownshipE_deg"]  = in.ownshipE_deg;
    vars["circlePoleSW"]  = in.circlePoleSW;

    // ── Evaluate the full DML graph ───────────────────────────────────────────
    const auto result = m_impl.evaluateRaw(std::move(vars));

    auto get = [&](const char* id) -> double {
        auto it = result.find(id);
        return it != result.end() ? it->second : 0.0;
    };

    return Outputs{
        get("el"),
        get("ail"),
        get("rdr"),
        get("PWR"),
    };
}

} // namespace Aetherion::Serialization

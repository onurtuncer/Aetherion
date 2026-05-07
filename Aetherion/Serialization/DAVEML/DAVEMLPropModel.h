// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// DAVEMLPropModel.h
//
// DAVE-ML propulsion model evaluator for F16_prop.dml.
//
// Wraps DAVEMLAeroModel::evaluateRaw() with propulsion-specific
// input/output variable names and SI unit conversion.
//
// Inputs (file units)   → DAVE-ML varID
//   PWR  [0–100 %]      → "PWR"
//   ALT  [ft]           → "ALT"
//   RMACH [–]           → "RMACH"
//
// Outputs (file units)  → DAVE-ML varID
//   FEX [lbf]           → "FEX"   body-X thrust (+ fwd)
//   FEY [lbf]           → "FEY"   body-Y thrust (+ right)
//   FEZ [lbf]           → "FEZ"   body-Z thrust (+ down)
//   TEL [ft·lbf]        → "TEL"   rolling  thrust moment
//   TEM [ft·lbf]        → "TEM"   pitching thrust moment
//   TEN [ft·lbf]        → "TEN"   yawing   thrust moment
//
// SI output conversion:
//   1 lbf    = 4.448 221 615 N
//   1 ft·lbf = 1.355 817 948 N·m
//
// CppAD compatibility: evaluate<S>() works for S = double and
// S = CppAD::AD<double> via DAVEMLAeroModel::evaluateRaw<S>.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>
#include <string>

namespace Aetherion::Serialization {

/// @brief Propulsion model loaded from a DAVE-ML file.
class DAVEMLPropModel
{
public:
    // ── Conversion constants ──────────────────────────────────────────────────
    static constexpr double kLbf_N     = 4.448221615260751;
    static constexpr double kFtLbf_Nm  = 1.355817948329279;

    // ── Input / output structs ────────────────────────────────────────────────

    template<class S>
    struct Inputs {
        S pwr_pct {};   ///< Power lever angle [0–100 %]
        S alt_ft  {};   ///< Altitude [ft]
        S mach    {};   ///< Mach number [–]
    };

    /// @brief Thrust forces [N] and moments [N·m] in body axes.
    template<class S>
    struct Outputs {
        S fx_N  {};   ///< Body-X thrust force  [N, + fwd]
        S fy_N  {};   ///< Body-Y thrust force  [N, + right]
        S fz_N  {};   ///< Body-Z thrust force  [N, + down]
        S mx_Nm {};   ///< Rolling  thrust moment [N·m]
        S my_Nm {};   ///< Pitching thrust moment [N·m]
        S mz_Nm {};   ///< Yawing   thrust moment [N·m]
    };

    // ── Construction ──────────────────────────────────────────────────────────

    /// @brief Load and parse the DAVE-ML propulsion file.
    explicit DAVEMLPropModel(const std::string& path)
        : m_engine(path) {}

    // ── Evaluation ────────────────────────────────────────────────────────────

    /// @brief Evaluate thrust at the given flight condition.
    ///        Returns forces in N and moments in N·m.
    template<class S>
    Outputs<S> evaluate(const Inputs<S>& in) const
    {
        std::unordered_map<std::string, S> vars;
        vars["PWR"]   = in.pwr_pct;
        vars["ALT"]   = in.alt_ft;
        vars["RMACH"] = in.mach;

        auto result = m_engine.evaluateRaw<S>(std::move(vars));

        auto get = [&](const char* id) -> S {
            auto it = result.find(id);
            return it != result.end() ? it->second : S(0.0);
        };

        Outputs<S> out;
        out.fx_N  = get("FEX") * S(kLbf_N);
        out.fy_N  = get("FEY") * S(kLbf_N);
        out.fz_N  = get("FEZ") * S(kLbf_N);
        out.mx_Nm = get("TEL") * S(kFtLbf_Nm);
        out.my_Nm = get("TEM") * S(kFtLbf_Nm);
        out.mz_Nm = get("TEN") * S(kFtLbf_Nm);
        return out;
    }

    /// @brief Returns thrust force in body-X [lbf] — convenience method.
    double thrustX_lbf(double pwr_pct, double alt_ft, double mach) const
    {
        Inputs<double> in{ pwr_pct, alt_ft, mach };
        return evaluate(in).fx_N / kLbf_N;
    }

private:
    DAVEMLAeroModel m_engine;  ///< General DAVE-ML evaluation engine
};

} // namespace Aetherion::Serialization

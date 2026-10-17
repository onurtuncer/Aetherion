// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// DAVEMLAeroModel.h
//
// DAVE-ML aerodynamic model evaluator for the F-16 aero file (F16_aero.dml).
//
// Supported DAVE-ML features (beyond DAVEMLReader):
//   - <breakpointDef>  — named breakpoint vectors.
//   - <function> / <griddedTableDef>  — 1-D and 2-D (and N-D) linear
//     interpolation tables.
//   - Extended MathML  — <abs>, <piecewise>/<piece>/<otherwise>,
//     comparison ops <lt>, <gt>, <leq>, <geq>, <eq>.
//   - Topological evaluation order (Kahn's algorithm over the dep. graph).
//   - independentVarRef min / max / extrapolate, and variableDef minValue /
//     maxValue on supplied inputs as well as computed variables. A table is
//     checked against its breakpoints when the file is loaded.
//
// CppAD note
// ──────────
// DAVEMLAeroModel::evaluate<S>() works with S = double and S = CppAD::AD<double>.
// Breakpoint-segment selection and piecewise conditions are comparisons on S.
// For AD they are evaluated on the current values and recorded on the tape,
// so a tape is exact in the region (segments, branches) it was recorded in and
// ADFun::compare_change_number() > 0 after Forward(0, x) says that x lies in
// another region: the holder of the tape must then re-record at x (the Newton
// evaluators in ODE/RKMK/Core/Newton.h do). Note that ADFun's change
// counting is off by default; call compare_change_count(1) on the tape.
// Interpolation weights are computed in S, so first derivatives are exact
// inside a segment. Min/max limits on variables, and the independentVarRef
// limits a table input is held to, use CondExpLt/CondExpGt and need no
// re-recording.
// ------------------------------------------------------------------------------

#pragma once

#include <limits>
#include <string>
#include <vector>
#include <unordered_map>
#include <stdexcept>

namespace Aetherion::Serialization {

/// @brief Parsed and evaluatable DAVE-ML aerodynamic model.
///
/// Instantiate once from the .dml file, then call evaluate\<S\>() as many
/// times as needed.  The templated evaluate() is the only runtime entry
/// point; everything else is set up during construction.
class DAVEMLAeroModel
{
public:
    // ── Input / output structs ────────────────────────────────────────────────

    /// @brief Aerodynamic inputs in the units expected by the DAVE-ML file.
    template<class S>
    struct Inputs {
        S vt_fps  {};  ///< True airspeed [ft/s]
        S alpha_deg{};  ///< Angle of attack [deg]
        S beta_deg {};  ///< Sideslip angle [deg]
        S p_rps   {};   ///< Roll  rate [rad/s]
        S q_rps   {};   ///< Pitch rate [rad/s]
        S r_rps   {};   ///< Yaw   rate [rad/s]
        S el_deg  {};   ///< Elevator deflection [deg]
        S ail_deg {};   ///< Aileron  deflection [deg]
        S rdr_deg {};   ///< Rudder   deflection [deg]
    };

    /// @brief Non-dimensional aerodynamic coefficients.
    template<class S>
    struct Outputs {
        S cx{};  ///< Body-axis X-force coefficient (+ fwd)
        S cy{};  ///< Body-axis Y-force coefficient (+ right)
        S cz{};  ///< Body-axis Z-force coefficient (+ down)
        S cl{};  ///< Rolling-moment  coefficient  (+ RWD)
        S cm{};  ///< Pitching-moment coefficient  (+ ANU)
        S cn{};  ///< Yawing-moment   coefficient  (+ ANR)
    };

    // ── Construction ──────────────────────────────────────────────────────────

    /// @brief Parse the DAVE-ML aero file and build the evaluation graph.
    /// @throws std::runtime_error  on XML parse failure or missing data.
    explicit DAVEMLAeroModel(const std::string& path);

    // ── Reference geometry (read from file) ───────────────────────────────────
    [[nodiscard]] double srefFt2()  const noexcept { return m_srefFt2; }  ///< Reference wing area [ft²]
    [[nodiscard]] double cbarFt()   const noexcept { return m_cbarFt; }   ///< Mean aerodynamic chord [ft]
    [[nodiscard]] double bspanFt()  const noexcept { return m_bspanFt; }  ///< Wing span [ft]

    // ── Evaluation ────────────────────────────────────────────────────────────

    /// @brief Evaluate the aerodynamic model at a given operating point.
    ///
    /// Works with S = double or S = CppAD::AD\<double\>.
    /// @param in  Operating-point inputs (see @c Inputs\<S\>).
    /// @return    Six non-dimensional force and moment coefficients.
    template<class S>
    Outputs<S> evaluate(const Inputs<S>& in) const;

    /// @brief Generic evaluation: caller supplies any varID→value map;
    ///        returns the full computed variable map including all outputs.
    ///        Used by PropModel and other specialisations.
    template<class S>
    std::unordered_map<std::string, S>
    evaluateRaw(std::unordered_map<std::string, S> vars) const;

    // ── Internal data (public for the .cpp template helpers) ─────────────────

    /// @brief Sorted breakpoint vector for one dimension.
    using BpVec = std::vector<double>;

    /// @brief Gridded look-up table (N-D, 1-D or 2-D for the F-16 model).
    struct GridTable {
        std::vector<std::string> inputVarIDs;  ///< ordered independent var IDs
        std::vector<std::string> bpIDs;        ///< matching breakpoint-def IDs
        std::vector<double>      data;         ///< row-major flattened data
        std::vector<std::size_t> dims;         ///< [n0, n1, ...] sizes per dim
        /// Per dimension, the range the input is held to before the segment
        /// search: independentVarRef min / max, and the breakpoint range on
        /// each side where extrapolate does not allow going past it. Infinite
        /// when unbounded. Fixed at load, so the clamp needs no re-recording.
        std::vector<double>      lo;
        std::vector<double>      hi;
    };

    /// @brief One evaluation step in the topologically sorted graph.
    struct EvalStep {
        std::string varID;
        std::string mathmlXml;  ///< non-empty when type == Calc or OutputCalc
        bool        isTable{ false };
        bool        isInput{ false };
        bool        isConst{ false };
        double      constVal{ 0.0 };
        /// Clamp from variableDef minValue / maxValue, applied to the computed
        /// value or, for an input, to the supplied one.
        bool   hasMinMax{ false };
        double minVal{  -std::numeric_limits<double>::infinity() };
        double maxVal{   std::numeric_limits<double>::infinity() };
    };

    // ── Internal helpers ──────────────────────────────────────────────────────

    /// @brief N-D linear interpolation (AD-safe).
    template<class S>
    S interpolate(const GridTable& fn,
                  const std::unordered_map<std::string, S>& vars) const;

    /// @brief Recursive MathML evaluator (extended: abs, piecewise, lt…).
    template<class S>
    S evalMathMLNode(const void* /* pugi::xml_node* */ xmlNode,
                     const std::unordered_map<std::string, S>& vars) const;

    template<class S>
    S evalMathMLStr(const std::string& xml,
                    const std::unordered_map<std::string, S>& vars) const;

private:
    double m_srefFt2{ 300.0 };  ///< Reference wing area [ft²]
    double m_cbarFt { 11.32 };  ///< Mean aerodynamic chord [ft]
    double m_bspanFt{ 30.0  };  ///< Wing span [ft]

    std::unordered_map<std::string, BpVec>     m_bps;     ///< bpID → breakpoints
    std::unordered_map<std::string, GridTable> m_tables;  ///< varID → table
    std::vector<EvalStep>                      m_steps;   ///< topo-sorted graph
};

} // namespace Aetherion::Serialization

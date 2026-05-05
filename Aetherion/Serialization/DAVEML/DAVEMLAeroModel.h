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
//
// CppAD note
// ──────────
// DAVEMLAeroModel::evaluate<S>() works with S = double and S = CppAD::AD<double>.
// Breakpoint-segment selection uses the plain-double value of the input (via
// CppAD::Value() in the AD path) so that indices are always integers, while
// interpolation weights are computed with full S precision — giving correct
// first derivatives everywhere except at breakpoint boundaries.
//
// Piecewise branches are resolved with the double value of the condition
// operand (again via CppAD::Value), then a CppAD-safe conditional expression
// is used: CondExpLt(cond, S(0), s_then, s_else).  This keeps the tape
// smooth for optimisation.
// ------------------------------------------------------------------------------

#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <stdexcept>

namespace Aetherion::Serialization {

/// @brief Parsed and evaluatable DAVE-ML aerodynamic model.
///
/// Instantiate once from the .dml file, then call evaluate<S>() as many
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

    // ── Reference geometry (read from file) ───────────────────────────────────
    double sref_ft2{ 300.0 };  ///< Reference wing area [ft²]
    double cbar_ft { 11.32 };  ///< Mean aerodynamic chord [ft]
    double bspan_ft{ 30.0  };  ///< Wing span [ft]

    // ── Construction ──────────────────────────────────────────────────────────

    /// @brief Parse the DAVE-ML aero file and build the evaluation graph.
    /// @throws std::runtime_error  on XML parse failure or missing data.
    explicit DAVEMLAeroModel(const std::string& path);

    // ── Evaluation ────────────────────────────────────────────────────────────

    /// @brief Evaluate the aerodynamic model at a given operating point.
    ///
    /// Works with S = double or S = CppAD::AD<double>.
    /// @param in  Operating-point inputs (see @c Inputs<S>).
    /// @return    Six non-dimensional force and moment coefficients.
    template<class S>
    Outputs<S> evaluate(const Inputs<S>& in) const;

    /// @brief Generic evaluation: caller supplies any varID→value map;
    ///        returns the full computed variable map including all outputs.
    ///        Used by PropModel and other specialisations.
    template<class S>
    std::unordered_map<std::string, S>
    evaluateRaw(std::unordered_map<std::string, S> inputs) const;

    // ── Internal data (public for the .cpp template helpers) ─────────────────

    /// @brief Sorted breakpoint vector for one dimension.
    using BpVec = std::vector<double>;

    /// @brief Gridded look-up table (N-D, 1-D or 2-D for the F-16 model).
    struct GridTable {
        std::vector<std::string> inputVarIDs;  ///< ordered independent var IDs
        std::vector<std::string> bpIDs;        ///< matching breakpoint-def IDs
        std::vector<double>      data;         ///< row-major flattened data
        std::vector<std::size_t> dims;         ///< [n0, n1, ...] sizes per dim
    };

    /// @brief One evaluation step in the topologically sorted graph.
    struct EvalStep {
        std::string varID;
        std::string mathmlXml;  ///< non-empty when type == Calc or OutputCalc
        bool        isTable{ false };
        bool        isInput{ false };
        bool        isConst{ false };
        double      constVal{ 0.0 };
    };

    std::unordered_map<std::string, BpVec>     m_bps;     ///< bpID → breakpoints
    std::unordered_map<std::string, GridTable> m_tables;  ///< varID → table
    std::vector<EvalStep>                      m_steps;   ///< topo-sorted graph

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
};

} // namespace Aetherion::Serialization

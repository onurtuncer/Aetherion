// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// DAVEMLReader.h
//
// Minimal DAVE-ML 2.0 reader for extracting <variableDef> initial values.
//
// Supported features (only what the F-16 model files require):
//   - <variableDef varID="..." units="..." initialValue="...">
//   - Calculated outputs: simple MathML arithmetic (<apply> with
//     <times/>, <minus/>, <plus/>, <divide/>, <cn>, <ci>).
//   - Unit conversion to SI for: slugft2, slug, ft, pct.
//     All other unit strings are passed through unchanged (value × 1.0).
//
// NOT supported (not needed for the F-16 inertia / basic aero files):
//   - Breakpoint / gridded table function lookup
//   - checkData / checkInputs / checkOutputs
//   - Signal routing or complex MathML beyond basic arithmetic
//
// CppAD note: this reader returns plain double values.  These are used
// as configuration constants passed into CppAD-aware policies —
// the inertia tensor and mass are never differentiated through.
// ------------------------------------------------------------------------------

#pragma once

#include <string>
#include <unordered_map>
#include <stdexcept>

namespace Aetherion::Serialization {

/// @brief Lightweight DAVE-ML variableDef reader.
///
/// Loads a DAVE-ML XML file and makes the declared variables available
/// by their @c varID.  Calculated outputs (those with a @c \<calculation\>
/// child) are evaluated lazily on first access using the current input
/// values.  Input variables can be set with @c setInput().
///
/// Unit conversions are applied by @c getValueSI(); they cover:
///   @c slugft2 → kg·m², @c slug → kg, @c ft → m, @c pct → dimensionless.
class DAVEMLReader
{
public:
    /// @brief Load and parse the given DAVE-ML file.
    /// @throws std::runtime_error  if the file cannot be opened or parsed.
    explicit DAVEMLReader(const std::string& path);

    /// @brief Returns the initial value in the units declared in the file.
    /// @throws std::out_of_range   if @p varID is not found.
    double getValue(const std::string& varID) const;

    /// @brief Returns the value converted to SI units.
    ///
    /// Conversion table:
    ///   slugft2 → × 1.355 817 948  (kg·m²)
    ///   slug    → × 14.593 902 937 (kg)
    ///   ft      → × 0.3048         (m)
    ///   pct     → × 0.01
    ///   all other units → × 1.0    (assumed already SI or dimensionless)
    ///
    /// @throws std::out_of_range   if @p varID is not found.
    double getValueSI(const std::string& varID) const;

    /// @brief Override the value of an input variable (for re-evaluating
    ///        calculated outputs).  Has no effect on constants.
    void setInput(const std::string& varID, double value);

    /// @brief Returns @c true if the given @p varID exists in the file.
    bool hasVar(const std::string& varID) const;

    /// @brief Internal per-variable record — public for .cpp helper access.
    struct VarEntry {
        double        value      { 0.0 };
        std::string   units;
        bool          isCalculated{ false };
        std::string   mathmlXml;   ///< serialised MathML for re-evaluation
    };

    std::unordered_map<std::string, VarEntry> m_vars;

    /// @brief Evaluate a simple MathML <apply> tree; returns the result.
    double evalMathML(const std::string& xml) const;
};

} // namespace Aetherion::Serialization

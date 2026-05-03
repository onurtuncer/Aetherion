// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// SnapshotTraits.h
//
// Compile-time snapshot format selection.
//
//   SnapshotFormat::One  — 38-column Aetherion-extended output (Snapshot1).
//                          Includes v_eci, q_body_to_eci in addition to all
//                          31 NASA reference columns.
//
//   SnapshotFormat::Two  — 31-column NASA TM-2015-218675 compatible output
//                          (Snapshot2).  Column names match the reference
//                          *_si_units.csv files exactly, enabling direct
//                          comparison without the "only N shared columns"
//                          message from compare_sim_validation.py.
//
// Usage
// ─────
//   // Write a Snapshot2-format header and row:
//   SnapshotTraits<SnapshotFormat::Two>::write_header(csv);
//   SnapshotTraits<SnapshotFormat::Two>::write_row(csv, snap2);
//
//   // Query the snapshot type:
//   using S = SnapshotTraits<SnapshotFormat::One>::type;  // → Snapshot1
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/Snapshot2.h>
#include <Aetherion/Simulation/Snapshot3.h>
#include <Aetherion/Simulation/Snapshot4.h>
#include <Aetherion/Simulation/Snapshot5.h>
#include <Aetherion/Simulation/Snapshot6.h>

namespace Aetherion::Simulation {

/// @brief Selects the output column schema for simulation CSV files.
enum class SnapshotFormat : int {
    One   = 1, ///< 38 cols — Aetherion-extended (Snapshot1).
    Two   = 2, ///< 31 cols — NASA reference (Snapshot2).
    Three = 3, ///< Placeholder — extend Snapshot3 for the assigned scenario.
    Four  = 4, ///< Placeholder — extend Snapshot4 for the assigned scenario.
    Five  = 5, ///< Placeholder — extend Snapshot5 for the assigned scenario.
    Six   = 6, ///< Placeholder — extend Snapshot6 for the assigned scenario.
};

/// @brief Primary template — intentionally undefined; specialise for each format.
template<SnapshotFormat F>
struct SnapshotTraits;

// ─────────────────────────────────────────────────────────────────────────────
// SnapshotFormat::One  →  Snapshot1
// ─────────────────────────────────────────────────────────────────────────────
template<>
struct SnapshotTraits<SnapshotFormat::One>
{
    using type = Snapshot1;
    static constexpr SnapshotFormat format = SnapshotFormat::One;
    static constexpr int column_count = 38;

    static void write_header(std::ostream& os) { Snapshot1_WriteCsvHeader(os); }
    static void write_row(std::ostream& os, const Snapshot1& s) { Snapshot1_WriteCsvRow(os, s); }
};

// ─────────────────────────────────────────────────────────────────────────────
// SnapshotFormat::Two  →  Snapshot2
// ─────────────────────────────────────────────────────────────────────────────
template<>
struct SnapshotTraits<SnapshotFormat::Two>
{
    using type = Snapshot2;
    static constexpr SnapshotFormat format = SnapshotFormat::Two;
    static constexpr int column_count = 31;

    static void write_header(std::ostream& os) { Snapshot2_WriteCsvHeader(os); }
    static void write_row(std::ostream& os, const Snapshot2& s) { Snapshot2_WriteCsvRow(os, s); }
};

// ─────────────────────────────────────────────────────────────────────────────
// SnapshotFormat::Three  →  Snapshot3  (placeholder — 31 cols, mirrors Two)
// ─────────────────────────────────────────────────────────────────────────────
template<>
struct SnapshotTraits<SnapshotFormat::Three>
{
    using type = Snapshot3;
    static constexpr SnapshotFormat format = SnapshotFormat::Three;
    static constexpr int column_count = static_cast<int>(Snapshot3CsvTraits::kColumnCount);
    static void write_header(std::ostream& os) { Snapshot3_WriteCsvHeader(os); }
    static void write_row(std::ostream& os, const Snapshot3& s) { Snapshot3_WriteCsvRow(os, s); }
};

// ─────────────────────────────────────────────────────────────────────────────
// SnapshotFormat::Four  →  Snapshot4  (placeholder)
// ─────────────────────────────────────────────────────────────────────────────
template<>
struct SnapshotTraits<SnapshotFormat::Four>
{
    using type = Snapshot4;
    static constexpr SnapshotFormat format = SnapshotFormat::Four;
    static constexpr int column_count = static_cast<int>(Snapshot4CsvTraits::kColumnCount);
    static void write_header(std::ostream& os) { Snapshot4_WriteCsvHeader(os); }
    static void write_row(std::ostream& os, const Snapshot4& s) { Snapshot4_WriteCsvRow(os, s); }
};

// ─────────────────────────────────────────────────────────────────────────────
// SnapshotFormat::Five  →  Snapshot5  (placeholder)
// ─────────────────────────────────────────────────────────────────────────────
template<>
struct SnapshotTraits<SnapshotFormat::Five>
{
    using type = Snapshot5;
    static constexpr SnapshotFormat format = SnapshotFormat::Five;
    static constexpr int column_count = static_cast<int>(Snapshot5CsvTraits::kColumnCount);
    static void write_header(std::ostream& os) { Snapshot5_WriteCsvHeader(os); }
    static void write_row(std::ostream& os, const Snapshot5& s) { Snapshot5_WriteCsvRow(os, s); }
};

// ─────────────────────────────────────────────────────────────────────────────
// SnapshotFormat::Six  →  Snapshot6  (placeholder)
// ─────────────────────────────────────────────────────────────────────────────
template<>
struct SnapshotTraits<SnapshotFormat::Six>
{
    using type = Snapshot6;
    static constexpr SnapshotFormat format = SnapshotFormat::Six;
    static constexpr int column_count = static_cast<int>(Snapshot6CsvTraits::kColumnCount);
    static void write_header(std::ostream& os) { Snapshot6_WriteCsvHeader(os); }
    static void write_row(std::ostream& os, const Snapshot6& s) { Snapshot6_WriteCsvRow(os, s); }
};

// ─────────────────────────────────────────────────────────────────────────────
// Convenience aliases
// ─────────────────────────────────────────────────────────────────────────────
using Snapshot1Traits = SnapshotTraits<SnapshotFormat::One>;
using Snapshot2Traits = SnapshotTraits<SnapshotFormat::Two>;
using Snapshot3Traits = SnapshotTraits<SnapshotFormat::Three>;
using Snapshot4Traits = SnapshotTraits<SnapshotFormat::Four>;
using Snapshot5Traits = SnapshotTraits<SnapshotFormat::Five>;
using Snapshot6Traits = SnapshotTraits<SnapshotFormat::Six>;

} // namespace Aetherion::Simulation

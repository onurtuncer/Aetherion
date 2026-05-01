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

namespace Aetherion::Simulation {

/// @brief Selects the output column schema for simulation CSV files.
enum class SnapshotFormat : int {
    One = 1,   ///< 38 columns — all Aetherion fields (Snapshot1).
    Two = 2,   ///< 31 columns — exact NASA reference schema (Snapshot2).
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
// Convenience aliases
// ─────────────────────────────────────────────────────────────────────────────
using Snapshot1Traits = SnapshotTraits<SnapshotFormat::One>;
using Snapshot2Traits = SnapshotTraits<SnapshotFormat::Two>;

} // namespace Aetherion::Simulation

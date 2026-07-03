// ------------------------------------------------------------------------------
// Project: Aetherion — Catch2 tests for SnapshotTraits
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Aetherion/Simulation/SnapshotTraits.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <type_traits>

using namespace Aetherion::Simulation;

// ---------------------------------------------------------------------------
// SnapshotFormat::One  ->  Snapshot1
// ---------------------------------------------------------------------------

TEST_CASE("SnapshotTraits<One>: type alias is Snapshot1", "[SnapshotTraits]")
{
    STATIC_REQUIRE(std::is_same_v<SnapshotTraits<SnapshotFormat::One>::type, Snapshot1>);
    REQUIRE(SnapshotTraits<SnapshotFormat::One>::column_count == 38);
    REQUIRE(SnapshotTraits<SnapshotFormat::One>::format == SnapshotFormat::One);
}

TEST_CASE("SnapshotTraits<One>: write_header/write_row delegate to Snapshot1 free functions", "[SnapshotTraits]")
{
    std::ostringstream oss;
    Snapshot1Traits::write_header(oss);
    Snapshot1Traits::write_row(oss, Snapshot1{});

    const std::string out = oss.str();
    REQUIRE(out.find("time") != std::string::npos);
    REQUIRE(std::count(out.begin(), out.end(), '\n') == 2);
}

// ---------------------------------------------------------------------------
// SnapshotFormat::Two  ->  Snapshot2
// ---------------------------------------------------------------------------

TEST_CASE("SnapshotTraits<Two>: type alias is Snapshot2", "[SnapshotTraits]")
{
    STATIC_REQUIRE(std::is_same_v<SnapshotTraits<SnapshotFormat::Two>::type, Snapshot2>);
    REQUIRE(SnapshotTraits<SnapshotFormat::Two>::column_count == 31);
    REQUIRE(SnapshotTraits<SnapshotFormat::Two>::format == SnapshotFormat::Two);
}

TEST_CASE("SnapshotTraits<Two>: write_header/write_row delegate to Snapshot2 free functions", "[SnapshotTraits]")
{
    Snapshot2 s{};
    s.gePosition_m.setZero();
    s.feVelocity_m_s.setZero();

    std::ostringstream oss;
    Snapshot2Traits::write_header(oss);
    Snapshot2Traits::write_row(oss, s);

    const std::string out = oss.str();
    REQUIRE(out.find("time") != std::string::npos);
    REQUIRE(std::count(out.begin(), out.end(), '\n') == 2);
}

TEST_CASE("SnapshotTraits<Two>: header column count matches Snapshot2CsvTraits", "[SnapshotTraits]")
{
    std::ostringstream oss;
    Snapshot2Traits::write_header(oss);
    const std::string header = oss.str().substr(0, oss.str().find('\n'));
    const auto commas = std::count(header.begin(), header.end(), ',');
    REQUIRE(static_cast<std::size_t>(commas + 1) == Snapshot2CsvTraits::kColumnCount);
}

// ---------------------------------------------------------------------------
// Convenience aliases resolve to the matching specialisation
// ---------------------------------------------------------------------------

TEST_CASE("SnapshotTraits: Snapshot1Traits/Snapshot2Traits aliases match their formats", "[SnapshotTraits]")
{
    STATIC_REQUIRE(std::is_same_v<Snapshot1Traits, SnapshotTraits<SnapshotFormat::One>>);
    STATIC_REQUIRE(std::is_same_v<Snapshot2Traits, SnapshotTraits<SnapshotFormat::Two>>);
}

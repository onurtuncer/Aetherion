// ------------------------------------------------------------------------------
// Project: Aetherion — Catch2 tests for Snapshot2 CSV writer
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "Aetherion/Simulation/Snapshot2.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace Aetherion::Simulation;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Split a single line on commas — no quote handling needed (pure numeric CSV).
static std::vector<std::string> split_csv(const std::string& line)
{
    std::vector<std::string> out;
    std::string tok;
    for (char c : line)
    {
        if (c == ',') { out.push_back(tok); tok.clear(); }
        else { tok += c; }
    }
    out.push_back(tok);
    return out;
}

/// Parse the header line (first line) into column-name tokens.
static std::vector<std::string> parse_header(const std::string& csv)
{
    auto nl = csv.find('\n');
    return split_csv(csv.substr(0, nl));
}

static std::vector<std::string> parse_first_row(const std::string& csv)
{
    auto first_nl = csv.find('\n');
    auto second_nl = csv.find('\n', first_nl + 1);

    // Extract the data line, then strip any trailing \r\n (MSVC emits \r\n on Windows)
    std::string line = csv.substr(first_nl + 1, second_nl - first_nl - 1);
    if (!line.empty() && line.back() == '\r')
        line.pop_back();

    return split_csv(line);
}

/// Build a Snapshot2 where every scalar field = its 1-based column index,
/// so we can verify each value lands in exactly the right column.
static Snapshot2 make_sentinel()
{
    Snapshot2 s;
    s.time = 1.0;
    s.gePosition_m = { 2.0, 3.0, 4.0 };
    s.feVelocity_m_s = { 5.0, 6.0, 7.0 };
    s.altitudeMsl_m = 8.0;
    s.longitude_rad = 9.0;
    s.latitude_rad = 10.0;
    s.localGravity_m_s2 = 11.0;
    s.eulerAngle_rad_Yaw = 12.0;
    s.eulerAngle_rad_Pitch = 13.0;
    s.eulerAngle_rad_Roll = 14.0;
    s.bodyAngularRateWrtEi_rad_s_Roll = 15.0;
    s.bodyAngularRateWrtEi_rad_s_Pitch = 16.0;
    s.bodyAngularRateWrtEi_rad_s_Yaw = 17.0;
    s.altitudeRateWrtMsl_m_s = 18.0;
    s.speedOfSound_m_s = 19.0;
    s.airDensity_kg_m3 = 20.0;
    s.ambientPressure_Pa = 21.0;
    s.ambientTemperature_K = 22.0;
    s.aero_bodyForce_N_X = 23.0;
    s.aero_bodyForce_N_Y = 24.0;
    s.aero_bodyForce_N_Z = 25.0;
    s.aero_bodyMoment_Nm_L = 26.0;
    s.aero_bodyMoment_Nm_M = 27.0;
    s.aero_bodyMoment_Nm_N = 28.0;
    s.mach = 29.0;
    s.dynamicPressure_Pa = 30.0;
    s.trueAirspeed_m_s = 31.0;
    return s;
}

// ---------------------------------------------------------------------------
// 1. Traits — compile-time descriptor
// ---------------------------------------------------------------------------

TEST_CASE("Snapshot2 Traits: column count is 31", "[traits][Snapshot2]")
{
    REQUIRE(Snapshot2CsvTraits::kColumnCount == 31);
}

TEST_CASE("Snapshot2 Traits: no column name is empty", "[traits][Snapshot2]")
{
    for (auto name : Snapshot2CsvTraits::kColumnNames)
        REQUIRE_FALSE(name.empty());
}

TEST_CASE("Snapshot2 Traits: all column names are unique", "[traits][Snapshot2]")
{
    std::vector<std::string_view> names(
        Snapshot2CsvTraits::kColumnNames.begin(),
        Snapshot2CsvTraits::kColumnNames.end());
    std::sort(names.begin(), names.end());
    auto dup = std::adjacent_find(names.begin(), names.end());
    REQUIRE(dup == names.end());
}

TEST_CASE("Snapshot2 Traits: first column is 'time'", "[traits][Snapshot2]")
{
    REQUIRE(Snapshot2CsvTraits::kColumnNames.front() == "time");
}

TEST_CASE("Snapshot2 Traits: last column is 'trueAirspeed_m_s'", "[traits][Snapshot2]")
{
    REQUIRE(Snapshot2CsvTraits::kColumnNames.back() == "trueAirspeed_m_s");
}

TEST_CASE("Snapshot2 Traits: no internal-only columns (v_eci / quaternion) are present", "[traits][Snapshot2]")
{
    const auto& n = Snapshot2CsvTraits::kColumnNames;
    for (auto name : n)
    {
        REQUIRE(name.find("v_eci") == std::string_view::npos);
        REQUIRE(name.find("q_body_to_eci") == std::string_view::npos);
    }
}

TEST_CASE("Snapshot2 Traits: Vector3d columns are contiguous in X/Y/Z order", "[traits][Snapshot2]")
{
    const auto& n = Snapshot2CsvTraits::kColumnNames;
    for (const char* base : { "gePosition_m_", "feVelocity_m_s_" })
    {
        std::string sx = std::string(base) + "X";
        // NOLINTNEXTLINE(readability-qualified-auto) - array::const_iterator is a class on MSVC, not a pointer
        const auto it = std::find(n.begin(), n.end(), std::string_view{ sx });
        REQUIRE(it != n.end());
        REQUIRE(*(it + 1) == std::string(base) + "Y");
        REQUIRE(*(it + 2) == std::string(base) + "Z");
    }
}

// ---------------------------------------------------------------------------
// 2. Header writer
// ---------------------------------------------------------------------------

TEST_CASE("Snapshot2 Header: column count matches kColumnCount", "[header][Snapshot2]")
{
    std::ostringstream oss;
    Snapshot2_WriteCsvHeader(oss);
    auto cols = parse_header(oss.str());
    REQUIRE(cols.size() == Snapshot2CsvTraits::kColumnCount);
}

TEST_CASE("Snapshot2 Header: column names match kColumnNames in order", "[header][Snapshot2]")
{
    std::ostringstream oss;
    Snapshot2_WriteCsvHeader(oss);
    auto cols = parse_header(oss.str());
    for (std::size_t i = 0; i < Snapshot2CsvTraits::kColumnCount; ++i)
        REQUIRE(cols[i] == Snapshot2CsvTraits::kColumnNames[i]);
}

TEST_CASE("Snapshot2 Header: terminated with a single newline", "[header][Snapshot2]")
{
    std::ostringstream oss;
    Snapshot2_WriteCsvHeader(oss);
    const auto& s = oss.str();
    REQUIRE(s.back() == '\n');
    REQUIRE(std::count(s.begin(), s.end(), '\n') == 1);
}

TEST_CASE("Snapshot2 Header: no leading or trailing comma", "[header][Snapshot2]")
{
    std::ostringstream oss;
    Snapshot2_WriteCsvHeader(oss);
    const std::string out = oss.str();
    auto line = out.substr(0, out.find('\n'));
    REQUIRE(line.front() != ',');
    REQUIRE(line.back() != ',');
}

// ---------------------------------------------------------------------------
// 3. Row writer — structural
// ---------------------------------------------------------------------------

TEST_CASE("Snapshot2 Row: value count matches header column count", "[row][Snapshot2]")
{
    std::ostringstream oss;
    Snapshot2_WriteCsvHeader(oss);
    Snapshot2_WriteCsvRow(oss, make_sentinel());

    auto header_cols = parse_header(oss.str());
    auto row_vals = parse_first_row(oss.str());
    REQUIRE(row_vals.size() == header_cols.size());
}

TEST_CASE("Snapshot2 Row: terminated with a single newline", "[row][Snapshot2]")
{
    std::ostringstream oss;
    Snapshot2_WriteCsvRow(oss, make_sentinel());
    const auto& s = oss.str();
    REQUIRE(s.back() == '\n');
    REQUIRE(std::count(s.begin(), s.end(), '\n') == 1);
}

TEST_CASE("Snapshot2 Row: no leading or trailing comma", "[row][Snapshot2]")
{
    std::ostringstream oss;
    Snapshot2_WriteCsvRow(oss, make_sentinel());
    const std::string out = oss.str();
    auto line = out.substr(0, out.find('\n'));
    REQUIRE(line.front() != ',');
    REQUIRE(line.back() != ',');
}

TEST_CASE("Snapshot2 Row: no empty fields (no adjacent commas)", "[row][Snapshot2]")
{
    std::ostringstream oss;
    Snapshot2_WriteCsvRow(oss, make_sentinel());
    REQUIRE(oss.str().find(",,") == std::string::npos);
}

// ---------------------------------------------------------------------------
// 4. Row writer — value correctness (sentinel pattern)
//    Column index is 0-based; sentinel value = (col_index + 1).0
// ---------------------------------------------------------------------------

TEST_CASE("Snapshot2 Row: each sentinel value lands in the correct column", "[row][values][Snapshot2]")
{
    std::ostringstream oss;
    Snapshot2_WriteCsvHeader(oss);
    Snapshot2_WriteCsvRow(oss, make_sentinel());

    auto header = parse_header(oss.str());
    auto row = parse_first_row(oss.str());

    for (std::size_t i = 0; i < row.size(); ++i)
    {
        double parsed = std::stod(row[i]);
        auto expected = static_cast<double>(i + 1);
        CAPTURE(header[i], i, parsed, expected);
        REQUIRE(parsed == Catch::Approx(expected).epsilon(1e-12));
    }
}

// ---------------------------------------------------------------------------
// 5. Row writer — special double values
// ---------------------------------------------------------------------------

TEST_CASE("Snapshot2 Row: zero-initialised scalar fields produce correct default values", "[row][values][Snapshot2]")
{
    // Snapshot2's plain double members are zero-initialised in-class, but
    // gePosition_m/feVelocity_m_s (Eigen::Vector3d) have NO in-class
    // initialiser and are left UNINITIALIZED by the default constructor
    // (undefined behaviour to read them) — so those columns are skipped here,
    // mirroring how test_Snapshot1 skips the uninitialised quaternion.
    Snapshot2 s{};
    s.gePosition_m.setZero();
    s.feVelocity_m_s.setZero();

    std::ostringstream oss;
    Snapshot2_WriteCsvHeader(oss);
    Snapshot2_WriteCsvRow(oss, s);

    auto header = parse_header(oss.str());
    auto row = parse_first_row(oss.str());

    REQUIRE(row.size() == 31);

    for (std::size_t i = 0; i < row.size(); ++i)
    {
        CAPTURE(header[i]);
        REQUIRE(std::stod(row[i]) == Catch::Approx(0.0).margin(1e-30));
    }
}

TEST_CASE("Snapshot2 Row: negative values round-trip correctly", "[row][values][Snapshot2]")
{
    Snapshot2 s = make_sentinel();
    s.time = -1.23456789012345e4;
    s.altitudeMsl_m = -9.87654321098765e3;

    std::ostringstream oss;
    Snapshot2_WriteCsvHeader(oss);
    Snapshot2_WriteCsvRow(oss, s);

    auto header = parse_header(oss.str());
    auto row = parse_first_row(oss.str());

    auto col = [&](const char* name) -> double {
        auto it = std::find(header.begin(), header.end(), name);
        REQUIRE(it != header.end());
        return std::stod(row[std::distance(header.begin(), it)]);
        };

    REQUIRE(col("time") == Catch::Approx(s.time).epsilon(1e-14));
    REQUIRE(col("altitudeMsl_m") == Catch::Approx(s.altitudeMsl_m).epsilon(1e-14));
}

TEST_CASE("Snapshot2 Row: large exponent values survive scientific notation round-trip", "[row][values][Snapshot2]")
{
    Snapshot2 s = make_sentinel();
    s.gePosition_m = { 6.371e6, -6.371e6, 1.0e7 };
    s.ambientPressure_Pa = 1.01325e5;

    std::ostringstream oss;
    Snapshot2_WriteCsvHeader(oss);
    Snapshot2_WriteCsvRow(oss, s);

    auto header = parse_header(oss.str());
    auto row = parse_first_row(oss.str());

    auto col = [&](const char* name) -> double {
        auto it = std::find(header.begin(), header.end(), name);
        REQUIRE(it != header.end());
        return std::stod(row[std::distance(header.begin(), it)]);
        };

    REQUIRE(col("gePosition_m_X") == Catch::Approx(6.371e6).epsilon(1e-14));
    REQUIRE(col("gePosition_m_Y") == Catch::Approx(-6.371e6).epsilon(1e-14));
    REQUIRE(col("gePosition_m_Z") == Catch::Approx(1.0e7).epsilon(1e-14));
    REQUIRE(col("ambientPressure_Pa") == Catch::Approx(1.01325e5).epsilon(1e-14));
}

// ---------------------------------------------------------------------------
// 6. Multi-row output
// ---------------------------------------------------------------------------

TEST_CASE("Snapshot2 Multi-row: N rows produce N+1 lines (header + N data lines)", "[multi-row][Snapshot2]")
{
    std::ostringstream oss;
    Snapshot2_WriteCsvHeader(oss);
    for (int i = 0; i < 5; ++i)
    {
        Snapshot2 s = make_sentinel();
        s.time = static_cast<double>(i);
        Snapshot2_WriteCsvRow(oss, s);
    }
    const std::string out = oss.str();
    REQUIRE(std::count(out.begin(), out.end(), '\n') == 6);
}

TEST_CASE("Snapshot2 Multi-row: time column is monotonically increasing", "[multi-row][Snapshot2]")
{
    std::ostringstream oss;
    Snapshot2_WriteCsvHeader(oss);

    const int N = 10;
    for (int i = 0; i < N; ++i)
    {
        Snapshot2 s = make_sentinel();
        s.time = i * 0.01;
        Snapshot2_WriteCsvRow(oss, s);
    }

    std::istringstream iss(oss.str());
    std::string line;
    std::getline(iss, line); // skip header

    double prev = -1.0;
    while (std::getline(iss, line))
    {
        if (line.empty()) continue;
        double t = std::stod(split_csv(line)[0]);
        REQUIRE(t > prev);
        prev = t;
    }
}

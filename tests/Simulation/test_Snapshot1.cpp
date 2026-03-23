// ------------------------------------------------------------------------------
// Project: Aetherion — Catch2 tests for Snapshot1 CSV writer
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------

#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "Aetherion/Simulation/Snapshot1.h"

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

/// Build a Snapshot1 where every distinct scalar field = its 1-based column index,
/// so we can verify each value lands in exactly the right column.
static Snapshot1 make_sentinel()
{
    Snapshot1 s;
    s.time = 1.0;
    s.gePosition_m = { 2.0,  3.0,  4.0 };
    s.feVelocity_m_s = { 5.0,  6.0,  7.0 };
    s.v_eci = { 8.0,  9.0, 10.0 };
    s.altitudeMsl_m = 11.0;
    s.longitude_rad = 12.0;
    s.latitude_rad = 13.0;
    s.localGravity_m_s2 = 14.0;
    s.eulerAngle_rad_Yaw = 15.0;
    s.eulerAngle_rad_Pitch = 16.0;
    s.eulerAngle_rad_Roll = 17.0;
    s.q_body_to_eci = { 18.0, 19.0, 20.0, 21.0 }; // w,x,y,z
    s.bodyAngularRateWrtEi_rad_s_Roll = 22.0;
    s.bodyAngularRateWrtEi_rad_s_Pitch = 23.0;
    s.bodyAngularRateWrtEi_rad_s_Yaw = 24.0;
    s.altitudeRateWrtMsl_m_s = 25.0;
    s.speedOfSound_m_s = 26.0;
    s.airDensity_kg_m3 = 27.0;
    s.ambientPressure_Pa = 28.0;
    s.ambientTemperature_K = 29.0;
    s.aero_bodyForce_N_X = 30.0;
    s.aero_bodyForce_N_Y = 31.0;
    s.aero_bodyForce_N_Z = 32.0;
    s.aero_bodyMoment_Nm_L = 33.0;
    s.aero_bodyMoment_Nm_M = 34.0;
    s.aero_bodyMoment_Nm_N = 35.0;
    s.mach = 36.0;
    s.dynamicPressure_Pa = 37.0;
    s.trueAirspeed_m_s = 38.0;
    return s;
}

// ---------------------------------------------------------------------------
// 1. Traits — compile-time descriptor
// ---------------------------------------------------------------------------

TEST_CASE("Traits: column count is 38", "[traits]")
{
    REQUIRE(Snapshot1CsvTraits::kColumnCount == 38);
}

TEST_CASE("Traits: no column name is empty", "[traits]")
{
    for (auto name : Snapshot1CsvTraits::kColumnNames)
        REQUIRE_FALSE(name.empty());
}

TEST_CASE("Traits: all column names are unique", "[traits]")
{
    std::vector<std::string_view> names(
        Snapshot1CsvTraits::kColumnNames.begin(),
        Snapshot1CsvTraits::kColumnNames.end());
    std::sort(names.begin(), names.end());
    auto dup = std::adjacent_find(names.begin(), names.end());
    REQUIRE(dup == names.end());
}

TEST_CASE("Traits: first column is 'time'", "[traits]")
{
    REQUIRE(Snapshot1CsvTraits::kColumnNames.front() == "time");
}

TEST_CASE("Traits: last column is 'trueAirspeed_m_s'", "[traits]")
{
    REQUIRE(Snapshot1CsvTraits::kColumnNames.back() == "trueAirspeed_m_s");
}

TEST_CASE("Traits: quaternion columns appear W-first and are contiguous", "[traits]")
{
    const auto& n = Snapshot1CsvTraits::kColumnNames;
    auto it_w = std::find(n.begin(), n.end(), std::string_view{ "q_body_to_eci_W" });
    REQUIRE(it_w != n.end());
    REQUIRE(*(it_w + 1) == "q_body_to_eci_X");
    REQUIRE(*(it_w + 2) == "q_body_to_eci_Y");
    REQUIRE(*(it_w + 3) == "q_body_to_eci_Z");
}

TEST_CASE("Traits: Vector3d columns are contiguous in X/Y/Z order", "[traits]")
{
    const auto& n = Snapshot1CsvTraits::kColumnNames;
    for (const char* base : { "gePosition_m_", "feVelocity_m_s_", "v_eci_" })
    {
        std::string sx = std::string(base) + "X";
        auto it = std::find(n.begin(), n.end(), std::string_view{ sx });
        REQUIRE(it != n.end());
        REQUIRE(*(it + 1) == std::string(base) + "Y");
        REQUIRE(*(it + 2) == std::string(base) + "Z");
    }
}

// ---------------------------------------------------------------------------
// 2. Header writer
// ---------------------------------------------------------------------------

TEST_CASE("Header: column count matches kColumnCount", "[header]")
{
    std::ostringstream oss;
    Snapshot1_WriteCsvHeader(oss);
    auto cols = parse_header(oss.str());
    REQUIRE(cols.size() == Snapshot1CsvTraits::kColumnCount);
}

TEST_CASE("Header: column names match kColumnNames in order", "[header]")
{
    std::ostringstream oss;
    Snapshot1_WriteCsvHeader(oss);
    auto cols = parse_header(oss.str());
    for (std::size_t i = 0; i < Snapshot1CsvTraits::kColumnCount; ++i)
        REQUIRE(cols[i] == Snapshot1CsvTraits::kColumnNames[i]);
}

TEST_CASE("Header: terminated with a single newline", "[header]")
{
    std::ostringstream oss;
    Snapshot1_WriteCsvHeader(oss);
    const auto& s = oss.str();
    REQUIRE(s.back() == '\n');
    // Only one newline — the header is exactly one line
    REQUIRE(std::count(s.begin(), s.end(), '\n') == 1);
}

TEST_CASE("Header: no leading or trailing comma", "[header]")
{
    std::ostringstream oss;
    Snapshot1_WriteCsvHeader(oss);
    const std::string out = oss.str();
    auto line = out.substr(0, out.find('\n'));
    REQUIRE(line.front() != ',');
    REQUIRE(line.back() != ',');
}

// ---------------------------------------------------------------------------
// 3. Row writer — structural
// ---------------------------------------------------------------------------

TEST_CASE("Row: value count matches header column count", "[row]")
{
    std::ostringstream oss;
    Snapshot1_WriteCsvHeader(oss);
    Snapshot1_WriteCsvRow(oss, make_sentinel());

    auto header_cols = parse_header(oss.str());
    auto row_vals = parse_first_row(oss.str());
    REQUIRE(row_vals.size() == header_cols.size());
}

TEST_CASE("Row: terminated with a single newline", "[row]")
{
    std::ostringstream oss;
    Snapshot1_WriteCsvRow(oss, Snapshot1{});
    const auto& s = oss.str();
    REQUIRE(s.back() == '\n');
    REQUIRE(std::count(s.begin(), s.end(), '\n') == 1);
}

TEST_CASE("Row: no leading or trailing comma", "[row]")
{
    std::ostringstream oss;
    Snapshot1_WriteCsvRow(oss, Snapshot1{});
    const std::string out = oss.str();
    auto line = out.substr(0, out.find('\n'));
    REQUIRE(line.front() != ',');
    REQUIRE(line.back() != ',');
}

TEST_CASE("Row: no empty fields (no adjacent commas)", "[row]")
{
    std::ostringstream oss;
    Snapshot1_WriteCsvRow(oss, make_sentinel());
    REQUIRE(oss.str().find(",,") == std::string::npos);
}

// ---------------------------------------------------------------------------
// 4. Row writer — value correctness (sentinel pattern)
//    Column index is 0-based; sentinel value = (col_index + 1).0
// ---------------------------------------------------------------------------

TEST_CASE("Row: each sentinel value lands in the correct column", "[row][values]")
{
    std::ostringstream oss;
    Snapshot1_WriteCsvHeader(oss);
    Snapshot1_WriteCsvRow(oss, make_sentinel());

    auto header = parse_header(oss.str());
    auto row = parse_first_row(oss.str());

    // For each column, expected value = (col_index + 1).0
    for (std::size_t i = 0; i < row.size(); ++i)
    {
        double parsed = std::stod(row[i]);
        double expected = static_cast<double>(i + 1);
        CAPTURE(header[i], i, parsed, expected);
        REQUIRE(parsed == Catch::Approx(expected).epsilon(1e-12));
    }
}

// ---------------------------------------------------------------------------
// 5. Row writer — special double values
// ---------------------------------------------------------------------------

TEST_CASE("Row: zero snapshot produces correct default values", "[row][values]")
{
    // Snapshot1{} zero-initialises all plain double members.
    // Eigen::Quaterniond default constructor leaves coefficients UNINITIALIZED
    // (undefined behaviour to read them) — so the quaternion columns are skipped!
    std::ostringstream oss;
    Snapshot1_WriteCsvHeader(oss);
    Snapshot1_WriteCsvRow(oss, Snapshot1{});

    auto header = parse_header(oss.str());
    auto row = parse_first_row(oss.str());

    REQUIRE(row.size() == 38);

    // Columns to skip — Eigen::Quaterniond is uninitialized on default construction
    const std::vector<std::string> skip = {
        "q_body_to_eci_W", "q_body_to_eci_X", "q_body_to_eci_Y", "q_body_to_eci_Z"
    };

    for (std::size_t i = 0; i < row.size(); ++i)
    {
        if (std::find(skip.begin(), skip.end(), header[i]) != skip.end())
            continue;
        CAPTURE(header[i]);
        REQUIRE(std::stod(row[i]) == Catch::Approx(0.0).margin(1e-30));
    }
}

TEST_CASE("Row: negative values round-trip correctly", "[row][values]")
{
    Snapshot1 s;
    s.time = -1.23456789012345e4;
    s.altitudeMsl_m = -9.87654321098765e3;

    std::ostringstream oss;
    Snapshot1_WriteCsvHeader(oss);
    Snapshot1_WriteCsvRow(oss, s);

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

TEST_CASE("Row: large exponent values survive scientific notation round-trip", "[row][values]")
{
    Snapshot1 s;
    s.gePosition_m = { 6.371e6, -6.371e6, 1.0e7 };
    s.ambientPressure_Pa = 1.01325e5;

    std::ostringstream oss;
    Snapshot1_WriteCsvHeader(oss);
    Snapshot1_WriteCsvRow(oss, s);

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

TEST_CASE("Multi-row: N rows produce N+1 lines (header + N data lines)", "[multi-row]")
{
    std::ostringstream oss;
    Snapshot1_WriteCsvHeader(oss);
    for (int i = 0; i < 5; ++i)
    {
        Snapshot1 s;
        s.time = static_cast<double>(i);
        Snapshot1_WriteCsvRow(oss, s);
    }
    const std::string out = oss.str();
    REQUIRE(std::count(out.begin(), out.end(), '\n') == 6);
}

TEST_CASE("Multi-row: time column is monotonically increasing", "[multi-row]")
{
    std::ostringstream oss;
    Snapshot1_WriteCsvHeader(oss);

    const int N = 10;
    for (int i = 0; i < N; ++i)
    {
        Snapshot1 s;
        s.time = i * 0.01;
        Snapshot1_WriteCsvRow(oss, s);
    }

    // Walk every data line (i.e. row) and verify time column ascends
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
// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_DAVEMLAeroModelLimits.cpp
//
// Regression tests for the second pre-DOI audit's findings N03 and N04.
//
// N03. A function's independentVarRef carries min, max and extrapolate, and
//      a variableDef's minValue / maxValue apply to an input the caller
//      supplies as much as to a computed variable. The table limits were
//      read and ignored, so a table with extrapolate="neither" (every table
//      of the F-16 deck) continued its edge segments' lines past the
//      breakpoints, and a supplied input went through unclamped.
// N04. A dataTable shorter than its breakpoint grid was accepted and read
//      past its end at evaluation, with nothing to say so in a Release
//      build. A table is now checked against its breakpoints when the file
//      is loaded.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>
#include <cppad/cppad.hpp>

#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

using namespace Aetherion::Serialization;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

    using AD = CppAD::AD<double>;

    /// `body` wrapped in a DAVEfunc element in a temp file named after `tag`,
    /// removed again when the loader returns or throws.
    struct TempDML
    {
        std::filesystem::path path;
        TempDML(const std::string& tag, const std::string& body)
            : path(std::filesystem::temp_directory_path() / ("_ae_test_daveml_limits_" + tag + ".dml"))
        {
            std::ofstream f(path);
            f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<DAVEfunc>\n"
              << body << "\n</DAVEfunc>\n";
        }
        TempDML(const TempDML&) = delete;
        TempDML& operator=(const TempDML&) = delete;
        TempDML(TempDML&&) = delete;
        TempDML& operator=(TempDML&&) = delete;
        ~TempDML()
        {
            std::error_code ec;
            std::filesystem::remove(path, ec);
        }
    };

    DAVEMLAeroModel loadDML(const std::string& tag, const std::string& body)
    {
        const TempDML f(tag, body);
        return DAVEMLAeroModel(f.path.string());
    }

    // y = table(x): breakpoints 0, 1, 2 and values 0, 1, 4, slope 1 on the first
    // segment and 3 on the second, so the lines continue to -0.5 at x = -0.5 and
    // to 5.5 at x = 2.5 while the edge values are 0 and 4.
    std::string tableDML(const std::string& ivrAttrs,
        const std::string& bpVals = "0.0, 1.0, 2.0",
        const std::string& data = "0.0, 1.0, 4.0",
        const std::string& bpRefs = R"(<bpRef bpID="XBP"/>)")
    {
        return R"(<variableDef varID="x" units="nd" initialValue="0.5"><isInput/></variableDef>
  <variableDef varID="y" units="nd"><isOutput/></variableDef>
  <breakpointDef name="xbp" bpID="XBP" units="nd"><bpVals>)" +
               bpVals + R"(</bpVals></breakpointDef>
  <function name="table">
    <independentVarRef varID="x" )" +
               ivrAttrs + R"(/>
    <dependentVarRef varID="y"/>
    <functionDefn name="table_fn"><griddedTableDef name="table_t" gtID="table_t_def">
      <breakpointRefs>)" +
               bpRefs + R"(</breakpointRefs>
      <dataTable>)" +
               data + R"(</dataTable>
    </griddedTableDef></functionDefn>
  </function>)";
    }

    // y = 2 a + b / 10 on a in [0, 1], b in [0, 10]; row-major, last index fastest.
    std::string twoDimDML(const std::string& data = "0.0, 1.0, 2.0, 3.0")
    {
        return R"(<variableDef varID="a" units="nd" initialValue="0"><isInput/></variableDef>
  <variableDef varID="b" units="nd" initialValue="0"><isInput/></variableDef>
  <variableDef varID="y" units="nd"><isOutput/></variableDef>
  <breakpointDef name="abp" bpID="ABP" units="nd"><bpVals>0.0, 1.0</bpVals></breakpointDef>
  <breakpointDef name="bbp" bpID="BBP" units="nd"><bpVals>0.0, 10.0</bpVals></breakpointDef>
  <function name="table2">
    <independentVarRef varID="a"/>
    <independentVarRef varID="b"/>
    <dependentVarRef varID="y"/>
    <functionDefn name="t2"><griddedTableDef name="t2d" gtID="t2d_def">
      <breakpointRefs><bpRef bpID="ABP"/><bpRef bpID="BBP"/></breakpointRefs>
      <dataTable>)" +
               data + R"(</dataTable>
    </griddedTableDef></functionDefn>
  </function>)";
    }

    // x held by its variableDef; y = x.
    std::string inputLimitDML(const std::string& attrs)
    {
        return R"(<variableDef varID="x" units="nd" initialValue="0" )" + attrs + R"(><isInput/></variableDef>
  <variableDef varID="y" units="nd"><calculation><math><ci>x</ci></math></calculation><isOutput/></variableDef>)";
    }

    double y(const DAVEMLAeroModel& m, double x)
    {
        return m.evaluateRaw<double>({ { "x", x } }).at("y");
    }

    /// Tape y(x) at x0 with change tracking on.
    CppAD::ADFun<double> tapeY(const DAVEMLAeroModel& m, double x0)
    {
        std::vector<AD> ax{ AD(x0) };
        CppAD::Independent(ax);
        std::vector<AD> ay{ m.evaluateRaw<AD>({ { "x", ax[0] } }).at("y") };
        CppAD::ADFun<double> f(ax, ay);
        f.compare_change_count(1);
        return f;
    }

} // namespace

TEST_CASE("DAVEMLAeroModel limits: extrapolate says what happens past the breakpoints",
    "[daveml_aero][limits][regression]")
{
    SECTION("neither, the default, holds the input at the edge")
    {
        for (const char* attrs : { "", R"(extrapolate="neither")" }) {
            const auto m = loadDML("neither", tableDML(attrs));
            CHECK_THAT(y(m, -0.5), WithinAbs(0.0, 1e-12));
            CHECK_THAT(y(m, 2.5), WithinAbs(4.0, 1e-12));
            CHECK_THAT(y(m, 0.5), WithinAbs(0.5, 1e-12));
            CHECK_THAT(y(m, 1.5), WithinAbs(2.5, 1e-12));
        }
    }
    SECTION("min lets the first segment's line continue below")
    {
        const auto m = loadDML("min", tableDML(R"(extrapolate="min")"));
        CHECK_THAT(y(m, -0.5), WithinAbs(-0.5, 1e-12));
        CHECK_THAT(y(m, 2.5), WithinAbs(4.0, 1e-12));
    }
    SECTION("max lets the last segment's line continue above")
    {
        const auto m = loadDML("max", tableDML(R"(extrapolate="max")"));
        CHECK_THAT(y(m, -0.5), WithinAbs(0.0, 1e-12));
        CHECK_THAT(y(m, 2.5), WithinAbs(5.5, 1e-12));
    }
    SECTION("both extrapolates on both sides")
    {
        const auto m = loadDML("both", tableDML(R"(extrapolate="both")"));
        CHECK_THAT(y(m, -0.5), WithinAbs(-0.5, 1e-12));
        CHECK_THAT(y(m, 2.5), WithinAbs(5.5, 1e-12));
    }
    SECTION("anything else is rejected when the file is loaded")
    {
        REQUIRE_THROWS_WITH(loadDML("bad_mode", tableDML(R"(extrapolate="always")")),
            ContainsSubstring("extrapolate"));
    }
}

TEST_CASE("DAVEMLAeroModel limits: min and max narrow the input range in any mode",
    "[daveml_aero][limits][regression]")
{
    // Tighter than the breakpoints: held at 0.5 and 1.5.
    const auto tight = loadDML("tight", tableDML(R"(min="0.5" max="1.5" extrapolate="neither")"));
    CHECK_THAT(y(tight, 0.0), WithinAbs(0.5, 1e-12));
    CHECK_THAT(y(tight, 2.0), WithinAbs(2.5, 1e-12));
    CHECK_THAT(y(tight, 1.0), WithinAbs(1.0, 1e-12));

    // Wider than the breakpoints, with extrapolation: the lines continue to
    // the limits and the input is held there.
    const auto wide = loadDML("wide", tableDML(R"(min="-1" max="3" extrapolate="both")"));
    CHECK_THAT(y(wide, -2.0), WithinAbs(-1.0, 1e-12)); // first line at x = -1
    CHECK_THAT(y(wide, 5.0), WithinAbs(7.0, 1e-12));   // last line at x = 3
    CHECK_THAT(y(wide, 2.5), WithinAbs(5.5, 1e-12));

    // The F-16 deck's form: neither, with the limits on the breakpoints.
    const auto f16 = loadDML("f16form", tableDML(R"(min="0.0" max="2.0" extrapolate="neither")"));
    CHECK_THAT(y(f16, -0.5), WithinAbs(0.0, 1e-12));
    CHECK_THAT(y(f16, 2.5), WithinAbs(4.0, 1e-12));

    REQUIRE_THROWS_WITH(loadDML("crossed", tableDML(R"(min="3" max="2" extrapolate="neither")")),
        ContainsSubstring("min > max"));
}

TEST_CASE("DAVEMLAeroModel limits: holding an input is a CondExp, so a tape stays valid across the limit",
    "[daveml_aero][limits][cppad]")
{
    const auto m = loadDML("tape", tableDML(R"(min="0.0" max="2.0" extrapolate="neither")"));
    auto f = tapeY(m, -0.5);

    // Held at the edge: value 0 and derivative 0, no change reported.
    CHECK_THAT(f.Forward(0, std::vector<double>{ -0.3 })[0], WithinAbs(0.0, 1e-12));
    CHECK(f.compare_change_number() == 0);
    CHECK_THAT(f.Jacobian(std::vector<double>{ -0.3 })[0], WithinAbs(0.0, 1e-12));

    // Into the first segment: the same tape gives the model and its slope.
    CHECK_THAT(f.Forward(0, std::vector<double>{ 0.5 })[0], WithinAbs(0.5, 1e-12));
    CHECK(f.compare_change_number() == 0);
    CHECK_THAT(f.Jacobian(std::vector<double>{ 0.5 })[0], WithinAbs(1.0, 1e-12));

    // Another segment is still reported.
    f.Forward(0, std::vector<double>{ 1.5 });
    CHECK(f.compare_change_number() > 0);

    // double and AD agree everywhere, held or not.
    for (double x : { -0.5, 0.25, 1.0, 1.75, 2.5 }) {
        auto g = tapeY(m, x);
        CHECK_THAT(g.Forward(0, std::vector<double>{ x })[0], WithinAbs(y(m, x), 1e-12));
    }
}

TEST_CASE("DAVEMLAeroModel limits: variableDef minValue / maxValue apply to a supplied input",
    "[daveml_aero][limits][regression]")
{
    SECTION("both limits")
    {
        const auto m = loadDML("in_both", inputLimitDML(R"(minValue="0" maxValue="2")"));
        const auto hi = m.evaluateRaw<double>({ { "x", 4.0 } });
        CHECK_THAT(hi.at("y"), WithinAbs(2.0, 1e-12));
        CHECK_THAT(hi.at("x"), WithinAbs(2.0, 1e-12)); // the held input is what comes back
        CHECK_THAT(m.evaluateRaw<double>({ { "x", -1.0 } }).at("y"), WithinAbs(0.0, 1e-12));
        CHECK_THAT(m.evaluateRaw<double>({ { "x", 1.0 } }).at("y"), WithinAbs(1.0, 1e-12));
    }
    SECTION("one limit only")
    {
        const auto lo = loadDML("in_min", inputLimitDML(R"(minValue="0")"));
        CHECK_THAT(lo.evaluateRaw<double>({ { "x", -1.0 } }).at("y"), WithinAbs(0.0, 1e-12));
        CHECK_THAT(lo.evaluateRaw<double>({ { "x", 4.0 } }).at("y"), WithinAbs(4.0, 1e-12));

        const auto hi = loadDML("in_max", inputLimitDML(R"(maxValue="2")"));
        CHECK_THAT(hi.evaluateRaw<double>({ { "x", -1.0 } }).at("y"), WithinAbs(-1.0, 1e-12));
        CHECK_THAT(hi.evaluateRaw<double>({ { "x", 4.0 } }).at("y"), WithinAbs(2.0, 1e-12));
    }
    SECTION("AD: held value, zero slope, no re-recording")
    {
        const auto m = loadDML("in_ad", inputLimitDML(R"(minValue="0" maxValue="2")"));
        auto f = tapeY(m, 4.0);
        CHECK_THAT(f.Forward(0, std::vector<double>{ 4.0 })[0], WithinAbs(2.0, 1e-12));
        CHECK_THAT(f.Jacobian(std::vector<double>{ 4.0 })[0], WithinAbs(0.0, 1e-12));
        CHECK_THAT(f.Forward(0, std::vector<double>{ 1.0 })[0], WithinAbs(1.0, 1e-12));
        CHECK(f.compare_change_number() == 0);
        CHECK_THAT(f.Jacobian(std::vector<double>{ 1.0 })[0], WithinAbs(1.0, 1e-12));
    }
    SECTION("through evaluate(): alpha held to the deck's range")
    {
        const auto m = loadDML("in_alpha", R"(
  <variableDef varID="alpha" units="deg" initialValue="0" minValue="-10" maxValue="45"><isInput/></variableDef>
  <variableDef varID="cx" units="nd"><calculation><math><ci>alpha</ci></math></calculation><isOutput/></variableDef>)");
        DAVEMLAeroModel::Inputs<double> in;
        in.alpha_deg = 60.0;
        CHECK_THAT(m.evaluate(in).cx, WithinAbs(45.0, 1e-12));
        in.alpha_deg = -20.0;
        CHECK_THAT(m.evaluate(in).cx, WithinAbs(-10.0, 1e-12));
        in.alpha_deg = 5.0;
        CHECK_THAT(m.evaluate(in).cx, WithinAbs(5.0, 1e-12));
    }
}

TEST_CASE("DAVEMLAeroModel load: a table is checked against its breakpoints",
    "[daveml_aero][validation][regression]")
{
    const char* const kNeither = R"(extrapolate="neither")";

    SECTION("a dataTable shorter or longer than the grid is rejected")
    {
        // The audit's probe: three breakpoints, one value -- read past the
        // end of the data at evaluation, with nothing to say so in Release.
        REQUIRE_THROWS_WITH(loadDML("short", tableDML(kNeither, "0.0, 1.0, 2.0", "0.0")),
            ContainsSubstring("dataTable has 1 value(s), the breakpoints need 3"));
        REQUIRE_THROWS_WITH(loadDML("long", tableDML(kNeither, "0.0, 1.0, 2.0", "0.0, 1.0, 4.0, 9.0")),
            ContainsSubstring("dataTable has 4 value(s), the breakpoints need 3"));
    }
    SECTION("breakpoints must be defined, at least two, and strictly increasing")
    {
        REQUIRE_THROWS_WITH(loadDML("one_bp", tableDML(kNeither, "0.0", "0.0")),
            ContainsSubstring("at least 2"));
        REQUIRE_THROWS_WITH(loadDML("unsorted", tableDML(kNeither, "0.0, 2.0, 1.0")),
            ContainsSubstring("strictly increasing"));
        REQUIRE_THROWS_WITH(loadDML("repeated", tableDML(kNeither, "0.0, 1.0, 1.0")),
            ContainsSubstring("strictly increasing"));
        REQUIRE_THROWS_WITH(loadDML("undefined", tableDML(kNeither, "0.0, 1.0, 2.0", "0.0, 1.0, 4.0",
                                                     R"(<bpRef bpID="NOPE"/>)")),
            ContainsSubstring("not defined"));
    }
    SECTION("one bpRef per independentVarRef")
    {
        REQUIRE_THROWS_WITH(loadDML("two_refs", tableDML(kNeither, "0.0, 1.0, 2.0", "0.0, 1.0, 4.0",
                                                    R"(<bpRef bpID="XBP"/><bpRef bpID="XBP"/>)")),
            ContainsSubstring("1 independentVarRef(s) but 2 bpRef(s)"));
    }
    SECTION("a 2-D table of the right size loads, and is held at its edges on both axes")
    {
        const auto m = loadDML("two_d", twoDimDML());
        auto y2 = [&](double a, double b) {
            return m.evaluateRaw<double>({ { "a", a }, { "b", b } }).at("y");
        };
        CHECK_THAT(y2(1.0, 0.0), WithinAbs(2.0, 1e-12));
        CHECK_THAT(y2(0.0, 10.0), WithinAbs(1.0, 1e-12));
        CHECK_THAT(y2(0.5, 5.0), WithinAbs(1.5, 1e-12));
        CHECK_THAT(y2(5.0, 20.0), WithinAbs(3.0, 1e-12));  // held at (1, 10)
        CHECK_THAT(y2(-1.0, -5.0), WithinAbs(0.0, 1e-12)); // held at (0, 0)
        CHECK_THAT(y2(0.5, 20.0), WithinAbs(2.0, 1e-12));  // held on b only

        REQUIRE_THROWS_WITH(loadDML("two_d_short", twoDimDML("0.0, 1.0, 2.0")),
            ContainsSubstring("dataTable has 3 value(s), the breakpoints need 4"));
    }
}

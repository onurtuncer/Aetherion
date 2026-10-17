// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_DAVEMLAeroModelErrors.cpp
//
// DAVEMLAeroModel on small hand-written DML files: variableDef minValue /
// maxValue limits through both entry points and both scalar types, and the
// diagnostics for MathML the evaluator does not support. Each case writes
// its own file name so parallel ctest processes do not collide.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>
#include <cppad/cppad.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <tuple>
#include <unordered_map>
#include <vector>

using namespace Aetherion::Serialization;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

    using AD = CppAD::AD<double>;

    /// Writes `body` wrapped in a DAVEfunc element to a temp file named after
    /// `tag`, loads it, and removes the file again, also when loading throws.
    DAVEMLAeroModel loadDML(const std::string& tag, const std::string& body)
    {
        struct TempFile
        {
            std::filesystem::path path;
            ~TempFile()
            {
                std::error_code ec;
                std::filesystem::remove(path, ec);
            }
        } file{ std::filesystem::temp_directory_path() / ("_ae_test_daveml_" + tag + ".dml") };
        {
            std::ofstream f(file.path);
            f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<DAVEfunc>\n"
              << body << "\n</DAVEfunc>\n";
        }
        return DAVEMLAeroModel(file.path.string());
    }

    // cx follows alpha but is limited to [-1, 2]; cm is limited above only.
    const char* const kLimitedDML = R"(
  <variableDef varID="alpha" units="deg" initialValue="0"><isInput/></variableDef>
  <variableDef varID="cx" units="nd" minValue="-1" maxValue="2">
    <calculation><math><apply><times/><cn>1</cn><ci>alpha</ci></apply></math></calculation>
  </variableDef>
  <variableDef varID="cm" units="nd" maxValue="0.5">
    <calculation><math><apply><divide/><ci>alpha</ci><cn>10</cn></apply></math></calculation>
  </variableDef>)";

    std::string calcVar(const std::string& mathBody)
    {
        return R"(<variableDef varID="x" units="nd" initialValue="2"><isInput/></variableDef>
  <variableDef varID="y" units="nd"><calculation><math>)" +
               mathBody + R"(</math></calculation></variableDef>)";
    }

} // namespace

TEST_CASE("DAVEMLAeroModel: variable limits clamp in evaluate (double)", "[daveml_aero][limits]")
{
    const auto m = loadDML("limits_d", kLimitedDML);

    DAVEMLAeroModel::Inputs<double> in;
    in.alpha_deg = 5.0;
    auto out = m.evaluate(in);
    CHECK_THAT(out.cx, WithinAbs(2.0, 1e-12)); // clamped to maxValue
    CHECK_THAT(out.cm, WithinAbs(0.5, 1e-12)); // 0.5, at the limit

    in.alpha_deg = -5.0;
    out = m.evaluate(in);
    CHECK_THAT(out.cx, WithinAbs(-1.0, 1e-12)); // clamped to minValue
    CHECK_THAT(out.cm, WithinAbs(-0.5, 1e-12)); // no lower limit

    in.alpha_deg = 1.5;
    out = m.evaluate(in);
    CHECK_THAT(out.cx, WithinAbs(1.5, 1e-12)); // inside the band
}

TEST_CASE("DAVEMLAeroModel: variable limits clamp in evaluate (AD<double>)", "[daveml_aero][limits][cppad]")
{
    const auto m = loadDML("limits_ad", kLimitedDML);

    for (const double alpha : { 5.0, -5.0, 1.5 }) {
        DAVEMLAeroModel::Inputs<AD> in;
        in.alpha_deg = alpha;
        const auto out = m.evaluate(in);
        const double expected = std::clamp(alpha, -1.0, 2.0);
        CHECK_THAT(CppAD::Value(out.cx), WithinAbs(expected, 1e-12));
    }
}

TEST_CASE("DAVEMLAeroModel: variable limits clamp in evaluateRaw", "[daveml_aero][limits]")
{
    const auto m = loadDML("limits_raw", kLimitedDML);

    auto rd = m.evaluateRaw(std::unordered_map<std::string, double>{ { "alpha", 9.0 } });
    CHECK_THAT(rd["cx"], WithinAbs(2.0, 1e-12));
    rd = m.evaluateRaw(std::unordered_map<std::string, double>{ { "alpha", -9.0 } });
    CHECK_THAT(rd["cx"], WithinAbs(-1.0, 1e-12));

    // AD path on a recorded tape: the clamp is a CondExp, so the limited
    // output is flat (zero derivative) outside the band and 1 inside it.
    for (const auto& [alpha, value, slope] : { std::tuple{ 9.0, 2.0, 0.0 }, std::tuple{ 0.5, 0.5, 1.0 } }) {
        std::vector<AD> ax{ AD(alpha) };
        CppAD::Independent(ax);
        auto ra = m.evaluateRaw(std::unordered_map<std::string, AD>{ { "alpha", ax[0] } });
        std::vector<AD> ay{ ra["cx"] };
        CppAD::ADFun<double> f(ax, ay);
        const std::vector<double> x{ alpha };
        CHECK_THAT(f.Forward(0, x)[0], WithinAbs(value, 1e-12));
        CHECK_THAT(f.Jacobian(x)[0], WithinAbs(slope, 1e-12));
    }
}

TEST_CASE("DAVEMLAeroModel: unsupported MathML is reported", "[daveml_aero][mathml][negative]")
{
    using Raw = std::unordered_map<std::string, double>;

    SECTION("operator")
    {
        const auto m = loadDML("bad_op", calcVar("<apply><factorial/><ci>x</ci></apply>"));
        REQUIRE_THROWS_WITH(m.evaluateRaw(Raw{}), ContainsSubstring("unsupported operator <factorial>"));
    }
    SECTION("arithmetic with the wrong arity")
    {
        const auto m = loadDML("bad_arity", calcVar("<apply><divide/><ci>x</ci></apply>"));
        REQUIRE_THROWS_WITH(m.evaluateRaw(Raw{}), ContainsSubstring("unsupported operator <divide>"));
    }
    SECTION("csymbol")
    {
        const auto m = loadDML("bad_csymbol", calcVar("<apply><csymbol>hypot</csymbol><ci>x</ci><ci>x</ci></apply>"));
        REQUIRE_THROWS_WITH(m.evaluateRaw(Raw{}), ContainsSubstring("unsupported csymbol 'hypot'"));
    }
    SECTION("element")
    {
        const auto m = loadDML("bad_element", calcVar("<mtext>two</mtext>"));
        REQUIRE_THROWS_WITH(m.evaluateRaw(Raw{}), ContainsSubstring("unexpected element <mtext>"));
    }
    SECTION("variable")
    {
        // `ghost` is referenced but never defined anywhere in the file.
        auto evaluate = [] {
            const auto m = loadDML("bad_var", calcVar("<apply><plus/><ci>x</ci><ci>ghost</ci></apply>"));
            return m.evaluateRaw(Raw{});
        };
        REQUIRE_THROWS_WITH(evaluate(), ContainsSubstring("ghost"));
    }
}

TEST_CASE("DAVEMLAeroModel: a table referencing an undefined breakpoint set", "[daveml_aero][negative]")
{
    // Rejected when the file is loaded, not at the first evaluation (second
    // pre-DOI audit, finding N04: a table is checked against its breakpoints).
    REQUIRE_THROWS_WITH(loadDML("bad_bp", R"(
  <variableDef varID="alpha" units="deg" initialValue="0"><isInput/></variableDef>
  <variableDef varID="cz" units="nd"/>
  <function name="cz_fn">
    <independentVarRef varID="alpha"/>
    <dependentVarRef varID="cz"/>
    <functionDefn>
      <griddedTableDef>
        <breakpointRefs><bpRef bpID="nowhere"/></breakpointRefs>
        <dataTable>1, 2, 3</dataTable>
      </griddedTableDef>
    </functionDefn>
  </function>)"),
        ContainsSubstring("breakpoint 'nowhere', which is not defined"));
}

TEST_CASE("DAVEMLAeroModel: a function whose table cannot be found is rejected at load",
    "[daveml_aero][negative][regression]")
{
    // Second pre-DOI audit, finding N09: a griddedTableRef to an undefined
    // gtID was skipped, the file loaded, and y evaluated to its initialValue
    // (7) with nothing to say the table was never read.
    const std::string head = R"(
  <variableDef varID="x" units="nd" initialValue="0"><isInput/></variableDef>
  <variableDef varID="y" units="nd" initialValue="7"><isOutput/></variableDef>
  <breakpointDef bpID="XBP"><bpVals>0, 1</bpVals></breakpointDef>)";

    SECTION("griddedTableRef to an undefined griddedTableDef")
    {
        REQUIRE_THROWS_WITH(loadDML("missing_gt", head + R"(
  <function name="y_fn"><independentVarRef varID="x"/><dependentVarRef varID="y"/>
    <functionDefn><griddedTableRef gtID="missing_table"/></functionDefn></function>)"),
            ContainsSubstring("table for 'y' refers to griddedTableDef 'missing_table', which is not defined"));
    }
    SECTION("the same reference resolves when the table is defined")
    {
        const auto m = loadDML("present_gt", head + R"(
  <griddedTableDef gtID="y_table"><breakpointRefs><bpRef bpID="XBP"/></breakpointRefs>
    <dataTable>10, 20</dataTable></griddedTableDef>
  <function name="y_fn"><independentVarRef varID="x"/><dependentVarRef varID="y"/>
    <functionDefn><griddedTableRef gtID="y_table"/></functionDefn></function>)");
        CHECK_THAT(m.evaluateRaw(std::unordered_map<std::string, double>{ { "x", 0.5 } }).at("y"),
            WithinAbs(15.0, 1e-12));
    }
    SECTION("ungridded table")
    {
        REQUIRE_THROWS_WITH(loadDML("ungridded", head + R"(
  <function name="y_fn"><independentVarRef varID="x"/><dependentVarRef varID="y"/>
    <functionDefn><ungriddedTableDef><dataPoint>0 10</dataPoint></ungriddedTableDef></functionDefn></function>)"),
            ContainsSubstring("ungridded tables are not supported"));
    }
    SECTION("no table at all")
    {
        REQUIRE_THROWS_WITH(loadDML("no_table", head + R"(
  <function name="y_fn"><independentVarRef varID="x"/><dependentVarRef varID="y"/>
    <functionDefn/></function>)"),
            ContainsSubstring("has no griddedTableDef or griddedTableRef"));
    }
    SECTION("no dependentVarRef")
    {
        REQUIRE_THROWS_WITH(loadDML("no_dep", head + R"(
  <function name="y_fn"><independentVarRef varID="x"/>
    <functionDefn><griddedTableRef gtID="whatever"/></functionDefn></function>)"),
            ContainsSubstring("function 'y_fn' has no dependentVarRef"));
    }
}

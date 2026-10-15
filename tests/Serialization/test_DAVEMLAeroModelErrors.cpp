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
#include <tuple>
#include <unordered_map>
#include <vector>

using namespace Aetherion::Serialization;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

using AD = CppAD::AD<double>;

/// Writes `body` wrapped in a DAVEfunc element to a temp file named after
/// `tag`, loads it, and removes the file again.
DAVEMLAeroModel loadDML(const std::string& tag, const std::string& body)
{
    const auto path = std::filesystem::temp_directory_path() / ("_ae_test_daveml_" + tag + ".dml");
    {
        std::ofstream f(path);
        f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<DAVEfunc>\n" << body << "\n</DAVEfunc>\n";
    }
    DAVEMLAeroModel m(path.string());
    std::filesystem::remove(path);
    return m;
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
  <variableDef varID="y" units="nd"><calculation><math>)" + mathBody + R"(</math></calculation></variableDef>)";
}

} // namespace

TEST_CASE("DAVEMLAeroModel: variable limits clamp in evaluate (double)", "[daveml_aero][limits]")
{
    const auto m = loadDML("limits_d", kLimitedDML);

    DAVEMLAeroModel::Inputs<double> in;
    in.alpha_deg = 5.0;
    auto out = m.evaluate(in);
    CHECK_THAT(out.cx, WithinAbs(2.0, 1e-12));   // clamped to maxValue
    CHECK_THAT(out.cm, WithinAbs(0.5, 1e-12));   // 0.5, at the limit

    in.alpha_deg = -5.0;
    out = m.evaluate(in);
    CHECK_THAT(out.cx, WithinAbs(-1.0, 1e-12));  // clamped to minValue
    CHECK_THAT(out.cm, WithinAbs(-0.5, 1e-12));  // no lower limit

    in.alpha_deg = 1.5;
    out = m.evaluate(in);
    CHECK_THAT(out.cx, WithinAbs(1.5, 1e-12));   // inside the band
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
    const auto m = loadDML("bad_bp", R"(
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
  </function>)");

    REQUIRE_THROWS_WITH(m.evaluateRaw(std::unordered_map<std::string, double>{ { "alpha", 1.0 } }),
                        ContainsSubstring("breakpoint 'nowhere' not found"));
}

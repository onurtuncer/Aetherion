// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_DAVEMLAeroModelTapes.cpp
//
// Regression for the pre-DOI audit's finding F01 at the model level.
//
// A CppAD tape of a DAVE-ML model records one region: the <piece> taken by
// every <piecewise> and the segment of every table. Until v0.17.2 the
// decisions selecting that region were made on double values extracted with
// CppAD::Value(), which recorded nothing: a tape recorded at x = +1 of a
// model that is 10 for x < 0 and 20 otherwise returned 20 at x = -1, and the
// F-16 Cz tape recorded at alpha = 5 deg returned the wrong segment's line at
// 25 deg, with nothing to say so. The decisions are now comparisons on the AD
// values, which CppAD records, so compare_change_number() after a zero-order
// Forward says whether the tape is valid where it was just evaluated. These
// tests pin that contract: the tape is exact inside its region, and reports
// when it is used outside it.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cppad/cppad.hpp>

#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

using namespace Aetherion::Serialization;
using Catch::Matchers::WithinAbs;

#ifndef DAVEML_AERO_FILE
#define DAVEML_AERO_FILE ""
#endif

namespace {

    using AD = CppAD::AD<double>;

    const char* const kPiecewiseDML = R"(<?xml version="1.0" encoding="UTF-8"?>
<DAVEfunc>
  <variableDef varID="x" units="nd" initialValue="1.0"><isInput/></variableDef>
  <variableDef varID="y" units="nd">
    <calculation><math>
      <piecewise>
        <piece><cn>10</cn><apply><lt/><ci>x</ci><cn>0</cn></apply></piece>
        <otherwise><cn>20</cn></otherwise>
      </piecewise>
    </math></calculation>
    <isOutput/>
  </variableDef>
</DAVEfunc>)";

    // y = table(x) with breakpoints 0, 1, 2 and values 0, 1, 4: slope 1 on the
    // first segment, slope 3 on the second. extrapolate="neither" with min 0 and
    // max 2, as the F-16 deck's tables, so outside [0, 2] the input is held at
    // the edge.
    const char* const kTableDML = R"(<?xml version="1.0" encoding="UTF-8"?>
<DAVEfunc>
  <variableDef varID="x" units="nd" initialValue="0.5"><isInput/></variableDef>
  <variableDef varID="y" units="nd"><isOutput/></variableDef>
  <breakpointDef name="xbp" bpID="XBP" units="nd">
    <bpVals> 0.0, 1.0, 2.0 </bpVals>
  </breakpointDef>
  <function name="table">
    <independentVarRef varID="x" min="0.0" max="2.0" extrapolate="neither"/>
    <dependentVarRef varID="y"/>
    <functionDefn name="table_fn">
      <griddedTableDef name="table_t" gtID="table_t_def">
        <breakpointRefs><bpRef bpID="XBP"/></breakpointRefs>
        <dataTable> 0.0, 1.0, 4.0 </dataTable>
      </griddedTableDef>
    </functionDefn>
  </function>
</DAVEfunc>)";

    std::string writeDML(const char* name, const char* text)
    {
        const std::string path = std::string("_ae_tapes_") + name + ".dml";
        std::ofstream(path) << text;
        return path;
    }

    /// Tape y(x) of a single-input, single-output raw model at x0, with change
    /// tracking on (an ADFun has it off by default).
    CppAD::ADFun<double> tapeRaw(DAVEMLAeroModel& m, double x0)
    {
        std::vector<AD> ax{ AD(x0) };
        CppAD::Independent(ax);
        const auto out = m.evaluateRaw<AD>({ { "x", ax[0] } });
        std::vector<AD> ay{ out.at("y") };
        CppAD::ADFun<double> f(ax, ay);
        f.compare_change_count(1);
        return f;
    }

    double direct(DAVEMLAeroModel& m, double x)
    {
        return m.evaluateRaw<double>({ { "x", x } }).at("y");
    }

} // namespace

TEST_CASE("DAVEMLAeroModel tape: a piecewise branch is recorded and its change is reported",
    "[daveml_aero][cppad][regression]")
{
    DAVEMLAeroModel m(writeDML("piecewise", kPiecewiseDML));
    auto f = tapeRaw(m, +1.0);

    // Inside the recorded region the tape is the model.
    CHECK_THAT(f.Forward(0, std::vector<double>{ 3.0 })[0], WithinAbs(20.0, 1e-12));
    CHECK(f.compare_change_number() == 0);

    // Outside it the tape still returns the recorded branch (a tape cannot do
    // otherwise) -- but now says so. Before the fix the count was 0 here.
    CHECK_THAT(f.Forward(0, std::vector<double>{ -1.0 })[0], WithinAbs(20.0, 1e-12));
    CHECK(f.compare_change_number() > 0);
    CHECK_THAT(direct(m, -1.0), WithinAbs(10.0, 1e-12));

    // Re-recording where the evaluation is wanted gives the model again.
    auto g = tapeRaw(m, -1.0);
    CHECK_THAT(g.Forward(0, std::vector<double>{ -1.0 })[0], WithinAbs(10.0, 1e-12));
    CHECK(g.compare_change_number() == 0);
}

TEST_CASE("DAVEMLAeroModel tape: a table segment is recorded and its change is reported",
    "[daveml_aero][cppad][regression]")
{
    DAVEMLAeroModel m(writeDML("table", kTableDML));
    auto f = tapeRaw(m, 0.5);

    // Same segment: exact value and exact derivative, no change reported.
    CHECK_THAT(f.Forward(0, std::vector<double>{ 0.7 })[0], WithinAbs(0.7, 1e-12));
    CHECK(f.compare_change_number() == 0);
    CHECK_THAT(f.Jacobian(std::vector<double>{ 0.7 })[0], WithinAbs(1.0, 1e-12));

    // Other segment: the tape extrapolates the first segment's line (1.5,
    // slope 1) where the model gives 2.5 with slope 3 -- and reports it.
    CHECK_THAT(f.Forward(0, std::vector<double>{ 1.5 })[0], WithinAbs(1.5, 1e-12));
    CHECK(f.compare_change_number() > 0);
    CHECK_THAT(direct(m, 1.5), WithinAbs(2.5, 1e-12));

    auto g = tapeRaw(m, 1.5);
    CHECK_THAT(g.Forward(0, std::vector<double>{ 1.5 })[0], WithinAbs(2.5, 1e-12));
    CHECK_THAT(g.Jacobian(std::vector<double>{ 1.5 })[0], WithinAbs(3.0, 1e-12));
    CHECK(g.compare_change_number() == 0);

    // Outside the table: the independentVarRef says extrapolate="neither"
    // with min 0 and max 2, so the input is held at the edge and the model
    // gives the edge values, not the segments' lines (this test used to pin
    // the lines, -0.5 and 5.5: second pre-DOI audit, finding N03). The hold
    // is a CondExp, so a tape recorded at -0.5 -- held at 0, first segment --
    // is still the model at 0.5 and reports the change at 1.5 like any other.
    CHECK_THAT(direct(m, -0.5), WithinAbs(0.0, 1e-12));
    CHECK_THAT(direct(m, 2.5), WithinAbs(4.0, 1e-12));
    auto h = tapeRaw(m, -0.5);
    CHECK_THAT(h.Forward(0, std::vector<double>{ -0.3 })[0], WithinAbs(0.0, 1e-12));
    CHECK(h.compare_change_number() == 0);
    CHECK_THAT(h.Jacobian(std::vector<double>{ -0.3 })[0], WithinAbs(0.0, 1e-12));
    CHECK_THAT(h.Forward(0, std::vector<double>{ 0.5 })[0], WithinAbs(0.5, 1e-12));
    CHECK(h.compare_change_number() == 0); // still the first segment
    h.Forward(0, std::vector<double>{ 1.5 });
    CHECK(h.compare_change_number() > 0);
}

TEST_CASE("DAVEMLAeroModel tape: the F-16 Cz tape reports leaving its alpha segment",
    "[daveml_aero][cppad][regression]")
{
    const std::string file = DAVEML_AERO_FILE;
    if (file.empty()) return;
    DAVEMLAeroModel m(file);

    auto inputsAt = [](const AD& alpha) {
        DAVEMLAeroModel::Inputs<AD> in;
        in.vt_fps = AD(300.0);
        in.alpha_deg = alpha;
        in.beta_deg = AD(0.0);
        in.p_rps = in.q_rps = in.r_rps = AD(0.0);
        in.el_deg = in.ail_deg = in.rdr_deg = AD(0.0);
        return in;
    };
    auto directCz = [&](double alpha) {
        DAVEMLAeroModel::Inputs<double> in;
        in.vt_fps = 300.0;
        in.alpha_deg = alpha;
        in.beta_deg = 0.0;
        in.p_rps = in.q_rps = in.r_rps = 0.0;
        in.el_deg = in.ail_deg = in.rdr_deg = 0.0;
        return m.evaluate(in).cz;
    };

    std::vector<AD> ax{ AD(5.0) };
    CppAD::Independent(ax);
    const auto out = m.evaluate(inputsAt(ax[0]));
    std::vector<AD> ay{ out.cz };
    CppAD::ADFun<double> f(ax, ay);
    f.compare_change_count(1);

    // Within the 5..10 deg segment the tape is the model.
    CHECK_THAT(f.Forward(0, std::vector<double>{ 7.0 })[0], WithinAbs(directCz(7.0), 1e-12));
    CHECK(f.compare_change_number() == 0);

    // The audit's probe: at 25 deg the tape (-1.676) is not the model
    // (-1.646); the tape now reports that it has left its region.
    const double taped = f.Forward(0, std::vector<double>{ 25.0 })[0];
    CHECK(std::abs(taped - directCz(25.0)) > 1e-3);
    CHECK(f.compare_change_number() > 0);
}

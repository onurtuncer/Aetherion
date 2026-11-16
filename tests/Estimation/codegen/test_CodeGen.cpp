// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// CppAD code-generation compatibility (TODO-sensor-fusion.md, "Things to
// check"; step 8). generate_estimation_cg taped the process step and the
// three measurement models of kNavBaroGnss16 with
// CppAD::AD<CppAD::cg::CG<double>>, in both error conventions, and wrote
// plain C for each map and its Jacobian; this binary compiles that C. Here
// the generated code is checked against the double maps (values) and
// against CppAD::AD<double> tapes of the same source (Jacobians), at two
// points that between them take both sides of every CondExp on the path:
// a branch frozen into the generated code at the taping point would show
// at the other one.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <cppad/cppad.hpp>

#include "CodeGenModels.h"
#include "estimation_cg.h"

namespace CGT = Aetherion::Estimation::CodeGenTest;
using AD = CppAD::AD<double>;

namespace {
    // Generated and reference evaluate the same expressions in a different
    // order (the generator folds constants and shares subexpressions), so
    // they agree to round-off of the largest terms, not of each entry. The
    // check is per output block (x1, Phi, Qd; z, H, R): the norm of the
    // difference relative to the norm of the reference block. A block that is
    // zero by construction (the error and the innovations at delta = 0: the
    // reference is round-off, |ref| < kZero) is compared relative to the
    // inputs instead, whose round-off it is (|p| ~ 1e4 m carries 2e-12 m).
    constexpr double kTol = 1e-12;
    constexpr double kZero = 1e-9;

    /// max over output blocks of |a - ref| / |ref| (/ |x| if |ref| < kZero),
    /// for a vector of outputs (stride 1) or a row-major Jacobian (stride = n_in).
    double blockRelDiff(const CGT::ModelSpec& m,
        const std::vector<double>& x,
        const std::vector<double>& a,
        const std::vector<double>& ref,
        int stride)
    {
        double x2 = 0.0;
        for (const double xi : x)
            x2 += xi * xi;
        double worst = 0.0;
        std::size_t o = 0;
        for (const int rows : CGT::outputBlocks(m.map)) {
            const std::size_t end = o + static_cast<std::size_t>(rows) * static_cast<std::size_t>(stride);
            double d2 = 0.0, r2 = 0.0;
            for (std::size_t i = o; i < end; ++i) {
                d2 += (a[i] - ref[i]) * (a[i] - ref[i]);
                r2 += ref[i] * ref[i];
            }
            const double r = std::sqrt(r2);
            worst = std::max(worst, std::sqrt(d2) / (r < kZero ? std::sqrt(x2) : r));
            o = end;
        }
        return worst;
    }

    bool allFinite(const std::vector<double>& v)
    {
        return std::all_of(v.begin(), v.end(), [](double d) { return std::isfinite(d); });
    }

    const aeth_cg_model* generated(const CGT::ModelSpec& m)
    {
        for (const aeth_cg_model& g : aeth_cg_models)
            if (m.name == g.name)
                return &g;
        return nullptr;
    }

    /// Row-major Jacobian of model m at x by a CppAD::AD<double> tape.
    std::vector<double> referenceJacobian(const CGT::ModelSpec& m, const std::vector<double>& x)
    {
        std::vector<AD> ax(x.begin(), x.end());
        CppAD::Independent(ax);
        const std::vector<AD> ay = CGT::evaluate<AD>(m, CGT::castFrame<AD>(CGT::testFrame()), ax);
        CppAD::ADFun<double> f(ax, ay);
        return f.Jacobian(x);
    }

    struct Result
    {
        double value;
        double jacobian;
    };

    Result compare(const CGT::ModelSpec& m, const CGT::Point& p)
    {
        const aeth_cg_model* g = generated(m);
        REQUIRE(g != nullptr);
        REQUIRE(g->n_in == m.nIn);
        REQUIRE(g->n_out == m.nOut);

        const std::vector<double> x = CGT::inputs(m, p);
        const std::vector<double> yRef = CGT::evaluate<double>(m, CGT::testFrame(), x);
        const std::vector<double> jRef = referenceJacobian(m, x);
        REQUIRE(jRef.size() == static_cast<std::size_t>(m.nIn) * static_cast<std::size_t>(m.nOut));

        std::vector<double> y(yRef.size()), j(jRef.size());
        g->value(x.data(), y.data());
        g->jacobian(x.data(), j.data());
        REQUIRE(allFinite(y));
        REQUIRE(allFinite(j));
        return { blockRelDiff(m, x, y, yRef, 1), blockRelDiff(m, x, j, jRef, m.nIn) };
    }
} // namespace

TEST_CASE("CodeGen: generated C for kNavBaroGnss16 matches double values and AD<double> Jacobians",
    "[estimation][codegen]")
{
    REQUIRE(AETH_CG_MODEL_COUNT == CGT::kModels.size());
    const CGT::Point a = CGT::pointA();
    const CGT::Point b = CGT::pointB();
    for (const CGT::ModelSpec& m : CGT::kModels) {
        const Result ra = compare(m, a);
        const Result rb = compare(m, b);
        INFO(std::string(m.name) << ": point A value " << ra.value << ", Jacobian " << ra.jacobian << "; point B value "
                                 << rb.value << ", Jacobian " << rb.jacobian);
        CHECK(ra.value < kTol);
        CHECK(ra.jacobian < kTol);
        CHECK(rb.value < kTol);
        CHECK(rb.jacobian < kTol);
    }
}

// The generated Jacobians are the ones the filter needs: d/d delta at
// delta = 0 of the process error map is Phi, of an innovation is H. They
// match the analytic step<C>() and the models' H to the accuracy the
// existing double/AD<double> tests established (Phi to discretisation order,
// H to round-off).
TEST_CASE("CodeGen: generated Jacobians reproduce Phi and H at delta = 0", "[estimation][codegen]")
{
    const CGT::Point a = CGT::pointA();
    for (const CGT::ModelSpec& m : CGT::kModels) {
        if (m.map == CGT::Map::ProcessStep)
            continue;
        const aeth_cg_model* g = generated(m);
        REQUIRE(g != nullptr);
        const std::vector<double> x = CGT::inputs(m, a);
        std::vector<double> j(static_cast<std::size_t>(m.nIn) * static_cast<std::size_t>(m.nOut));
        g->jacobian(x.data(), j.data());

        // Columns 0..15 of the Jacobian are d/d delta.
        const int rows = m.map == CGT::Map::ProcessError ? CGT::kN : CGT::measurementDim(m.map);
        std::vector<double> dDelta;
        for (int c = 0; c < CGT::kN; ++c)
            for (int r = 0; r < rows; ++r)
                dDelta.push_back(j[static_cast<std::size_t>(r * m.nIn + c)]); // column-major, like the models

        std::vector<double> analytic;
        double tol = 0.0;
        if (m.map == CGT::Map::ProcessError) {
            const CGT::ModelSpec& step = CGT::kModels[m.convention == CGT::kRI ? 2 : 3];
            const std::vector<double> xs = CGT::inputs(step, a);
            const std::vector<double> ys = CGT::evaluate<double>(step, CGT::testFrame(), xs);
            analytic.assign(ys.begin() + CGT::kState, ys.begin() + CGT::kState + CGT::kN * CGT::kN); // Phi
            tol = 1e-6;                                                                              // as test_ProcessModel.cpp: analytic vs linearised step agree to O(dt^3)
        } else {
            const std::vector<double> y = CGT::evaluate<double>(m, CGT::testFrame(), x);
            const int k = CGT::measurementDim(m.map);
            analytic.assign(y.begin() + k, y.begin() + k + k * CGT::kN); // H
            tol = 1e-9;
        }
        double d2 = 0.0, r2 = 0.0;
        for (std::size_t i = 0; i < analytic.size(); ++i) {
            d2 += (dDelta[i] - analytic[i]) * (dDelta[i] - analytic[i]);
            r2 += analytic[i] * analytic[i];
        }
        const double e = std::sqrt(d2 / r2);
        INFO(std::string(m.name) << ": generated d/d delta vs analytic, relative " << e);
        CHECK(e < tol);
    }
}

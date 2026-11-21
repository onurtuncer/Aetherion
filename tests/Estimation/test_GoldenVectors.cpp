// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Golden vectors of the reference filter (TODO-sensor-fusion.md, step 8;
// doc/codegen.rst). The files in data/Estimation/GoldenVectors/ are what
// Hemerion replays against its embedded filter, so they must stay what the
// reference filter computes: this test reads each committed file, runs
// InvariantEKF<double, C> over the file's own inputs and requires the file's
// outputs within the file's tolerances. It needs no generated code and runs
// in every build; test_EkfCodeGen.cpp replays the same files through the
// generated C.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>

#include "golden/GoldenVectors.h"

namespace Est = Aetherion::Estimation;
namespace Golden = Aetherion::Estimation::Golden;
using Est::ErrorConvention;

namespace {
    const Est::Configuration& kCfg = Est::kNavBaroGnss16;

    Golden::File load(ErrorConvention c)
    {
        const std::string path =
            std::string(AETHERION_GOLDEN_VECTORS_DIR) + "/nav-baro-gnss16_" + std::string(Est::toString(c)) + ".txt";
        std::ifstream in(path);
        INFO(path);
        REQUIRE(in.good());
        return Golden::read(in);
    }

    Golden::File reference(ErrorConvention c, const Golden::File& f)
    {
        return c == ErrorConvention::RightInvariant ? Golden::runReference<ErrorConvention::RightInvariant>(kCfg, f)
                                                    : Golden::runReference<ErrorConvention::LeftInvariant>(kCfg, f);
    }

    std::size_t count(const Golden::File& f, std::string_view kind)
    {
        std::size_t n = 0;
        for (const Golden::Record& r : f.records) n += r.kind == kind ? 1 : 0;
        return n;
    }
} // namespace

TEST_CASE("GoldenVectors: the committed files carry the configuration and its hash", "[estimation][golden]")
{
    for (const ErrorConvention c : { ErrorConvention::RightInvariant, ErrorConvention::LeftInvariant }) {
        const Golden::File f = load(c);
        CHECK(f.configuration == "nav-baro-gnss16");
        CHECK(f.convention == Est::toString(c));
        CHECK(f.hash == kCfg.hash(c));
        CHECK(f.errorDim == kCfg.dimension());
        CHECK(f.noiseDim == Est::noiseDimension(kCfg));
        REQUIRE(f.tolerances.count("float64") == 1);
        for (const std::string_view q : Golden::kQuantities)
            CHECK(f.tolerances.at("float64").count(std::string(q)) == 1);
        // Every call of the filter interface is in the sequence, including
        // predicts over part of an IMU interval.
        CHECK(count(f, "predict") > 0);
        CHECK(count(f, "gnss_position") > 0);
        CHECK(count(f, "gnss_velocity") > 0);
        CHECK(count(f, "barometer") > 0);
        std::size_t partial = 0;
        for (const Golden::Record& r : f.records) partial += (r.kind == "predict" && r.input[6] < 0.009) ? 1 : 0;
        CHECK(partial == 2 * count(f, "gnss_position"));
    }
    // The default convention's hash is the one test_Configuration.cpp pins.
    CHECK(load(Est::kErrorConvention).hash == kCfg.hash());
}

TEST_CASE("GoldenVectors: the reference filter reproduces the committed outputs", "[estimation][golden]")
{
    for (const ErrorConvention c : { ErrorConvention::RightInvariant, ErrorConvention::LeftInvariant }) {
        const Golden::File golden = load(c);
        const Golden::Comparison result = Golden::compare(golden, reference(c, golden));
        INFO(Est::toString(c) << ": " << result.summary());
        CHECK(result.passed());
    }
}

TEST_CASE("GoldenVectors: the comparison fails on a wrong filter or a wrong input", "[estimation][golden]")
{
    const Golden::File golden = load(ErrorConvention::RightInvariant);

    // The other convention is a different filter on the same inputs.
    const Golden::Comparison other = Golden::compare(golden, reference(ErrorConvention::LeftInvariant, golden));
    INFO("left-invariant filter on right-invariant vectors: " << other.summary());
    CHECK_FALSE(other.passed());

    // One gyro reading of the first predict off by 1e-6 rad/s.
    Golden::File perturbed = golden;
    perturbed.records.front().input[0] += 1e-6;
    const Golden::Comparison gyro = Golden::compare(golden, reference(ErrorConvention::RightInvariant, perturbed));
    INFO("gyro + 1e-6 rad/s: " << gyro.summary());
    CHECK_FALSE(gyro.passed());

    // One barometer reading off by a micrometre.
    perturbed = golden;
    for (Golden::Record& r : perturbed.records)
        if (r.kind == "barometer") {
            r.input[0] += 1e-6;
            break;
        }
    CHECK_FALSE(Golden::compare(golden, reference(ErrorConvention::RightInvariant, perturbed)).passed());

    // A deviation that is not a number fails; it does not compare false.
    Golden::File broken = golden;
    broken.records.back().covariance[0] = std::nan("");
    CHECK_FALSE(Golden::compare(golden, broken).passed());
}

TEST_CASE("GoldenVectors: writing and reading a file is exact", "[estimation][golden]")
{
    const Golden::File golden = load(ErrorConvention::LeftInvariant);
    std::stringstream s;
    Golden::write(s, golden);
    const Golden::File again = Golden::read(s);
    REQUIRE(again.records.size() == golden.records.size());
    CHECK(again.hash == golden.hash);
    CHECK(again.frame == golden.frame);
    CHECK(again.qcDiag == golden.qcDiag);
    CHECK(again.state0 == golden.state0);
    CHECK(again.covariance0 == golden.covariance0);
    bool same = true;
    for (std::size_t k = 0; k < golden.records.size(); ++k) {
        const Golden::Record& a = again.records[k];
        const Golden::Record& b = golden.records[k];
        same = same && a.kind == b.kind && a.input == b.input && a.nis == b.nis && a.z == b.z && a.state == b.state &&
               a.covariance == b.covariance;
    }
    CHECK(same);

    // A file whose qc_diag the reference filter cannot be given is refused.
    Golden::File bad = golden;
    bad.qcDiag[1] *= 2.0;
    CHECK_THROWS(reference(ErrorConvention::LeftInvariant, bad));
}

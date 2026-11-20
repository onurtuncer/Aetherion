// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// The step-8 generator tool (TODO-sensor-fusion.md, step 8; doc/codegen.rst).
// generate_ekf_c wrote aeth_ekf_ri.{c,h} and aeth_ekf_li.{c,h} for
// kNavBaroGnss16 into the build tree; this binary compiles that C. Checked:
//
//   - the generated header carries the configuration's dimension, block
//     offsets and hash, and the layout of EkfMaps.h;
//   - every generated function reproduces the double reference (the filter's
//     own model code) at two points that between them take both sides of
//     every CondExp, in a frame other than the one the code was taped in;
//   - the generated Jacobians (Phi, H) agree with CppAD::AD<double>
//     derivatives of the nonlinear maps;
//   - a filter built from the generated functions alone, with its own
//     covariance algebra, replays the golden vectors within their stated
//     tolerances, and fails them when it should;
//   - the files carry the caller's banner and the provenance lines.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

#include <cppad/cppad.hpp>

#include <Eigen/Dense>

#include "../golden/GoldenVectors.h"
#include "CodeGenModels.h"
#include "EkfMaps.h"
#include "aeth_ekf_li.h"
#include "aeth_ekf_ri.h"

namespace Est = Aetherion::Estimation;
namespace CG = Aetherion::Estimation::CodeGen;
namespace CGT = Aetherion::Estimation::CodeGenTest;
namespace Golden = Aetherion::Estimation::Golden;
using Est::ErrorConvention;

namespace {
    const Est::Configuration& kCfg = Est::kNavBaroGnss16;

    using Fn = void (*)(const double*, double*);

    /// The generated functions of one convention.
    struct Api
    {
        ErrorConvention convention;
        std::string stem;
        Fn propagate, transition, gnssPosition, gnssVelocity, barometer, retract;

        [[nodiscard]] Fn function(CG::Function f) const
        {
            switch (f) {
                case CG::Function::Propagate:
                    return propagate;
                case CG::Function::Transition:
                    return transition;
                case CG::Function::GnssPosition:
                    return gnssPosition;
                case CG::Function::GnssVelocity:
                    return gnssVelocity;
                case CG::Function::Barometer:
                    return barometer;
                default:
                    return retract;
            }
        }

        [[nodiscard]] std::vector<double> call(CG::Function f, const std::vector<double>& in) const
        {
            REQUIRE(static_cast<int>(in.size()) == CG::totalSize(CG::inputSegments(kCfg, f)));
            std::vector<double> out(static_cast<std::size_t>(CG::totalSize(CG::outputSegments(kCfg, f))));
            function(f)(in.data(), out.data());
            return out;
        }
    };

    const Api kRi{ ErrorConvention::RightInvariant, "aeth_ekf_ri", aeth_ekf_ri_propagate, aeth_ekf_ri_transition,
        aeth_ekf_ri_gnss_position, aeth_ekf_ri_gnss_velocity, aeth_ekf_ri_barometer, aeth_ekf_ri_retract };
    const Api kLi{ ErrorConvention::LeftInvariant, "aeth_ekf_li", aeth_ekf_li_propagate, aeth_ekf_li_transition,
        aeth_ekf_li_gnss_position, aeth_ekf_li_gnss_velocity, aeth_ekf_li_barometer, aeth_ekf_li_retract };

    /// The inputs of function f at a point of the compatibility fixture, in the
    /// given frame. A measurement is the one predicted at retract(x, delta), so
    /// the innovation is zero at point A (delta = 0) and large at point B.
    std::vector<double> inputsAt(
        const Api& api, CG::Function f, const CGT::Point& p, const Est::LciFrame<double>& frame)
    {
        Eigen::VectorXd delta(kCfg.dimension());
        for (int i = 0; i < kCfg.dimension(); ++i) delta[i] = p.delta[static_cast<std::size_t>(i)];
        const Est::EstimationState<double> displaced = api.convention == ErrorConvention::RightInvariant
                                                           ? p.x.retract<ErrorConvention::RightInvariant>(kCfg, delta)
                                                           : p.x.retract<ErrorConvention::LeftInvariant>(kCfg, delta);
        const Est::MeasurementModels<double> models(frame);
        Eigen::Vector3d y = displaced.X.p;
        if (f == CG::Function::GnssVelocity) y = displaced.X.v;

        std::vector<double> in;
        for (const CG::Segment& s : CG::inputSegments(kCfg, f)) {
            if (s.name == "state")
                CG::appendState(p.x, in);
            else if (s.name == "frame") {
                const std::vector<double> packed = CG::packFrame(frame);
                in.insert(in.end(), packed.begin(), packed.end());
            } else if (s.name == "gyro")
                in.insert(in.end(), { p.gyro[0], p.gyro[1], p.gyro[2] });
            else if (s.name == "accel")
                in.insert(in.end(), { p.accel[0], p.accel[1], p.accel[2] });
            else if (s.name == "dt")
                in.push_back(p.dt);
            else if (s.name == "qc_diag")
                in.insert(in.end(), p.qcDiag.begin(), p.qcDiag.end());
            else if (s.name == "delta")
                in.insert(in.end(), p.delta.begin(), p.delta.end());
            else if (s.name == "meas_cov")
                in.insert(in.end(), { 2.25, 0.3, -0.1, 0.3, 2.5, 0.2, -0.1, 0.2, 9.0 });
            else if (s.name == "meas_sigma")
                in.push_back(2.0);
            else if (s.name == "y" && s.size == 1)
                in.push_back(models.predictBarometer(displaced));
            else if (s.name == "y")
                in.insert(in.end(), { y[0], y[1], y[2] });
        }
        return in;
    }

    double norm(const double* v, int size)
    {
        double s = 0.0;
        for (int i = 0; i < size; ++i) s += v[i] * v[i];
        return std::sqrt(s);
    }

    // As in test_CodeGen.cpp: generated and reference evaluate the same
    // expressions in a different order, so they agree to round-off of the
    // largest terms. Per output segment, |a - ref| / |ref|; a segment that is
    // zero by construction (|ref| < kZero) is taken relative to the inputs,
    // whose round-off it is. The innovation z = y - h(x) is always taken
    // relative to max(|z|, |inputs|): it is a difference of terms the size of
    // the inputs, and the barometer's h(x) goes through the ECI position
    // (~6.4e6 m). Measured: the barometer innovations differ by 9.3e-10 m,
    // one ulp of 6.4e6, which is 4.5e-11 of a 20.8 m innovation.
    constexpr double kTol = 1e-12;
    constexpr double kZero = 1e-9;

    double worstSegmentDifference(
        CG::Function f, const std::vector<double>& in, const std::vector<double>& a, const std::vector<double>& ref)
    {
        const double inputs = norm(in.data(), static_cast<int>(in.size()));
        double worst = 0.0;
        int o = 0;
        for (const CG::Segment& s : CG::outputSegments(kCfg, f)) {
            std::vector<double> d(static_cast<std::size_t>(s.size));
            for (int i = 0; i < s.size; ++i)
                d[static_cast<std::size_t>(i)] =
                    a[static_cast<std::size_t>(o + i)] - ref[static_cast<std::size_t>(o + i)];
            double r = norm(ref.data() + o, s.size);
            if (r < kZero || s.name == "z") r = std::max(r, inputs);
            worst = std::max(worst, norm(d.data(), s.size) / r);
            o += s.size;
        }
        return worst;
    }

    /// A frame that is not the one the code was taped in.
    Est::LciFrame<double> otherFrame()
    {
        constexpr double deg = std::numbers::pi / 180.0;
        return Est::LciFrame<double>::FromGeodetic(-33.9 * deg, 151.2 * deg, 40.0, 4.1);
    }

    std::string slurp(const std::string& path)
    {
        std::ifstream in(path, std::ios::binary);
        REQUIRE(in.good());
        std::ostringstream s;
        s << in.rdbuf();
        return s.str();
    }

    Golden::File loadGolden(ErrorConvention c)
    {
        const std::string path =
            std::string(AETHERION_GOLDEN_VECTORS_DIR) + "/nav-baro-gnss16_" + std::string(Est::toString(c)) + ".txt";
        std::ifstream in(path);
        REQUIRE(in.good());
        return Golden::read(in);
    }

    using RowMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    /// A filter made of the generated functions and nothing from the reference
    /// filter: what a consumer of the generated code writes. The covariance
    /// algebra is deliberately not the reference's (explicit inverse of the
    /// innovation covariance instead of an LDLT solve). `dtScale` perturbs
    /// every predict, to show that the comparison can fail.
    Golden::File runGenerated(const Api& api, Golden::File file, double dtScale = 1.0)
    {
        const int n = kCfg.dimension();
        const auto seg = [](CG::Function f, std::string_view name) {
            return static_cast<std::size_t>(CG::offsetOf(CG::outputSegments(kCfg, f), name));
        };
        std::vector<double> x = file.state0;
        Eigen::MatrixXd P = Golden::unpackUpper(file.covariance0, n);

        for (Golden::Record& r : file.records) {
            if (r.kind == "predict") {
                std::vector<double> in = x;
                in.insert(in.end(), r.input.begin(), r.input.end());
                in.back() *= dtScale;
                in.insert(in.end(), file.frame.begin(), file.frame.end());
                const std::vector<double> x1 = api.call(CG::Function::Propagate, in);
                in.insert(in.end(), file.qcDiag.begin(), file.qcDiag.end());
                const std::vector<double> t = api.call(CG::Function::Transition, in);
                const Eigen::Map<const RowMatrix> Phi(t.data() + seg(CG::Function::Transition, "phi"), n, n);
                const Eigen::Map<const RowMatrix> Qd(t.data() + seg(CG::Function::Transition, "qd"), n, n);
                P = Phi * P * Phi.transpose() + Qd;
                x = x1;
            } else {
                const CG::Function f =
                    r.kind == "gnss_position"
                        ? CG::Function::GnssPosition
                        : (r.kind == "gnss_velocity" ? CG::Function::GnssVelocity : CG::Function::Barometer);
                const int m = Golden::measurementDim(r.kind);
                std::vector<double> in = x;
                in.insert(in.end(), r.input.begin(), r.input.end());
                if (f == CG::Function::Barometer) in.insert(in.end(), file.frame.begin(), file.frame.end());
                const std::vector<double> out = api.call(f, in);
                const Eigen::Map<const Eigen::VectorXd> z(out.data() + seg(f, "z"), m);
                const Eigen::Map<const RowMatrix> H(out.data() + seg(f, "h"), m, n);
                const Eigen::Map<const RowMatrix> R(out.data() + seg(f, "r"), m, m);
                const Eigen::MatrixXd Sinv = (H * P * H.transpose() + R).inverse();
                const Eigen::MatrixXd K = P * H.transpose() * Sinv;
                const Eigen::VectorXd delta = K * z;
                std::vector<double> rin = x;
                rin.insert(rin.end(), delta.data(), delta.data() + n);
                x = api.call(CG::Function::Retract, rin);
                const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(n, n) - K * H;
                P = IKH * P * IKH.transpose() + K * R * K.transpose();
                r.nis = z.dot(Sinv * z);
                r.z.assign(z.data(), z.data() + m);
            }
            P = (0.5 * (P + P.transpose())).eval();
            r.state = x;
            r.covariance = Golden::packUpper(P);
        }
        return file;
    }

    /// Jacobian with respect to delta (the first kN inputs) of a fixture map at
    /// point p by a CppAD::AD<double> tape, row-major (rows x kN).
    std::vector<double> adJacobianInDelta(const CGT::ModelSpec& m, const CGT::Point& p, int rows)
    {
        using AD = CppAD::AD<double>;
        const std::vector<double> x = CGT::inputs(m, p);
        std::vector<AD> ax(x.begin(), x.end());
        CppAD::Independent(ax);
        const std::vector<AD> ay = CGT::evaluate<AD>(m, CGT::castFrame<AD>(CGT::testFrame()), ax);
        CppAD::ADFun<double> fun(ax, ay);
        const std::vector<double> j = fun.Jacobian(x);
        std::vector<double> d;
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < CGT::kN; ++c) d.push_back(j[static_cast<std::size_t>(r * m.nIn + c)]);
        return d;
    }

    const CGT::ModelSpec& fixtureModel(CGT::Map map, ErrorConvention c)
    {
        for (const CGT::ModelSpec& m : CGT::kModels)
            if (m.map == map && m.convention == c) return m;
        FAIL("no such fixture model");
        return CGT::kModels[0];
    }
} // namespace

TEST_CASE("EkfCodeGen: the generated header carries dimension, block offsets and hash", "[estimation][codegen]")
{
    // Read from the configuration, never from a literal layout.
    STATIC_REQUIRE(AETH_EKF_RI_ERROR_DIM == Est::kNavBaroGnss16.dimension());
    STATIC_REQUIRE(AETH_EKF_LI_ERROR_DIM == Est::kNavBaroGnss16.dimension());
    STATIC_REQUIRE(AETH_EKF_RI_NOISE_DIM == Est::noiseDimension(Est::kNavBaroGnss16));
    STATIC_REQUIRE(AETH_EKF_RI_BLOCK_NAV_CORE_OFFSET == Est::kNavBaroGnss16.offset(Est::BlockId::NavCore));
    STATIC_REQUIRE(AETH_EKF_RI_BLOCK_GYRO_BIAS_OFFSET == Est::kNavBaroGnss16.offset(Est::BlockId::GyroBias));
    STATIC_REQUIRE(AETH_EKF_RI_BLOCK_ACCEL_BIAS_OFFSET == Est::kNavBaroGnss16.offset(Est::BlockId::AccelBias));
    STATIC_REQUIRE(AETH_EKF_RI_BLOCK_BARO_BIAS_OFFSET == Est::kNavBaroGnss16.offset(Est::BlockId::BaroBias));
    STATIC_REQUIRE(AETH_EKF_LI_BLOCK_BARO_BIAS_OFFSET == Est::kNavBaroGnss16.offset(Est::BlockId::BaroBias));
    STATIC_REQUIRE(AETH_EKF_RI_BLOCK_NAV_CORE_DIM == Est::blockSpec(Est::BlockId::NavCore).dim);
    STATIC_REQUIRE(AETH_EKF_RI_BLOCK_BARO_BIAS_DIM == Est::blockSpec(Est::BlockId::BaroBias).dim);
    STATIC_REQUIRE(AETH_EKF_RI_ERROR_VELOCITY ==
                   Est::kNavBaroGnss16.offset(Est::BlockId::NavCore) + Est::NavCoreLayout::kVelocity);
    STATIC_REQUIRE(AETH_EKF_RI_ERROR_POSITION ==
                   Est::kNavBaroGnss16.offset(Est::BlockId::NavCore) + Est::NavCoreLayout::kPosition);
    STATIC_REQUIRE(
        AETH_EKF_RI_NOISE_BARO_BIAS_WALK_OFFSET == Est::noiseOffset(Est::kNavBaroGnss16, Est::BlockId::BaroBias));
    STATIC_REQUIRE(AETH_EKF_RI_STATE_DIM == CG::StateLayout::kDim);
    STATIC_REQUIRE(AETH_EKF_RI_STATE_BARO_BIAS == CG::StateLayout::kBaroBias);
    STATIC_REQUIRE(AETH_EKF_RI_FRAME_DIM == CG::FrameLayout::kDim);
    STATIC_REQUIRE(AETH_EKF_RI_ERROR_CONVENTION_RIGHT_INVARIANT == 1);
    STATIC_REQUIRE(AETH_EKF_LI_ERROR_CONVENTION_RIGHT_INVARIANT == 0);

    CHECK(AETH_EKF_RI_CONFIG_HASH == kCfg.hash(ErrorConvention::RightInvariant));
    CHECK(AETH_EKF_LI_CONFIG_HASH == kCfg.hash(ErrorConvention::LeftInvariant));
    CHECK(AETH_EKF_RI_CONFIG_HASH != AETH_EKF_LI_CONFIG_HASH);
    CHECK(std::string(AETH_EKF_RI_CONFIG_NAME) == "nav-baro-gnss16");
    CHECK(std::string(AETH_EKF_RI_ERROR_CONVENTION) == Est::toString(ErrorConvention::RightInvariant));
    CHECK(std::string(AETH_EKF_LI_ERROR_CONVENTION) == Est::toString(ErrorConvention::LeftInvariant));
    CHECK(std::stoull(AETH_EKF_RI_CONFIG_HASH_STRING, nullptr, 16) == AETH_EKF_RI_CONFIG_HASH);

    // Function layouts: the macros are the offsets of EkfMaps.h.
    const auto in = [](CG::Function f, std::string_view s) { return CG::offsetOf(CG::inputSegments(kCfg, f), s); };
    const auto out = [](CG::Function f, std::string_view s) { return CG::offsetOf(CG::outputSegments(kCfg, f), s); };
    CHECK(AETH_EKF_RI_PROPAGATE_N_IN == CG::totalSize(CG::inputSegments(kCfg, CG::Function::Propagate)));
    CHECK(AETH_EKF_RI_PROPAGATE_N_OUT == CG::totalSize(CG::outputSegments(kCfg, CG::Function::Propagate)));
    CHECK(AETH_EKF_RI_PROPAGATE_IN_FRAME == in(CG::Function::Propagate, "frame"));
    CHECK(AETH_EKF_RI_TRANSITION_N_IN == CG::totalSize(CG::inputSegments(kCfg, CG::Function::Transition)));
    CHECK(AETH_EKF_RI_TRANSITION_N_OUT == 2 * kCfg.dimension() * kCfg.dimension());
    CHECK(AETH_EKF_RI_TRANSITION_IN_QC_DIAG == in(CG::Function::Transition, "qc_diag"));
    CHECK(AETH_EKF_RI_TRANSITION_OUT_QD == out(CG::Function::Transition, "qd"));
    CHECK(AETH_EKF_RI_GNSS_POSITION_IN_MEAS_COV == in(CG::Function::GnssPosition, "meas_cov"));
    CHECK(AETH_EKF_RI_GNSS_POSITION_OUT_H == out(CG::Function::GnssPosition, "h"));
    CHECK(AETH_EKF_RI_GNSS_VELOCITY_OUT_R == out(CG::Function::GnssVelocity, "r"));
    CHECK(AETH_EKF_RI_BAROMETER_IN_FRAME == in(CG::Function::Barometer, "frame"));
    CHECK(AETH_EKF_RI_BAROMETER_N_OUT == CG::totalSize(CG::outputSegments(kCfg, CG::Function::Barometer)));
    CHECK(AETH_EKF_RI_RETRACT_IN_DELTA == in(CG::Function::Retract, "delta"));
    CHECK(AETH_EKF_LI_TRANSITION_N_IN == AETH_EKF_RI_TRANSITION_N_IN);
    CHECK(AETH_EKF_RI_TRANSITION_N_TMP > 0);
}

TEST_CASE("EkfCodeGen: every generated function reproduces the double reference", "[estimation][codegen]")
{
    const std::vector<CGT::Point> points{ CGT::pointA(), CGT::pointB() };
    const std::vector<Est::LciFrame<double>> frames{ CGT::testFrame(), otherFrame() };
    for (const Api* api : { &kRi, &kLi })
        for (const CG::Function f : CG::kFunctions) {
            double worst = 0.0;
            for (const CGT::Point& p : points)
                for (const Est::LciFrame<double>& frame : frames) {
                    const std::vector<double> in = inputsAt(*api, f, p, frame);
                    const std::vector<double> ref = CG::evaluate<double>(kCfg, api->convention, f, in);
                    const std::vector<double> y = api->call(f, in);
                    REQUIRE(std::all_of(y.begin(), y.end(), [](double d) { return std::isfinite(d); }));
                    worst = std::max(worst, worstSegmentDifference(f, in, y, ref));
                }
            INFO(api->stem << "_" << CG::name(f) << ": worst relative difference per output segment " << worst);
            CHECK(worst < kTol);
        }
}

TEST_CASE("EkfCodeGen: a perturbed generated output fails the value check", "[estimation][codegen]")
{
    // The check above can fail: one entry of Phi off by a relative 1e-10.
    const CG::Function f = CG::Function::Transition;
    const std::vector<double> in = inputsAt(kRi, f, CGT::pointA(), otherFrame());
    const std::vector<double> ref = CG::evaluate<double>(kCfg, kRi.convention, f, in);
    std::vector<double> y = kRi.call(f, in);
    REQUIRE(worstSegmentDifference(f, in, y, ref) < kTol);
    const auto largest = std::max_element(y.begin(), y.begin() + kCfg.dimension() * kCfg.dimension(),
        [](double a, double b) { return std::abs(a) < std::abs(b); });
    *largest *= 1.0 + 1e-10;
    CHECK(worstSegmentDifference(f, in, y, ref) > kTol);
}

// The generated Jacobians against CppAD: d/d delta at delta = 0 of the
// nonlinear error map is Phi (to the discretisation order test_ProcessModel.cpp
// established), of the innovation fed the measurement predicted at
// retract(x, delta) is H (to round-off). The AD<double> tapes are those of the
// compatibility fixture, in its frame.
TEST_CASE("EkfCodeGen: generated Phi and H agree with CppAD::AD<double> derivatives", "[estimation][codegen]")
{
    const CGT::Point a = CGT::pointA();
    const int n = kCfg.dimension();
    const auto relative = [](const double* generated, const std::vector<double>& ad) {
        std::vector<double> d(ad.size());
        for (std::size_t i = 0; i < ad.size(); ++i) d[i] = generated[i] - ad[i];
        return norm(d.data(), static_cast<int>(d.size())) / norm(ad.data(), static_cast<int>(ad.size()));
    };
    for (const Api* api : { &kRi, &kLi }) {
        {
            const std::vector<double> t =
                api->call(CG::Function::Transition, inputsAt(*api, CG::Function::Transition, a, CGT::testFrame()));
            const double e =
                relative(t.data(), adJacobianInDelta(fixtureModel(CGT::Map::ProcessError, api->convention), a, n));
            INFO(api->stem << ": generated Phi vs AD<double> d(error)/d(delta), relative " << e);
            CHECK(e < 1e-6);
        }
        const std::vector<std::pair<CG::Function, CGT::Map>> measurements{ { CG::Function::GnssPosition,
                                                                               CGT::Map::GnssPosition },
            { CG::Function::GnssVelocity, CGT::Map::GnssVelocity }, { CG::Function::Barometer, CGT::Map::Barometer } };
        for (const auto& [f, map] : measurements) {
            const std::vector<double> out = api->call(f, inputsAt(*api, f, a, CGT::testFrame()));
            const int m = CGT::measurementDim(map);
            const double e = relative(out.data() + CG::offsetOf(CG::outputSegments(kCfg, f), "h"),
                adJacobianInDelta(fixtureModel(map, api->convention), a, m));
            INFO(api->stem << "_" << CG::name(f) << ": generated H vs AD<double> dz/d(delta), relative " << e);
            CHECK(e < 1e-9);
        }
    }
}

TEST_CASE("EkfCodeGen: a filter built from the generated C replays the golden vectors", "[estimation][codegen][golden]")
{
    for (const Api* api : { &kRi, &kLi }) {
        const Golden::File golden = loadGolden(api->convention);
        // The contract: refuse a file whose hash is not the generated code's.
        REQUIRE(golden.hash == (api == &kRi ? AETH_EKF_RI_CONFIG_HASH : AETH_EKF_LI_CONFIG_HASH));
        REQUIRE(golden.errorDim == AETH_EKF_RI_ERROR_DIM);

        const Golden::Comparison c = Golden::compare(golden, runGenerated(*api, golden));
        INFO(api->stem << ": " << c.summary());
        CHECK(c.passed());
        for (const std::string_view q : Golden::kQuantities) CHECK(c.worst.count(std::string(q)) == 1);

        // The comparison can fail: every dt longer by a relative 1e-6 ...
        const Golden::Comparison slow = Golden::compare(golden, runGenerated(*api, golden, 1.0 + 1e-6));
        INFO(api->stem << ", dt * (1 + 1e-6): " << slow.summary());
        CHECK_FALSE(slow.passed());
        // ... or the other convention's code on this convention's vectors.
        const Golden::Comparison other = Golden::compare(golden, runGenerated(api == &kRi ? kLi : kRi, golden));
        CHECK_FALSE(other.passed());
    }
}

TEST_CASE("EkfCodeGen: the generated files carry the banner and the provenance lines", "[estimation][codegen]")
{
    const std::string banner = slurp(AETHERION_EKF_TEST_BANNER);
    const std::string marker = std::string("AETHERION-GENERATED-") + "ESTIMATOR-CODE";
    for (const Api* api : { &kRi, &kLi })
        for (const char* extension : { ".c", ".h" }) {
            const std::string text = slurp(std::string(AETHERION_EKF_GENERATED_DIR) + "/" + api->stem + extension);
            INFO(api->stem << extension);
            CHECK(text.rfind("/*\n", 0) == 0);
            std::istringstream lines(banner);
            for (std::string line; std::getline(lines, line);) {
                if (!line.empty() && line.back() == '\r') line.pop_back();
                if (!line.empty()) CHECK(text.find(" * " + line + "\n") != std::string::npos);
            }
            CHECK(text.find(marker) != std::string::npos);
            CHECK(text.find("generate_ekf_c, commit " AETHERION_EKF_TEST_COMMIT "\n") != std::string::npos);
            CHECK(text.find("CppADCodeGen: " AETHERION_CPPADCG_VERSION "\n") != std::string::npos);
            CHECK(text.find(api == &kRi ? AETH_EKF_RI_CONFIG_HASH_STRING : AETH_EKF_LI_CONFIG_HASH_STRING) !=
                  std::string::npos);
            // The test banner's licence line is the only SPDX identifier: the
            // tool adds none of its own.
            CHECK(text.find("SPDX-License-Identifier", text.find("SPDX-License-Identifier") + 1) == std::string::npos);
        }
}

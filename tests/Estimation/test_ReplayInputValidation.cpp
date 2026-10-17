// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Regression tests for the second pre-DOI audit's finding N06 and its
// covariance note.
//
// N06. The replay skipped an IMU record whose time did not advance, but the
//      next interval averaged with logs.imu[k-1] -- possibly that skipped
//      record: a duplicate t = 0 stamp with a 100 rad/s roll rate between two
//      zero-gyro samples turned the final attitude by 0.5 rad. And nothing
//      checked the inputs: a NaN gyro reading reached the propagation and
//      made every later estimate NaN (ANEES NaN). Records are now checked
//      before use, rejected ones are counted, and each interval averages with
//      the last accepted sample.
// Covariance. An update checked that S = H P H^T + R factorises but not that
//      R is a covariance: R = -0.5 I on P = I left S = 0.5 I and drove the
//      posterior position variance to -1. R must now be finite, symmetric and
//      positive semi-definite.
//
// The tests use records that differ from the accepted ones -- a duplicate
// with the same readings cannot show which sample was averaged.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include <Aetherion/Estimation/EKF.h>
#include <Aetherion/Estimation/Replay/Replay.h>
#include <Aetherion/ODE/RKMK/Lie/SO3.h>

using namespace Aetherion::Estimation;
using namespace Aetherion::Estimation::Replay;
using Catch::Matchers::ContainsSubstring;

namespace {
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    constexpr double kInf = std::numeric_limits<double>::infinity();

    const Configuration kNavOnly({ BlockId::NavCore }, {});

    ImuRecord imuAt(double t, const Eigen::Vector3d& gyro = Eigen::Vector3d::Zero())
    {
        ImuRecord r;
        r.partTime = t;
        r.hostTime = t;
        r.gyro_rad_s = gyro;
        r.accel_m_s2 = Eigen::Vector3d(0.0, 0.0, 9.8);
        return r;
    }

    TruthRecord truthAt(double t)
    {
        TruthRecord r;
        r.time = t;
        r.vNed_m_s = Eigen::Vector3d::Zero();
        return r;
    }

    /// The audit's log: 0 and 0.01 s, zero gyro; or n samples at 100 Hz.
    LogSet logs(int n = 2)
    {
        LogSet l;
        for (int k = 0; k < n; ++k)
            l.imu.push_back(imuAt(0.01 * k));
        l.truth = { truthAt(0.0), truthAt(0.01 * (n - 1)) };
        return l;
    }

    double angleBetween(const ReplayResult& a, const ReplayResult& b)
    {
        const Eigen::Matrix3d dR = b.states.back().estimate.X.R * a.states.back().estimate.X.R.transpose();
        return Aetherion::ODE::RKMK::Lie::SO3::Log_R(dR).norm();
    }
} // namespace

TEST_CASE("Replay: a skipped duplicate IMU stamp does not influence the estimate",
    "[estimation][replay][validation][regression]")
{
    for (const ErrorConvention c : { ErrorConvention::LeftInvariant, ErrorConvention::RightInvariant })
    {
        const ReplayResult base = replay(c, logs(), kNavOnly, {});

        // The audit's probe: a duplicate t = 0 stamp with a 100 rad/s roll rate
        // between the two zero-gyro samples. It used to turn the estimate by 0.5 rad.
        LogSet withDup = logs();
        withDup.imu.insert(withDup.imu.begin() + 1, imuAt(0.0, Eigen::Vector3d(100.0, 0.0, 0.0)));
        const ReplayResult dup = replay(c, withDup, kNavOnly, {});

        CHECK(angleBetween(base, dup) == 0.0);
        CHECK(dup.summary.skippedImuSamples == 1);
        CHECK(dup.summary.imuSamples == base.summary.imuSamples);
        CHECK(dup.states.back().estimate.X.p == base.states.back().estimate.X.p);
        CHECK(dup.states.back().estimate.X.v == base.states.back().estimate.X.v);
    }
}

TEST_CASE("Replay: an out-of-order IMU stamp in the middle of a log is skipped without effect",
    "[estimation][replay][validation][regression]")
{
    const ReplayResult base = replay(ErrorConvention::LeftInvariant, logs(51), kNavOnly, {});

    LogSet l = logs(51);
    l.imu.insert(l.imu.begin() + 21, imuAt(0.105, Eigen::Vector3d(0.0, 3.0, -2.0))); // after 0.20 s: out of order
    const ReplayResult r = replay(ErrorConvention::LeftInvariant, l, kNavOnly, {});

    CHECK(angleBetween(base, r) == 0.0);
    CHECK(r.summary.skippedImuSamples == 1);
}

TEST_CASE("Replay: IMU samples that are not finite are rejected and counted",
    "[estimation][replay][validation][regression]")
{
    const ReplayResult base = replay(ErrorConvention::LeftInvariant, logs(51), kNavOnly, {});

    SECTION("NaN gyro, the audit's probe on a longer log")
    {
        LogSet l = logs(51);
        l.imu.insert(l.imu.begin() + 10, imuAt(0.095, Eigen::Vector3d(kNaN, 0.0, 0.0)));
        const ReplayResult r = replay(ErrorConvention::LeftInvariant, l, kNavOnly, {});
        CHECK(r.states.back().estimate.X.R.allFinite());
        CHECK(std::isfinite(r.summary.aneesNav));
        CHECK(r.summary.rejectedImuSamples == 1);
        CHECK(angleBetween(base, r) == 0.0);
    }
    SECTION("infinite acceleration, NaN part time, NaN host time")
    {
        LogSet l = logs(51);
        ImuRecord a = imuAt(0.095);
        a.accel_m_s2.x() = kInf;
        ImuRecord b = imuAt(0.135);
        b.partTime = kNaN;
        ImuRecord h = imuAt(0.175);
        h.hostTime = kNaN;
        l.imu.insert(l.imu.begin() + 18, h);
        l.imu.insert(l.imu.begin() + 14, b);
        l.imu.insert(l.imu.begin() + 10, a);
        const ReplayResult r = replay(ErrorConvention::LeftInvariant, l, kNavOnly, {});
        CHECK(r.summary.rejectedImuSamples == 3);
        CHECK(r.states.back().estimate.X.R.allFinite());
        CHECK(angleBetween(base, r) == 0.0);
        CHECK(r.summary.timeBase.samples == 51); // the clock fit used the valid samples only
    }
    SECTION("fewer than two valid samples is an error, not a NaN result")
    {
        LogSet l = logs(2);
        l.imu[1].gyro_rad_s.x() = kNaN;
        REQUIRE_THROWS_WITH(replay(ErrorConvention::LeftInvariant, l, kNavOnly, {}),
            ContainsSubstring("fewer than two valid IMU samples"));
    }
}

TEST_CASE("Replay: truth records that are not finite are rejected; truth is put in time order",
    "[estimation][replay][validation]")
{
    const ReplayResult base = replay(ErrorConvention::LeftInvariant, logs(51), kNavOnly, {});

    LogSet l = logs(51);
    TruthRecord bad = truthAt(0.25);
    bad.lat_deg = kNaN;
    l.truth.push_back(bad);
    std::swap(l.truth.front(), l.truth.back()); // NaN first, valid records out of order
    const ReplayResult r = replay(ErrorConvention::LeftInvariant, l, kNavOnly, {});

    CHECK(r.summary.rejectedTruthRecords == 1);
    CHECK(std::isfinite(r.summary.aneesNav));
    CHECK(r.summary.aneesNav == base.summary.aneesNav);

    LogSet none = logs(51);
    for (TruthRecord& t : none.truth)
        t.time = kNaN;
    REQUIRE_THROWS_WITH(replay(ErrorConvention::LeftInvariant, none, kNavOnly, {}),
        ContainsSubstring("no valid truth"));
}

TEST_CASE("Replay: a measurement whose time is not finite is rejected and counted",
    "[estimation][replay][validation]")
{
    LogSet l = logs(51);
    BaroRecord b;
    b.partTime = kNaN; // unwraps to NaN
    b.hostTime = 0.2;
    b.pressure_Pa = 90000.0;
    l.baro.push_back(b);
    const ReplayResult r = replay(ErrorConvention::LeftInvariant, l, kNavBaroGnss16, {});
    CHECK(r.summary.rejectedMeasurements == 1);
    CHECK(r.summary.baroUpdates == 0);
    CHECK(r.states.back().estimate.X.R.allFinite());
}

TEST_CASE("EKF: an update whose R is not a covariance is refused and changes nothing",
    "[estimation][ekf][validation][regression]")
{
    const Configuration cfg({ BlockId::NavCore }, { MeasurementId::GnssPosition });
    InvariantEKF<double, ErrorConvention::LeftInvariant> k(cfg, LciFrame<double>::FromGeodetic(0, 0, 0, 0), {});
    k.initialise({}, Eigen::MatrixXd::Identity(9, 9));
    const Eigen::MatrixXd P0 = k.covariance();

    auto refused = [&](const Eigen::Matrix3d& R) {
        const auto u = k.updateGnssPosition(Eigen::Vector3d::Zero(), R);
        return !u.applied && u.rejected && k.covariance() == P0;
    };

    // The audit's probe: S = P + R = 0.5 I is positive, and the posterior
    // position variance came out -1.
    CHECK(refused(-0.5 * Eigen::Matrix3d::Identity()));

    Eigen::Matrix3d indefinite = Eigen::Matrix3d::Identity();
    indefinite(2, 2) = -0.1;
    CHECK(refused(indefinite));

    Eigen::Matrix3d asymmetric = Eigen::Matrix3d::Identity();
    asymmetric(0, 1) = 0.5;
    CHECK(refused(asymmetric));

    Eigen::Matrix3d nan = Eigen::Matrix3d::Identity();
    nan(1, 1) = kNaN;
    CHECK(refused(nan));

    // A valid covariance -- including a singular one, a perfect measurement on
    // one axis -- is applied, and P stays positive semi-definite.
    Eigen::Matrix3d singular = Eigen::Matrix3d::Identity();
    singular(2, 2) = 0.0;
    const auto u = k.updateGnssPosition(Eigen::Vector3d::Zero(), singular);
    CHECK(u.applied);
    CHECK_FALSE(u.rejected);
    CHECK(k.covariance().diagonal().minCoeff() >= 0.0);
}

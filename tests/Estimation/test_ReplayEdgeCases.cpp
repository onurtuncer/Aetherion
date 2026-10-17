// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Sensor fusion step 6, replay harness: the paths the end-to-end run in
// test_Replay.cpp does not take. A one-second stationary log built in memory
// carries one of each awkward record -- a fix without nominal_time_s, a fix
// before the first IMU sample, an epoch without a fix, a 2D fix, a duplicate
// IMU stamp -- and the loaders are fed every kind of missing input.
// ------------------------------------------------------------------------------

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>

#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Estimation/Replay/Csv.h>
#include <Aetherion/Estimation/Replay/Logs.h>
#include <Aetherion/Estimation/Replay/Replay.h>
#include <Aetherion/Estimation/Replay/TimeBase.h>

using namespace Aetherion::Estimation;
using namespace Aetherion::Estimation::Replay;
using Catch::Matchers::ContainsSubstring;

namespace {
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    constexpr double kLat0 = 36.0192, kLon0 = -75.6744, kAlt0 = 3000.0;

    constexpr std::string_view kImuHeader = "sample_index,part_time_s,host_time_s,accel_x_mps2,accel_y_mps2,accel_z_mps2,"
                                            "gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s";
    constexpr std::string_view kBaroHeader = "sample_index,part_time_s,host_time_s,pressure_pa,temperature_c";
    constexpr std::string_view kTruthHeader = "iterations, time, f16::out.alt_m[REAL], f16::out.lat_deg[REAL], "
                                              "f16::out.lon_deg[REAL], f16::out.v_north_m_s[REAL], "
                                              "f16::out.v_east_m_s[REAL], f16::out.v_down_m_s[REAL], "
                                              "f16::out.yaw_rad[REAL], f16::out.pitch_rad[REAL], "
                                              "f16::out.roll_rad[REAL]";
    constexpr std::string_view kSidecar = "lat0_deg=36.0192\nlon0_deg=-75.6744\nalt0_ft=9842.5\n"
                                          "gps_latency_s=0.1\nrealtime_factor=0\n";

    GpsRecord fix(double nominal, double host, bool valid, int fixType, bool withVelocity)
    {
        GpsRecord g;
        g.valid = valid;
        g.nominalTime = nominal;
        g.hostTime = host;
        g.lat_deg = kLat0;
        g.lon_deg = kLon0;
        g.alt_m = kAlt0;
        g.horizontalAccuracy_m = 1.5;
        g.verticalAccuracy_m = 3.0;
        g.fixType = fixType;
        g.hasVelocity = withVelocity;
        g.speedAccuracy_m_s = 0.0; // not reported: the harness falls back to ReplayOptions::gnssVelSigma_m_s
        return g;
    }

    /// One second at rest at the LCI origin, 100 Hz IMU, with one of each awkward record.
    LogSet stationaryLogs()
    {
        LogSet logs;
        logs.run.lat0_deg = kLat0;
        logs.run.lon0_deg = kLon0;
        logs.run.alt0_m = kAlt0;
        logs.run.gpsLatency_s = 0.1;

        for (int k = 0; k <= 100; ++k)
        {
            const double t = 0.01 * k;
            ImuRecord r;
            r.partTime = t;
            r.hostTime = 1000.0 + t;
            r.gyro_rad_s = Eigen::Vector3d::Zero();
            r.accel_m_s2 = Eigen::Vector3d(0.0, 0.0, -9.8);
            logs.imu.push_back(r);
            if (k == 50)
                logs.imu.push_back(r); // duplicate stamp: skipped
        }
        for (double t : { 0.0, 1.0 })
        {
            TruthRecord tr;
            tr.time = t;
            tr.lat_deg = kLat0;
            tr.lon_deg = kLon0;
            tr.alt_m = kAlt0;
            tr.vNed_m_s = Eigen::Vector3d::Zero();
            logs.truth.push_back(tr);
        }

        logs.gps.push_back(fix(kNaN, 1000.0 + 0.2 + 0.1, true, LogContract::Gps::kFixType3D, true)); // host - latency = 0.2
        logs.gps.push_back(fix(-0.5, 999.6, true, LogContract::Gps::kFixType3D, true));              // before the first IMU sample
        logs.gps.push_back(fix(0.4, 1000.5, false, 0, false));                                       // epoch without a fix
        logs.gps.push_back(fix(0.6, 1000.7, true, 1, false));                                        // 2D: below minFixType
        logs.gps.push_back(fix(0.805, 1000.9, true, LogContract::Gps::kFixType3D, false));           // splits an IMU interval

        logs.baro.push_back({ 0.3, 1000.3, Aetherion::Environment::US1976Atmosphere(kAlt0).p, 15.0 });
        return logs;
    }
} // namespace

TEST_CASE("Replay: empty logs are rejected", "[estimation][replay][negative]")
{
    LogSet logs = stationaryLogs();
    logs.imu.clear();
    REQUIRE_THROWS_WITH(replay(ErrorConvention::RightInvariant, logs, kNavBaroGnss16, {}),
        ContainsSubstring("no IMU samples"));
    logs = stationaryLogs();
    logs.truth.clear();
    REQUIRE_THROWS_WITH(replay(ErrorConvention::LeftInvariant, logs, kNavBaroGnss16, {}), ContainsSubstring("no truth"));
}

TEST_CASE("Replay: awkward records on a stationary log, default sigmas", "[estimation][replay]")
{
    const LogSet logs = stationaryLogs();
    ReplayOptions opt;
    opt.outputEvery_s = 0.1; // initialSigma left empty: the harness's defaults apply

    for (const ErrorConvention c : { ErrorConvention::RightInvariant, ErrorConvention::LeftInvariant })
    {
        const ReplayResult r = replay(c, logs, kNavBaroGnss16, opt);
        const ReplaySummary& s = r.summary;
        INFO(toString(c));

        CHECK(s.convention == c);
        CHECK(s.imuSamples == 100); // 102 records, the first starts the run, the duplicate is skipped
        CHECK(s.duration_s == Catch::Approx(1.0));
        // The fix before t0 is dropped silently; the dropout and the 2D fix are counted.
        CHECK(s.skippedFixes == 2);
        CHECK(s.gnssPositionUpdates == 2);
        CHECK(s.gnssVelocityUpdates == 1);
        CHECK(s.baroUpdates == 1);
        // The fix without nominal_time_s is placed at the mapped arrival less the latency.
        REQUIRE_FALSE(r.updates.empty());
        CHECK(r.updates.front().t == Catch::Approx(0.2).margin(1e-6));
        CHECK(r.updates.front().id == MeasurementId::GnssPosition);
        // Decimated output: t0 and every 0.1 s.
        CHECK(r.states.size() == 11);
        CHECK(r.states.front().sigma.size() == 16);
        // At rest from the truth, the estimate stays put.
        CHECK(s.posRms_m < 1.0);
        CHECK(std::isfinite(s.aneesNav));
    }
}

TEST_CASE("Replay: measurements the configuration does not use are ignored", "[estimation][replay]")
{
    const LogSet logs = stationaryLogs();
    const Configuration posOnly{ { BlockId::NavCore }, { MeasurementId::GnssPosition } };
    REQUIRE(posOnly.valid());

    ReplayOptions opt;
    opt.initialError = Eigen::VectorXd::Zero(posOnly.dimension());
    opt.initialError[0] = 0.001; // a small attitude perturbation, no bias blocks to perturb

    const ReplayResult r = replay<ErrorConvention::RightInvariant>(logs, posOnly, opt);
    CHECK(r.summary.gnssPositionUpdates == 2);
    CHECK(r.summary.gnssVelocityUpdates == 0);
    CHECK(r.summary.baroUpdates == 0);
    CHECK(r.states.size() == 101); // every IMU sample
    CHECK(r.states.front().error.size() == 9);
}

TEST_CASE("Replay: writers emit one column per state and one row per record", "[estimation][replay][output]")
{
    const ReplayResult r = replay(ErrorConvention::RightInvariant, stationaryLogs(), kNavBaroGnss16, {});

    std::ostringstream states, updates, summary;
    writeStates(states, r, kNavBaroGnss16);
    writeUpdates(updates, r);
    writeSummary(summary, r.summary);

    std::istringstream in(states.str());
    std::string header, firstRow;
    std::getline(in, header);
    std::getline(in, firstRow);
    const auto commas = [](const std::string& s) { return std::count(s.begin(), s.end(), ','); };
    CHECK(commas(header) == commas(firstRow));
    CHECK_THAT(header, ContainsSubstring("sigma_15") && ContainsSubstring("est_bp"));
    const auto lines = [](const std::string& s) { return static_cast<std::size_t>(std::count(s.begin(), s.end(), '\n')); };
    CHECK(lines(states.str()) == r.states.size() + 1);

    CHECK_THAT(updates.str(), ContainsSubstring("t_s,measurement,dim,nis"));
    CHECK(lines(updates.str()) == r.updates.size() + 1);

    CHECK_THAT(summary.str(), ContainsSubstring("convention:") && ContainsSubstring("fixes skipped") &&
                                  ContainsSubstring("bias NEES"));
}

TEST_CASE("Replay: truth interpolation clamps at the ends and wraps longitude", "[estimation][replay][truth]")
{
    constexpr double kDeg = std::numbers::pi / 180.0;
    std::vector<TruthRecord> truth(2);
    truth[0].time = 10.0;
    truth[0].lat_deg = 1.0;
    truth[0].lon_deg = 179.0;
    truth[0].alt_m = 100.0;
    truth[0].vNed_m_s = Eigen::Vector3d(10.0, 0.0, 0.0);
    truth[0].yaw_rad = std::numbers::pi - 0.1;
    truth[1] = truth[0];
    truth[1].time = 20.0;
    truth[1].lat_deg = 2.0;
    truth[1].lon_deg = -179.0; // across the antimeridian
    truth[1].alt_m = 200.0;
    truth[1].vNed_m_s = Eigen::Vector3d(20.0, 0.0, 0.0);
    truth[1].yaw_rad = -std::numbers::pi + 0.1; // across +-pi

    const GeodeticTruth mid = Replay::detail::interpolateTruth(truth, 15.0);
    CHECK(mid.lat_rad == Catch::Approx(1.5 * kDeg));
    CHECK(std::abs(std::remainder(mid.lon_rad - std::numbers::pi, 2.0 * std::numbers::pi)) < 1e-12);
    CHECK(mid.alt_m == Catch::Approx(150.0));
    CHECK(mid.vNed[0] == Catch::Approx(15.0));
    CHECK(std::abs(std::remainder(mid.yaw_rad - std::numbers::pi, 2.0 * std::numbers::pi)) < 1e-12);

    CHECK(Replay::detail::interpolateTruth(truth, 0.0).alt_m == Catch::Approx(100.0));  // before the first record
    CHECK(Replay::detail::interpolateTruth(truth, 99.0).alt_m == Catch::Approx(200.0)); // after the last
    truth.resize(1);
    CHECK(Replay::detail::interpolateTruth(truth, 12.0).alt_m == Catch::Approx(100.0)); // a single record
}

TEST_CASE("Replay timing: the barometer part clock is unwrapped against the host clock", "[estimation][replay][time]")
{
    const ReplayTiming timing;
    TimeBaseFit fit; // host == part
    const RunConfiguration run;
    const double W = LogContract::Baro::kPartTimeWrap_s;
    const BaroRecord b{ 3.0, W + 3.0, 101325.0, 15.0 }; // the register has wrapped once
    CHECK(timing.baro(b, fit, run) == Catch::Approx(W + 3.0));
}

TEST_CASE("Logs: the loaders reject missing inputs", "[estimation][replay][logs][negative]")
{
    auto streams = [](std::stringstream& imu, std::stringstream& truth) {
        imu << kImuHeader << "\n";
        truth << kTruthHeader << "\n";
    };

    SECTION("IMU and truth are always required")
    {
        std::stringstream imu, truth;
        streams(imu, truth);
        REQUIRE_THROWS_WITH(loadLogs(LogStreams{ nullptr, nullptr, nullptr, &truth, kSidecar }, kNavBaroGnss16),
            ContainsSubstring("imu_samples.csv is required"));
        REQUIRE_THROWS_WITH(loadLogs(LogStreams{ nullptr, &imu, nullptr, nullptr, kSidecar }, kNavBaroGnss16),
            ContainsSubstring("truth log is required"));
    }
    SECTION("GPS and baro when the configuration uses them")
    {
        std::stringstream imu, truth;
        streams(imu, truth);
        REQUIRE_THROWS_WITH(loadLogs(LogStreams{ nullptr, &imu, nullptr, &truth, kSidecar }, kNavBaroGnss16),
            ContainsSubstring("gps_fixes.csv is required"));

        std::stringstream imu2, truth2;
        streams(imu2, truth2);
        const Configuration baroOnly{ { BlockId::NavCore, BlockId::BaroBias }, { MeasurementId::Barometer } };
        REQUIRE_THROWS_WITH(loadLogs(LogStreams{ nullptr, &imu2, nullptr, &truth2, kSidecar }, baroOnly),
            ContainsSubstring("baro_samples.csv is required"));

        std::stringstream imu3, truth3, baro3;
        streams(imu3, truth3);
        baro3 << kBaroHeader << "\n0,0.5,1000.5,70000.0,-5.0\n";
        const LogSet logs = loadLogs(LogStreams{ nullptr, &imu3, &baro3, &truth3, kSidecar }, baroOnly);
        CHECK(logs.gps.empty());
        REQUIRE(logs.baro.size() == 1);
        CHECK(logs.baro[0].temperature_C == Catch::Approx(-5.0));
    }
    SECTION("a sidecar value that is not a number")
    {
        REQUIRE_THROWS_WITH(runConfiguration("lat0_deg=north\nlon0_deg=0\nalt0_ft=0\ngps_latency_s=0\nrealtime_factor=0\n"),
            ContainsSubstring("lat0_deg is not a number"));
        REQUIRE_THROWS_WITH(runConfiguration("lat0_deg=1\n"), ContainsSubstring("missing key(s)"));
    }
}

TEST_CASE("Logs: loading from a directory", "[estimation][replay][logs]")
{
    const auto dir = std::filesystem::temp_directory_path() / "aetherion_replay_edge_test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const auto truthPath = dir / "truth.csv";
    const Configuration baroOnly{ { BlockId::NavCore, BlockId::BaroBias }, { MeasurementId::Barometer } };

    // Nothing there yet.
    REQUIRE_THROWS_WITH(loadLogs(dir, truthPath, baroOnly), ContainsSubstring("cannot open"));

    std::ofstream(dir / "imu_samples.csv") << kImuHeader << "\n0,0.0,1000.0,0,0,-9.8,0,0,0\n";
    std::ofstream(dir / "baro_samples.csv") << kBaroHeader << "\n0,0.0,1000.0,70000.0,-5.0\n";
    std::ofstream(truthPath) << kTruthHeader << "\n0, 0.0, 3000.0, 36.0, -75.0, 0, 0, 0, 0, 0, 0\n";

    // The sidecar defaults to the truth file with .config; absent, that is an error.
    REQUIRE_THROWS_WITH(loadLogs(dir, truthPath, baroOnly), ContainsSubstring("truth.config"));

    // An explicit sidecar path; the GPS log is not needed by this configuration.
    const auto sidecar = dir / "run.cfg";
    std::ofstream(sidecar) << kSidecar;
    const LogSet logs = loadLogs(dir, truthPath, baroOnly, sidecar);
    CHECK(logs.imu.size() == 1);
    CHECK(logs.baro.size() == 1);
    CHECK(logs.truth.size() == 1);
    CHECK(logs.run.gpsLatency_s == Catch::Approx(0.1));

    // A configuration with GNSS needs gps_fixes.csv.
    REQUIRE_THROWS_WITH(loadLogs(dir, truthPath, kNavBaroGnss16, sidecar), ContainsSubstring("gps_fixes.csv"));

    std::filesystem::remove_all(dir);
}

TEST_CASE("CSV reader: empty input, BOM, cell parsing, files", "[estimation][replay][csv]")
{
    const Configuration baroOnly{ { BlockId::NavCore, BlockId::BaroBias }, { MeasurementId::Barometer } };

    std::stringstream empty;
    REQUIRE_THROWS_WITH(readCsv(empty, LogContract::Log::BaroSamples, baroOnly, "baro"), ContainsSubstring("empty file"));

    std::stringstream bom;
    bom << "\xEF\xBB\xBF" << kBaroHeader << "\n0,1.5,1001.5,TRUE,junk\n";
    const CsvTable t = readCsv(bom, LogContract::Log::BaroSamples, baroOnly);
    REQUIRE(t.has("sample_index"));
    CHECK(t.at(0, "part_time_s") == Catch::Approx(1.5));
    CHECK(t.at(0, "pressure_pa") == 1.0);        // boolean spelling
    CHECK(std::isnan(t.at(0, "temperature_c"))); // not a number

    CHECK(Replay::detail::parseCell(" False ") == 0.0);
    CHECK(Replay::detail::parseCell("True") == 1.0);
    CHECK(std::isnan(Replay::detail::parseCell("   ")));

    const auto path = std::filesystem::temp_directory_path() / "aetherion_csv_edge_test.csv";
    REQUIRE_THROWS_WITH(readCsvFile(path / "missing.csv", LogContract::Log::BaroSamples, baroOnly),
        ContainsSubstring("cannot open"));
    std::ofstream(path) << kBaroHeader << "\n0,0,0,101325,15\n1,0.1,0.1,101300,15\n";
    CHECK(readCsvFile(path, LogContract::Log::BaroSamples, baroOnly).rows.size() == 2);
    std::filesystem::remove(path);
}

TEST_CASE("Time-base fit: mismatched inputs", "[estimation][replay][time][negative]")
{
    REQUIRE_THROWS_WITH(fitTimeBase({ 0.0, 1.0 }, { 0.0 }), ContainsSubstring("differ in length"));
    REQUIRE_THROWS_WITH(fitTimeBase({}, {}), ContainsSubstring("fewer than two"));
}

TEST_CASE("EKF: updates for measurements outside the configuration are not applied", "[estimation][ekf]")
{
    constexpr double kDeg = std::numbers::pi / 180.0;
    const Configuration posOnly{ { BlockId::NavCore }, { MeasurementId::GnssPosition } };
    const LciFrame<double> frame = LciFrame<double>::FromGeodetic(kLat0 * kDeg, kLon0 * kDeg, kAlt0, 0.0);
    InvariantEKF<double, ErrorConvention::RightInvariant> ekf(posOnly, frame, ProcessNoise<double>{ 2e-4, 5e-3, 2e-6, 2e-5, 0.05 });

    const auto v = ekf.updateGnssVelocity(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
    CHECK_FALSE(v.applied);
    CHECK(v.dim == 0);
    const auto b = ekf.updateBarometer(kAlt0, 1.0);
    CHECK_FALSE(b.applied);
}

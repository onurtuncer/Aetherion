// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Sensor fusion step 6, replay harness: the CSV reader against the truth
// log's quirks, the time-base fit, the truth -> LCI mapping and its Euler
// inverse, and log loading against the header lines Hemerion writes.
// ------------------------------------------------------------------------------

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <numbers>
#include <random>
#include <sstream>
#include <string>

#include <Aetherion/Coordinate/InertialToLocal.h>
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Estimation/Replay/Csv.h>
#include <Aetherion/Estimation/Replay/Logs.h>
#include <Aetherion/Estimation/Replay/Replay.h>
#include <Aetherion/Estimation/Replay/TimeBase.h>
#include <Aetherion/Estimation/Replay/TruthState.h>

#include "SyntheticTruth.h"

#include <filesystem>
#include <fstream>
#include <iomanip>

using namespace Aetherion::Estimation;
using namespace Aetherion::Estimation::Replay;
using Catch::Matchers::ContainsSubstring;
namespace WGS84 = Aetherion::Environment::WGS84;

namespace {
    constexpr double kDeg = std::numbers::pi / 180.0;

    // Header lines as Hemerion writes them (see test_LogContract.cpp).
    constexpr std::string_view kGpsHeader = "fix_index,nominal_time_s,host_time_s,latitude_deg,longitude_deg,altitude_m,"
                                            "ground_speed_mps,course_deg,horizontal_accuracy_m,vertical_accuracy_m,"
                                            "num_satellites,fix_type";
    constexpr std::string_view kImuHeader = "sample_index,part_time_s,host_time_s,accel_x_mps2,accel_y_mps2,accel_z_mps2,"
                                            "gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s";
    constexpr std::string_view kBaroHeader = "sample_index,part_time_s,host_time_s,pressure_pa,temperature_c";
    constexpr std::string_view kTruthHeader = "iterations, time, f16::out.alt_m[REAL], f16::out.lat_deg[REAL], "
                                              "f16::out.lon_deg[REAL], f16::out.v_north_m_s[REAL], "
                                              "f16::out.v_east_m_s[REAL], f16::out.v_down_m_s[REAL], "
                                              "f16::out.yaw_rad[REAL], f16::out.pitch_rad[REAL], "
                                              "f16::out.roll_rad[REAL], f16::out.p_rad_s[REAL], f16::out.staged[BOOL]";

    const Configuration kGnssPosBaro{ { BlockId::NavCore, BlockId::GyroBias, BlockId::AccelBias, BlockId::BaroBias },
        { MeasurementId::GnssPosition, MeasurementId::Barometer } };
} // namespace

TEST_CASE("CSV reader: truth-log separators, type suffixes, booleans, blanks", "[estimation][replay][csv]")
{
    std::stringstream in;
    in << kTruthHeader << "\n"
       << "0, 0.0, 3052.0, 36.0192, -75.6744, 100.0, 100.0, 0.0, 0.7854, 0.046, 0.0, 0.0, false\n"
       << "\n"
       << "1, 0.01, 3052.1, 36.0192, -75.6744, 100.0, 100.0, 0.0, 0.7854, 0.046, 0.0, 0.0, true\n";
    const CsvTable t = readCsv(in, LogContract::Log::Truth, kGnssPosBaro, "truth");
    REQUIRE(t.rows.size() == 2);
    REQUIRE(t.at(1, "time") == Catch::Approx(0.01));
    REQUIRE(t.at(0, "out.alt_m") == Catch::Approx(3052.0));
    REQUIRE(t.at(0, "out.staged") == 0.0);
    REQUIRE(t.at(1, "out.staged") == 1.0);
    REQUIRE(t.has("out.p_rad_s"));
    REQUIRE_THROWS_WITH(t.column("nope"), ContainsSubstring("nope"));

    // A short row is padded with NaN, a long one is an error.
    std::stringstream shortRow;
    shortRow << kBaroHeader << "\n0,0.0,0.0,101325.0\n";
    const CsvTable s = readCsv(shortRow, LogContract::Log::BaroSamples, kGnssPosBaro);
    REQUIRE(std::isnan(s.at(0, "temperature_c")));
    std::stringstream longRow;
    longRow << kBaroHeader << "\n0,0.0,0.0,101325.0,15.0,extra\n";
    REQUIRE_THROWS_WITH(readCsv(longRow, LogContract::Log::BaroSamples, kGnssPosBaro, "baro"),
        ContainsSubstring("6 cells"));

    // A renamed column fails before any row is read.
    std::stringstream renamed;
    renamed << "sample_index,part_time_s,host_time_s,pressure_hpa,temperature_c\n0,0,0,1013,15\n";
    REQUIRE_THROWS_WITH(readCsv(renamed, LogContract::Log::BaroSamples, kGnssPosBaro), ContainsSubstring("pressure_pa"));
}

TEST_CASE("Time-base fit recovers a skewed, offset host clock", "[estimation][replay][time]")
{
    Aetherion::Estimation::Testing::PortableGaussian g(7);
    auto jitter = [&] { return 50e-6 * g(); }; // 50 us of stamping jitter
    std::vector<double> part, host;
    const double offset = 1234.5678, scale = 1.0 + 2.5e-5; // 25 ppm
    for (int k = 0; k < 6000; ++k)
    {
        const double t = k * 0.01;
        part.push_back(t);
        host.push_back(offset + scale * t + jitter());
    }
    const TimeBaseFit f = fitTimeBase(part, host);
    REQUIRE(f.samples == 6000);
    REQUIRE(f.offset == Catch::Approx(offset).margin(5e-6));
    REQUIRE(f.scale == Catch::Approx(scale).margin(1e-7));
    REQUIRE(f.rmsResidual == Catch::Approx(50e-6).epsilon(0.1));
    REQUIRE(f.maxResidual < 300e-6);
    REQUIRE(f.toPart(f.toHost(17.3)) == Catch::Approx(17.3));
    // Mapping a host stamp back is good to the jitter floor, far under 1 ms.
    REQUIRE(std::abs(f.toPart(offset + scale * 42.0) - 42.0) < 1e-5);

    REQUIRE_THROWS(fitTimeBase({ 1.0 }, { 2.0 }));
    REQUIRE_THROWS(fitTimeBase({ 1.0, 1.0 }, { 2.0, 3.0 }));
}

TEST_CASE("Truth -> LCI state and back to NED Euler angles", "[estimation][replay][truth]")
{
    const double lat0 = 36.0192 * kDeg, lon0 = -75.6744 * kDeg, h0 = 3.0;
    const LciFrame<double> frame = LciFrame<double>::FromGeodetic(lat0, lon0, h0, 0.0);

    // At rest at the origin at t = 0, level, heading 45 deg.
    GeodeticTruth g;
    g.t = 0.0;
    g.lat_rad = lat0;
    g.lon_rad = lon0;
    g.alt_m = h0;
    g.yaw_rad = 45.0 * kDeg;
    const auto x0 = truthToState(g, frame);
    REQUIRE(x0.X.p.norm() < 1e-6);
    // Earth-relative rest is inertially w_e x r, eastward.
    const auto r_E = Aetherion::Coordinate::GeodeticToECEF(lat0, lon0, h0);
    REQUIRE(x0.X.v.norm() == Catch::Approx(WGS84::kRotationRate_rad_s * std::hypot(r_E[0], r_E[1])).epsilon(1e-9));
    REQUIRE(x0.X.v[1] > 0.0);
    REQUIRE(std::abs(x0.X.v[0]) < 1e-6);
    // Attitude: body -> LCI equals body -> NED at t0 at the origin.
    REQUIRE((x0.X.R - rotationNedFromBody(45.0 * kDeg, 0.0, 0.0)).norm() < 1e-12);
    const Eigen::Vector3d e0 = eulerNedFromState(x0, 0.0, frame);
    REQUIRE(e0[0] == Catch::Approx(45.0 * kDeg).margin(1e-9));
    REQUIRE(std::abs(e0[1]) < 1e-9);
    REQUIRE(std::abs(e0[2]) < 1e-9);

    // 100 s later, 2 km north and 3 km up, flying north-east at 172 m/s,
    // banked: the Euler round trip must survive the Earth's rotation.
    GeodeticTruth g1;
    g1.t = 100.0;
    g1.lat_rad = lat0 + 2000.0 / 6371e3;
    g1.lon_rad = lon0;
    g1.alt_m = 3000.0;
    g1.vNed = Eigen::Vector3d(122.0, 122.0, -3.0);
    g1.yaw_rad = 45.0 * kDeg;
    g1.pitch_rad = 2.6 * kDeg;
    g1.roll_rad = -20.0 * kDeg;
    const auto x1 = truthToState(g1, frame);
    const Eigen::Vector3d e1 = eulerNedFromState(x1, g1.t, frame);
    REQUIRE(e1[0] == Catch::Approx(g1.yaw_rad).margin(1e-9));
    REQUIRE(e1[1] == Catch::Approx(g1.pitch_rad).margin(1e-9));
    REQUIRE(e1[2] == Catch::Approx(g1.roll_rad).margin(1e-9));
    // and the position is where the GNSS mapping puts the same fix.
    const MeasurementModels<double> m(frame);
    REQUIRE((x1.X.p - m.gnssPositionLci({ g1.lat_rad, g1.lon_rad, g1.alt_m }, frame.earthRotationAngle(100.0))).norm() <
            1e-9);
}

TEST_CASE("Log set loads from streams against the contract headers", "[estimation][replay][logs]")
{
    std::stringstream gps, imu, baro, truth;
    // One fix, then an epoch without one as Hemerion writes it: every field after nominal_time_s empty.
    gps << kGpsHeader << "\n0,0.1,1000.1,36.0192,-75.6744,3052.0,172.4,45.0,1.5,3.0,9,2\n1,0.2,,,,,,,,,,\n";
    imu << kImuHeader << "\n0,0.00,1000.00,0.1,0.0,-9.8,0.0,0.0,0.0\n1,0.01,1000.01,0.1,0.0,-9.8,0.0,0.0,0.0\n";
    baro << kBaroHeader << "\n0,0.0,1000.0,70000.0,-5.0\n";
    truth << kTruthHeader << "\n0, 0.0, 3052.0, 36.0192, -75.6744, 121.9, 121.9, 0.0, 0.7854, 0.046, 0.0, 0.0, false\n";
    const std::string sidecar = "check_case=11\nlat0_deg=36.0192\nlon0_deg=-75.6744\nalt0_ft=10013\n"
                                "gps_latency_s=0.2\nrealtime_factor=0\n";

    const LogSet logs = loadLogs(LogStreams{ &gps, &imu, &baro, &truth, sidecar }, kGnssPosBaro);
    REQUIRE(logs.imu.size() == 2);
    REQUIRE(logs.imu[1].partTime == Catch::Approx(0.01));
    REQUIRE(logs.imu[1].accel_m_s2[2] == Catch::Approx(-9.8));
    REQUIRE(logs.gps.size() == 2);
    REQUIRE(logs.gps[0].valid);
    REQUIRE(logs.gps[0].fixType == LogContract::Gps::kFixType3D);
    REQUIRE(logs.gps[0].nominalTime == Catch::Approx(0.1));
    REQUIRE_FALSE(logs.gps[0].hasVelocity);
    REQUIRE_FALSE(logs.gps[1].valid);
    REQUIRE(logs.gps[1].nominalTime == Catch::Approx(0.2));
    REQUIRE(logs.gps[1].fixType == 0);
    REQUIRE(logs.gps[0].horizontalAccuracy_m == Catch::Approx(1.5));
    REQUIRE(logs.baro[0].pressure_Pa == Catch::Approx(70000.0));
    REQUIRE(logs.truth[0].vNed_m_s[0] == Catch::Approx(121.9));
    REQUIRE(logs.truth[0].yaw_rad == Catch::Approx(0.7854));
    REQUIRE(logs.run.alt0_m == Catch::Approx(10013.0 * 0.3048));
    REQUIRE(logs.run.gpsLatency_s == Catch::Approx(0.2));
    REQUIRE(logs.run.raw.at("check_case") == "11");

    // The 16-state configuration needs GNSS velocity, which this GPS log lacks.
    std::stringstream gps2;
    gps2 << kGpsHeader << "\n";
    std::stringstream imu2, baro2, truth2;
    imu2 << kImuHeader << "\n";
    baro2 << kBaroHeader << "\n";
    truth2 << kTruthHeader << "\n";
    REQUIRE_THROWS_WITH(loadLogs(LogStreams{ &gps2, &imu2, &baro2, &truth2, sidecar }, kNavBaroGnss16),
        ContainsSubstring("vel_north_mps"));

    // A configuration without the barometer does not need the baro log.
    const Configuration gnssOnly{ { BlockId::NavCore, BlockId::GyroBias, BlockId::AccelBias },
        { MeasurementId::GnssPosition } };
    std::stringstream gps3, imu3, truth3;
    gps3 << kGpsHeader << "\n";
    imu3 << kImuHeader << "\n";
    truth3 << kTruthHeader << "\n";
    REQUIRE_NOTHROW(loadLogs(LogStreams{ &gps3, &imu3, nullptr, &truth3, sidecar }, gnssOnly));
}

// ---------------------------------------------------------------------------
// End to end: synthetic logs in Hemerion's formats from the analytic truth,
// written to files, loaded, replayed, judged.
// ---------------------------------------------------------------------------
namespace {
    struct SyntheticLogs
    {
        std::filesystem::path dir;
        std::filesystem::path truth;
        double hostOffset = 1000.0, hostScale = 1.0 + 2e-5, gpsLatency = 0.2;
    };

    /// Hemerion's drawn sensor errors (its step 3) in the synthetic logs: the
    /// IMU biases are drawn per seed from the filter's prior and walk at the
    /// filter's densities, the barometer carries a constant pressure bias in
    /// pascals added to the pressure (Hemerion's BMP390 model), and the truth
    /// log carries every drawn-error column plus the plant's static pressure
    /// and temperature, as Hemerion writes them. Scale error and misalignment
    /// are logged as zero; their equivalent bias is tested on its own.
    struct DrawnSpec
    {
        double gyroBiasSigma = 0.005, accelBiasSigma = 0.1, pressureBiasSigma_Pa = 100.0;
        double gyroBiasWalk = 2e-6, accelBiasWalk = 2e-5; ///< [unit/sqrt(s)], as ProcessNoise
    };

    constexpr std::string_view kDrawnTruthColumns =
        ", f16::out.P_Pa[REAL], f16::out.T_K[REAL], imu::accel_bias_x_mps2[REAL], imu::accel_bias_y_mps2[REAL], "
        "imu::accel_bias_z_mps2[REAL], imu::gyro_bias_x_rad_s[REAL], imu::gyro_bias_y_rad_s[REAL], "
        "imu::gyro_bias_z_rad_s[REAL], imu::accel_scale_error_x[REAL], imu::accel_scale_error_y[REAL], "
        "imu::accel_scale_error_z[REAL], imu::gyro_scale_error_x[REAL], imu::gyro_scale_error_y[REAL], "
        "imu::gyro_scale_error_z[REAL], imu::accel_misalignment_x_rad[REAL], imu::accel_misalignment_y_rad[REAL], "
        "imu::accel_misalignment_z_rad[REAL], imu::gyro_misalignment_x_rad[REAL], imu::gyro_misalignment_y_rad[REAL], "
        "imu::gyro_misalignment_z_rad[REAL], baro::pressure_bias_pa[REAL], baro::temperature_bias_c[REAL]";

    SyntheticLogs writeSyntheticLogs(double T, unsigned seed, const DrawnSpec* drawn = nullptr)
    {
        using Aetherion::Estimation::Testing::SyntheticTruth;
        const double lat0 = 36.0192 * kDeg, lon0 = -75.6744 * kDeg, alt0_ft = 10013.0;
        const LciFrame<double> frame = LciFrame<double>::FromGeodetic(lat0, lon0, alt0_ft * 0.3048, 0.0);
        const MeasurementModels<double> meas(frame);
        const SyntheticTruth truth;

        SyntheticLogs out;
        // Separate directories: CTest runs the test cases as separate processes, possibly at once.
        out.dir = std::filesystem::temp_directory_path() /
                  (drawn ? "aetherion_replay_test_drawn" : "aetherion_replay_test");
        std::filesystem::create_directories(out.dir);
        out.truth = out.dir / "f16_truth.csv";

        Aetherion::Estimation::Testing::PortableGaussian N01(seed);
        Eigen::Vector3d gyroBias(0.002, -0.001, 0.0015), accelBias(0.05, -0.02, 0.03);
        double baroBias = 12.0;
        const double gyroNd = 5e-5, accelNd = 7e-4, sigH = 1.5, sigV = 3.0, sigVel = 0.1, sigBaro = 1.0;
        const double dt = 0.01;
        // The drawn errors come from their own stream, so the sensor noise of a
        // seed is the same sequence with or without them.
        Aetherion::Estimation::Testing::PortableGaussian D01(seed + 7919U);
        double pressureBias_Pa = 0.0;
        if (drawn)
        {
            gyroBias = D01.vector(drawn->gyroBiasSigma, drawn->gyroBiasSigma, drawn->gyroBiasSigma);
            accelBias = D01.vector(drawn->accelBiasSigma, drawn->accelBiasSigma, drawn->accelBiasSigma);
            pressureBias_Pa = drawn->pressureBiasSigma_Pa * D01();
            baroBias = 0.0; // the bias is in the pressure instead
        }
        auto host = [&](double t) { return out.hostOffset + out.hostScale * t + 20e-6 * N01(); };

        // Geodetic view of the LCI truth at t.
        auto geodetic = [&](double t, double& lat, double& lon, double& h, Eigen::Vector3d& vNed, Eigen::Vector3d& euler) {
            const EstimationState<double> x = truth.at(t);
            const double era = frame.earthRotationAngle(t);
            const Eigen::Vector3d r_W = frame.toEci(x.X.p);
            const auto r_E = Aetherion::Coordinate::ECIToECEF(std::array<double, 3>{ r_W[0], r_W[1], r_W[2] }, era);
            Aetherion::Coordinate::ECEFToGeodeticWGS84(r_E, lat, lon, h);
            const Eigen::Vector3d v_W = frame.rotateToEci(x.X.v);
            const auto v_Ei = Aetherion::Coordinate::ECIToECEF(std::array<double, 3>{ v_W[0], v_W[1], v_W[2] }, era);
            const double w = WGS84::kRotationRate_rad_s;
            const std::array<double, 3> v_E{ v_Ei[0] + w * r_E[1], v_Ei[1] - w * r_E[0], v_Ei[2] }; // minus w_e x r_E
            const auto vn = Aetherion::Coordinate::ECEFToNED(v_E, lat, lon);
            vNed = Eigen::Vector3d(vn[0], vn[1], vn[2]);
            euler = eulerNedFromState(x, t, frame);
        };

        std::ofstream imu(out.dir / "imu_samples.csv"), gps(out.dir / "gps_fixes.csv"), baro(out.dir / "baro_samples.csv"),
            tr(out.truth), sc(out.dir / "f16_truth.config");
        imu << std::setprecision(12) << kImuHeader << "\n";
        gps << std::setprecision(12)
            << "fix_index,nominal_time_s,host_time_s,latitude_deg,longitude_deg,altitude_m,ground_speed_mps,course_deg,"
               "horizontal_accuracy_m,vertical_accuracy_m,num_satellites,fix_type,vel_north_mps,vel_east_mps,vel_down_mps,"
               "speed_accuracy_mps\n";
        baro << std::setprecision(12) << kBaroHeader << "\n";
        tr << std::setprecision(12) << kTruthHeader << (drawn ? kDrawnTruthColumns : std::string_view{}) << "\n";
        sc << "check_case=11\nlat0_deg=" << std::setprecision(12) << lat0 / kDeg << "\nlon0_deg=" << lon0 / kDeg
           << "\nalt0_ft=" << alt0_ft << "\ngps_latency_s=" << out.gpsLatency << "\nrealtime_factor=0\n";

        const int n = static_cast<int>(std::lround(T / dt));
        int fixIndex = 0, baroIndex = 0;
        for (int k = 0; k <= n; ++k)
        {
            const double t = k * dt;
            if (drawn && k > 0) // the biases walk; the truth row logs the value this step's sample uses
            {
                gyroBias += D01.vector(1.0, 1.0, 1.0) * drawn->gyroBiasWalk * std::sqrt(dt);
                accelBias += D01.vector(1.0, 1.0, 1.0) * drawn->accelBiasWalk * std::sqrt(dt);
            }
            // IMU: the instantaneous reading at t (no mid-step trick: that is the harness's job).
            const ImuSample<double> ideal = truth.imu(t, frame);
            const Eigen::Vector3d g =
                ideal.gyro_rad_s + gyroBias + N01.vector(1.0, 1.0, 1.0) * gyroNd / std::sqrt(dt);
            const Eigen::Vector3d a =
                ideal.accel_m_s2 + accelBias + N01.vector(1.0, 1.0, 1.0) * accelNd / std::sqrt(dt);
            imu << k << ',' << t << ',' << host(t) << ',' << a[0] << ',' << a[1] << ',' << a[2] << ',' << g[0] << ',' << g[1]
                << ',' << g[2] << '\n';

            double lat = 0.0, lon = 0.0, h = 0.0;
            Eigen::Vector3d vNed, euler;
            geodetic(t, lat, lon, h, vNed, euler);
            tr << k << ", " << t << ", " << h << ", " << lat / kDeg << ", " << lon / kDeg << ", " << vNed[0] << ", " << vNed[1]
               << ", " << vNed[2] << ", " << euler[0] << ", " << euler[1] << ", " << euler[2] << ", 0.0, false";
            if (drawn)
            {
                const auto air = Aetherion::Environment::US1976Atmosphere(h);
                tr << ", " << air.p << ", " << air.T << ", " << accelBias[0] << ", " << accelBias[1] << ", " << accelBias[2]
                   << ", " << gyroBias[0] << ", " << gyroBias[1] << ", " << gyroBias[2]
                   << ", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, " << pressureBias_Pa << ", 0.25";
            }
            tr << "\n";

            if (k % 20 == 0) // 5 Hz fix, delivered gpsLatency later on the host clock
            {
                const bool noFix = (fixIndex % 50 == 7); // the occasional dropout: Hemerion writes the epoch with empty fields
                const double latN = lat + (sigH * N01()) / 6371e3, lonN = lon + (sigH * N01()) / (6371e3 * std::cos(lat));
                const double hN = h + sigV * N01();
                const Eigen::Vector3d vN = vNed + N01.vector(1.0, 1.0, 1.0) * sigVel;
                if (noFix)
                    gps << fixIndex++ << ',' << t << ",,,,,,,,,,,,,,\n";
                else
                    gps << fixIndex++ << ',' << t << ',' << host(t + out.gpsLatency) << ',' << latN / kDeg << ',' << lonN / kDeg
                        << ',' << hN << ',' << vN.head<2>().norm() << ',' << std::atan2(vN[1], vN[0]) / kDeg << ',' << sigH << ','
                        << sigV << ",9," << LogContract::Gps::kFixType3D << ',' << vN[0] << ',' << vN[1] << ',' << vN[2] << ','
                        << sigVel << '\n';
            }
            if (k % 10 == 0) // 10 Hz barometer: pressure of (h + bias + noise)
            {
                const double p =
                    Aetherion::Environment::US1976Atmosphere(h + baroBias + sigBaro * N01()).p + pressureBias_Pa;
                baro << baroIndex++ << ',' << t << ',' << host(t) << ',' << p << ",15.0\n";
            }
        }
        return out;
    }
} // namespace

TEST_CASE("Replay: synthetic Hemerion logs through the harness, both conventions", "[estimation][replay][e2e]")
{
    const SyntheticLogs files = writeSyntheticLogs(120.0, 3);
    const LogSet logs = loadLogs(files.dir, files.truth, kNavBaroGnss16);
    REQUIRE(logs.imu.size() == 12001);
    REQUIRE(logs.gps.size() == 601);
    REQUIRE(logs.gps[0].hasVelocity);
    REQUIRE(logs.run.gpsLatency_s == Catch::Approx(files.gpsLatency));

    ReplayOptions opt;
    opt.noise = ProcessNoise<double>{ 5e-5, 7e-4, 2e-6, 2e-5, 0.05 };
    opt.initialError = Eigen::VectorXd::Zero(16);
    // Heading initialised from truth, as the plan says, with a realistic
    // initialisation error; the large-heading-error behaviour of the two
    // conventions is test_EKF.cpp's subject, not the harness's.
    opt.initialError.segment<3>(0) = Eigen::Vector3d(0.1 * kDeg, -0.1 * kDeg, 0.2 * kDeg);
    opt.initialError.segment<3>(6) = Eigen::Vector3d(5.0, -4.0, 3.0);
    opt.initialSigma = Eigen::VectorXd(16);
    opt.initialSigma << 2.0 * kDeg, 2.0 * kDeg, 5.0 * kDeg, 2.0, 2.0, 2.0, 30.0, 30.0, 30.0, 0.005, 0.005, 0.005, 0.1,
        0.1, 0.1, 20.0;

    for (const ErrorConvention c : { ErrorConvention::RightInvariant, ErrorConvention::LeftInvariant })
    {
        const ReplayResult r = replay(c, logs, kNavBaroGnss16, opt);
        const ReplaySummary& s = r.summary;
        std::ostringstream text;
        writeSummary(text, s);
        INFO(text.str());

        // The time base is recovered from the IMU log to the stamping jitter.
        CHECK(s.timeBase.offset == Catch::Approx(files.hostOffset).margin(1e-5));
        CHECK(s.timeBase.scale == Catch::Approx(files.hostScale).margin(1e-7));
        CHECK(s.timeBase.maxResidual < 1e-4);

        CHECK(s.imuSamples == 12000);
        // 601 epochs, 12 of them without a fix (fix_index 7, 57, ..., 557);
        // validity from nominal_time_s, so the one at t = 0 is usable too.
        CHECK(s.gnssPositionUpdates + s.skippedFixes == 601);
        CHECK(s.skippedFixes == 12);
        CHECK(s.gnssVelocityUpdates == s.gnssPositionUpdates);
        CHECK(s.baroUpdates == 1201);

        // Consistent, and at the sensor floor where the measurements reach.
        CHECK(s.aneesNav > 0.3);
        CHECK(s.aneesNav < 3.0);
        CHECK(s.nisGnssPosition > 0.4);
        CHECK(s.nisGnssPosition < 2.5);
        CHECK(s.nisGnssVelocity > 0.4);
        CHECK(s.nisGnssVelocity < 2.5);
        CHECK(s.nisBaro > 0.4);
        CHECK(s.nisBaro < 2.5);
        // Bounds checked over seeds 1-20 in both conventions (2026-10-14), all
        // passing; worst cases ANEES 0.44 / 2.04, position 0.49 m, tilt 1.24 deg,
        // yaw 2.78 deg. The conventions are indistinguishable over those seeds
        // (tilt 0.43 deg mean for both). On seed 3 alone LI has 0.11 deg of
        // tilt against RI's 0.90 deg; that is noise, not a convention effect.
        CHECK(s.posRms_m < 0.6);
        CHECK(s.velRms_m_s < 0.15);
        CHECK(s.tiltRms_deg < 1.5);
        CHECK(s.yawRms_deg < 3.0);

        // This truth log has no drawn-error columns: nav-core NEES only, and the summary says so.
        CHECK_FALSE(s.aneesGyroBias.valid);
        CHECK_FALSE(s.aneesAccelBias.valid);
        CHECK_FALSE(s.aneesBaroBias.valid);
        CHECK_FALSE(s.aneesFull.valid);
        CHECK_FALSE(s.leak.valid);
        CHECK(text.str().find("navigation core only") != std::string::npos);

        // The output writers run and agree with the rows.
        std::ostringstream states, updates;
        writeStates(states, r, kNavBaroGnss16);
        writeUpdates(updates, r);
        CHECK(states.str().find("nees_nav") != std::string::npos);
        CHECK(updates.str().find("gnss_position") != std::string::npos);
        CHECK(r.states.size() == 12001);
    }
}

TEST_CASE("Drawn errors: pressure bias in metres, unmodelled IMU terms as equivalent bias", "[estimation][replay][bias]")
{
    namespace Env = Aetherion::Environment;
    // Hemerion's check (its step 3): -44.7 Pa drawn at case 11's altitude
    // (alt0 10013 ft) read as +5.04 m by the filter's bias state. A negative
    // pressure bias is a positive pressure-altitude bias.
    const double h = 10013.0 * 0.3048;
    const auto air = Env::US1976Atmosphere(h);
    const double db = pressureBiasToMetres(-44.7, air.p);
    CHECK(db == Catch::Approx(5.04).margin(0.01));
    // and it is -dp / (rho g) at the flight's pressure, rho = P / (R T), g at h.
    constexpr double R = 287.05287, g0 = 9.80665, Re = 6356766.0;
    const double g = g0 * (Re / (Re + h)) * (Re / (Re + h));
    CHECK(db == Catch::Approx(44.7 / (air.p / (R * air.T) * g)).epsilon(1e-3));
    CHECK(pressureBiasToMetres(30.0, air.p) < 0.0);
    CHECK(pressureBiasToMetres(0.0, air.p) == 0.0);

    // y = diag(1 + s) (I + skew(m)) x + b: the equivalent bias is exactly y - b - x.
    const Eigen::Vector3d s(2e-3, -1e-3, 3e-3), m(1e-3, -2e-3, 0.5e-3), x(3.0, -1.0, -9.5), b(0.02, -0.01, 0.03);
    Eigen::Matrix3d K;
    K << 1.0, -m.z(), m.y(), m.z(), 1.0, -m.x(), -m.y(), m.x(), 1.0;
    const Eigen::Vector3d y = (Eigen::Vector3d::Ones() + s).asDiagonal() * (K * x) + b;
    CHECK((equivalentImuBias(s, m, y, b) - (y - b - x)).norm() < 1e-12);
    // First order: s .* x + m x x. Misalignment about z couples x into y with
    // the skew's sign; what is left on x is second order, m_z^2 |x|.
    const Eigen::Vector3d onlyMz = equivalentImuBias(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 1e-3),
        Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d::Zero());
    CHECK(onlyMz[1] == Catch::Approx(1e-2).epsilon(1e-5));
    CHECK(std::abs(onlyMz[0]) < 2e-5);
    CHECK(onlyMz[2] == 0.0);
    CHECK(equivalentImuBias(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), y, b).norm() == 0.0);
}

TEST_CASE("Truth log: drawn-error columns are optional and bound as groups", "[estimation][replay][logs][bias]")
{
    const std::string header = std::string(kTruthHeader) + std::string(kDrawnTruthColumns);
    std::stringstream full;
    full << header << "\n"
         << "0, 0.0, 3052.0, 36.0192, -75.6744, 121.9, 121.9, 0.0, 0.7854, 0.046, 0.0, 0.0, false, 69650.0, 268.3, "
            "0.01, -0.02, 0.03, 1e-3, -2e-3, 3e-3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -44.7, 0.2\n";
    const std::vector<TruthRecord> r = truthRecords(readCsv(full, LogContract::Log::Truth, kGnssPosBaro, "truth"));
    REQUIRE(r.size() == 1);
    CHECK(r[0].drawn.hasImuBias);
    CHECK(r[0].drawn.accelBias_m_s2[1] == Catch::Approx(-0.02));
    CHECK(r[0].drawn.gyroBias_rad_s[2] == Catch::Approx(3e-3));
    CHECK(r[0].drawn.hasImuScaleMisalignment);
    CHECK(r[0].drawn.hasPressureBias);
    CHECK(r[0].drawn.pressureBias_Pa == Catch::Approx(-44.7));
    CHECK(r[0].drawn.staticPressure_Pa == Catch::Approx(69650.0));

    // One column of the IMU bias group missing: the group is absent, the log still loads.
    std::string partial = header;
    partial.replace(partial.find("imu::gyro_bias_z_rad_s"), std::string_view("imu::gyro_bias_z_rad_s").size(), "imu::other");
    std::stringstream p;
    p << partial << "\n"
      << "0, 0.0, 3052.0, 36.0192, -75.6744, 121.9, 121.9, 0.0, 0.7854, 0.046, 0.0, 0.0, false, 69650.0, 268.3, "
         "0.01, -0.02, 0.03, 1e-3, -2e-3, 3e-3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -44.7, 0.2\n";
    const std::vector<TruthRecord> q = truthRecords(readCsv(p, LogContract::Log::Truth, kGnssPosBaro, "truth"));
    CHECK_FALSE(q[0].drawn.hasImuBias);
    CHECK(q[0].drawn.gyroBias_rad_s.isZero());
    CHECK(q[0].drawn.hasPressureBias);

    // Without any of them: nothing drawn, the plant's pressure not logged.
    std::stringstream none;
    none << kTruthHeader << "\n0, 0.0, 3052.0, 36.0192, -75.6744, 121.9, 121.9, 0.0, 0.7854, 0.046, 0.0, 0.0, false\n";
    const std::vector<TruthRecord> n = truthRecords(readCsv(none, LogContract::Log::Truth, kGnssPosBaro, "truth"));
    CHECK_FALSE(n[0].drawn.hasImuBias);
    CHECK_FALSE(n[0].drawn.hasImuScaleMisalignment);
    CHECK_FALSE(n[0].drawn.hasPressureBias);
    CHECK(std::isnan(n[0].drawn.staticPressure_Pa));
}

TEST_CASE("Replay: NEES per block against the drawn errors, seeds 1-20, both conventions", "[estimation][replay][e2e][bias]")
{
    const DrawnSpec spec;
    ReplayOptions opt;
    opt.noise = ProcessNoise<double>{ 5e-5, 7e-4, 2e-6, 2e-5, 0.05 };
    opt.initialError = Eigen::VectorXd::Zero(16);
    opt.initialError.segment<3>(0) = Eigen::Vector3d(0.1 * kDeg, -0.1 * kDeg, 0.2 * kDeg);
    opt.initialError.segment<3>(6) = Eigen::Vector3d(5.0, -4.0, 3.0);
    opt.initialSigma = Eigen::VectorXd(16);
    // Same as the nav-core test; the bias priors are the sigmas the biases are drawn with
    // (the baro prior, 20 m, is wider than the 100 Pa ~ 12 m draw).
    opt.initialSigma << 2.0 * kDeg, 2.0 * kDeg, 5.0 * kDeg, 2.0, 2.0, 2.0, 30.0, 30.0, 30.0, spec.gyroBiasSigma,
        spec.gyroBiasSigma, spec.gyroBiasSigma, spec.accelBiasSigma, spec.accelBiasSigma, spec.accelBiasSigma, 20.0;

    // Over seeds 1-20 (2026-11-13, MSVC 19.50 Release), second-half ANEES / dim:
    //                 nav core     gyro bias    accel bias   baro bias    full (16)
    //   RI  mean      1.25         1.71         1.21         0.70         1.30
    //       range     0.52-2.71    0.32-7.03    0.16-5.52    0.12-2.26    0.73-3.59
    //   LI  mean      1.05         1.05         1.13         0.70         1.09
    //       range     0.38-2.12    0.22-3.63    0.16-5.82    0.12-2.29    0.51-2.26
    // One seed's time average of a slowly varying 3-dof error is close to one
    // chi-square draw, so the per-seed bounds are loose and the mean over the
    // seeds carries the claim. The baro block sits below 1 by design: the
    // drawn pressure bias is constant, the filter's bias walks (0.05 m/sqrt(s)).
    // A flipped IMU-bias sign or a wrong-signed pressure conversion puts the
    // affected block in the hundreds (checked when this test was written).
    for (const ErrorConvention c : { ErrorConvention::RightInvariant, ErrorConvention::LeftInvariant })
    {
        double sumG = 0.0, sumA = 0.0, sumB = 0.0, sumFull = 0.0;
        for (unsigned seed = 1; seed <= 20; ++seed)
        {
            const SyntheticLogs files = writeSyntheticLogs(120.0, seed, &spec);
            const LogSet logs = loadLogs(files.dir, files.truth, kNavBaroGnss16);
            const ReplayResult r = replay(c, logs, kNavBaroGnss16, opt);
            const ReplaySummary& s = r.summary;
            std::ostringstream text;
            writeSummary(text, s);
            INFO("seed " << seed << ", " << toString(c) << "\n"
                         << text.str());

            REQUIRE(s.aneesGyroBias.valid);
            REQUIRE(s.aneesAccelBias.valid);
            REQUIRE(s.aneesBaroBias.valid);
            REQUIRE(s.aneesFull.valid);
            CHECK(text.str().find("navigation core (9), gyro bias (3), accel bias (3), baro bias (1), full state") !=
                  std::string::npos);

            // The nav-core bound of the test above, unchanged, on every seed.
            CHECK(s.aneesNav > 0.3);
            CHECK(s.aneesNav < 3.0);
            CHECK(s.aneesGyroBias.anees < 10.0);
            CHECK(s.aneesAccelBias.anees < 8.0);
            CHECK(s.aneesBaroBias.anees < 4.0);
            CHECK(s.aneesFull.anees > 0.3);
            CHECK(s.aneesFull.anees < 5.0);

            // Scale and misalignment are logged as zero: no leak, and the NEES
            // with the leak added is the NEES.
            CHECK(s.leak.valid);
            CHECK(s.leak.gyroMax_rad_s.norm() == 0.0);
            CHECK(s.leak.accelMax_m_s2.norm() == 0.0);
            CHECK(s.leak.gyroBias.anees == Catch::Approx(s.aneesGyroBias.anees));
            CHECK(s.leak.accelBias.anees == Catch::Approx(s.aneesAccelBias.anees));

            sumG += s.aneesGyroBias.anees;
            sumA += s.aneesAccelBias.anees;
            sumB += s.aneesBaroBias.anees;
            sumFull += s.aneesFull.anees;

            if (seed == 1)
            {
                // Per-step output: every block's NEES and the truth it was taken against.
                std::ostringstream states;
                writeStates(states, r, kNavBaroGnss16);
                const std::string csv = states.str();
                CHECK(csv.find(",nees_gyro_bias,nees_accel_bias,nees_baro_bias,nees_full,") != std::string::npos);
                CHECK(csv.find("nan") == std::string::npos);
                CHECK(r.states.back().neesFull.valid);
                // The baro truth is the drawn pressure bias at the flight's pressure, in metres.
                const BiasTruth& bt = r.states.back().biasTruth;
                CHECK(bt.staticPressureLogged);
                CHECK(bt.baroBias_m == Catch::Approx(pressureBiasToMetres(bt.pressureBias_Pa, bt.staticPressure_Pa)));
                CHECK(bt.baroBias_m * bt.pressureBias_Pa < 0.0);
            }
        }
        INFO(toString(c) << " mean ANEES over seeds 1-20: gyro " << sumG / 20.0 << ", accel " << sumA / 20.0
                         << ", baro " << sumB / 20.0 << ", full " << sumFull / 20.0);
        CHECK(sumG / 20.0 > 0.4);
        CHECK(sumG / 20.0 < 2.5);
        CHECK(sumA / 20.0 > 0.4);
        CHECK(sumA / 20.0 < 2.5);
        CHECK(sumB / 20.0 > 0.3);
        CHECK(sumB / 20.0 < 2.5);
        CHECK(sumFull / 20.0 > 0.5);
        CHECK(sumFull / 20.0 < 2.0);
    }
}

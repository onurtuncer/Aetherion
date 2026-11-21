// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// generate_golden_vectors: writes the golden vectors of the reference filter
// for one configuration and error convention (TODO-sensor-fusion.md, step 8;
// format in GoldenVectors.h, description in doc/codegen.rst).
//
//   generate_golden_vectors --out <dir> [--config nav-baro-gnss16]
//                           [--convention ri|li] [--duration <seconds>]
//
// writes <dir>/<config>_<convention>.txt, e.g.
// data/Estimation/GoldenVectors/nav-baro-gnss16_right-invariant.txt.
//
// The sequence is short and built to exercise the filter's arithmetic, not to
// judge its tuning: the banking, pitching SyntheticTruth of the estimator
// tests from t = 20 s, a sizeable initial error (so that the first updates
// make large corrections), an IMU at 100 Hz with bias and noise, GNSS
// position and velocity at 10 Hz whose validity times fall inside an IMU
// interval (the interval is split, as the replay harness does), with a full
// 3 x 3 covariance, and a barometer at 25 Hz. The process noise is the replay
// default. The inputs are drawn here; the outputs are whatever
// InvariantEKF<double, C> computes from them (Golden::runReference).
// ------------------------------------------------------------------------------

#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "../SyntheticTruth.h"
#include "GoldenVectors.h"

namespace {
    namespace Est = Aetherion::Estimation;
    namespace Golden = Aetherion::Estimation::Golden;
    using Est::Testing::PortableGaussian;
    using Est::Testing::SyntheticTruth;
    using Eigen::Matrix3d;
    using Eigen::Vector3d;

    constexpr double kDeg = std::numbers::pi / 180.0;
    constexpr double kT0 = 20.0;         // start on the synthetic truth [s]
    constexpr double kDt = 0.01;         // IMU period [s]
    constexpr double kFixPeriod = 0.1;   // GNSS period [s]
    constexpr double kFixOffset = 0.033; // first fix validity time after the start [s]: inside an IMU interval
    constexpr int kBaroEvery = 4;        // barometer after every 4th IMU step (25 Hz)

    /// The tolerances of the "float64" set: what a double-precision
    /// implementation of the same filter is held to. Measured basis in
    /// TODO-sensor-fusion.md (step 8) and doc/codegen.rst; no float32 set until
    /// step 7b has chosen the numerical form.
    std::map<std::string, Golden::Tolerance> float64Tolerances()
    {
        return {
            { "attitude", { 1e-10, 0.0 } },
            { "velocity", { 1e-9, 1e-10 } },
            { "position", { 1e-8, 1e-10 } },
            { "gyro_bias", { 1e-11, 1e-8 } },
            { "accel_bias", { 1e-10, 1e-8 } },
            { "baro_bias", { 1e-9, 1e-8 } },
            { "covariance", { 0.0, 1e-8 } },
            { "innovation", { 1e-8, 1e-8 } },
            { "nis", { 1e-8, 1e-8 } },
        };
    }

    void append(std::vector<double>& v, const Vector3d& a) { v.insert(v.end(), { a[0], a[1], a[2] }); }

    /// A GNSS record: the measurement and its LCI covariance C diag(sigma^2) C^T.
    Golden::Record fix(
        const std::string& kind, const Vector3d& truth, const Vector3d& sigma, const Matrix3d& C, PortableGaussian& g)
    {
        const Vector3d y = truth + C * g.vector(sigma[0], sigma[1], sigma[2]);
        Matrix3d Sigma = C * sigma.cwiseAbs2().asDiagonal() * C.transpose();
        Sigma = (0.5 * (Sigma + Sigma.transpose())).eval();
        Golden::Record r;
        r.kind = kind;
        append(r.input, y);
        Est::CodeGen::appendRowMajor(Sigma, r.input);
        return r;
    }

    Golden::Record predict(const Est::ImuSample<double>& u, double dt)
    {
        Golden::Record r;
        r.kind = "predict";
        append(r.input, u.gyro_rad_s);
        append(r.input, u.accel_m_s2);
        r.input.push_back(dt);
        return r;
    }

    template <Est::ErrorConvention C>
    Golden::File build(const Est::Configuration& cfg, const std::string& configName, double duration)
    {
        namespace SO3 = Aetherion::ODE::RKMK::Lie::SO3;
        const Est::LciFrame<double> frame = Est::LciFrame<double>::FromGeodetic(41.1 * kDeg, 29.0 * kDeg, 120.0, 1.2);
        const Est::ProcessNoise<double> noise{ 2e-4, 5e-3, 2e-6, 2e-5, 0.05 }; // the replay default
        const SyntheticTruth truth;
        PortableGaussian g(20261120);

        const Vector3d gyroBias(1e-3, -5e-4, 8e-4), accelBias(0.03, -0.02, 0.04);
        const double baroBias = 5.0;

        Est::InvariantEKF<double, C> ekf(cfg, frame, noise);
        Golden::File f;
        f.configuration = configName;
        f.convention = std::string(Est::toString(C));
        f.hash = cfg.hash(C);
        f.errorDim = cfg.dimension();
        f.noiseDim = Est::noiseDimension(cfg);
        f.stateDim = Est::CodeGen::StateLayout::kDim;
        f.tolerances["float64"] = float64Tolerances();
        f.frame = Est::CodeGen::packFrame(frame);
        const Eigen::MatrixXd Qc = ekf.processModel().continuousNoiseCovariance(cfg, noise);
        for (Eigen::Index i = 0; i < Qc.rows(); ++i) f.qcDiag.push_back(Qc(i, i));

        // Initial estimate: the truth with a physical error, biases unknown (zero).
        Est::EstimationState<double> x0 = truth.at(kT0);
        x0.X.R = SO3::Exp_R(Vector3d(2.0 * kDeg, -1.5 * kDeg, 4.0 * kDeg)) * x0.X.R;
        x0.X.v += Vector3d(0.8, -0.5, 0.3);
        x0.X.p += Vector3d(6.0, -4.0, 5.0);
        Eigen::VectorXd sigma0 = Eigen::VectorXd::Zero(cfg.dimension());
        const int nav = cfg.offset(Est::BlockId::NavCore);
        sigma0.segment<3>(nav + Est::NavCoreLayout::kAttitude) = Vector3d(3.0 * kDeg, 3.0 * kDeg, 5.0 * kDeg);
        sigma0.segment<3>(nav + Est::NavCoreLayout::kVelocity).setConstant(1.0);
        sigma0.segment<3>(nav + Est::NavCoreLayout::kPosition).setConstant(10.0);
        if (const int o = cfg.offset(Est::BlockId::GyroBias); o >= 0) sigma0.segment<3>(o).setConstant(2e-3);
        if (const int o = cfg.offset(Est::BlockId::AccelBias); o >= 0) sigma0.segment<3>(o).setConstant(0.05);
        if (const int o = cfg.offset(Est::BlockId::BaroBias); o >= 0) sigma0[o] = 10.0;
        f.state0 = Est::CodeGen::packState(x0);
        f.covariance0 = Golden::packUpper(ekf.covarianceFromPhysical(x0, sigma0.cwiseAbs2().asDiagonal()));

        const Matrix3d Cfix = SO3::Exp_R(Vector3d(0.02, -0.01, 0.03)); // receiver axes -> LCI: a full covariance
        const double imuScale = 1.0 / std::sqrt(kDt);
        const int steps = static_cast<int>(std::lround(duration / kDt));
        int nextFix = 0;
        for (int k = 0; k < steps; ++k) {
            const double t = k * kDt;
            Est::ImuSample<double> u = truth.imu(kT0 + t + 0.5 * kDt, frame);
            u.gyro_rad_s +=
                gyroBias + g.vector(noise.gyroNoise * imuScale, noise.gyroNoise * imuScale, noise.gyroNoise * imuScale);
            u.accel_m_s2 += accelBias + g.vector(noise.accelNoise * imuScale, noise.accelNoise * imuScale,
                                            noise.accelNoise * imuScale);

            const double tFix = kFixOffset + nextFix * kFixPeriod;
            if (tFix > t && tFix < t + kDt) {
                const Est::EstimationState<double> at = truth.at(kT0 + tFix);
                f.records.push_back(predict(u, tFix - t));
                f.records.push_back(fix("gnss_position", at.X.p, Vector3d(1.5, 1.5, 3.0), Cfix, g));
                f.records.push_back(fix("gnss_velocity", at.X.v, Vector3d(0.1, 0.1, 0.2), Cfix, g));
                f.records.push_back(predict(u, t + kDt - tFix));
                ++nextFix;
            } else {
                f.records.push_back(predict(u, kDt));
            }
            if ((k + 1) % kBaroEvery == 0) {
                Est::EstimationState<double> at = truth.at(kT0 + t + kDt);
                at.baroBias = baroBias;
                Golden::Record r;
                r.kind = "barometer";
                r.input = { ekf.measurementModels().predictBarometer(at) + 0.3 * g(), 1.0 };
                f.records.push_back(r);
            }
        }
        return Golden::runReference<C>(cfg, f);
    }

    int run(const std::map<std::string, std::string>& args)
    {
        const auto get = [&args](const std::string& key, const std::string& fallback) {
            const auto it = args.find(key);
            return it == args.end() ? fallback : it->second;
        };
        const std::string configName = get("config", "nav-baro-gnss16");
        if (configName != "nav-baro-gnss16")
            throw std::invalid_argument("unknown configuration '" + configName + "' (known: nav-baro-gnss16)");
        const Est::Configuration& cfg = Est::kNavBaroGnss16;
        const double duration = std::stod(get("duration", "1.0"));
        const std::string convention =
            get("convention", Est::kErrorConvention == Est::ErrorConvention::RightInvariant ? "ri" : "li");
        if (args.find("out") == args.end() || (convention != "ri" && convention != "li") || !(duration > 0.0))
            throw std::invalid_argument("bad or missing arguments");

        const Golden::File f = convention == "ri"
                                   ? build<Est::ErrorConvention::RightInvariant>(cfg, configName, duration)
                                   : build<Est::ErrorConvention::LeftInvariant>(cfg, configName, duration);
        const std::filesystem::path dir(args.at("out"));
        std::filesystem::create_directories(dir);
        const std::filesystem::path file = dir / (configName + "_" + f.convention + ".txt");
        std::ofstream os(file, std::ios::binary);
        Golden::write(os, f);
        if (!os) throw std::runtime_error("cannot write " + file.string());
        std::cout << file.string() << ": " << f.records.size() << " records, hash 0x" << std::hex << f.hash << "\n";
        return 0;
    }
} // namespace

int main(int argc, char** argv)
{
    try {
        std::map<std::string, std::string> args;
        for (int i = 1; i + 1 < argc; i += 2) {
            const std::string key = argv[i];
            if (key.rfind("--", 0) != 0) throw std::invalid_argument("expected '--option value', got '" + key + "'");
            args[key.substr(2)] = argv[i + 1];
        }
        if (argc % 2 == 0) throw std::invalid_argument("an option has no value");
        return run(args);
    } catch (const std::invalid_argument& e) {
        std::cerr << "generate_golden_vectors: " << e.what() << "\n"
                  << "usage: generate_golden_vectors --out <dir> [--config nav-baro-gnss16] [--convention ri|li] "
                     "[--duration <seconds>]\n";
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "generate_golden_vectors: " << e.what() << "\n";
        return 1;
    }
}

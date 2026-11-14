// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// CodeGenModels.h
//
// The estimator maps the CppADCodeGen compatibility check generates C for
// (TODO-sensor-fusion.md, "Things to check" and step 8), written once on a
// scalar S so that the same source is
//
//   - taped with CppAD::AD<CppAD::cg::CG<double>> by generate_estimation_cg,
//     which writes plain C for each map and its dense Jacobian;
//   - taped with CppAD::AD<double> by test_estimation_codegen, whose Jacobian
//     is the reference the generated one is checked against;
//   - evaluated in double, the reference for the generated values.
//
// Every map takes one flat input vector and returns one flat output vector,
// the shape the generated C functions have. The state is packed as
// [R (9, column-major), v, p, b_g, b_a, b_p] (22 entries), configuration
// kNavBaroGnss16, both error conventions:
//
//   process_error_C   in  [delta (16), state (22), gyro, accel, dt]   (45)
//                     out error_C(propagate(x), propagate(retract_C(x, delta)))
//                     -- d/d delta at delta = 0 is Phi
//   process_step_C    in  [state (22), gyro, accel, dt, diag Qc (13)] (42)
//                     out [x1 (22), Phi (16 x 16), Qd (16 x 16)] of step<C>
//   gnss_position_C,  in  [delta (16), state (22)]                     (38)
//   gnss_velocity_C,  out [z (m), H (m x 16), R (m x m)] of the model at x,
//   barometer_C           fed the measurement predicted at retract_C(x, delta)
//                     -- dz/d delta at delta = 0 is H
//
// Matrices are flattened column-major (Eigen's storage order).
// This is a test fixture, not the step-8 generator tool.
// ------------------------------------------------------------------------------

#pragma once

#include <array>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <string_view>
#include <vector>

#include <Eigen/Dense>

#include <Aetherion/Estimation/MeasurementModels.h>
#include <Aetherion/Estimation/ProcessModel.h>

namespace Aetherion::Estimation::CodeGenTest {

    inline constexpr const Configuration& kCfg = kNavBaroGnss16;
    inline constexpr int kN = 16;     // error-state dimension
    inline constexpr int kState = 22; // packed nominal state
    inline constexpr int kNoise = 13; // noise dimension of kNavBaroGnss16

    enum class Map
    {
        ProcessError,
        ProcessStep,
        GnssPosition,
        GnssVelocity,
        Barometer
    };

    struct ModelSpec
    {
        std::string_view name; ///< C identifier stem, also the Catch2 section name
        Map map;
        ErrorConvention convention;
        int nIn;
        int nOut;
    };

    [[nodiscard]] constexpr int measurementDim(Map m) { return m == Map::Barometer ? 1 : 3; }

    [[nodiscard]] constexpr int inputDim(Map m)
    {
        switch (m) {
            case Map::ProcessError:
                return kN + kState + 7;
            case Map::ProcessStep:
                return kState + 7 + kNoise;
            default:
                return kN + kState;
        }
    }

    [[nodiscard]] constexpr int outputDim(Map m)
    {
        switch (m) {
            case Map::ProcessError:
                return kN;
            case Map::ProcessStep:
                return kState + 2 * kN * kN;
            default: {
                const int k = measurementDim(m);
                return k + k * kN + k * k;
            }
        }
    }

    /// Sizes of the output blocks of a map, in output order: [x1, Phi, Qd],
    /// [z, H, R] or the single error vector.
    [[nodiscard]] inline std::vector<int> outputBlocks(Map m)
    {
        switch (m) {
            case Map::ProcessError:
                return { kN };
            case Map::ProcessStep:
                return { kState, kN * kN, kN * kN };
            default: {
                const int k = measurementDim(m);
                return { k, k * kN, k * k };
            }
        }
    }

    inline constexpr ErrorConvention kRI = ErrorConvention::RightInvariant;
    inline constexpr ErrorConvention kLI = ErrorConvention::LeftInvariant;

    inline constexpr std::array<ModelSpec, 10> kModels{ {
        { "process_error_ri", Map::ProcessError, kRI, inputDim(Map::ProcessError), outputDim(Map::ProcessError) },
        { "process_error_li", Map::ProcessError, kLI, inputDim(Map::ProcessError), outputDim(Map::ProcessError) },
        { "process_step_ri", Map::ProcessStep, kRI, inputDim(Map::ProcessStep), outputDim(Map::ProcessStep) },
        { "process_step_li", Map::ProcessStep, kLI, inputDim(Map::ProcessStep), outputDim(Map::ProcessStep) },
        { "gnss_position_ri", Map::GnssPosition, kRI, inputDim(Map::GnssPosition), outputDim(Map::GnssPosition) },
        { "gnss_position_li", Map::GnssPosition, kLI, inputDim(Map::GnssPosition), outputDim(Map::GnssPosition) },
        { "gnss_velocity_ri", Map::GnssVelocity, kRI, inputDim(Map::GnssVelocity), outputDim(Map::GnssVelocity) },
        { "gnss_velocity_li", Map::GnssVelocity, kLI, inputDim(Map::GnssVelocity), outputDim(Map::GnssVelocity) },
        { "barometer_ri", Map::Barometer, kRI, inputDim(Map::Barometer), outputDim(Map::Barometer) },
        { "barometer_li", Map::Barometer, kLI, inputDim(Map::Barometer), outputDim(Map::Barometer) },
    } };

    // ---- the frame and the evaluation points ----------------------------------

    inline constexpr double kDeg = std::numbers::pi / 180.0;

    /// Kitty Hawk, 3 m ellipsoidal height, an arbitrary ERA: the frame of
    /// test_ProcessModel.cpp and test_MeasurementModels.cpp.
    inline LciFrame<double> testFrame() { return LciFrame<double>::FromGeodetic(36.0192 * kDeg, -75.6744 * kDeg, 3.0, 0.3); }

    template <class S>
    LciFrame<S> castFrame(const LciFrame<double>& f)
    {
        LciFrame<S> g;
        g.r_W0 = f.r_W0.template cast<S>();
        g.R_WL = f.R_WL.template cast<S>();
        g.era0_rad = S(f.era0_rad);
        return g;
    }

    /// An evaluation point: a perturbation, a nominal state and an IMU step.
    struct Point
    {
        std::array<double, kN> delta{};
        EstimationState<double> x;
        Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel = Eigen::Vector3d::Zero();
        double dt = 0.01;
        std::array<double, kNoise> qcDiag{};
    };

    /// Point A: the state of the existing double/AD<double> tests at delta = 0.
    /// Exp(w dt) takes its trigonometric branch (|w dt| ~ 1e-3), every Log its
    /// series branch (the error of a state against itself is the identity).
    inline Point pointA()
    {
        namespace SO3 = ODE::RKMK::Lie::SO3;
        Point p;
        p.x.X.R = SO3::Exp_R(Eigen::Vector3d(0.3, -0.2, 1.1));
        p.x.X.v = Eigen::Vector3d(120.0, 40.0, -5.0);
        p.x.X.p = Eigen::Vector3d(8000.0, -3000.0, -1500.0);
        p.x.gyroBias = Eigen::Vector3d(0.002, -0.001, 0.0015);
        p.x.accelBias = Eigen::Vector3d(0.05, -0.02, 0.03);
        p.x.baroBias = 12.0;
        p.gyro = Eigen::Vector3d(0.1, -0.05, 0.2);
        p.accel = Eigen::Vector3d(2.0, -1.0, -9.0);
        p.dt = 0.01;
        for (int i = 0; i < kNoise; ++i)
            p.qcDiag[static_cast<std::size_t>(i)] = 1e-4 * (1.0 + 0.1 * i);
        return p;
    }

    /// Point B: the opposite branches. The bias-corrected rate is ~2e-4 rad/s,
    /// so |w dt| ~ 2e-6 and Exp(w dt) takes its series branch (theta^2 < 1e-8);
    /// delta carries a 0.25 rad rotation, so retract's Exp and the error's Log
    /// take their trigonometric branches. Generated code that had a branch
    /// frozen at the taping point (A) cannot match the reference here.
    inline Point pointB()
    {
        namespace SO3 = ODE::RKMK::Lie::SO3;
        Point p;
        p.delta = { 0.15, -0.1, 0.17, 1.5, -0.7, 0.3, 25.0, -12.0, 6.0, 4e-4, -3e-4, 2e-4, 0.02, -0.01, 0.015, 3.0 };
        p.x.X.R = SO3::Exp_R(Eigen::Vector3d(-1.2, 0.4, 2.3));
        p.x.X.v = Eigen::Vector3d(-60.0, 210.0, 12.0);
        p.x.X.p = Eigen::Vector3d(-15000.0, 22000.0, -4200.0);
        p.x.gyroBias = Eigen::Vector3d(-0.003, 0.002, 0.001);
        p.x.accelBias = Eigen::Vector3d(-0.04, 0.06, 0.01);
        p.x.baroBias = -7.5;
        p.gyro = p.x.gyroBias + Eigen::Vector3d(1e-4, -2e-4, 5e-5);
        p.accel = Eigen::Vector3d(-1.0, 3.0, -9.5);
        p.dt = 0.005;
        for (int i = 0; i < kNoise; ++i)
            p.qcDiag[static_cast<std::size_t>(i)] = 2e-3 / (1.0 + i);
        return p;
    }

    // ---- packing -----------------------------------------------------------------

    template <class S>
    void packState(const EstimationState<double>& x, std::vector<S>& out)
    {
        for (int c = 0; c < 3; ++c)
            for (int r = 0; r < 3; ++r)
                out.push_back(S(x.X.R(r, c)));
        for (const Eigen::Vector3d* v : { &x.X.v, &x.X.p, &x.gyroBias, &x.accelBias })
            for (int i = 0; i < 3; ++i)
                out.push_back(S((*v)[i]));
        out.push_back(S(x.baroBias));
    }

    template <class S>
    EstimationState<S> unpackState(const std::vector<S>& in, std::size_t o)
    {
        EstimationState<S> x;
        for (int c = 0; c < 3; ++c)
            for (int r = 0; r < 3; ++r)
                x.X.R(r, c) = in[o++];
        for (Eigen::Matrix<S, 3, 1>* v : { &x.X.v, &x.X.p, &x.gyroBias, &x.accelBias })
            for (int i = 0; i < 3; ++i)
                (*v)[i] = in[o++];
        x.baroBias = in[o];
        return x;
    }

    template <class S>
    void packMatrix(const Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>& M, std::vector<S>& out)
    {
        for (Eigen::Index c = 0; c < M.cols(); ++c)
            for (Eigen::Index r = 0; r < M.rows(); ++r)
                out.push_back(M(r, c));
    }

    /// The flat input vector of model m at point p.
    inline std::vector<double> inputs(const ModelSpec& m, const Point& p)
    {
        std::vector<double> in;
        if (m.map != Map::ProcessStep)
            in.assign(p.delta.begin(), p.delta.end());
        packState(p.x, in);
        if (m.map == Map::ProcessError || m.map == Map::ProcessStep) {
            for (int i = 0; i < 3; ++i)
                in.push_back(p.gyro[i]);
            for (int i = 0; i < 3; ++i)
                in.push_back(p.accel[i]);
            in.push_back(p.dt);
        }
        if (m.map == Map::ProcessStep)
            in.insert(in.end(), p.qcDiag.begin(), p.qcDiag.end());
        if (static_cast<int>(in.size()) != m.nIn)
            throw std::logic_error("CodeGenModels: input size mismatch");
        return in;
    }

    // ---- the maps ------------------------------------------------------------------

    template <class S>
    using VecXS = Eigen::Matrix<S, Eigen::Dynamic, 1>;
    template <class S>
    using MatXS = Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>;

    template <ErrorConvention C, class S>
    std::vector<S> evaluateConvention(Map map, const LciFrame<S>& frame, const std::vector<S>& in)
    {
        std::vector<S> out;
        if (map == Map::ProcessStep) {
            const StrapdownProcessModel<S> model(frame);
            const EstimationState<S> x = unpackState(in, 0);
            const std::size_t o = kState;
            const ImuSample<S> u{ { in[o], in[o + 1], in[o + 2] }, { in[o + 3], in[o + 4], in[o + 5] } };
            const S dt = in[o + 6];
            MatXS<S> Qc = MatXS<S>::Zero(kNoise, kNoise);
            for (int i = 0; i < kNoise; ++i)
                Qc(i, i) = in[o + 7 + static_cast<std::size_t>(i)];
            MatXS<S> Phi, Qd;
            const EstimationState<S> x1 = model.template step<C>(kCfg, x, u, dt, Qc, Phi, Qd);
            // x1 in the same layout as the packed input state.
            for (int c = 0; c < 3; ++c)
                for (int r = 0; r < 3; ++r)
                    out.push_back(x1.X.R(r, c));
            for (const Eigen::Matrix<S, 3, 1>* v : { &x1.X.v, &x1.X.p, &x1.gyroBias, &x1.accelBias })
                for (int i = 0; i < 3; ++i)
                    out.push_back((*v)[i]);
            out.push_back(x1.baroBias);
            packMatrix(Phi, out);
            packMatrix(Qd, out);
            return out;
        }

        VecXS<S> delta(kN);
        for (int i = 0; i < kN; ++i)
            delta[i] = in[static_cast<std::size_t>(i)];
        const EstimationState<S> x = unpackState(in, kN);
        const EstimationState<S> truth = x.template retract<C>(kCfg, delta);

        if (map == Map::ProcessError) {
            const StrapdownProcessModel<S> model(frame);
            const std::size_t o = kN + kState;
            const ImuSample<S> u{ { in[o], in[o + 1], in[o + 2] }, { in[o + 3], in[o + 4], in[o + 5] } };
            const S dt = in[o + 6];
            const VecXS<S> xi = model.propagate(x, u, dt).template error<C>(kCfg, model.propagate(truth, u, dt));
            for (int i = 0; i < kN; ++i)
                out.push_back(xi[i]);
            return out;
        }

        const MeasurementModels<S> m(frame);
        Eigen::Matrix<S, 3, 3> Sigma = Eigen::Matrix<S, 3, 3>::Zero();
        Sigma(0, 0) = S(1.5 * 1.5);
        Sigma(1, 1) = S(1.5 * 1.5);
        Sigma(2, 2) = S(3.0 * 3.0);
        VecXS<S> z;
        MatXS<S> H, R;
        switch (map) {
            case Map::GnssPosition:
                m.template gnssPosition<C>(kCfg, x, MeasurementModels<S>::predictGnssPosition(truth), Sigma, z, H, R);
                break;
            case Map::GnssVelocity:
                m.template gnssVelocity<C>(kCfg, x, MeasurementModels<S>::predictGnssVelocity(truth), Sigma, z, H, R);
                break;
            default:
                m.template barometer<C>(kCfg, x, m.predictBarometer(truth), S(2.0), z, H, R);
                break;
        }
        for (Eigen::Index i = 0; i < z.size(); ++i)
            out.push_back(z[i]);
        packMatrix(H, out);
        packMatrix(R, out);
        return out;
    }

    /// Evaluate model m on scalar S.
    template <class S>
    std::vector<S> evaluate(const ModelSpec& m, const LciFrame<S>& frame, const std::vector<S>& in)
    {
        std::vector<S> out = m.convention == kRI ? evaluateConvention<kRI>(m.map, frame, in)
                                                 : evaluateConvention<kLI>(m.map, frame, in);
        if (static_cast<int>(out.size()) != m.nOut)
            throw std::logic_error("CodeGenModels: output size mismatch");
        return out;
    }

} // namespace Aetherion::Estimation::CodeGenTest

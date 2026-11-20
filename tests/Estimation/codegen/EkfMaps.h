// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// EkfMaps.h
//
// The interface of the generated filter code (sensor fusion step 8): the
// functions generate_ekf_c emits for one configuration and one error
// convention, each as a map from one flat input vector to one flat output
// vector, written once on a scalar S so that the same source is
//
//   - taped with CppAD::AD<CppAD::cg::CG<double>> by generate_ekf_c, which
//     writes it out as plain C;
//   - evaluated in double, the reference the generated C is checked against
//     (test_EkfCodeGen.cpp) and the packing the golden vectors use
//     (golden/GoldenVectors.h).
//
// Every map calls the reference filter's own model code (ProcessModel.h,
// MeasurementModels.h, EstimationState.h): the generated functions are those
// expressions, not a second derivation.
//
//   propagate       f: one IMU step of the nonlinear model
//                   in  [state, gyro, accel, dt, frame]
//                   out [state]
//   transition      the Jacobian side of f: discrete error transition Phi and
//                   process noise Qd of step<C> over the same IMU step
//                   in  [state, gyro, accel, dt, frame, qc_diag]
//                   out [phi (n x n), qd (n x n)]
//   gnss_position,  h and its Jacobian: innovation z, H and the measurement
//   gnss_velocity   covariance R in the convention's innovation coordinates
//                   in  [state, y (LCI), meas_cov (3 x 3, LCI)]
//                   out [z (3), h (3 x n), r (3 x 3)]
//   barometer       in  [state, y (pressure altitude, m), meas_sigma, frame]
//                   out [z (1), h (1 x n), r (1)]
//   retract         the update's state correction x <- retract<C>(x, delta)
//                   in  [state, delta (n)]
//                   out [state]
//
// n = cfg.dimension() and the noise dimension come from the configuration;
// nothing here names a state index. A measurement function exists only if the
// configuration carries the measurement.
//
// Packing. The nominal state is [R (9), v, p, b_g, b_a, b_p] (StateLayout; a
// state always carries every library block, see EstimationState.h). The LCI
// frame is [r_W0 (3), R_WL (9)]; era0 is not an input because no generated
// function uses it (measurements arrive in LCI, as at the InvariantEKF
// interface). All matrices are row-major, the natural order for C.
// ------------------------------------------------------------------------------

#pragma once

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Dense>

#include <Aetherion/Estimation/MeasurementModels.h>
#include <Aetherion/Estimation/ProcessModel.h>

namespace Aetherion::Estimation::CodeGen {

    enum class Function
    {
        Propagate,
        Transition,
        GnssPosition,
        GnssVelocity,
        Barometer,
        Retract
    };

    inline constexpr std::array<Function, 6> kFunctions{ Function::Propagate, Function::Transition,
        Function::GnssPosition, Function::GnssVelocity, Function::Barometer, Function::Retract };

    /// C identifier stem of a function.
    [[nodiscard]] constexpr std::string_view name(Function f)
    {
        switch (f) {
            case Function::Propagate:
                return "propagate";
            case Function::Transition:
                return "transition";
            case Function::GnssPosition:
                return "gnss_position";
            case Function::GnssVelocity:
                return "gnss_velocity";
            case Function::Barometer:
                return "barometer";
            case Function::Retract:
                return "retract";
        }
        return "?";
    }

    /// True if the configuration has function f: a measurement function exists
    /// only for a measurement the configuration carries.
    [[nodiscard]] constexpr bool carried(const Configuration& cfg, Function f)
    {
        switch (f) {
            case Function::GnssPosition:
                return cfg.has(MeasurementId::GnssPosition);
            case Function::GnssVelocity:
                return cfg.has(MeasurementId::GnssVelocity);
            case Function::Barometer:
                return cfg.has(MeasurementId::Barometer);
            default:
                return true;
        }
    }

    /// Packed nominal state: offsets and total size.
    namespace StateLayout {
        inline constexpr int kAttitude = 0;   ///< R, body -> LCI, row-major (9)
        inline constexpr int kVelocity = 9;   ///< v in LCI [m/s]
        inline constexpr int kPosition = 12;  ///< p in LCI [m]
        inline constexpr int kGyroBias = 15;  ///< [rad/s], body
        inline constexpr int kAccelBias = 18; ///< [m/s^2], body
        inline constexpr int kBaroBias = 21;  ///< [m] of pressure altitude
        inline constexpr int kDim = 22;
    } // namespace StateLayout

    /// Packed LCI frame: offsets and total size.
    namespace FrameLayout {
        inline constexpr int kOrigin = 0;   ///< r_W0, ECI position of the origin [m]
        inline constexpr int kRotation = 3; ///< R_WL, LCI -> ECI, row-major (9)
        inline constexpr int kDim = 12;
    } // namespace FrameLayout

    /// A named run of entries in a function's input or output vector.
    struct Segment
    {
        std::string_view name;
        int size;
    };

    [[nodiscard]] inline std::vector<Segment> inputSegments(const Configuration& cfg, Function f)
    {
        const int n = cfg.dimension();
        const Segment state{ "state", StateLayout::kDim };
        const Segment frame{ "frame", FrameLayout::kDim };
        switch (f) {
            case Function::Propagate:
                return { state, { "gyro", 3 }, { "accel", 3 }, { "dt", 1 }, frame };
            case Function::Transition:
                return { state, { "gyro", 3 }, { "accel", 3 }, { "dt", 1 }, frame, { "qc_diag", noiseDimension(cfg) } };
            case Function::GnssPosition:
            case Function::GnssVelocity:
                return { state, { "y", 3 }, { "meas_cov", 9 } };
            case Function::Barometer:
                return { state, { "y", 1 }, { "meas_sigma", 1 }, frame };
            case Function::Retract:
                return { state, { "delta", n } };
        }
        return {};
    }

    [[nodiscard]] inline std::vector<Segment> outputSegments(const Configuration& cfg, Function f)
    {
        const int n = cfg.dimension();
        switch (f) {
            case Function::Propagate:
            case Function::Retract:
                return { { "state", StateLayout::kDim } };
            case Function::Transition:
                return { { "phi", n * n }, { "qd", n * n } };
            case Function::GnssPosition:
            case Function::GnssVelocity:
                return { { "z", 3 }, { "h", 3 * n }, { "r", 9 } };
            case Function::Barometer:
                return { { "z", 1 }, { "h", n }, { "r", 1 } };
        }
        return {};
    }

    [[nodiscard]] inline int totalSize(const std::vector<Segment>& segments)
    {
        int n = 0;
        for (const Segment& s : segments)
            n += s.size;
        return n;
    }

    /// Offset of the named segment; throws if there is none.
    [[nodiscard]] inline int offsetOf(const std::vector<Segment>& segments, std::string_view segment)
    {
        int o = 0;
        for (const Segment& s : segments) {
            if (s.name == segment)
                return o;
            o += s.size;
        }
        throw std::logic_error("EkfMaps: no segment named " + std::string(segment));
    }

    // ---- packing -----------------------------------------------------------------

    template <class S, class Derived>
    void appendRowMajor(const Eigen::MatrixBase<Derived>& M, std::vector<S>& out)
    {
        for (Eigen::Index r = 0; r < M.rows(); ++r)
            for (Eigen::Index c = 0; c < M.cols(); ++c)
                out.push_back(M(r, c));
    }

    template <class S>
    void appendState(const EstimationState<S>& x, std::vector<S>& out)
    {
        appendRowMajor(x.X.R, out);
        for (const Eigen::Matrix<S, 3, 1>* v : { &x.X.v, &x.X.p, &x.gyroBias, &x.accelBias })
            for (int i = 0; i < 3; ++i)
                out.push_back((*v)[i]);
        out.push_back(x.baroBias);
    }

    template <class S>
    [[nodiscard]] Eigen::Matrix<S, 3, 1> readVec3(const std::vector<S>& in, int o)
    {
        const auto u = static_cast<std::size_t>(o);
        return { in[u], in[u + 1], in[u + 2] };
    }

    template <class S>
    [[nodiscard]] Eigen::Matrix<S, 3, 3> readMat3(const std::vector<S>& in, int o)
    {
        Eigen::Matrix<S, 3, 3> M;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                M(r, c) = in[static_cast<std::size_t>(o + 3 * r + c)];
        return M;
    }

    template <class S>
    [[nodiscard]] EstimationState<S> readState(const std::vector<S>& in, int o)
    {
        EstimationState<S> x;
        x.X.R = readMat3(in, o + StateLayout::kAttitude);
        x.X.v = readVec3(in, o + StateLayout::kVelocity);
        x.X.p = readVec3(in, o + StateLayout::kPosition);
        x.gyroBias = readVec3(in, o + StateLayout::kGyroBias);
        x.accelBias = readVec3(in, o + StateLayout::kAccelBias);
        x.baroBias = in[static_cast<std::size_t>(o + StateLayout::kBaroBias)];
        return x;
    }

    template <class S>
    [[nodiscard]] LciFrame<S> readFrame(const std::vector<S>& in, int o)
    {
        LciFrame<S> f;
        f.r_W0 = readVec3(in, o + FrameLayout::kOrigin);
        f.R_WL = readMat3(in, o + FrameLayout::kRotation);
        return f;
    }

    [[nodiscard]] inline std::vector<double> packState(const EstimationState<double>& x)
    {
        std::vector<double> out;
        appendState(x, out);
        return out;
    }

    [[nodiscard]] inline std::vector<double> packFrame(const LciFrame<double>& f)
    {
        std::vector<double> out{ f.r_W0[0], f.r_W0[1], f.r_W0[2] };
        appendRowMajor(f.R_WL, out);
        return out;
    }

    // ---- the maps ------------------------------------------------------------------

    namespace detail {
        template <class S>
        using VecXS = Eigen::Matrix<S, Eigen::Dynamic, 1>;
        template <class S>
        using MatXS = Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>;

        template <class S>
        [[nodiscard]] ImuSample<S> readImu(const std::vector<S>& in, const std::vector<Segment>& seg)
        {
            return { readVec3(in, offsetOf(seg, "gyro")), readVec3(in, offsetOf(seg, "accel")) };
        }

        template <ErrorConvention C, class S>
        void transition(const Configuration& cfg, const std::vector<S>& in, std::vector<S>& out)
        {
            const std::vector<Segment> seg = inputSegments(cfg, Function::Transition);
            const StrapdownProcessModel<S> model(readFrame(in, offsetOf(seg, "frame")));
            const int m = noiseDimension(cfg);
            const int q = offsetOf(seg, "qc_diag");
            MatXS<S> Qc = MatXS<S>::Zero(m, m);
            for (int i = 0; i < m; ++i)
                Qc(i, i) = in[static_cast<std::size_t>(q + i)];
            MatXS<S> Phi, Qd;
            const S dt = in[static_cast<std::size_t>(offsetOf(seg, "dt"))];
            (void)model.template step<C>(cfg, readState(in, 0), readImu(in, seg), dt, Qc, Phi, Qd);
            appendRowMajor(Phi, out);
            appendRowMajor(Qd, out);
        }

        template <ErrorConvention C, class S>
        void measurement(const Configuration& cfg, Function f, const std::vector<S>& in, std::vector<S>& out)
        {
            const std::vector<Segment> seg = inputSegments(cfg, f);
            const EstimationState<S> x = readState(in, 0);
            const int y = offsetOf(seg, "y");
            VecXS<S> z;
            MatXS<S> H, R;
            if (f == Function::Barometer) {
                const MeasurementModels<S> models(readFrame(in, offsetOf(seg, "frame")));
                const S sigma = in[static_cast<std::size_t>(offsetOf(seg, "meas_sigma"))];
                models.template barometer<C>(cfg, x, in[static_cast<std::size_t>(y)], sigma, z, H, R);
            } else {
                // The GNSS models do not use the frame: the fix is in LCI already.
                const MeasurementModels<S> models{ LciFrame<S>{} };
                const Eigen::Matrix<S, 3, 3> Sigma = readMat3(in, offsetOf(seg, "meas_cov"));
                if (f == Function::GnssPosition)
                    models.template gnssPosition<C>(cfg, x, readVec3(in, y), Sigma, z, H, R);
                else
                    models.template gnssVelocity<C>(cfg, x, readVec3(in, y), Sigma, z, H, R);
            }
            for (Eigen::Index i = 0; i < z.size(); ++i)
                out.push_back(z[i]);
            appendRowMajor(H, out);
            appendRowMajor(R, out);
        }
    } // namespace detail

    /// Evaluate function f of configuration cfg in error convention C on scalar S.
    template <ErrorConvention C, class S>
    [[nodiscard]] std::vector<S> evaluate(const Configuration& cfg, Function f, const std::vector<S>& in)
    {
        const std::vector<Segment> seg = inputSegments(cfg, f);
        if (!carried(cfg, f) || static_cast<int>(in.size()) != totalSize(seg))
            throw std::logic_error("EkfMaps: function not carried or input size mismatch: " + std::string(name(f)));

        std::vector<S> out;
        switch (f) {
            case Function::Propagate: {
                const StrapdownProcessModel<S> model(readFrame(in, offsetOf(seg, "frame")));
                const S dt = in[static_cast<std::size_t>(offsetOf(seg, "dt"))];
                appendState(model.propagate(readState(in, 0), detail::readImu(in, seg), dt), out);
                break;
            }
            case Function::Transition:
                detail::transition<C>(cfg, in, out);
                break;
            case Function::Retract: {
                const int n = cfg.dimension();
                const int o = offsetOf(seg, "delta");
                detail::VecXS<S> delta(n);
                for (int i = 0; i < n; ++i)
                    delta[i] = in[static_cast<std::size_t>(o + i)];
                appendState(readState(in, 0).template retract<C>(cfg, delta), out);
                break;
            }
            default:
                detail::measurement<C>(cfg, f, in, out);
                break;
        }
        if (static_cast<int>(out.size()) != totalSize(outputSegments(cfg, f)))
            throw std::logic_error("EkfMaps: output size mismatch: " + std::string(name(f)));
        return out;
    }

    /// evaluate() with the convention chosen at run time.
    template <class S>
    [[nodiscard]] std::vector<S>
    evaluate(const Configuration& cfg, ErrorConvention convention, Function f, const std::vector<S>& in)
    {
        return convention == ErrorConvention::RightInvariant ? evaluate<ErrorConvention::RightInvariant, S>(cfg, f, in)
                                                             : evaluate<ErrorConvention::LeftInvariant, S>(cfg, f, in);
    }

} // namespace Aetherion::Estimation::CodeGen

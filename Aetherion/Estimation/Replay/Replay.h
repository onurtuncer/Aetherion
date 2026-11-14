// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Replay.h
//
// The replay harness (sensor fusion step 6): runs the reference filter
// (EKF.h) over a LogSet of Hemerion's co-simulation logs and judges it
// against the truth log with NEES and NIS.
//
// Time. Every record is put on the simulation time base before anything
// else happens. What Hemerion's writers stamp (surveyed at its origin/main
// 7da6057, 2026-10-13):
//   IMU   part_time_s  the sensor's own clock; equal to simulation time
//                      unless the FMU's clock skew/jitter options are on.
//         host_time_s  arrival of the burst holding the sample, on the
//                      flight computer's clock -- bursty (one stamp per
//                      communication step) and, unpaced (rtf 0), with no
//                      fixed relation to simulation time at all.
//   baro  part_time_s  the BMP390 SENSORTIME register: simulation time,
//                      wrapping every 512 s. Unwrapped against the
//                      fit-mapped host time.
//   GNSS  nominal_time_s  the fix's epoch in simulation time, index-based,
//                      so the latency cancels in it. The validity time.
//                      Hemerion 7da6057 writes fix_count * fix_period
//                      with fix_count from 1, one period late (fix 1 is
//                      the t = 0 state); logs from it need -fix_period.
//         host_time_s  arrival on the flight computer's clock, latency
//                      included (NAV-PVT carries no time of its own). Used
//                      only if nominal_time_s is absent: mapped through
//                      the fit over the IMU log, minus the run's
//                      gps_latency_s.
// The fit host = offset + scale * part over the IMU log (TimeBase.h) is
// therefore a cross-check and a fallback, not the primary clock; its
// residual is reported. This is the replay reference, which may know the
// latency; the flight filter has to recover it from the arrival stamps
// (Hemerion step 10) and is judged against this reference by the golden
// vectors (step 9). The choices are in ReplayTiming so a different log
// convention is one struct away, not a rewrite.
//
// Order. Events are merged by time. Each IMU interval [t_k-1, t_k] is
// propagated with the readings held at the average of its two bounding
// samples (the trapezoidal rule, second order for a smooth signal, where
// holding the earlier sample would leave a ZOH lag of half a period that
// reads as a bias). A measurement whose validity time falls inside the
// interval splits it: predict to that time, apply, predict the remainder.
// Not "apply after the IMU step that reaches it": a fix stamped a few
// microseconds after an IMU boundary would then be applied a whole period
// late, and at 120 m/s a mean lag of half a 10 ms period is a 0.6 m
// along-track bias that rotates with the heading (seen on synthetic logs;
// ANEES 3.8 with every NIS at 1).
//
// Initialisation. From the truth at the first IMU sample: position,
// velocity and attitude (heading from truth, as the plan says; never from
// GNSS course), optionally perturbed; biases at zero. The LCI frame's
// origin is the sidecar's lat0/lon0/alt0, era0 = 0.
//
// Output. One row per IMU sample with the estimate, the truth, the error in
// the filter's own coordinates, its NEES, and the 1-sigma of each state;
// one row per update with its NIS; and a summary.
//
// NEES per block. The navigation core's NEES needs only the plant's truth.
// The bias blocks' NEES needs the sensors' realised errors, which Hemerion's
// FMUs publish since its be5a19a (its step 3) as optional truth columns
// (LogContract::OptionalTruth). When they are present the harness reports
// NEES per block -- nav core (9), gyro bias (3), accel bias (3), baro bias
// (1) -- and over the whole state; when they are absent it reports the nav
// core only. The summary says which. Every NEES is e^T P_bb^-1 e over the
// block's own marginal covariance, in the filter's error coordinates. The
// bias errors are additive in both conventions (truth - estimate,
// EstimationState::error); the conventions differ in the covariance.
//
// Bias conventions, checked against Hemerion's models (its
// imu_noise_model.h, bmp390_measurement_model.h, f16_trim_ecos host):
//   IMU   reading = scale * ((I + skew(m)) * truth) + bias + noise, per axis,
//         in the body frame (x nose, the plant's body axes; no mounting
//         rotation), gyro truth the body rate relative to ECI. The filter
//         uses w = w~ - b_g, a = a~ - b_a in the same body frame, so the
//         drawn bias *is* the bias state, same sign, same frame.
//   baro  pressure = ambient + pressure_bias_pa. The filter reads pressure
//         altitude y = h_p(p) (pressureAltitude_m: the standard atmosphere
//         inverted, geopotential converted to geometric height) and models
//         y = h + b_p, so b_p absorbs everything that separates the noise-free
//         reading h_p(P + dp) from the geometric height h. Its NEES truth
//         (maintainer decision 2026-11-13) is the sum of two terms, both in
//         metres of geometric height:
//           sensor      h_p(P + dp) - h_p(P)   the drawn pressure bias at the
//                       flight's ambient pressure P; -dp / (rho g) to first
//                       order (pressureBiasToMetres). A negative pressure bias
//                       is a positive altitude bias.
//           atmosphere  h_p(P) - h             the day's atmosphere, with P the
//                       plant's out.P_Pa and h its out.alt_m; zero to
//                       round-off on a standard day. It is what the random
//                       walk on b_p is for: it grows with height on a
//                       non-standard day.
//         truth b_p = sensor + atmosphere = h_p(P + dp) - h. Both terms are
//         reported as diagnostics. Without out.P_Pa the atmosphere term is
//         unknown: the standard atmosphere at h stands in for P, the truth is
//         the sensor term only, and the summary says so.
// Scale-factor error and misalignment are not modelled. The equivalent bias
// they put on each reading, (M - I) M^-1 (y - b) with M = diag(1 + s)
// (I + skew(m)), is computed per step from the logged readings and reported
// beside the bias NEES (rms, max, and the NEES with it added to the truth),
// so a bias NEES above 1 can be attributed.
// ------------------------------------------------------------------------------

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <functional>
#include <numbers>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Dense>

#include <Aetherion/Environment/Atmosphere.h>

#include <Aetherion/Estimation/EKF.h>
#include <Aetherion/Estimation/Replay/Logs.h>
#include <Aetherion/Estimation/Replay/TimeBase.h>
#include <Aetherion/Estimation/Replay/TruthState.h>

namespace Aetherion::Estimation::Replay {

    /// Which clock each record lives on, as functions of the record, the
    /// time-base fit and the run configuration. The defaults are the Hemerion
    /// convention as understood on 2026-10-13 (file header).
    struct ReplayTiming
    {
        std::function<double(const ImuRecord&, const TimeBaseFit&, const RunConfiguration&)> imu =
            [](const ImuRecord& r, const TimeBaseFit&, const RunConfiguration&) { return r.partTime; };

        /// The part clock, unwrapped: the multiple of the wrap period that puts
        /// it nearest the fit-mapped host time. Exact while the host mapping is
        /// good to a quarter period (128 s), which even an unpaced run is.
        std::function<double(const BaroRecord&, const TimeBaseFit&, const RunConfiguration&)> baro =
            [](const BaroRecord& r, const TimeBaseFit& f, const RunConfiguration&) {
                constexpr double W = LogContract::Baro::kPartTimeWrap_s;
                const double ref = f.toPart(r.hostTime);
                return r.partTime + W * std::round((ref - r.partTime) / W);
            };

        /// nominal_time_s when the log has it, else the mapped arrival time less the latency.
        std::function<double(const GpsRecord&, const TimeBaseFit&, const RunConfiguration&)> gps =
            [](const GpsRecord& r, const TimeBaseFit& f, const RunConfiguration& run) {
                if (std::isfinite(r.nominalTime))
                    return r.nominalTime;
                return f.toPart(r.hostTime) - run.gpsLatency_s;
            };
    };

    struct ReplayOptions
    {
        ProcessNoise<double> noise{ 2e-4, 5e-3, 2e-6, 2e-5, 0.05 }; ///< Hemerion's IMU densities at 100 Hz (its step 4)
        double baroSigma_m = 1.0;                                   ///< pressure-altitude noise given to the filter
        double gnssVelSigma_m_s = 0.1;                              ///< used when the log carries no speed accuracy
        double gnssHorizSigma_m = 1.5;                              ///< used when a fix carries no horizontal accuracy
        double gnssVertSigma_m = 3.0;                               ///< used when a fix carries no vertical accuracy
        double gnssHorizScale = 1.0;                                ///< sigma_N = sigma_E = gnssHorizScale * horizontal_accuracy_m
        int minFixType = LogContract::Gps::kFixType3D;              ///< Hemerion's enum (0 none, 1 2D, 2 3D); fixes below it are skipped
                                                                    /// Initial 1-sigma in physical coordinates: attitude (3, rad), velocity (3), position (3), then the bias blocks.
        Eigen::VectorXd initialSigma;
        /// Perturbation added to the truth at initialisation, in the same physical coordinates (default none).
        Eigen::VectorXd initialError;
        ReplayTiming timing;
        double outputEvery_s = 0.0; ///< 0: every IMU sample
    };

    struct UpdateRow
    {
        double t = 0.0;
        MeasurementId id{};
        int dim = 0;
        double nis = 0.0;
    };

    /// The part of a NEES that exists only when the truth carries what it needs.
    struct OptionalNees
    {
        bool valid = false;
        double value = 0.0;
    };

    /// The bias blocks' truth at one time, from the optional drawn-error columns.
    struct BiasTruth
    {
        bool imuBias = false;              ///< gyro and accel bias known
        bool imuScaleMisalignment = false; ///< scale-factor error and misalignment known
        bool baroBias = false;             ///< pressure bias known: baroBiasTruth_m and baroSensorTerm_m are valid
        Eigen::Vector3d gyroBias_rad_s = Eigen::Vector3d::Zero();
        Eigen::Vector3d accelBias_m_s2 = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyroScaleError = Eigen::Vector3d::Zero(), accelScaleError = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyroMisalignment_rad = Eigen::Vector3d::Zero(), accelMisalignment_rad = Eigen::Vector3d::Zero();
        double pressureBias_Pa = 0.0;
        double staticPressure_Pa = 0.0; ///< the flight's ambient pressure P the sensor term used
        /// out.P_Pa logged: P is the plant's, and baroAtmosphereTerm_m is valid and part of baroBiasTruth_m.
        /// Otherwise P is the standard atmosphere at the truth altitude and the truth is the sensor term only.
        bool staticPressureLogged = false;
        /// Sensor term, h_p(P + dp) - h_p(P) [m of geometric height]: the drawn pressure bias at P
        /// (pressureBiasToMetres).
        double baroSensorTerm_m = 0.0;
        /// Atmosphere term, h_p(P) - h [m of geometric height], with h the truth's out.alt_m: the day's
        /// atmosphere, zero to round-off on a standard day. Valid only if staticPressureLogged.
        double baroAtmosphereTerm_m = 0.0;
        /// The baro-bias block's NEES truth: baroSensorTerm_m + baroAtmosphereTerm_m = h_p(P + dp) - h, what
        /// separates the noise-free pressure altitude from the geometric height (maintainer decision
        /// 2026-11-13). Sensor term only when !staticPressureLogged.
        double baroBiasTruth_m = 0.0;
    };

    struct StateRow
    {
        double t = 0.0;
        EstimationState<double> estimate;
        EstimationState<double> truth; ///< bias members from the drawn errors where the log has them, else zero
        Eigen::VectorXd error;         ///< filter coordinates, navigation core + biases (truth - estimate)
        Eigen::VectorXd sigma;         ///< sqrt(diag P)
        double neesNav = 0.0;          ///< navigation core only (9 dof)
        OptionalNees neesGyroBias;     ///< 3 dof, against the drawn gyro bias
        OptionalNees neesAccelBias;    ///< 3 dof, against the drawn accel bias
        OptionalNees neesBaroBias;     ///< 1 dof, against BiasTruth::baroBiasTruth_m (sensor + atmosphere)
        OptionalNees neesFull;         ///< every block the configuration carries
        BiasTruth biasTruth;
        bool hasLeak = false;                                     ///< scale/misalignment known: the two below are meaningful
        Eigen::Vector3d gyroLeak_rad_s = Eigen::Vector3d::Zero(); ///< equivalent bias of the unmodelled terms
        Eigen::Vector3d accelLeak_m_s2 = Eigen::Vector3d::Zero(); ///< likewise
        OptionalNees neesGyroBiasLeak;                            ///< against drawn bias + equivalent bias
        OptionalNees neesAccelBiasLeak;                           ///< likewise
        Eigen::Vector3d eulerErr_rad;                             ///< yaw, pitch, roll error in local NED
    };

    /// Mean NEES / dof over the second half, for a block whose truth may be absent.
    struct BlockAnees
    {
        bool valid = false; ///< at least one row of the second half had the truth
        double anees = 0.0;
        std::size_t rows = 0;
    };

    /// The unmodelled IMU terms as equivalent bias over the second half.
    struct LeakSummary
    {
        bool valid = false;
        Eigen::Vector3d gyroRms_rad_s = Eigen::Vector3d::Zero(), gyroMax_rad_s = Eigen::Vector3d::Zero();
        Eigen::Vector3d accelRms_m_s2 = Eigen::Vector3d::Zero(), accelMax_m_s2 = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyroBiasSigma_rad_s = Eigen::Vector3d::Zero(); ///< the filter's mean 1-sigma, for scale
        Eigen::Vector3d accelBiasSigma_m_s2 = Eigen::Vector3d::Zero();
        BlockAnees gyroBias, accelBias; ///< NEES with the equivalent bias added to the truth
    };

    struct ReplaySummary
    {
        ErrorConvention convention{};
        TimeBaseFit timeBase;
        std::size_t imuSamples = 0, gnssPositionUpdates = 0, gnssVelocityUpdates = 0, baroUpdates = 0, skippedFixes = 0;
        // Records that never reached the filter. None of them can influence the estimate.
        std::size_t rejectedImuSamples = 0;   ///< a time or a reading not finite
        std::size_t skippedImuSamples = 0;    ///< duplicate or out-of-order time stamp
        std::size_t rejectedTruthRecords = 0; ///< a time or a field not finite
        std::size_t rejectedMeasurements = 0; ///< GNSS / baro record whose time is not finite
        std::size_t rejectedUpdates = 0;      ///< refused by the filter: value or covariance invalid (UpdateResult::rejected)
        double duration_s = 0.0;
        double aneesNav = 0.0;                                                        ///< mean NEES of the navigation core over the second half, / 9
        BlockAnees aneesGyroBias, aneesAccelBias, aneesBaroBias, aneesFull;           ///< second half, / dim; valid only with the drawn errors
        LeakSummary leak;                                                             ///< unmodelled scale and misalignment
        double posRms_m = 0.0, velRms_m_s = 0.0, tiltRms_deg = 0.0, yawRms_deg = 0.0; ///< second half
        double nisGnssPosition = 0.0, nisGnssVelocity = 0.0, nisBaro = 0.0;           ///< mean NIS / dim, second half
        /// The last state row, kept for the summary's end-of-run bias line.
        bool hasFinalBaro = false;
        double finalBaroEstimate_m = 0.0, finalBaroSigma_m = 0.0;
        BiasTruth finalBiasTruth;
        /// Second-half rows with a baro-bias truth, and those of them whose truth includes the atmosphere term.
        std::size_t baroTruthRows = 0, baroTruthRowsWithAtmosphere = 0;
    };

    struct ReplayResult
    {
        ReplaySummary summary;
        std::vector<StateRow> states;
        std::vector<UpdateRow> updates;
    };

    /// A barometer pressure bias as the filter's bias state sees it: metres of
    /// pressure altitude, h_p(P + dp) - h_p(P) at the flight's ambient pressure
    /// P, through the same inversion the filter applies to the reading
    /// (pressureAltitude_m, geometric height). To first order -dp / (rho g) with
    /// rho and g of the standard atmosphere at P; a negative dp reads high. This
    /// is the sensor term of the baro-bias truth only (BiasTruth::baroBiasTruth_m).
    [[nodiscard]] inline double pressureBiasToMetres(double pressureBias_Pa, double staticPressure_Pa)
    {
        return pressureAltitude_m(staticPressure_Pa + pressureBias_Pa) - pressureAltitude_m(staticPressure_Pa);
    }

    /// What an IMU triad's unmodelled scale-factor error s and misalignment m add
    /// to a reading y whose bias is b, as an equivalent bias:
    ///   y = M x + b + n,  M = diag(1 + s) (I + skew(m)),  leak = (M - I) x,
    /// with x recovered as M^-1 (y - b) (the white noise n passes through at
    /// order s n, far below the leak itself).
    [[nodiscard]] inline Eigen::Vector3d equivalentImuBias(const Eigen::Vector3d& scaleError,
        const Eigen::Vector3d& misalignment_rad,
        const Eigen::Vector3d& reading,
        const Eigen::Vector3d& bias)
    {
        Eigen::Matrix3d K;
        K << 1.0, -misalignment_rad.z(), misalignment_rad.y(), misalignment_rad.z(), 1.0, -misalignment_rad.x(),
            -misalignment_rad.y(), misalignment_rad.x(), 1.0;
        const Eigen::Matrix3d M = (Eigen::Vector3d::Ones() + scaleError).asDiagonal() * K;
        const Eigen::Vector3d x = M.partialPivLu().solve(reading - bias);
        return (M - Eigen::Matrix3d::Identity()) * x;
    }

    namespace detail {
        /// The truth rows either side of t and the interpolation weight.
        struct Bracket
        {
            const TruthRecord* a = nullptr;
            const TruthRecord* b = nullptr;
            double w = 0.0;
        };

        [[nodiscard]] inline Bracket bracket(const std::vector<TruthRecord>& truth, double t)
        {
            auto it =
                std::lower_bound(truth.begin(), truth.end(), t, [](const TruthRecord& r, double tt) { return r.time < tt; });
            Bracket br;
            br.b = (it == truth.end()) ? &truth.back() : &*it;
            br.a = (it == truth.begin()) ? br.b : &*(it - 1);
            br.w = (br.b->time > br.a->time) ? std::clamp((t - br.a->time) / (br.b->time - br.a->time), 0.0, 1.0) : 0.0;
            return br;
        }

        /// The drawn errors at t, interpolated like the rest of the truth. A group
        /// counts only if both bracketing rows carry it.
        [[nodiscard]] inline BiasTruth interpolateBiasTruth(const std::vector<TruthRecord>& truth, double t)
        {
            const Bracket br = bracket(truth, t);
            const DrawnErrors& A = br.a->drawn;
            const DrawnErrors& B = br.b->drawn;
            const double w = br.w;
            BiasTruth out;
            if (A.hasImuBias && B.hasImuBias)
            {
                out.imuBias = true;
                out.gyroBias_rad_s = A.gyroBias_rad_s + w * (B.gyroBias_rad_s - A.gyroBias_rad_s);
                out.accelBias_m_s2 = A.accelBias_m_s2 + w * (B.accelBias_m_s2 - A.accelBias_m_s2);
            }
            if (A.hasImuScaleMisalignment && B.hasImuScaleMisalignment)
            {
                out.imuScaleMisalignment = true; // drawn once per run: the earlier row's values
                out.gyroScaleError = A.gyroScaleError;
                out.accelScaleError = A.accelScaleError;
                out.gyroMisalignment_rad = A.gyroMisalignment_rad;
                out.accelMisalignment_rad = A.accelMisalignment_rad;
            }
            if (A.hasPressureBias && B.hasPressureBias)
            {
                const double alt = br.a->alt_m + w * (br.b->alt_m - br.a->alt_m);
                const double pa = A.staticPressure_Pa, pb = B.staticPressure_Pa;
                out.staticPressureLogged = std::isfinite(pa) && std::isfinite(pb) && pa > 0.0 && pb > 0.0;
                // Without the plant's pressure, the standard atmosphere at the
                // truth altitude stands in (exact for a standard day).
                out.staticPressure_Pa =
                    out.staticPressureLogged ? pa + w * (pb - pa) : Environment::US1976Atmosphere(alt).p;
                out.pressureBias_Pa = A.pressureBias_Pa + w * (B.pressureBias_Pa - A.pressureBias_Pa);
                out.baroSensorTerm_m = pressureBiasToMetres(out.pressureBias_Pa, out.staticPressure_Pa);
                out.baroBiasTruth_m = out.baroSensorTerm_m;
                if (out.staticPressureLogged)
                {
                    // Per row, then interpolated: each row's P and altitude are
                    // one plant evaluation, so on a standard day each row's
                    // term is zero to round-off and so is the interpolant
                    // (interpolating P itself would leave its curvature).
                    const double atmA = pressureAltitude_m(pa) - br.a->alt_m;
                    const double atmB = pressureAltitude_m(pb) - br.b->alt_m;
                    out.baroAtmosphereTerm_m = atmA + w * (atmB - atmA);
                    out.baroBiasTruth_m += out.baroAtmosphereTerm_m;
                }
                out.baroBias = std::isfinite(out.baroBiasTruth_m);
            }
            return out;
        }

        [[nodiscard]] inline GeodeticTruth interpolateTruth(const std::vector<TruthRecord>& truth, double t)
        {
            constexpr double kDeg = std::numbers::pi / 180.0;
            const Bracket br = bracket(truth, t);
            const TruthRecord* a = br.a;
            const TruthRecord* b = br.b;
            const double w = br.w;
            auto lerp = [w](double x, double y) { return x + w * (y - x); };
            auto lerpAngle = [w](double x, double y) {
                const double d = std::remainder(y - x, 2.0 * std::numbers::pi);
                return x + w * d;
            };
            GeodeticTruth g;
            g.t = t;
            g.lat_rad = lerp(a->lat_deg, b->lat_deg) * kDeg;
            g.lon_rad = lerpAngle(a->lon_deg * kDeg, b->lon_deg * kDeg);
            g.alt_m = lerp(a->alt_m, b->alt_m);
            g.vNed = a->vNed_m_s + w * (b->vNed_m_s - a->vNed_m_s);
            g.yaw_rad = lerpAngle(a->yaw_rad, b->yaw_rad);
            g.pitch_rad = lerp(a->pitch_rad, b->pitch_rad);
            g.roll_rad = lerp(a->roll_rad, b->roll_rad);
            return g;
        }

        /// The host <- part clock fit over the IMU log.
        [[nodiscard]] inline TimeBaseFit fitImuTimeBase(const std::vector<ImuRecord>& imu)
        {
            std::vector<double> part, host;
            part.reserve(imu.size());
            host.reserve(imu.size());
            for (const ImuRecord& r : imu)
            {
                part.push_back(r.partTime);
                host.push_back(r.hostTime);
            }
            return fitTimeBase(part, host);
        }

        /// Initial 1-sigma in physical coordinates when the options give none of the right size.
        [[nodiscard]] inline Eigen::VectorXd defaultInitialSigma(const Configuration& cfg)
        {
            constexpr double kDeg = std::numbers::pi / 180.0;
            Eigen::VectorXd sig = Eigen::VectorXd::Zero(cfg.dimension());
            sig.segment<3>(cfg.offset(BlockId::NavCore) + NavCoreLayout::kAttitude).setConstant(1.0 * kDeg);
            sig.segment<3>(cfg.offset(BlockId::NavCore) + NavCoreLayout::kVelocity).setConstant(1.0);
            sig.segment<3>(cfg.offset(BlockId::NavCore) + NavCoreLayout::kPosition).setConstant(10.0);
            if (const int o = cfg.offset(BlockId::GyroBias); o >= 0)
                sig.segment<3>(o).setConstant(2e-3);
            if (const int o = cfg.offset(BlockId::AccelBias); o >= 0)
                sig.segment<3>(o).setConstant(0.05);
            if (const int o = cfg.offset(BlockId::BaroBias); o >= 0)
                sig[o] = 20.0;
            return sig;
        }

        /// Physical perturbation of x: world-frame attitude error, dv, dp, biases.
        inline void perturb(EstimationState<double>& x, const Configuration& cfg, const Eigen::VectorXd& err)
        {
            const int nav = cfg.offset(BlockId::NavCore);
            const Eigen::Vector3d dth = err.segment<3>(nav + NavCoreLayout::kAttitude);
            x.X.R = ODE::RKMK::Lie::SO3::Exp_R(dth) * x.X.R;
            x.X.v += err.segment<3>(nav + NavCoreLayout::kVelocity);
            x.X.p += err.segment<3>(nav + NavCoreLayout::kPosition);
            if (const int o = cfg.offset(BlockId::GyroBias); o >= 0)
                x.gyroBias += err.segment<3>(o);
            if (const int o = cfg.offset(BlockId::AccelBias); o >= 0)
                x.accelBias += err.segment<3>(o);
            if (const int o = cfg.offset(BlockId::BaroBias); o >= 0)
                x.baroBias += err[o];
        }

        /// Fill the summary's statistics over the second half of [t0, tEnd].
        inline void summarise(ReplayResult& res, double t0, double tEnd)
        {
            constexpr double kDeg = std::numbers::pi / 180.0;
            ReplaySummary& S = res.summary;
            S.duration_s = tEnd - t0;
            const double tHalf = t0 + 0.5 * S.duration_s;
            double nees = 0.0, pos = 0.0, vel = 0.0, tilt = 0.0, yaw = 0.0;
            std::size_t cnt = 0;
            for (const StateRow& r : res.states)
            {
                if (r.t < tHalf)
                    continue;
                nees += r.neesNav;
                pos += (r.estimate.X.p - r.truth.X.p).squaredNorm();
                vel += (r.estimate.X.v - r.truth.X.v).squaredNorm();
                tilt += r.eulerErr_rad[1] * r.eulerErr_rad[1] + r.eulerErr_rad[2] * r.eulerErr_rad[2];
                yaw += r.eulerErr_rad[0] * r.eulerErr_rad[0];
                ++cnt;
            }
            if (cnt > 0)
            {
                const auto c = static_cast<double>(cnt);
                S.aneesNav = nees / c / 9.0;
                S.posRms_m = std::sqrt(pos / c);
                S.velRms_m_s = std::sqrt(vel / c);
                S.tiltRms_deg = std::sqrt(tilt / c / 2.0) / kDeg;
                S.yawRms_deg = std::sqrt(yaw / c) / kDeg;
            }
            double nisP = 0.0, nisV = 0.0, nisB = 0.0;
            std::size_t nP = 0, nV = 0, nB = 0;
            for (const UpdateRow& u : res.updates)
            {
                if (u.t < tHalf)
                    continue;
                switch (u.id)
                {
                    case MeasurementId::GnssPosition:
                        nisP += u.nis / u.dim;
                        ++nP;
                        break;
                    case MeasurementId::GnssVelocity:
                        nisV += u.nis / u.dim;
                        ++nV;
                        break;
                    case MeasurementId::Barometer:
                        nisB += u.nis / u.dim;
                        ++nB;
                        break;
                }
            }
            S.nisGnssPosition = nP ? nisP / static_cast<double>(nP) : 0.0;
            S.nisGnssVelocity = nV ? nisV / static_cast<double>(nV) : 0.0;
            S.nisBaro = nB ? nisB / static_cast<double>(nB) : 0.0;
        }

        /// Mean of an optional per-row NEES over the rows from tHalf on, / dof.
        template <class Get>
        [[nodiscard]] BlockAnees blockAnees(const std::vector<StateRow>& rows, double tHalf, int dof, Get get)
        {
            BlockAnees b;
            double sum = 0.0;
            for (const StateRow& r : rows)
            {
                const OptionalNees& n = get(r);
                if (r.t < tHalf || !n.valid)
                    continue;
                sum += n.value;
                ++b.rows;
            }
            b.valid = b.rows > 0;
            b.anees = b.valid ? sum / static_cast<double>(b.rows) / dof : 0.0;
            return b;
        }

        /// The bias blocks' share of the summary: per-block ANEES, the unmodelled
        /// IMU terms as equivalent bias, and the end-of-run barometer bias.
        inline void summariseBiases(ReplayResult& res, const Configuration& cfg, double tHalf)
        {
            ReplaySummary& S = res.summary;
            S.aneesGyroBias = blockAnees(res.states, tHalf, 3, [](const StateRow& r) -> const OptionalNees& { return r.neesGyroBias; });
            S.aneesAccelBias = blockAnees(res.states, tHalf, 3, [](const StateRow& r) -> const OptionalNees& { return r.neesAccelBias; });
            S.aneesBaroBias = blockAnees(res.states, tHalf, 1, [](const StateRow& r) -> const OptionalNees& { return r.neesBaroBias; });
            S.aneesFull = blockAnees(res.states, tHalf, cfg.dimension(), [](const StateRow& r) -> const OptionalNees& { return r.neesFull; });

            LeakSummary& L = S.leak;
            L.gyroBias = blockAnees(res.states, tHalf, 3, [](const StateRow& r) -> const OptionalNees& { return r.neesGyroBiasLeak; });
            L.accelBias = blockAnees(res.states, tHalf, 3, [](const StateRow& r) -> const OptionalNees& { return r.neesAccelBiasLeak; });
            const int bg = cfg.offset(BlockId::GyroBias), ba = cfg.offset(BlockId::AccelBias);
            std::size_t n = 0;
            for (const StateRow& r : res.states)
            {
                if (r.t < tHalf || !r.hasLeak)
                    continue;
                L.gyroRms_rad_s += r.gyroLeak_rad_s.cwiseAbs2();
                L.accelRms_m_s2 += r.accelLeak_m_s2.cwiseAbs2();
                L.gyroMax_rad_s = L.gyroMax_rad_s.cwiseMax(r.gyroLeak_rad_s.cwiseAbs());
                L.accelMax_m_s2 = L.accelMax_m_s2.cwiseMax(r.accelLeak_m_s2.cwiseAbs());
                if (bg >= 0)
                    L.gyroBiasSigma_rad_s += r.sigma.segment<3>(bg);
                if (ba >= 0)
                    L.accelBiasSigma_m_s2 += r.sigma.segment<3>(ba);
                ++n;
            }
            L.valid = n > 0;
            if (L.valid)
            {
                const auto c = static_cast<double>(n);
                L.gyroRms_rad_s = (L.gyroRms_rad_s / c).cwiseSqrt();
                L.accelRms_m_s2 = (L.accelRms_m_s2 / c).cwiseSqrt();
                L.gyroBiasSigma_rad_s /= c;
                L.accelBiasSigma_m_s2 /= c;
            }

            for (const StateRow& r : res.states)
            {
                if (r.t < tHalf || !r.biasTruth.baroBias)
                    continue;
                ++S.baroTruthRows;
                if (r.biasTruth.staticPressureLogged)
                    ++S.baroTruthRowsWithAtmosphere;
            }

            if (const int bp = cfg.offset(BlockId::BaroBias); bp >= 0 && !res.states.empty())
            {
                const StateRow& last = res.states.back();
                S.hasFinalBaro = true;
                S.finalBaroEstimate_m = last.estimate.baroBias;
                S.finalBaroSigma_m = last.sigma[bp];
                S.finalBiasTruth = last.biasTruth;
            }
        }
    } // namespace detail

    namespace detail {
        [[nodiscard]] inline bool finite(const ImuRecord& r)
        {
            return std::isfinite(r.partTime) && std::isfinite(r.hostTime) && r.gyro_rad_s.allFinite() &&
                   r.accel_m_s2.allFinite();
        }

        [[nodiscard]] inline bool finite(const TruthRecord& r)
        {
            return std::isfinite(r.time) && std::isfinite(r.lat_deg) && std::isfinite(r.lon_deg) &&
                   std::isfinite(r.alt_m) && r.vNed_m_s.allFinite() && std::isfinite(r.yaw_rad) &&
                   std::isfinite(r.pitch_rad) && std::isfinite(r.roll_rad);
        }

        struct ValidInputs
        {
            std::vector<ImuRecord> imu;
            std::vector<TruthRecord> truth; ///< in time order
        };

        // Inputs are checked before anything uses them (second pre-DOI audit,
        // N06): a NaN gyro reached the propagation and made every later
        // estimate NaN, and a non-finite time would reach the clock fit and
        // the event order. Rejected records are counted in the summary; none
        // of them reaches the filter. The truth is put in time order, which
        // interpolateTruth relies on.
        [[nodiscard]] inline ValidInputs validInputs(const LogSet& logs, ReplaySummary& summary)
        {
            ValidInputs v;
            v.imu.reserve(logs.imu.size());
            for (const ImuRecord& r : logs.imu)
            {
                if (finite(r))
                    v.imu.push_back(r);
                else
                    ++summary.rejectedImuSamples;
            }
            if (v.imu.size() < 2)
                throw std::runtime_error(std::format(
                    "replay: fewer than two valid IMU samples ({} rejected as not finite)", summary.rejectedImuSamples));

            v.truth.reserve(logs.truth.size());
            for (const TruthRecord& r : logs.truth)
            {
                if (finite(r))
                    v.truth.push_back(r);
                else
                    ++summary.rejectedTruthRecords;
            }
            if (v.truth.empty())
                throw std::runtime_error("replay: no valid truth record");
            std::stable_sort(v.truth.begin(), v.truth.end(),
                [](const TruthRecord& a, const TruthRecord& b) { return a.time < b.time; });
            return v;
        }

        /// e_b^T P_bb^-1 e_b over one block of the error vector.
        [[nodiscard]] inline double blockNees(const Eigen::VectorXd& e, const Eigen::MatrixXd& P, int offset, int dim)
        {
            const Eigen::VectorXd eb = e.segment(offset, dim);
            const Eigen::MatrixXd Pb = P.block(offset, offset, dim, dim);
            return eb.dot(Pb.ldlt().solve(eb));
        }

        /// The bias blocks' NEES of a row whose truth biases are filled in, and
        /// the full-state NEES when every carried block has its truth.
        template <ErrorConvention C>
        inline void biasNees(StateRow& row, const Configuration& cfg, const Eigen::MatrixXd& P)
        {
            const BiasTruth& bt = row.biasTruth;
            bool full = true;
            auto one = [&](BlockId id, bool known, OptionalNees& out) {
                const int o = cfg.offset(id);
                if (o < 0)
                    return;
                full = full && known;
                if (known)
                    out = { true, blockNees(row.error, P, o, blockSpec(id).dim) };
            };
            one(BlockId::GyroBias, bt.imuBias, row.neesGyroBias);
            one(BlockId::AccelBias, bt.imuBias, row.neesAccelBias);
            one(BlockId::BaroBias, bt.baroBias, row.neesBaroBias);
            if (full)
                row.neesFull = { true, row.error.dot(P.ldlt().solve(row.error)) };

            // Attribution: the same NEES with the unmodelled terms' equivalent bias added to the truth.
            if (!row.hasLeak || !bt.imuBias)
                return;
            EstimationState<double> withLeak = row.truth;
            withLeak.gyroBias += row.gyroLeak_rad_s;
            withLeak.accelBias += row.accelLeak_m_s2;
            const Eigen::VectorXd e = row.estimate.template error<C>(cfg, withLeak);
            if (const int o = cfg.offset(BlockId::GyroBias); o >= 0)
                row.neesGyroBiasLeak = { true, blockNees(e, P, o, 3) };
            if (const int o = cfg.offset(BlockId::AccelBias); o >= 0)
                row.neesAccelBiasLeak = { true, blockNees(e, P, o, 3) };
        }

        /// One output row at t: estimate, truth (with the drawn biases where the
        /// log has them), errors in filter coordinates and every NEES the truth
        /// allows. `u` is the IMU reading held over the interval that ended at t.
        template <ErrorConvention C>
        [[nodiscard]] StateRow recordRow(const InvariantEKF<double, C>& ekf,
            const Configuration& cfg,
            const std::vector<TruthRecord>& truth,
            const LciFrame<double>& frame,
            double t,
            const ImuSample<double>& u)
        {
            StateRow row;
            row.t = t;
            row.estimate = ekf.state();
            row.truth = truthToState(interpolateTruth(truth, t), frame);
            row.biasTruth = interpolateBiasTruth(truth, t);
            const BiasTruth& bt = row.biasTruth;
            if (bt.imuBias)
            {
                row.truth.gyroBias = bt.gyroBias_rad_s;
                row.truth.accelBias = bt.accelBias_m_s2;
            }
            if (bt.baroBias)
                row.truth.baroBias = bt.baroBiasTruth_m; // sensor + atmosphere terms
            if (bt.imuScaleMisalignment)
            {
                row.hasLeak = true;
                row.gyroLeak_rad_s =
                    equivalentImuBias(bt.gyroScaleError, bt.gyroMisalignment_rad, u.gyro_rad_s, bt.gyroBias_rad_s);
                row.accelLeak_m_s2 =
                    equivalentImuBias(bt.accelScaleError, bt.accelMisalignment_rad, u.accel_m_s2, bt.accelBias_m_s2);
            }

            const Eigen::MatrixXd& P = ekf.covariance();
            const int nav = cfg.offset(BlockId::NavCore);
            row.error = ekf.state().template error<C>(cfg, row.truth);
            row.sigma = P.diagonal().cwiseSqrt();
            row.neesNav = blockNees(row.error, P, nav, 9);
            biasNees<C>(row, cfg, P);

            const Eigen::Vector3d eulerEst = eulerNedFromState(ekf.state(), t, frame);
            const Eigen::Vector3d eulerTruth = eulerNedFromState(row.truth, t, frame);
            for (int i = 0; i < 3; ++i)
                row.eulerErr_rad[i] = std::remainder(eulerEst[i] - eulerTruth[i], 2.0 * std::numbers::pi);
            return row;
        }
    } // namespace detail

    template <ErrorConvention C>
    [[nodiscard]] ReplayResult replay(const LogSet& logs, const Configuration& cfg, const ReplayOptions& opt)
    {
        constexpr double kDeg = std::numbers::pi / 180.0;
        if (logs.imu.empty())
            throw std::runtime_error("replay: no IMU samples");
        if (logs.truth.empty())
            throw std::runtime_error("replay: no truth");

        ReplayResult res;
        res.summary.convention = C;

        const detail::ValidInputs valid = detail::validInputs(logs, res.summary);
        const std::vector<ImuRecord>& imu = valid.imu;
        const std::vector<TruthRecord>& truth = valid.truth;

        res.summary.timeBase = detail::fitImuTimeBase(imu);
        const TimeBaseFit& fit = res.summary.timeBase;

        // Frame and filter.
        const LciFrame<double> frame =
            LciFrame<double>::FromGeodetic(logs.run.lat0_deg * kDeg, logs.run.lon0_deg * kDeg, logs.run.alt0_m, 0.0);
        const MeasurementModels<double> meas(frame);
        InvariantEKF<double, C> ekf(cfg, frame, opt.noise);
        const int n = cfg.dimension();

        // Initial state from the truth at the first IMU sample.
        const double t0 = opt.timing.imu(imu.front(), fit, logs.run);
        if (!std::isfinite(t0))
            throw std::runtime_error("replay: the first IMU sample maps to a time that is not finite");
        EstimationState<double> x0 = truthToState(detail::interpolateTruth(truth, t0), frame);
        const Eigen::VectorXd sig = (opt.initialSigma.size() == n) ? opt.initialSigma : detail::defaultInitialSigma(cfg);
        if (opt.initialError.size() == n)
            detail::perturb(x0, cfg, opt.initialError);
        ekf.initialise(x0, ekf.covarianceFromPhysical(x0, Eigen::MatrixXd(sig.cwiseProduct(sig).asDiagonal())));

        // Measurement events on the simulation time base, in order.
        struct Event
        {
            double t;
            int kind; // 0 gps, 1 baro
            std::size_t index;
        };
        std::vector<Event> events;
        auto addEvent = [&](double t, int kind, std::size_t i) {
            if (std::isfinite(t))
                events.push_back({ t, kind, i });
            else
                ++res.summary.rejectedMeasurements; // would sort anywhere and split nothing sensibly
        };
        for (std::size_t i = 0; i < logs.gps.size(); ++i)
            addEvent(opt.timing.gps(logs.gps[i], fit, logs.run), 0, i);
        for (std::size_t i = 0; i < logs.baro.size(); ++i)
            addEvent(opt.timing.baro(logs.baro[i], fit, logs.run), 1, i);
        std::stable_sort(events.begin(), events.end(), [](const Event& a, const Event& b) { return a.t < b.t; });
        std::size_t nextEvent = 0;
        while (nextEvent < events.size() && events[nextEvent].t < t0)
            ++nextEvent; // measurements before the first IMU sample cannot be applied

        const bool useGnssPos = cfg.has(MeasurementId::GnssPosition);
        const bool useGnssVel = cfg.has(MeasurementId::GnssVelocity);
        const bool useBaro = cfg.has(MeasurementId::Barometer);

        auto applyEvent = [&](const Event& e) {
            if (e.kind == 0)
            {
                const GpsRecord& g = logs.gps[e.index];
                if (!g.valid || g.fixType < opt.minFixType)
                {
                    ++res.summary.skippedFixes;
                    return;
                }
                const double era = frame.earthRotationAngle(e.t);
                const GnssFix<double> fix{ g.lat_deg * kDeg, g.lon_deg * kDeg, g.alt_m };
                const Eigen::Matrix3d C_LN = meas.nedToLci(fix, era);
                if (useGnssPos)
                {
                    const double sh = (g.horizontalAccuracy_m > 0.0) ? opt.gnssHorizScale * g.horizontalAccuracy_m : opt.gnssHorizSigma_m;
                    const double sz = (g.verticalAccuracy_m > 0.0) ? g.verticalAccuracy_m : opt.gnssVertSigma_m;
                    const Eigen::Matrix3d SigmaN = Eigen::Vector3d(sh * sh, sh * sh, sz * sz).asDiagonal();
                    const auto r = ekf.updateGnssPosition(meas.gnssPositionLci(fix, era), C_LN * SigmaN * C_LN.transpose());
                    if (r.applied)
                    {
                        res.updates.push_back({ e.t, MeasurementId::GnssPosition, r.dim, r.nis });
                        ++res.summary.gnssPositionUpdates;
                    }
                    res.summary.rejectedUpdates += r.rejected ? 1U : 0U;
                }
                if (useGnssVel && g.hasVelocity)
                {
                    const double sv = (g.speedAccuracy_m_s > 0.0) ? g.speedAccuracy_m_s : opt.gnssVelSigma_m_s;
                    const Eigen::Matrix3d SigmaV = Eigen::Matrix3d::Identity() * sv * sv;
                    const auto r =
                        ekf.updateGnssVelocity(meas.gnssVelocityLci(fix, g.velNed_m_s, era), C_LN * SigmaV * C_LN.transpose());
                    if (r.applied)
                    {
                        res.updates.push_back({ e.t, MeasurementId::GnssVelocity, r.dim, r.nis });
                        ++res.summary.gnssVelocityUpdates;
                    }
                    res.summary.rejectedUpdates += r.rejected ? 1U : 0U;
                }
            } else if (useBaro)
            {
                const BaroRecord& b = logs.baro[e.index];
                const auto r = ekf.updateBarometer(pressureAltitude_m(b.pressure_Pa), opt.baroSigma_m);
                if (r.applied)
                {
                    res.updates.push_back({ e.t, MeasurementId::Barometer, r.dim, r.nis });
                    ++res.summary.baroUpdates;
                }
                res.summary.rejectedUpdates += r.rejected ? 1U : 0U;
            }
        };

        auto record = [&](double t, const ImuSample<double>& u) {
            res.states.push_back(detail::recordRow<C>(ekf, cfg, truth, frame, t, u));
        };

        // Main loop.
        double tPrev = t0;
        double tLastOut = -1e300;
        record(t0, ImuSample<double>{ imu.front().gyro_rad_s, imu.front().accel_m_s2 });
        tLastOut = t0;
        const ImuRecord* last = &imu.front(); // the last sample that was accepted
        for (std::size_t k = 1; k < imu.size(); ++k)
        {
            const ImuRecord& s = imu[k];
            const double t = opt.timing.imu(s, fit, logs.run);
            if (!std::isfinite(t))
            {
                ++res.summary.rejectedImuSamples;
                continue;
            }
            if (t <= tPrev)
            {
                ++res.summary.skippedImuSamples; // duplicate or out-of-order stamp: skip the sample
                continue;
            }
            // Readings held over [tPrev, t]: the average of the bounding
            // samples -- the one at tPrev being the last one accepted, not the
            // previous array element, which may be a skipped duplicate (second
            // pre-DOI audit, N06: a skipped duplicate with a different gyro
            // reading turned the estimate by 0.5 rad).
            const ImuRecord& u0 = *last;
            const ImuSample<double> u{ 0.5 * (u0.gyro_rad_s + s.gyro_rad_s), 0.5 * (u0.accel_m_s2 + s.accel_m_s2) };
            last = &s;

            // Split the interval at every measurement inside it.
            double tCur = tPrev;
            while (nextEvent < events.size() && events[nextEvent].t <= t)
            {
                const Event& e = events[nextEvent++];
                if (e.t > tCur)
                {
                    ekf.predict(u, e.t - tCur);
                    tCur = e.t;
                }
                applyEvent(e);
            }
            if (t > tCur)
                ekf.predict(u, t - tCur);
            tPrev = t;
            ++res.summary.imuSamples;

            if (opt.outputEvery_s <= 0.0 || t - tLastOut >= opt.outputEvery_s - 1e-9)
            {
                record(t, u);
                tLastOut = t;
            }
        }

        detail::summarise(res, t0, tPrev); // over the second half
        detail::summariseBiases(res, cfg, t0 + 0.5 * (tPrev - t0));
        return res;
    }

    /// Dispatch on a run-time convention.
    [[nodiscard]] inline ReplayResult
    replay(ErrorConvention c, const LogSet& logs, const Configuration& cfg, const ReplayOptions& opt)
    {
        return c == ErrorConvention::RightInvariant ? replay<ErrorConvention::RightInvariant>(logs, cfg, opt) : replay<ErrorConvention::LeftInvariant>(logs, cfg, opt);
    }

    // ---- output -----------------------------------------------------------------

    inline void writeStates(std::ostream& out, const ReplayResult& r, const Configuration& cfg)
    {
        constexpr double kDeg = std::numbers::pi / 180.0;
        const int n = cfg.dimension();
        out << "t_s,est_p_x,est_p_y,est_p_z,est_v_x,est_v_y,est_v_z,truth_p_x,truth_p_y,truth_p_z,truth_v_x,truth_v_y,"
               "truth_v_z,err_yaw_deg,err_pitch_deg,err_roll_deg,nees_nav";
        for (int i = 0; i < n; ++i)
            out << ",err_" << i;
        for (int i = 0; i < n; ++i)
            out << ",sigma_" << i;
        // Appended after the original columns so positional readers keep working.
        // A NEES or truth the log cannot support is an empty cell, never NaN.
        // truth_bp is the baro-bias NEES truth, sensor + atmosphere terms (sensor
        // only without out.P_Pa); the two terms follow at the end, the
        // atmosphere's empty without out.P_Pa.
        out << ",est_bg_x,est_bg_y,est_bg_z,est_ba_x,est_ba_y,est_ba_z,est_bp"
               ",nees_gyro_bias,nees_accel_bias,nees_baro_bias,nees_full"
               ",truth_bg_x,truth_bg_y,truth_bg_z,truth_ba_x,truth_ba_y,truth_ba_z,truth_bp,truth_bp_pa"
               ",leak_bg_x,leak_bg_y,leak_bg_z,leak_ba_x,leak_ba_y,leak_ba_z,nees_gyro_bias_leak,nees_accel_bias_leak"
               ",truth_bp_sensor_m,truth_bp_atmosphere_m\n";
        auto opt = [&out](const OptionalNees& n) {
            out << ',';
            if (n.valid)
                out << n.value;
        };
        auto vec = [&out](bool valid, const Eigen::Vector3d& v) {
            for (int i = 0; i < 3; ++i)
            {
                out << ',';
                if (valid)
                    out << v[i];
            }
        };
        auto scalar = [&out](bool valid, double v) {
            out << ',';
            if (valid)
                out << v;
        };
        for (const StateRow& s : r.states)
        {
            out << s.t;
            for (int i = 0; i < 3; ++i)
                out << ',' << s.estimate.X.p[i];
            for (int i = 0; i < 3; ++i)
                out << ',' << s.estimate.X.v[i];
            for (int i = 0; i < 3; ++i)
                out << ',' << s.truth.X.p[i];
            for (int i = 0; i < 3; ++i)
                out << ',' << s.truth.X.v[i];
            for (int i = 0; i < 3; ++i)
                out << ',' << s.eulerErr_rad[i] / kDeg;
            out << ',' << s.neesNav;
            for (int i = 0; i < n; ++i)
                out << ',' << s.error[i];
            for (int i = 0; i < n; ++i)
                out << ',' << s.sigma[i];
            for (int i = 0; i < 3; ++i)
                out << ',' << s.estimate.gyroBias[i];
            for (int i = 0; i < 3; ++i)
                out << ',' << s.estimate.accelBias[i];
            out << ',' << s.estimate.baroBias;
            opt(s.neesGyroBias);
            opt(s.neesAccelBias);
            opt(s.neesBaroBias);
            opt(s.neesFull);
            vec(s.biasTruth.imuBias, s.biasTruth.gyroBias_rad_s);
            vec(s.biasTruth.imuBias, s.biasTruth.accelBias_m_s2);
            scalar(s.biasTruth.baroBias, s.biasTruth.baroBiasTruth_m);
            scalar(s.biasTruth.baroBias, s.biasTruth.pressureBias_Pa);
            vec(s.hasLeak, s.gyroLeak_rad_s);
            vec(s.hasLeak, s.accelLeak_m_s2);
            opt(s.neesGyroBiasLeak);
            opt(s.neesAccelBiasLeak);
            scalar(s.biasTruth.baroBias, s.biasTruth.baroSensorTerm_m);
            scalar(s.biasTruth.baroBias && s.biasTruth.staticPressureLogged, s.biasTruth.baroAtmosphereTerm_m);
            out << '\n';
        }
    }

    inline void writeUpdates(std::ostream& out, const ReplayResult& r)
    {
        out << "t_s,measurement,dim,nis\n";
        for (const UpdateRow& u : r.updates)
            out << u.t << ',' << measurementSpec(u.id).name << ',' << u.dim << ',' << u.nis << '\n';
    }

    /// What the baro-bias NEES was taken against (BiasTruth::baroBiasTruth_m),
    /// and its two terms at the end of the run.
    inline void writeBaroTruthSummary(std::ostream& out, const ReplaySummary& s)
    {
        if (!s.aneesBaroBias.valid)
            return;
        const std::size_t all = s.baroTruthRows, withAtm = s.baroTruthRowsWithAtmosphere;
        if (withAtm == all)
            out << "baro-bias truth: sensor + atmosphere terms, h_p(P + dp) - h = [h_p(P + dp) - h_p(P)] + [h_p(P) - "
                   "h], P the plant's out.P_Pa, h its out.alt_m, geometric height\n";
        else if (withAtm == 0)
            out << "baro-bias truth: sensor term only, h_p(P + dp) - h_p(P) with P the standard atmosphere at out.alt_m; "
                   "out.P_Pa not logged, so the atmosphere term h_p(P) - h is unknown and left out\n";
        else
            out << "baro-bias truth: sensor + atmosphere terms on " << withAtm << " of " << all
                << " second-half rows; sensor term only on the rest, where out.P_Pa is not logged\n";

        const BiasTruth& b = s.finalBiasTruth;
        if (!s.hasFinalBaro || !b.baroBias)
            return;
        out << "baro bias at the end: estimate " << s.finalBaroEstimate_m << " +- " << s.finalBaroSigma_m
            << " m, truth " << b.baroBiasTruth_m << " m = sensor " << b.baroSensorTerm_m << " m (drawn "
            << b.pressureBias_Pa << " Pa at " << b.staticPressure_Pa << " Pa, "
            << (b.staticPressureLogged ? "plant's pressure" : "standard atmosphere") << ")";
        if (b.staticPressureLogged)
            out << " + atmosphere " << b.baroAtmosphereTerm_m << " m (h_p(P) - h)";
        else
            out << "; atmosphere term unknown without out.P_Pa";
        out << '\n';
    }

    /// Which NEES the summary reports, and the bias blocks' figures when the
    /// truth log carries the drawn sensor errors.
    inline void writeBiasSummary(std::ostream& out, const ReplaySummary& s)
    {
        const bool anyBias = s.aneesGyroBias.valid || s.aneesAccelBias.valid || s.aneesBaroBias.valid;
        if (!anyBias)
        {
            out << "NEES reported: navigation core only (9 dof); no bias NEES, the truth log carries no drawn sensor "
                   "errors\n";
            return;
        }
        out << "NEES reported: navigation core (9)";
        if (s.aneesGyroBias.valid)
            out << ", gyro bias (3)";
        if (s.aneesAccelBias.valid)
            out << ", accel bias (3)";
        if (s.aneesBaroBias.valid)
            out << ", baro bias (1)";
        if (s.aneesFull.valid)
            out << ", full state";
        out << ", against the drawn sensor errors in the truth log\n";
        auto anees = [&out](std::string_view name, const BlockAnees& b) {
            out << ", " << name << ' ';
            if (b.valid)
                out << b.anees;
            else
                out << "not available";
        };
        out << "second half: ANEES (/dim) nav core " << s.aneesNav;
        anees("gyro bias", s.aneesGyroBias);
        anees("accel bias", s.aneesAccelBias);
        anees("baro bias", s.aneesBaroBias);
        anees("full", s.aneesFull);
        out << '\n';

        const LeakSummary& L = s.leak;
        auto v3 = [&out](const Eigen::Vector3d& v) { out << '(' << v[0] << ' ' << v[1] << ' ' << v[2] << ')'; };
        if (L.valid)
        {
            out << "unmodelled IMU scale and misalignment as equivalent bias, second half (x y z): gyro rms ";
            v3(L.gyroRms_rad_s);
            out << " max ";
            v3(L.gyroMax_rad_s);
            out << " rad/s against bias 1-sigma ";
            v3(L.gyroBiasSigma_rad_s);
            out << "; accel rms ";
            v3(L.accelRms_m_s2);
            out << " max ";
            v3(L.accelMax_m_s2);
            out << " m/s^2 against bias 1-sigma ";
            v3(L.accelBiasSigma_m_s2);
            out << '\n'
                << "ANEES (/dim) with that equivalent bias added to the truth";
            anees("gyro bias", L.gyroBias);
            anees("accel bias", L.accelBias);
            out << '\n';
        }
        writeBaroTruthSummary(out, s);
    }

    inline void writeSummary(std::ostream& out, const ReplaySummary& s)
    {
        out << "convention: " << toString(s.convention) << '\n'
            << "time base: host = " << s.timeBase.offset << " + " << s.timeBase.scale << " * part over " << s.timeBase.samples
            << " IMU samples, residual rms " << s.timeBase.rmsResidual * 1e3 << " ms, max " << s.timeBase.maxResidual * 1e3
            << " ms\n"
            << "duration: " << s.duration_s << " s, " << s.imuSamples << " IMU steps, " << s.gnssPositionUpdates
            << " GNSS position, " << s.gnssVelocityUpdates << " GNSS velocity, " << s.baroUpdates << " barometer updates, "
            << s.skippedFixes << " fixes skipped\n"
            << "not used: " << s.rejectedImuSamples << " IMU samples not finite, " << s.skippedImuSamples
            << " IMU samples out of order, " << s.rejectedTruthRecords << " truth records not finite, "
            << s.rejectedMeasurements << " measurements with no finite time, " << s.rejectedUpdates
            << " updates refused by the filter\n"
            << "second half: ANEES (nav core, /9) " << s.aneesNav << ", NIS/dim GNSS pos " << s.nisGnssPosition
            << ", GNSS vel " << s.nisGnssVelocity << ", baro " << s.nisBaro << '\n'
            << "second half RMS: position " << s.posRms_m << " m, velocity " << s.velRms_m_s << " m/s, tilt " << s.tiltRms_deg
            << " deg, yaw " << s.yawRms_deg << " deg\n";
        writeBiasSummary(out, s);
    }

} // namespace Aetherion::Estimation::Replay

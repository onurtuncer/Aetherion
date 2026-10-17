// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// ProcessModel.h
//
// Strapdown process model of the estimator on SE2(3) x R^7 in the LCI frame
// (sensor fusion step 5; doc/sensor_fusion.rst "Kinematics" and "Error
// convention"). With the IMU readings w~, a~ (inertial rate and specific
// force, body frame), the bias-corrected w = w~ - b_g, a = a~ - b_a and
// gravity g_L(p) of LciFrame.h:
//
//   R_dot = R [w]x,   v_dot = R a + g_L(p),   p_dot = v,   b_dot = white.
//
// Three things are provided, all scalar-templated:
//
//   propagate(x, u, dt)          one IMU step of the nonlinear model (zero-
//                                order hold on the readings, rotation by
//                                Exp(w dt), specific force rotated by the
//                                mid-step attitude R Exp(w dt/2));
//   errorJacobians<C>(...)       the continuous error dynamics
//                                xi_dot = F xi + G n at a state, in error
//                                convention C;
//   step<C>(...)                 propagate plus the discrete transition
//                                Phi and process noise Qd over the step.
//
// step() averages F and G over the step (trapezoidal, evaluated at x_k and
// x_{k+1}) before discretising, Phi = I + F dt + (F dt)^2/2, Qd = G Qc G^T dt.
// This is a requirement found by the prototype's finite-difference check
// (scripts/iekf_convention_mc.py): the right-invariant bias coupling
// -Ad_Xhat contains [p]x R, which changes within one step at a rate of about
// |p||w|, so a start-of-step F is only first-order accurate for RI. LI is
// second-order either way; it gets the same treatment for symmetry.
//
// Error dynamics. With g = g_L(p), G = dg/dp (gravity gradient) and
// Ad = Ad_X (SE23.h), rows/columns indexed by the blocks' offsets:
//
//   RI:  F[nu,phi] = [g]x - G [p]x,  F[nu,rho] = G,  F[rho,nu] = I,
//        F[nav, b_g] = -Ad[:, phi],  F[nav, b_a] = -Ad[:, nu],
//        G_c[nav, n_g] =  Ad[:, phi], G_c[nav, n_a] =  Ad[:, nu];
//
//   LI:  F[phi,phi] = F[nu,nu] = F[rho,rho] = -[w]x,  F[nu,phi] = -[a]x,
//        F[nu,rho] = R^T G R,  F[rho,nu] = I,
//        F[phi, b_g] = -I,  F[nu, b_a] = -I,
//        G_c[phi, n_g] = I,  G_c[nu, n_a] = I;
//
//   both: G_c[b, n_b] = I for every random-walk block carried.
//
// Both were verified entry by entry against central differences of the full
// nonlinear step in the prototype, and are again in
// tests/Estimation/test_ProcessModel.cpp, in double and through CppAD.
//
// Noise vector n (continuous, PSD Qc = diag(ProcessNoise^2)): gyro white
// (3), accelerometer white (3), then one random-walk driver per carried bias
// block in rank order. noiseDimension() and noiseOffset() give its layout.
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <Aetherion/Estimation/Blocks.h>
#include <Aetherion/Estimation/Configuration.h>
#include <Aetherion/Estimation/EstimationState.h>
#include <Aetherion/Estimation/LciFrame.h>
#include <Aetherion/Estimation/SE23.h>
#include <Aetherion/ODE/RKMK/Lie/SO3.h>
#include <Aetherion/Spatial/Skew.h>

namespace Aetherion::Estimation {

    /// One IMU sample, body frame: inertial angular rate and specific force.
    template <class S>
    struct ImuSample
    {
        Eigen::Matrix<S, 3, 1> gyro_rad_s;
        Eigen::Matrix<S, 3, 1> accel_m_s2;
    };

    /// Continuous-time noise densities (square roots of the PSDs).
    template <class S>
    struct ProcessNoise
    {
        S gyroNoise = S(0);     ///< [rad/s/sqrt(Hz)]
        S accelNoise = S(0);    ///< [m/s^2/sqrt(Hz)]
        S gyroBiasWalk = S(0);  ///< [rad/s^2/sqrt(Hz)]
        S accelBiasWalk = S(0); ///< [m/s^3/sqrt(Hz)]
        S baroBiasWalk = S(0);  ///< [m/s/sqrt(Hz)]
    };

    /// Dimension of the noise vector n for a configuration.
    [[nodiscard]] constexpr int noiseDimension(const Configuration& cfg)
    {
        int n = 6; // IMU white noise
        for (const BlockSpec& s : kBlockLibrary)
            if (s.process == ProcessModelKind::RandomWalk && cfg.has(s.id))
                n += s.dim;
        return n;
    }

    /// Column of block b's random-walk driver in n, or -1 if b is not a carried
    /// random-walk block.
    [[nodiscard]] constexpr int noiseOffset(const Configuration& cfg, BlockId b)
    {
        int n = 6;
        for (const BlockSpec& s : kBlockLibrary)
        {
            if (s.process != ProcessModelKind::RandomWalk || !cfg.has(s.id))
                continue;
            if (s.id == b)
                return n;
            n += s.dim;
        }
        return -1;
    }

    template <class S>
    class StrapdownProcessModel
    {
    public:
        using Mat3 = Eigen::Matrix<S, 3, 3>;
        using Vec3 = Eigen::Matrix<S, 3, 1>;
        using MatX = Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>;
        using State = EstimationState<S>;

        explicit StrapdownProcessModel(LciFrame<S> frame)
            : frame_(std::move(frame))
        {}

        [[nodiscard]] const LciFrame<S>& frame() const { return frame_; }

        /// One step of the nonlinear model. Biases are random walks: their mean
        /// propagation is the identity.
        ///
        /// The specific force is rotated by the mid-step attitude R Exp(w dt/2),
        /// which is the velocity increment integral(R Exp(w t) a dt) to second
        /// order. Rotating by the start-of-step attitude instead drops the
        /// dt^2/2 [a]x coupling of the gyro bias into velocity, and the
        /// linearisation of the step then disagrees with Phi at first order (the
        /// finite-difference test shows a residual ratio of 2, not 8, per halving
        /// of dt).
        [[nodiscard]] State propagate(const State& x, const ImuSample<S>& u, const S& dt) const
        {
            namespace SO3 = ODE::RKMK::Lie::SO3;
            const Vec3 w = u.gyro_rad_s - x.gyroBias;
            const Vec3 a = u.accel_m_s2 - x.accelBias;
            const Mat3 R_mid = x.X.R * SO3::Exp_R(Vec3(w * (S(0.5) * dt)));
            const Vec3 a_L = R_mid * a + frame_.gravity(x.X.p);

            State out = x;
            out.X.p = x.X.p + x.X.v * dt + a_L * (S(0.5) * dt * dt);
            out.X.v = x.X.v + a_L * dt;
            out.X.R = x.X.R * SO3::Exp_R(Vec3(w * dt));
            return out;
        }

        /// Continuous error dynamics at x: F (n x n) and G (n x m) with
        /// n = cfg.dimension(), m = noiseDimension(cfg).
        template <ErrorConvention C>
        void errorJacobians(const Configuration& cfg, const State& x, const ImuSample<S>& u, MatX& F, MatX& G) const
        {
            const int n = cfg.dimension();
            F = MatX::Zero(n, n);
            G = MatX::Zero(n, noiseDimension(cfg));

            const int nav = cfg.offset(BlockId::NavCore);
            const int iPhi = nav + NavCoreLayout::kAttitude;
            const int iNu = nav + NavCoreLayout::kVelocity;
            const int iRho = nav + NavCoreLayout::kPosition;
            const int bg = cfg.offset(BlockId::GyroBias);
            const int ba = cfg.offset(BlockId::AccelBias);

            const Mat3 I = Mat3::Identity();
            const Vec3 g = frame_.gravity(x.X.p);
            const Mat3 Gg = frame_.gravityGradient(x.X.p);

            if constexpr (C == ErrorConvention::RightInvariant)
            {
                const typename SE23<S>::Mat9 Ad = SE23<S>::Adjoint(x.X);
                F.template block<3, 3>(iNu, iPhi) = Spatial::skew(g) - Gg * Spatial::skew(x.X.p);
                F.template block<3, 3>(iNu, iRho) = Gg;
                F.template block<3, 3>(iRho, iNu) = I;
                if (bg >= 0)
                    F.template block<9, 3>(nav, bg) = -Ad.template block<9, 3>(0, 0);
                if (ba >= 0)
                    F.template block<9, 3>(nav, ba) = -Ad.template block<9, 3>(0, 3);
                G.template block<9, 3>(nav, 0) = Ad.template block<9, 3>(0, 0);
                G.template block<9, 3>(nav, 3) = Ad.template block<9, 3>(0, 3);
            } else
            {
                const Vec3 w = u.gyro_rad_s - x.gyroBias;
                const Vec3 a = u.accel_m_s2 - x.accelBias;
                const Mat3 W = Spatial::skew(w);
                F.template block<3, 3>(iPhi, iPhi) = -W;
                F.template block<3, 3>(iNu, iPhi) = -Spatial::skew(a);
                F.template block<3, 3>(iNu, iNu) = -W;
                F.template block<3, 3>(iNu, iRho) = x.X.R.transpose() * Gg * x.X.R;
                F.template block<3, 3>(iRho, iNu) = I;
                F.template block<3, 3>(iRho, iRho) = -W;
                if (bg >= 0)
                    F.template block<3, 3>(iPhi, bg) = -I;
                if (ba >= 0)
                    F.template block<3, 3>(iNu, ba) = -I;
                G.template block<3, 3>(iPhi, 0) = I;
                G.template block<3, 3>(iNu, 3) = I;
            }

            for (const BlockSpec& s : kBlockLibrary)
            {
                if (s.process != ProcessModelKind::RandomWalk || !cfg.has(s.id))
                    continue;
                const int o = cfg.offset(s.id);
                const int k = noiseOffset(cfg, s.id);
                for (int i = 0; i < s.dim; ++i)
                    G(o + i, k + i) = S(1);
            }
        }

        /// Diagonal PSD matrix Qc of n for a configuration.
        [[nodiscard]] MatX continuousNoiseCovariance(const Configuration& cfg, const ProcessNoise<S>& q) const
        {
            MatX Qc = MatX::Zero(noiseDimension(cfg), noiseDimension(cfg));
            for (int i = 0; i < 3; ++i)
            {
                Qc(i, i) = q.gyroNoise * q.gyroNoise;
                Qc(3 + i, 3 + i) = q.accelNoise * q.accelNoise;
            }
            auto walk = [&](BlockId b, const S& density) {
                const int k = noiseOffset(cfg, b);
                if (k < 0)
                    return;
                for (int i = 0; i < blockSpec(b).dim; ++i)
                    Qc(k + i, k + i) = density * density;
            };
            walk(BlockId::GyroBias, q.gyroBiasWalk);
            walk(BlockId::AccelBias, q.accelBiasWalk);
            walk(BlockId::BaroBias, q.baroBiasWalk);
            return Qc;
        }

        /// Propagate x over one IMU step and return the discrete error transition
        /// Phi and process noise Qd for convention C (trapezoidal F, G; see the
        /// header comment). P propagates as Phi P Phi^T + Qd.
        template <ErrorConvention C>
        [[nodiscard]] State step(const Configuration& cfg,
            const State& x,
            const ImuSample<S>& u,
            const S& dt,
            const MatX& Qc,
            MatX& Phi,
            MatX& Qd) const
        {
            const State x1 = propagate(x, u, dt);

            MatX F0, G0, F1, G1;
            errorJacobians<C>(cfg, x, u, F0, G0);
            errorJacobians<C>(cfg, x1, u, F1, G1);
            const MatX F = S(0.5) * (F0 + F1);
            const MatX G = S(0.5) * (G0 + G1);

            const MatX Fd = F * dt;
            Phi = MatX::Identity(F.rows(), F.cols()) + Fd + S(0.5) * Fd * Fd;
            Qd = G * Qc * G.transpose() * dt;
            return x1;
        }

    private:
        LciFrame<S> frame_;
    };

} // namespace Aetherion::Estimation

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// EstimationState.h
//
// The estimator state SE2(3) x R^7 (sensor fusion step 5): the navigation
// core X = (R, v, p) plus the Euclidean bias blocks of Blocks.h. A state
// always carries every library block; which of them are *estimated*, and
// where they sit in the error vector, is the Configuration's business. A
// block the configuration does not carry is a known constant to the
// models (zero unless set).
//
// The two operations that tie the state to its error vector, in either
// error convention (Blocks.h, ErrorConvention):
//
//   retract<C>(cfg, delta)  : X <- Exp(delta_nav) X   (right-invariant)
//                             X <- X Exp(delta_nav)   (left-invariant)
//                             b <- b + delta_b
//   error<C>(cfg, other)    : the delta with other == this->retract<C>(delta),
//                             Log(other.X X^-1) or Log(X^-1 other.X)
//
// retract is what an update applies, error is what NEES and the Jacobian
// checks measure, and the two are exact inverses of each other (tested).
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <Aetherion/Estimation/Blocks.h>
#include <Aetherion/Estimation/Configuration.h>
#include <Aetherion/Estimation/SE23.h>

namespace Aetherion::Estimation {

    template <class S>
    struct EstimationState
    {
        using Vec3 = Eigen::Matrix<S, 3, 1>;
        using VecX = Eigen::Matrix<S, Eigen::Dynamic, 1>;
        using Tangent = typename SE23<S>::Tangent;

        SE23<S> X;                     ///< Navigation core: attitude body -> LCI, velocity, position in LCI
        Vec3 gyroBias = Vec3::Zero();  ///< [rad/s], body
        Vec3 accelBias = Vec3::Zero(); ///< [m/s^2], body
        S baroBias = S(0);             ///< [m] of pressure altitude

        template <ErrorConvention C>
        [[nodiscard]] EstimationState retract(const Configuration& cfg, const VecX& delta) const
        {
            EstimationState out = *this;
            const SE23<S> E = SE23<S>::Exp(delta.template segment<9>(cfg.offset(BlockId::NavCore)));
            if constexpr (C == ErrorConvention::RightInvariant)
                out.X = E * X;
            else
                out.X = X * E;
            if (const int o = cfg.offset(BlockId::GyroBias); o >= 0)
                out.gyroBias += delta.template segment<3>(o);
            if (const int o = cfg.offset(BlockId::AccelBias); o >= 0)
                out.accelBias += delta.template segment<3>(o);
            if (const int o = cfg.offset(BlockId::BaroBias); o >= 0)
                out.baroBias += delta[o];
            return out;
        }

        template <ErrorConvention C>
        [[nodiscard]] VecX error(const Configuration& cfg, const EstimationState& other) const
        {
            VecX xi = VecX::Zero(cfg.dimension());
            SE23<S> E;
            if constexpr (C == ErrorConvention::RightInvariant)
                E = other.X * X.inverse();
            else
                E = X.inverse() * other.X;
            xi.template segment<9>(cfg.offset(BlockId::NavCore)) = SE23<S>::Log(E);
            if (const int o = cfg.offset(BlockId::GyroBias); o >= 0)
                xi.template segment<3>(o) = other.gyroBias - gyroBias;
            if (const int o = cfg.offset(BlockId::AccelBias); o >= 0)
                xi.template segment<3>(o) = other.accelBias - accelBias;
            if (const int o = cfg.offset(BlockId::BaroBias); o >= 0)
                xi[o] = other.baroBias - baroBias;
            return xi;
        }
    };

} // namespace Aetherion::Estimation

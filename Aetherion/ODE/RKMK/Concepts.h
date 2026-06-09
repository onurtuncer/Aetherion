// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// Concepts.h
//
// C++20 concepts constraining the KinematicsField, VectorField, and Integrator
// contracts used throughout Aetherion::ODE::RKMK.
//
#pragma once
#include <concepts>
#include <Eigen/Core>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <Aetherion/RigidBody/InertialParameters.h>

namespace Aetherion::ODE::RKMK {

    // ------------------------------------------------------------------------------
    // KinematicsFieldOnSE3<KF, Scalar>
    //
    // KF must be callable as: kf(t, g, xi) -> Matrix<Scalar, 6, 1>
    // where xi is the body-frame twist in R^6.
    // ------------------------------------------------------------------------------
    template<typename KF, typename Scalar = double>
    concept KinematicsFieldOnSE3 = requires(
        const KF & kf,
        const Scalar & t,
        const Lie::SE3<Scalar>&g,
        const Eigen::Matrix<Scalar, 6, 1>&xi)
    {
        // same_as (not convertible_to) so wrong-dimension returns fail here.
        { kf(t, g, xi) } -> std::same_as<Eigen::Matrix<Scalar, 6, 1>>;
    };

    // ------------------------------------------------------------------------------
    // VectorFieldOnProductSE3<VF, N, Scalar>
    //
    // VF must be callable as: vf(t, g, x) -> Matrix<Scalar, N, 1>
    // ------------------------------------------------------------------------------
    template<typename VF, int N, typename Scalar = double>
    concept VectorFieldOnProductSE3 = requires(
        const VF & vf,
        const Scalar & t,
        const Lie::SE3<Scalar>&g,
        const Eigen::Matrix<Scalar, N, 1>&x)
    {
        { vf(t, g, x) } -> std::same_as<Eigen::Matrix<Scalar, N, 1>>;
    };

    // ------------------------------------------------------------------------------
    // RKMKIntegratorOnProductSE3<I, N, Scalar>
    //
    // I must expose:
    //   typename StepResult  -- with fields g1, x1, converged
    //   typename VecE        -- Eigen::Matrix<Scalar, N, 1>
    //   step(t0, g0, x0, h, opt) -> StepResult
    // ------------------------------------------------------------------------------
    template<typename I, int N, typename Scalar = double>
    concept RKMKIntegratorOnProductSE3 = requires(
        const I & integrator,
        const Scalar & t0,
        const Lie::SE3<Scalar>&g0,
        const typename I::VecE & x0,
        const Scalar & h,
        const Core::NewtonOptions & opt)
    {
        typename I::StepResult;
        typename I::VecE;

            requires (I::VecE::RowsAtCompileTime == N);

        { integrator.step(t0, g0, x0, h, opt) } -> std::same_as<typename I::StepResult>;

            requires requires(typename I::StepResult r) {
                { r.g1 }        -> std::convertible_to<Lie::SE3<Scalar>>;
                { r.x1 }        -> std::convertible_to<typename I::VecE>;
                { r.converged } -> std::convertible_to<bool>;
        };
    };

    // ------------------------------------------------------------------------------
    // ConstructibleFromInertialParameters<VF>
    // ------------------------------------------------------------------------------
    template<typename VF>
    concept ConstructibleFromInertialParameters =
        requires(const RigidBody::InertialParameters & ip) {
            { VF(ip) } -> std::same_as<VF>;
    };

    // ------------------------------------------------------------------------------
    // IntegratorFor<I, XiField, FField, N, Scalar>
    //
    // Full constraint for the Integrator template parameter in SixDoFStepper.
    // Requires both the RKMKIntegratorOnProductSE3 stepping contract and that I
    // is constructible from (XiField, FField) — matching the construction pattern
    // used inside SixDoFStepper::step().
    //
    // Modelling this concept enables alternative RKMK schemes (e.g. an explicit
    // RKMK4 for non-stiff phases, or a Gauss-Legendre variant) to be dropped into
    // SixDoFStepper / ISimulator without any changes to those classes.
    // ------------------------------------------------------------------------------
    template<typename I, typename XiField, typename FField, int N, typename Scalar = double>
    concept IntegratorFor =
        RKMKIntegratorOnProductSE3<I, N, Scalar> &&
        std::constructible_from<I, XiField, FField>;

} // namespace Aetherion::ODE::RKMK

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Concepts.h
//
// C++20 concepts constraining the KinematicsField, VectorField, and Integrator
// contracts used throughout Aetherion::ODE::RKMK.
//
// Dependency graph:
//
//   KinematicsFieldOnSE3          VectorFieldOnProductSE3<N>
//          |                                |
//          └──────────┬─────────────────────┘
//                     ▼
//            RKMKIntegratorOnProductSE3<N>
//                     ▲
//                     │  constrained by
//            RigidBody6DoFStepper<VF>
//
#pragma once
#include <concepts>
#include <Eigen/Core>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <Aetherion/RigidBody/InertialParameters.h>

namespace Aetherion::ODE::RKMK {

    // ------------------------------------------------------------------------------
    // KinematicsFieldOnSE3
    //
    // Models the "where" equation: given pose g and body-frame twist xi,
    // return the Lie algebra element se(3) that drives g-dot.
    //
    // Conforming types:
    //   - KinematicsXiField          (standard body-frame twist, [omega; v])
    //   - KinematicsQuaternionField  (if you ever factor out the quat path)
    //   - KinematicsDualQuatField    (dual quaternion kinematics)
    //
    // Intentionally does NOT see the full Euclidean state vector x — the
    // caller (the integrator) is responsible for slicing xi out of x before
    // dispatching here. This keeps each kinematic model a pure SE(3) mapping.
    // ------------------------------------------------------------------------------
    template<typename KF>
    concept KinematicsFieldOnSE3 = requires(
        const KF & kf,
        double                                  t,
        const Lie::SE3<double>&g,
        const Eigen::Matrix<double, 6, 1>&xi)
    {
        { kf(t, g, xi) } -> std::convertible_to<Eigen::Matrix<double, 6, 1>>;
    };

    // ------------------------------------------------------------------------------
    // VectorFieldOnProductSE3<N>
    //
    // Models the "why" equation: given time, pose, and full Euclidean state,
    // return x-dot (forces, torques, mass flow, etc.).
    //
    // N is the Euclidean dimension — 7 for the standard 6DoF stepper
    // (6 twist components + 1 mass), but left generic so the concept
    // works for reduced models (e.g. N=6 if mass is constant) or
    // extended ones (e.g. N=13 with fuel slosh states).
    //
    // Conforming types:
    //   - RigidBodyVectorField<CentralGravityPolicy>
    //   - RigidBodyVectorField<AeroTablePolicy>
    //   - any callable (t, g, x) -> Matrix<double, N, 1>
    // ------------------------------------------------------------------------------
    template<typename VF, int N>
    concept VectorFieldOnProductSE3 = requires(
        const VF & vf,
        double                                  t,
        const Lie::SE3<double>&g,
        const Eigen::Matrix<double, N, 1>&x)
    {
        { vf(t, g, x) } -> std::convertible_to<Eigen::Matrix<double, N, 1>>;
    };

    // ------------------------------------------------------------------------------
    // RKMKIntegratorOnProductSE3<N>
    //
    // A fully assembled integrator operating on SE(3) x R^N.
    // Constrains the integrator seam so that alternative RKMK schemes
    // (e.g. a Gauss-Legendre variant, an explicit RKMK4 for non-stiff phases)
    // can be dropped in without touching the stepper.
    //
    // Required nested types:
    //   StepResult   -- carries g1, x1, converged (and optionally: iterations,
    //                   residual, rejected)
    //   VecE         -- Eigen::Matrix<double, N, 1>
    //
    // Required method:
    //   step(t0, g0, x0, h, opt) -> StepResult
    // ------------------------------------------------------------------------------
    template<typename I, int N>
    concept RKMKIntegratorOnProductSE3 = requires(
        const I & integrator,
        double                                  t0,
        const Lie::SE3<double>&g0,
        const typename I::VecE & x0,
        double                                  h,
        const Core::NewtonOptions & opt)
    {
        typename I::StepResult;
        typename I::VecE;

        // VecE must actually be R^N — prevents silent dimension mismatches
        // when N is deduced from context.
            requires (I::VecE::RowsAtCompileTime == N);

        { integrator.step(t0, g0, x0, h, opt) } -> std::same_as<typename I::StepResult>;

        // StepResult shape constraint — nested requires checks the
        // associated type rather than the integrator itself.
            requires requires(typename I::StepResult r) {
                { r.g1 }        -> std::convertible_to<Lie::SE3<double>>;
                { r.x1 }        -> std::convertible_to<typename I::VecE>;
                { r.converged } -> std::convertible_to<bool>;
        };
    };

    // ------------------------------------------------------------------------------
    // ConstructibleFromInertialParameters
    //
    // Opt-in concept for the convenience constructor path on RigidBody6DoFStepper.
    // Only gates the single-argument constructor — policies with richer
    // construction (e.g. needing an atmosphere table or thrust curve) simply
    // don't model this concept and must use the explicit VF-taking constructor.
    // ------------------------------------------------------------------------------
    template<typename VF>
    concept ConstructibleFromInertialParameters =
        requires(const RigidBody::InertialParameters & ip) {
            { VF(ip) } -> std::same_as<VF>;
    };

} // namespace Aetherion::ODE::RKMK
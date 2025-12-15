// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// File: Aetherion/ODE/RKMK/Core/Concepts.h
//
// Minimal C++23 concepts for:
//  - scalar types (double, CppAD::AD<double>, etc.)
//  - value-semantics types
//  - Lie groups (value type with Identity/Exp/dexp_inv and composition)
//  - dynamics fields xi(t,g,x) and f(t,g,x) for product manifolds G × R^m
//
#pragma once

#include <concepts>
#include <type_traits>
#include <utility>

#include <Eigen/Dense>

#include <Aetherion/ODE/RKMK/Core/Scalar.h>

namespace Aetherion::ODE::RKMK::Core {

    // ----------------------------------------------------------------------------
    // Basic building blocks
    // ----------------------------------------------------------------------------

    template<class T>
    concept ValueSemantics =
        std::is_default_constructible_v<T> &&
        std::is_copy_constructible_v<T> &&
        std::is_move_constructible_v<T> &&
        std::is_copy_assignable_v<T> &&
        std::is_move_assignable_v<T>;

    template<class S>
    concept ScalarLike =
        requires(S a, S b) {
        // construction
        S{ 0 };
        S{ 1 };

        // arithmetic
        { a + b } -> std::convertible_to<S>;
        { a - b } -> std::convertible_to<S>;
        { a* b } -> std::convertible_to<S>;
        { a / b } -> std::convertible_to<S>;
        { -a }    -> std::convertible_to<S>;

        // comparisons (needed for CondExp wrappers on doubles)
        { a < b } -> std::convertible_to<bool>;
    };

    template<class S>
    concept ADCompatibleScalar =
        ScalarLike<S> && (std::is_arithmetic_v<std::decay_t<S>> || is_cppad_ad_v<S>);

    // We typically store Euclidean parts as Eigen column vectors.
    // Keep this intentionally lightweight: only require "vector-ish" Eigen API.
    template<class V>
    concept EigenVectorLike =
        requires(V v) {
        typename V::Scalar;
        { v.rows() } -> std::convertible_to<Eigen::Index>;
        { v.cols() } -> std::convertible_to<Eigen::Index>;
        { v.size() } -> std::convertible_to<Eigen::Index>;
        // element access (vector overload)
        { v(0) } -> std::convertible_to<typename V::Scalar>;
    } &&
        // Prefer compile-time vector shapes when available:
        (V::ColsAtCompileTime == 1 || V::RowsAtCompileTime == 1);

    // ----------------------------------------------------------------------------
    // Lie group concept (value semantics + required API)
    // ----------------------------------------------------------------------------
    //
    // We assume a group value type G exposes:
    //   using Scalar  = ...;
    //   using Tangent = ...;   // Lie algebra element type
    //
    //   static G Identity();
    //   G operator*(const G&) const;
    //
    //   static G Exp(const Tangent&);
    //   static Tangent dexp_inv(const Tangent& x, const Tangent& y);
    //
    // Notes:
    // - This is RKMK-oriented: dexp_inv is what you need in implicit stages.
    // - We do NOT require Log(), inverse(), Ad(), etc. here.
    //
    template<class G>
    concept LieGroup =
        ValueSemantics<G> &&
        requires {
        typename G::Scalar;
        typename G::Tangent;
    }&&
        ADCompatibleScalar<typename G::Scalar>&&
        requires(const G& g1, const G& g2,
    const typename G::Tangent& x,
        const typename G::Tangent& y) {
            { G::Identity() } -> std::same_as<G>;
            { g1* g2 }       -> std::same_as<G>;
            { G::Exp(x) }     -> std::same_as<G>;
            { G::dexp_inv(x, y) } -> std::same_as<typename G::Tangent>;
    };

    // Convenience if you model groups as templates SE3<Scalar>, SO3<Scalar>, ...
    template<template<class> class GTemplate, class S>
    concept LieGroupTemplate = LieGroup<GTemplate<S>>;

    // ----------------------------------------------------------------------------
    // Product-manifold dynamics fields
    // ----------------------------------------------------------------------------
    //
    // For product manifold: (g, x) in G × R^m
    // We model:
    //
    //   xi : (t, g, x) -> Tangent(G)   (a "twist"/algebra element)
    //   f  : (t, g, x) -> xdot in R^m  (Euclidean dynamics)
    //
    // These are the only signatures enforced here.
    //
    // NOTE: In your implementation, xi/f should be templated on Scalar so they work
    // for both double and CppAD::AD<double>.
    //
    template<class Xi, class G, class X>
    concept XiField =
        LieGroup<G> &&
        EigenVectorLike<X> &&
        std::regular_invocable<Xi, typename G::Scalar, const G&, const X&>&&
        std::same_as<std::invoke_result_t<Xi, typename G::Scalar, const G&, const X&>,
        typename G::Tangent>;

    template<class F, class G, class X>
    concept EuclidField =
        LieGroup<G> &&
        EigenVectorLike<X> &&
        std::regular_invocable<F, typename G::Scalar, const G&, const X&>&&
        std::same_as<std::invoke_result_t<F, typename G::Scalar, const G&, const X&>,
        X>;

    // Often you want both together:
    template<class Xi, class F, class G, class X>
    concept ProductDynamics =
        XiField<Xi, G, X>&& EuclidField<F, G, X>;

} // namespace Aetherion::ODE::RKMK::Core

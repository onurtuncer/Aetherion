// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Catch2 tests for: Aetherion/ODE/RKMK/Core/Concepts.h
//
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <type_traits>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include <Aetherion/ODE/RKMK/Core/Concepts.h>

namespace {
    namespace Core = Aetherion::ODE::RKMK::Core;

    // -----------------------------
    // Minimal value-semantics Lie group (no Eigen storage -> AD-friendly)
    // -----------------------------
    template<class S>
    struct MockGroup {
        using Scalar = S;
        using Tangent = std::array<S, 6>;

        // store a 4x4 matrix in row-major
        std::array<S, 16> T{};

        static MockGroup Identity() {
            MockGroup g;
            g.T.fill(S(0));
            g.T[0] = S(1);
            g.T[5] = S(1);
            g.T[10] = S(1);
            g.T[15] = S(1);
            return g;
        }

        friend MockGroup operator*(const MockGroup& a, const MockGroup& b) {
            MockGroup c;
            c.T.fill(S(0));
            for (int r = 0; r < 4; ++r) {
                for (int col = 0; col < 4; ++col) {
                    S acc = S(0);
                    for (int k = 0; k < 4; ++k) {
                        acc = acc + a.T[r * 4 + k] * b.T[k * 4 + col];
                    }
                    c.T[r * 4 + col] = acc;
                }
            }
            return c;
        }

        // Stubs (only signature matters for concepts)
        static MockGroup Exp(const Tangent&) { return Identity(); }

        static Tangent dexp_inv(const Tangent&, const Tangent& y) { return y; }
    };

    // -----------------------------
    // Dynamics fields on G × R^3
    // -----------------------------
    struct XiGood {
        template<class S>
        auto operator()(S /*t*/,
            const MockGroup<S>& /*g*/,
            const Eigen::Matrix<S, 3, 1>& x) const -> typename MockGroup<S>::Tangent {
            typename MockGroup<S>::Tangent xi{};
            xi[0] = x(0);
            xi[1] = x(1);
            xi[2] = x(2);
            xi[3] = S(0);
            xi[4] = S(0);
            xi[5] = S(0);
            return xi;
        }
    };

    struct FGood {
        template<class S>
        auto operator()(S /*t*/,
            const MockGroup<S>& /*g*/,
            const Eigen::Matrix<S, 3, 1>& x) const -> Eigen::Matrix<S, 3, 1> {
            return x;
        }
    };

    // Bad Xi: wrong return type
    struct XiBadReturn {
        template<class S>
        auto operator()(S, const MockGroup<S>&, const Eigen::Matrix<S, 3, 1>&) const -> int {
            return 0;
        }
    };

    // Bad f: wrong return vector shape
    struct FBadReturn {
        template<class S>
        auto operator()(S, const MockGroup<S>&, const Eigen::Matrix<S, 3, 1>&) const -> Eigen::Matrix<S, 4, 1> {
            return Eigen::Matrix<S, 4, 1>::Zero();
        }
    };

    struct NonCopyable {
        NonCopyable() = default;
        NonCopyable(const NonCopyable&) = delete;
        NonCopyable& operator=(const NonCopyable&) = delete;
        NonCopyable(NonCopyable&&) = default;
        NonCopyable& operator=(NonCopyable&&) = default;
    };

    struct NoOpsScalar {}; // should fail ScalarLike/ADCompatibleScalar

} // namespace

TEST_CASE("Concepts: basic scalar concepts", "[concepts][scalar]") {
    STATIC_REQUIRE(Core::ScalarLike<double>);
    STATIC_REQUIRE(Core::ADCompatibleScalar<double>);
    STATIC_REQUIRE(Core::ADCompatibleScalar<CppAD::AD<double>>);

    STATIC_REQUIRE_FALSE(Core::ScalarLike<NoOpsScalar>);
    STATIC_REQUIRE_FALSE(Core::ADCompatibleScalar<NoOpsScalar>);
}

TEST_CASE("Concepts: ValueSemantics", "[concepts][value]") {
    STATIC_REQUIRE(Core::ValueSemantics<int>);
    STATIC_REQUIRE(Core::ValueSemantics<MockGroup<double>>);

    STATIC_REQUIRE_FALSE(Core::ValueSemantics<NonCopyable>);
}

TEST_CASE("Concepts: EigenVectorLike", "[concepts][eigen]") {
    STATIC_REQUIRE(Core::EigenVectorLike<Eigen::Vector3d>);
    STATIC_REQUIRE(Core::EigenVectorLike<Eigen::VectorXd>);

    STATIC_REQUIRE_FALSE(Core::EigenVectorLike<Eigen::Matrix3d>);
    STATIC_REQUIRE_FALSE(Core::EigenVectorLike<Eigen::Matrix<double, 3, 4>>);
}

TEST_CASE("Concepts: LieGroup and LieGroupTemplate", "[concepts][lie]") {
    STATIC_REQUIRE(Core::LieGroup<MockGroup<double>>);
    STATIC_REQUIRE(Core::LieGroupTemplate<MockGroup, double>);
}

TEST_CASE("Concepts: XiField / EuclidField / ProductDynamics", "[concepts][fields]") {
    using G = MockGroup<double>;
    using X = Eigen::Vector3d;

    STATIC_REQUIRE(Core::XiField<XiGood, G, X>);
    STATIC_REQUIRE(Core::EuclidField<FGood, G, X>);
    STATIC_REQUIRE(Core::ProductDynamics<XiGood, FGood, G, X>);

    STATIC_REQUIRE_FALSE(Core::XiField<XiBadReturn, G, X>);
    STATIC_REQUIRE_FALSE(Core::EuclidField<FBadReturn, G, X>);
    STATIC_REQUIRE_FALSE(Core::ProductDynamics<XiBadReturn, FGood, G, X>);
    STATIC_REQUIRE_FALSE(Core::ProductDynamics<XiGood, FBadReturn, G, X>);
}

TEST_CASE("Concepts: fields reject non-vector Euclidean state X", "[concepts][fields][negative]") {
    using G = MockGroup<double>;
    using Xb = Eigen::Matrix3d; // not a vector

    STATIC_REQUIRE_FALSE(Core::XiField<XiGood, G, Xb>);
    STATIC_REQUIRE_FALSE(Core::EuclidField<FGood, G, Xb>);
}

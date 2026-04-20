// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Aetherion/Spatial/WrenchOps.h>
#include <Aetherion/Spatial/WrenchShift.h>

namespace {
    using Scalar = double;
    using namespace Aetherion::Spatial;
    using Vec3d  = Eigen::Vector3d;
    using Catch::Matchers::WithinAbs;
}

// =============================================================================
// ZeroWrench
// =============================================================================

TEST_CASE("ZeroWrench: force vector is all zeros", "[spatial][WrenchOps][zero]")
{
    auto w = ZeroWrench<Scalar>();
    CHECK(w.f.isZero());
}

// =============================================================================
// AddInPlace
// =============================================================================

TEST_CASE("AddInPlace: accumulates force components correctly", "[spatial][WrenchOps][add]")
{
    Wrench<Scalar> acc{};
    acc.f << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

    Wrench<Scalar> delta{};
    delta.f << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

    AddInPlace(acc, delta);

    Eigen::Matrix<Scalar, 6, 1> expected;
    expected << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6;
    CHECK(acc.f.isApprox(expected, 1e-12));
}

TEST_CASE("AddInPlace: adding zero wrench leaves accumulator unchanged", "[spatial][WrenchOps][add]")
{
    Wrench<Scalar> acc{};
    acc.f << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    const auto acc_copy = acc;

    AddInPlace(acc, ZeroWrench<Scalar>());

    CHECK(acc.f.isApprox(acc_copy.f, 1e-12));
}

TEST_CASE("AddInPlace: adding multiple wrenches is commutative in result", "[spatial][WrenchOps][add]")
{
    Wrench<Scalar> a{}, b{};
    a.f << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    b.f << 0.0, 2.0, 0.0, 0.0, 0.0, 0.0;

    Wrench<Scalar> acc1 = ZeroWrench<Scalar>();
    AddInPlace(acc1, a);
    AddInPlace(acc1, b);

    Wrench<Scalar> acc2 = ZeroWrench<Scalar>();
    AddInPlace(acc2, b);
    AddInPlace(acc2, a);

    CHECK(acc1.f.isApprox(acc2.f, 1e-12));
}

// =============================================================================
// ShiftWrenchToNewPoint
// =============================================================================

TEST_CASE("ShiftWrenchToNewPoint: zero offset leaves wrench unchanged", "[spatial][WrenchShift]")
{
    Wrench<Scalar> f{};
    f.f << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

    Vec3d r = Vec3d::Zero();
    auto shifted = ShiftWrenchToNewPoint(f, r);

    CHECK(shifted.f.isApprox(f.f, 1e-12));
}

TEST_CASE("ShiftWrenchToNewPoint: force component is unchanged after shift", "[spatial][WrenchShift]")
{
    // Force is invariant; only moment changes.
    Wrench<Scalar> f{};
    f.f << 0.0, 0.0, 0.0, 3.0, -1.0, 2.0;  // [moment=0 | force]

    Vec3d r(0.5, 0.0, 0.0);
    auto shifted = ShiftWrenchToNewPoint(f, r);

    // Force part (indices 3-5) must be unchanged
    CHECK(shifted.f.tail<3>().isApprox(f.f.tail<3>(), 1e-12));
}

TEST_CASE("ShiftWrenchToNewPoint: moment = M_P + r x F", "[spatial][WrenchShift]")
{
    // F = (0, 0, 10) at P, M_P = (0, 0, 0), r = P-Q = (1, 0, 0)
    // Expected moment at Q: (0,0,0) + (1,0,0)×(0,0,10) = (0,-10, 0)
    Wrench<Scalar> f{};
    f.f << 0.0, 0.0, 0.0,   // moment at P
           0.0, 0.0, 10.0;  // force

    Vec3d r(1.0, 0.0, 0.0);
    auto shifted = ShiftWrenchToNewPoint(f, r);

    CHECK_THAT(shifted.f(0), WithinAbs( 0.0, 1e-12));
    CHECK_THAT(shifted.f(1), WithinAbs(-10.0, 1e-12));
    CHECK_THAT(shifted.f(2), WithinAbs( 0.0, 1e-12));
}

TEST_CASE("ShiftWrenchToNewPoint: existing moment is preserved in shift", "[spatial][WrenchShift]")
{
    // F = (0, 0, 10), M_P = (1, 0, 0), r = (0, 1, 0)
    // r x F = (0,1,0)×(0,0,10) = (10, 0, 0)
    // moment at Q = (1,0,0) + (10,0,0) = (11, 0, 0)
    Wrench<Scalar> f{};
    f.f << 1.0, 0.0, 0.0,   // moment at P
           0.0, 0.0, 10.0;  // force

    Vec3d r(0.0, 1.0, 0.0);
    auto shifted = ShiftWrenchToNewPoint(f, r);

    CHECK_THAT(shifted.f(0), WithinAbs(11.0, 1e-12));
    CHECK_THAT(shifted.f(1), WithinAbs( 0.0, 1e-12));
    CHECK_THAT(shifted.f(2), WithinAbs( 0.0, 1e-12));
}

TEST_CASE("ShiftWrenchToNewPoint: pure moment (no force) is unaffected by shift", "[spatial][WrenchShift]")
{
    // When F = 0, r × F = 0, so moment does not change
    Wrench<Scalar> f{};
    f.f << 1.0, 2.0, 3.0,   // moment
           0.0, 0.0, 0.0;   // zero force

    Vec3d r(5.0, -3.0, 2.0);
    auto shifted = ShiftWrenchToNewPoint(f, r);

    CHECK(shifted.f.head<3>().isApprox(f.f.head<3>(), 1e-12));
}

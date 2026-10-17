// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_SpatialInertiaContract.cpp
//
// Regression tests for the mass-property contract of RigidBody::VectorField
// (pre-DOI audit findings F02 and F03).
//
// F02. InertialParameters carries the inertia tensor about the centre of mass
//      and the centre of mass relative to the body origin. VectorField used to
//      put the tensor straight into the rotational block of M while adding the
//      m[c]x coupling blocks, i.e. the parallel-axis term m[c]x[c]x^T was
//      missing. For m = 10 kg, I_C = 1, c = (1, 0, 0) that matrix has a
//      negative eigenvalue: not the kinetic-energy metric of any body. The
//      tests below pin M to Spatial::Inertia, require it positive definite, and
//      check the physics the shift governs: a torque about the centre of mass
//      must turn a body the same way whether its origin sits at the centre of
//      mass or not.
//
// F03. M and M_inv were built once from the constructor mass while gravity was
//      multiplied by the integrated mass state x(6), so a body whose mass state
//      had halved fell at g/2. The wrench policies now receive the mass M was
//      built with, and mass properties change only through
//      setInertialParameters().
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>

#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/Spatial/Inertia.h>
#include <Aetherion/Spatial/Wrench.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

using namespace Aetherion;
using namespace Aetherion::RigidBody;
using Catch::Matchers::WithinAbs;

namespace {

    using SE3d = ODE::RKMK::Lie::SE3<double>;
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Vec7 = Eigen::Matrix<double, 7, 1>;

    /// Gravity-policy stand-in that applies a fixed body-frame wrench, mass-independent.
    struct BodyWrenchPolicy
    {
        Vec6 w = Vec6::Zero();
        template <class S>
        Spatial::Wrench<S> operator()(const ODE::RKMK::Lie::SE3<S>&, S) const
        {
            Spatial::Wrench<S> out{};
            out.f = w.template cast<S>();
            return out;
        }
    };
    static_assert(FlightDynamics::GravityPolicy<BodyWrenchPolicy>);

    /// Uniform gravity of 10 m/s^2 along body -z, proportional to the mass it is handed.
    struct UniformGravityPolicy
    {
        template <class S>
        Spatial::Wrench<S> operator()(const ODE::RKMK::Lie::SE3<S>&, S mass) const
        {
            Spatial::Wrench<S> out{};
            out.f.setZero();
            out.f(5) = S(-10.0) * mass;
            return out;
        }
    };
    static_assert(FlightDynamics::GravityPolicy<UniformGravityPolicy>);

    InertialParameters params(double m, const Eigen::Vector3d& diag, const Eigen::Vector3d& c,
        const Eigen::Vector3d& products = Eigen::Vector3d::Zero())
    {
        InertialParameters ip;
        ip.mass_kg = m;
        ip.Ixx = diag.x();
        ip.Iyy = diag.y();
        ip.Izz = diag.z();
        ip.Ixy = products.x();
        ip.Iyz = products.y();
        ip.Ixz = products.z();
        ip.xbar_m = c.x();
        ip.ybar_m = c.y();
        ip.zbar_m = c.z();
        return ip;
    }

    Vec7 restState(double m)
    {
        Vec7 x = Vec7::Zero();
        x(6) = m;
        return x;
    }
}

TEST_CASE("Spatial inertia: VectorField::M is Spatial::Inertia about the body origin",
    "[spatial_inertia][contract][regression]")
{
    // General case: products of inertia and an offset in all three axes.
    const InertialParameters ip = params(12.5, { 3.0, 7.0, 9.0 }, { 0.8, -0.3, 0.45 }, { 0.2, -0.1, 0.35 });
    const VectorField<BodyWrenchPolicy> vf(ip);

    Eigen::Matrix3d I_C;
    I_C << ip.Ixx, -ip.Ixy, -ip.Ixz,
        -ip.Ixy, ip.Iyy, -ip.Iyz,
        -ip.Ixz, -ip.Iyz, ip.Izz;
    const Spatial::Inertia<double> ref(ip.mass_kg, I_C, Eigen::Vector3d(ip.xbar_m, ip.ybar_m, ip.zbar_m));

    CHECK((vf.M - ref.M).norm() < 1e-12 * ref.M.norm());
    CHECK((vf.M * vf.M_inv - Eigen::Matrix<double, 6, 6>::Identity()).norm() < 1e-12);
    CHECK(vf.mass_kg == ip.mass_kg);

    // The parallel-axis term is there: the rotational block is I_C + m [c]x [c]x^T.
    const Eigen::Vector3d c(ip.xbar_m, ip.ybar_m, ip.zbar_m);
    const Eigen::Matrix3d C = Spatial::skew(c);
    const Eigen::Matrix3d expected = I_C + ip.mass_kg * C * C.transpose();
    CHECK((vf.M.topLeftCorner<3, 3>() - expected).norm() < 1e-12 * expected.norm());
}

TEST_CASE("Spatial inertia: M is positive definite for a centre of mass off the origin",
    "[spatial_inertia][contract][regression]")
{
    // The audit's probe: 10 kg, unit CG inertia, CG 1 m ahead of the origin.
    // Without the parallel-axis term the smallest eigenvalue was -5.47.
    const InertialParameters ip = params(10.0, { 1.0, 1.0, 1.0 }, { 1.0, 0.0, 0.0 });
    const VectorField<BodyWrenchPolicy> vf(ip);

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(vf.M);
    REQUIRE(eig.info() == Eigen::Success);
    CHECK(eig.eigenvalues().minCoeff() > 0.0);
    CHECK((vf.M - vf.M.transpose()).norm() < 1e-12);

    // Rotational kinetic energy for a spin about the y axis through the origin:
    // 1/2 (I_yy + m x^2) w^2 = 1/2 (1 + 10) w^2, not 1/2 w^2.
    Vec6 nu = Vec6::Zero();
    nu(1) = 2.0;
    CHECK_THAT(0.5 * nu.dot(vf.M * nu), WithinAbs(0.5 * 11.0 * 4.0, 1e-12));
}

TEST_CASE("Spatial inertia: a torque about the centre of mass turns the body the same "
          "whether or not the origin is at the centre of mass",
    "[spatial_inertia][contract][regression]")
{
    // Body A: origin at the centre of mass. Body B: the same body described
    // from an origin displaced by c. A pure torque tau about the centre of
    // mass is the wrench [tau; 0] about either origin (no force, so no moment
    // transfer). From rest, A gives wdot = I_C^-1 tau; B must give the same
    // wdot, and its origin accelerates with c x wdot. With the parallel-axis
    // term missing, B's wdot came out wrong by (I_C + m[c]x[c]x)^-1 I_C.
    const Eigen::Vector3d diag(1.0, 2.0, 3.0);
    const Eigen::Vector3d c(1.0, 0.5, -0.3);
    const Eigen::Vector3d tau(0.1, 0.2, 0.3);

    BodyWrenchPolicy torque;
    torque.w.head<3>() = tau;

    const VectorField<BodyWrenchPolicy> A(params(10.0, diag, Eigen::Vector3d::Zero()), torque);
    const VectorField<BodyWrenchPolicy> B(params(10.0, diag, c), torque);

    const SE3d g = SE3d::Identity();
    const Vec7 xA = A(0.0, g, restState(10.0));
    const Vec7 xB = B(0.0, g, restState(10.0));

    const Eigen::Vector3d wdotA = xA.head<3>();
    const Eigen::Vector3d wdotB = xB.head<3>();
    const Eigen::Vector3d wdotExpected = diag.cwiseInverse().cwiseProduct(tau);

    CHECK((wdotA - wdotExpected).norm() < 1e-12);
    CHECK((wdotB - wdotExpected).norm() < 1e-12);
    CHECK(xA.segment<3>(3).norm() < 1e-12);                            // A's origin (the CM) does not accelerate
    CHECK((xB.segment<3>(3) - c.cross(wdotB)).norm() < 1e-12);         // B's origin swings about the CM
}

TEST_CASE("Mass state: free-fall acceleration is g whatever the integrated mass says",
    "[spatial_inertia][contract][mass][regression]")
{
    // Built at 10 kg, evaluated with the mass state at 5 kg: the audit's probe
    // gave -5 m/s^2 because gravity used x(6) and M_inv the constructor mass.
    const InertialParameters ip = params(10.0, { 1.0, 1.0, 1.0 }, Eigen::Vector3d::Zero());
    const VectorField<UniformGravityPolicy> vf(ip);
    const SE3d g = SE3d::Identity();

    for (double mState : { 10.0, 5.0, 20.0 })
    {
        const Vec7 xdot = vf(0.0, g, restState(mState));
        INFO("mass state " << mState);
        CHECK_THAT(xdot(5), WithinAbs(-10.0, 1e-12));
        CHECK(xdot.head<5>().norm() < 1e-12);
        CHECK(xdot(6) == 0.0);  // ConstantMassPolicy
    }
}

TEST_CASE("Mass state: a force divides by the mass M was built with, and follows setInertialParameters",
    "[spatial_inertia][contract][mass][regression]")
{
    BodyWrenchPolicy force;
    force.w(3) = 40.0;  // 40 N along body x

    VectorField<BodyWrenchPolicy> vf(params(10.0, { 1.0, 1.0, 1.0 }, Eigen::Vector3d::Zero()), force);
    const SE3d g = SE3d::Identity();

    CHECK_THAT(vf(0.0, g, restState(10.0))(3), WithinAbs(4.0, 1e-12));
    CHECK_THAT(vf(0.0, g, restState(2.0))(3), WithinAbs(4.0, 1e-12));   // the state does not scale M

    vf.setInertialParameters(params(5.0, { 1.0, 1.0, 1.0 }, Eigen::Vector3d::Zero()));
    CHECK(vf.mass_kg == 5.0);
    CHECK_THAT(vf(0.0, g, restState(5.0))(3), WithinAbs(8.0, 1e-12));
    CHECK_THAT(vf.M(3, 3), WithinAbs(5.0, 1e-12));
    CHECK_THAT(vf.M_inv(3, 3), WithinAbs(0.2, 1e-12));
}

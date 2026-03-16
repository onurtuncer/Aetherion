// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// test_ConceptConformance.cpp
//
// Catch2 concept conformance tests for Aetherion::ODE::RKMK concepts.
// Covers positive and negative cases for all four concepts.
//
// NOTE: All comments are ASCII-only. MSVC rejects Unicode in source files
// even with /utf-8 when the BOM is absent. No math symbols, arrows, or
// Greek letters anywhere in this translation unit.
//
#include <catch2/catch_test_macros.hpp>

#include <Aetherion/ODE/RKMK/Concepts.h>
#include <Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_ProductSE3.h>
#include <Aetherion/FlightDynamics/KinematicsXiField.h>
#include <Aetherion/FlightDynamics/RigidBodyVectorField.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>

#include <Eigen/Core>

using namespace Aetherion;
using namespace Aetherion::ODE::RKMK;

// ------------------------------------------------------------------------------
// Concrete aliases -- all templates fully specialised with Scalar=double
// ------------------------------------------------------------------------------
using KFd = FlightDynamics::KinematicsXiField<double>;
using CVFd = FlightDynamics::RigidBodyVectorField<FlightDynamics::CentralGravityPolicy>;
using Itgd = ODE::RKMK::Integrators::RadauIIA_RKMK_ProductSE3<KFd, CVFd, 7>;

// ------------------------------------------------------------------------------
// Test doubles
// ------------------------------------------------------------------------------
namespace {

    using SE3d = Lie::SE3<double>;

    // --- KinematicsField doubles ---

    struct GoodKinematics {
        Eigen::Matrix<double, 6, 1> operator()(
            const double&,
            const SE3d&,
            const Eigen::Matrix<double, 6, 1>& xi) const
        {
            return xi;
        }
    };

    // Returns wrong dimension -- must fail KinematicsFieldOnSE3.
    struct BadKinematics_WrongDim {
        Eigen::Matrix<double, 3, 1> operator()(
            const double&,
            const SE3d&,
            const Eigen::Matrix<double, 6, 1>&) const
        {
            return {};
        }
    };

    // Non-const operator() -- must fail KinematicsFieldOnSE3.
    struct BadKinematics_NonConst {
        Eigen::Matrix<double, 6, 1> operator()(
            const double&,
            const SE3d&,
            const Eigen::Matrix<double, 6, 1>&)   // no const
        {
            return {};
        }
    };

    // --- VectorField doubles ---

    struct GoodVectorField {
        Eigen::Matrix<double, 7, 1> operator()(
            const double&,
            const SE3d&,
            const Eigen::Matrix<double, 7, 1>&) const
        {
            return {};
        }
    };

    // Correct signature but N=6 -- must fail VectorFieldOnProductSE3<VF,7>.
    struct BadVectorField_WrongN {
        Eigen::Matrix<double, 6, 1> operator()(
            const double&,
            const SE3d&,
            const Eigen::Matrix<double, 6, 1>&) const
        {
            return {};
        }
    };

    // --- StepResult / Integrator doubles ---

    struct GoodStepResult {
        SE3d                      g1;
        Eigen::Matrix<double, 7, 1> x1;
        bool                      converged;
    };

    struct GoodIntegrator {
        using VecE = Eigen::Matrix<double, 7, 1>;
        using StepResult = GoodStepResult;

        StepResult step(
            const double&,
            const SE3d&,
            const VecE&,
            const double&,
            const Core::NewtonOptions&) const
        {
            return {};
        }
    };

    // StepResult missing 'converged' -- must fail RKMKIntegratorOnProductSE3.
    struct BadStepResult_NoConverged {
        SE3d                      g1;
        Eigen::Matrix<double, 7, 1> x1;
    };

    struct BadIntegrator_NoConverged {
        using VecE = Eigen::Matrix<double, 7, 1>;
        using StepResult = BadStepResult_NoConverged;

        StepResult step(
            const double&,
            const SE3d&,
            const VecE&,
            const double&,
            const Core::NewtonOptions&) const
        {
            return {};
        }
    };

    // VecE wrong compile-time dimension -- must fail RKMKIntegratorOnProductSE3<I,7>.
    struct BadIntegrator_WrongDim {
        using VecE = Eigen::Matrix<double, 6, 1>;   // 6 != 7
        using StepResult = GoodStepResult;

        StepResult step(
            const double&,
            const SE3d&,
            const VecE&,
            const double&,
            const Core::NewtonOptions&) const
        {
            return {};
        }
    };

    // Does not satisfy ConstructibleFromInertialParameters -- no matching ctor.
    struct NoBareCtorVF {
        Eigen::Matrix<double, 7, 1> operator()(
            const double&,
            const SE3d&,
            const Eigen::Matrix<double, 7, 1>&) const
        {
            return {};
        }
    };

} // namespace

// ------------------------------------------------------------------------------
// KinematicsFieldOnSE3
// ------------------------------------------------------------------------------
TEST_CASE("KinematicsFieldOnSE3 - positive conformance", "[concepts][kinematics]")
{
    STATIC_REQUIRE(KinematicsFieldOnSE3<KFd>);
    STATIC_REQUIRE(KinematicsFieldOnSE3<GoodKinematics>);
}

TEST_CASE("KinematicsFieldOnSE3 - negative conformance", "[concepts][kinematics]")
{
    STATIC_REQUIRE_FALSE(KinematicsFieldOnSE3<BadKinematics_WrongDim>);
    STATIC_REQUIRE_FALSE(KinematicsFieldOnSE3<BadKinematics_NonConst>);
    STATIC_REQUIRE_FALSE(KinematicsFieldOnSE3<int>);
}

// ------------------------------------------------------------------------------
// VectorFieldOnProductSE3
// ------------------------------------------------------------------------------
TEST_CASE("VectorFieldOnProductSE3 - positive conformance", "[concepts][vectorfield]")
{
    STATIC_REQUIRE(VectorFieldOnProductSE3<GoodVectorField, 7>);
    STATIC_REQUIRE(VectorFieldOnProductSE3<CVFd, 7>);
}

TEST_CASE("VectorFieldOnProductSE3 - negative conformance", "[concepts][vectorfield]")
{
    STATIC_REQUIRE_FALSE(VectorFieldOnProductSE3<BadVectorField_WrongN, 7>);
    STATIC_REQUIRE_FALSE(VectorFieldOnProductSE3<GoodVectorField, 6>);
    STATIC_REQUIRE_FALSE(VectorFieldOnProductSE3<int, 7>);
}

// ------------------------------------------------------------------------------
// RKMKIntegratorOnProductSE3
// ------------------------------------------------------------------------------
TEST_CASE("RKMKIntegratorOnProductSE3 - positive conformance", "[concepts][integrator]")
{
    STATIC_REQUIRE(RKMKIntegratorOnProductSE3<GoodIntegrator, 7>);
    STATIC_REQUIRE(RKMKIntegratorOnProductSE3<Itgd, 7>);
}

TEST_CASE("RKMKIntegratorOnProductSE3 - negative conformance", "[concepts][integrator]")
{
    STATIC_REQUIRE_FALSE(RKMKIntegratorOnProductSE3<BadIntegrator_NoConverged, 7>);
    STATIC_REQUIRE_FALSE(RKMKIntegratorOnProductSE3<BadIntegrator_WrongDim, 7>);
    STATIC_REQUIRE_FALSE(RKMKIntegratorOnProductSE3<int, 7>);
}

// ------------------------------------------------------------------------------
// ConstructibleFromInertialParameters
// ------------------------------------------------------------------------------
TEST_CASE("ConstructibleFromInertialParameters", "[concepts][construction]")
{
    STATIC_REQUIRE(ConstructibleFromInertialParameters<CVFd>);
    STATIC_REQUIRE_FALSE(ConstructibleFromInertialParameters<NoBareCtorVF>);
}
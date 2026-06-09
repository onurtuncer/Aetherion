// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// test_ConceptConformance.cpp
//
// Catch2 concept conformance tests for Aetherion::ODE::RKMK concepts.
// Covers positive and negative cases for all four concepts.

#include <catch2/catch_test_macros.hpp>

#include <Aetherion/ODE/RKMK/Concepts.h>
#include <Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_ProductSE3.h>
#include <Aetherion/RigidBody/KinematicsXiField.h>
#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>

#include <Eigen/Core>
#include <type_traits>

using namespace Aetherion;
using namespace Aetherion::ODE::RKMK;

// ------------------------------------------------------------------------------
// Concrete aliases -- all templates fully specialised with Scalar=double
// ------------------------------------------------------------------------------
using KFd = RigidBody::KinematicsXiField;
using CVFd = RigidBody::VectorField<FlightDynamics::CentralGravityPolicy>;
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

    // --- IntegratorFor doubles ---

    // Satisfies both RKMKIntegratorOnProductSE3<I,7> and
    // constructible_from<I, KFd, CVFd>.  Models a drop-in replacement integrator.
    struct GoodIntegratorFor {
        using VecE = Eigen::Matrix<double, 7, 1>;
        using StepResult = GoodStepResult;

        GoodIntegratorFor(const KFd&, const CVFd&) {}

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

    // Satisfies RKMKIntegratorOnProductSE3 but has no (KFd, CVFd) constructor.
    // Used as the negative case for IntegratorFor.
    // (GoodIntegrator, defined above, already plays this role.)

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

// ------------------------------------------------------------------------------
// IntegratorFor
// ------------------------------------------------------------------------------
TEST_CASE("IntegratorFor - positive: RadauIIA satisfies the concept", "[concepts][integrator_for]")
{
    // Itgd = RadauIIA_RKMK_ProductSE3<KFd, CVFd, 7>:
    //   - satisfies RKMKIntegratorOnProductSE3<Itgd, 7>
    //   - is constructible from (KFd, CVFd)
    STATIC_REQUIRE(IntegratorFor<Itgd, KFd, CVFd, 7>);
    STATIC_REQUIRE(IntegratorFor<GoodIntegratorFor, KFd, CVFd, 7>);
}

TEST_CASE("IntegratorFor - negative: not constructible from (XiField, FField)", "[concepts][integrator_for]")
{
    // GoodIntegrator satisfies RKMKIntegratorOnProductSE3 but has no (KFd, CVFd) ctor.
    STATIC_REQUIRE_FALSE(IntegratorFor<GoodIntegrator, KFd, CVFd, 7>);
    STATIC_REQUIRE_FALSE(IntegratorFor<int, KFd, CVFd, 7>);
}

TEST_CASE("IntegratorFor - negative: wrong Euclidean dimension", "[concepts][integrator_for]")
{
    // Itgd::VecE is R^7; checking against N=6 must fail.
    STATIC_REQUIRE_FALSE(IntegratorFor<Itgd, KFd, CVFd, 6>);
}

// ------------------------------------------------------------------------------
// SixDoFStepper: default IntegratorPolicy matches RadauIIA
// ------------------------------------------------------------------------------
TEST_CASE("SixDoFStepper: default Integrator alias is RadauIIA_RKMK_ProductSE3",
          "[concepts][stepper]")
{
    using DefaultStepper = RigidBody::SixDoFStepper<CVFd>;
    STATIC_REQUIRE(std::is_same_v<DefaultStepper::Integrator, Itgd>);
}

TEST_CASE("SixDoFStepper: custom IntegratorPolicy is accepted when it satisfies IntegratorFor",
          "[concepts][stepper]")
{
    using CustomStepper = RigidBody::SixDoFStepper<CVFd, 7, GoodIntegratorFor>;
    STATIC_REQUIRE(std::is_same_v<CustomStepper::Integrator, GoodIntegratorFor>);
}
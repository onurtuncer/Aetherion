// ------------------------------------------------------------------------------
// Project: Aetherion — Catch2 tests for the optional policy hooks of the
// simulation layer: ISimulator built from a pre-made VectorField, the
// protected accessors closed-loop simulators use, and MakeSnapshot2's
// air-data path for aero policies that carry a steady wind and a
// non-standard day.
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Aetherion/Examples/DraglessSphere/DraglessSphereTypes.h>
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/FlightDynamics/Policies/PropulsionPolicies.h>
#include <Aetherion/Simulation/ISimulator.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot2.h>

using namespace Aetherion;
using Examples::DraglessSphere::DraglessSphereVF;

namespace {

    RigidBody::InertialParameters makeIP()
    {
        RigidBody::InertialParameters ip;
        ip.mass_kg = 1.0;
        ip.Ixx = 1.0;
        ip.Iyy = 1.0;
        ip.Izz = 1.0;
        return ip;
    }

// 1 km above the equator at longitude 0 (ECI == ECEF at theta_gst = 0),
// body axes aligned with ECI, moving 100 m/s along ECI +Y (east).
    RigidBody::StateD lowState()
    {
        RigidBody::StateD s;
        s.g.R = Eigen::Matrix3d::Identity();
        s.g.q = Eigen::Quaterniond::Identity();
        s.g.p = Eigen::Vector3d(Environment::WGS84::kSemiMajorAxis_m + 1000.0, 0.0, 0.0);
        s.nu_B.setZero();
        s.nu_B(4) = 100.0;
        s.m = 1.0;
        return s;
    }

/// Zero-wrench aero policy exposing the optional hooks MakeSnapshot2 looks for.
    struct WindyDayAeroPolicy
    {
        Eigen::Vector3d wind{ Eigen::Vector3d::Zero() };
        Environment::AtmosphereOffsets atm{};

        template <class S>
        Spatial::Wrench<S> operator()(const ODE::RKMK::Lie::SE3<S>&, const Eigen::Matrix<S, 6, 1>&, S, S) const
        {
            Spatial::Wrench<S> w{};
            w.f.setZero();
            return w;
        }
        [[nodiscard]] const Eigen::Vector3d& windECEF() const noexcept { return wind; }
        [[nodiscard]] const Environment::AtmosphereOffsets& atmosphereOffsets() const noexcept { return atm; }
    };

    static_assert(FlightDynamics::AeroPolicy<WindyDayAeroPolicy>);

/// Simulator built from a pre-made vector field, with a hook that edits it in place.
    class PrebuiltFieldSimulator
        : public Simulation::ISimulator<DraglessSphereVF, Simulation::Snapshot1>
    {
    public:
        PrebuiltFieldSimulator(DraglessSphereVF vf, RigidBody::StateD s0)
            : ISimulator(std::move(vf), std::move(s0))
        {
            validate();  // the base's default check accepts anything
        }

        void scaleInertia(double k) { mutableVectorField().M *= k; }

        [[nodiscard]] Simulation::Snapshot1 snapshot() const noexcept override
        {
            return Simulation::MakeSnapshot1(time(), state(), 0.0, vectorField().gravity);
        }
    };

} // namespace

TEST_CASE("ISimulator: constructed from a pre-built VectorField", "[ISimulator]")
{
    DraglessSphereVF vf(makeIP(),
        FlightDynamics::J2GravityPolicy{},
        FlightDynamics::ZeroAeroPolicy{},
        FlightDynamics::ZeroPropulsionPolicy{});
    REQUIRE(vf.M(3, 3) == Catch::Approx(1.0));

    PrebuiltFieldSimulator sim(vf, lowState());
    REQUIRE(sim.time() == 0.0);
    REQUIRE(sim.vectorField().M(0, 0) == Catch::Approx(1.0));

    // Closed-loop simulators edit the field between steps through the
    // protected accessor; the change is visible through the public one.
    sim.scaleInertia(2.0);
    REQUIRE(sim.vectorField().M(0, 0) == Catch::Approx(2.0));
    REQUIRE(sim.vectorField().M(5, 5) == Catch::Approx(2.0));

    const auto res = sim.step(0.01);
    REQUIRE(res.converged);
    REQUIRE(sim.time() == Catch::Approx(0.01));
    REQUIRE(sim.snapshot().time == Catch::Approx(0.01));
}

TEST_CASE("MakeSnapshot2: calm air and a standard day match the zero aero policy", "[MakeSnapshot2]")
{
    const auto s = lowState();
    FlightDynamics::J2GravityPolicy gravity{};
    const auto ref = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity, FlightDynamics::ZeroAeroPolicy{});
    const auto calm = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity, WindyDayAeroPolicy{});
    REQUIRE(calm.trueAirspeed_m_s == ref.trueAirspeed_m_s);
    REQUIRE(calm.ambientTemperature_K == ref.ambientTemperature_K);
    REQUIRE(calm.airDensity_kg_m3 == ref.airDensity_kg_m3);
}

TEST_CASE("MakeSnapshot2: the policy's steady wind is removed from the air data", "[MakeSnapshot2]")
{
    const auto s = lowState();
    FlightDynamics::J2GravityPolicy gravity{};
    const auto calm = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity, WindyDayAeroPolicy{});

    // At (lat, lon) = (0, 0) ECEF +Y is local east.
    WindyDayAeroPolicy windy;
    windy.wind = Eigen::Vector3d(0.0, 10.0, 0.0);
    const auto snap = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity, windy);

    const Eigen::Vector3d v_air = calm.feVelocity_m_s - Eigen::Vector3d(0.0, 10.0, 0.0);
    REQUIRE(snap.trueAirspeed_m_s == Catch::Approx(v_air.norm()).epsilon(1e-9));
    REQUIRE(snap.trueAirspeed_m_s != Catch::Approx(calm.trueAirspeed_m_s));
    REQUIRE(snap.mach == Catch::Approx(snap.trueAirspeed_m_s / snap.speedOfSound_m_s));
    // Ground-relative quantities are untouched by the wind.
    REQUIRE(snap.feVelocity_m_s.isApprox(calm.feVelocity_m_s));
}

TEST_CASE("MakeSnapshot2: a non-standard day changes the reported atmosphere", "[MakeSnapshot2]")
{
    const auto s = lowState();
    FlightDynamics::J2GravityPolicy gravity{};
    const auto standard = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity, WindyDayAeroPolicy{});

    WindyDayAeroPolicy hot;
    hot.atm.deltaT_K = 15.0;
    const auto snap = Simulation::MakeSnapshot2(0.0, s, 0.0, gravity, hot);

    const auto expected = Environment::US1976Atmosphere(snap.altitudeMsl_m, hot.atm);
    REQUIRE(snap.ambientTemperature_K == Catch::Approx(expected.T));
    REQUIRE(snap.airDensity_kg_m3 == Catch::Approx(expected.rho));
    REQUIRE(snap.ambientTemperature_K > standard.ambientTemperature_K + 10.0);
}

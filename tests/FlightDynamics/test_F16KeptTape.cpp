// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_F16KeptTape.cpp
//
// The F-16 plant, stepped the way F16PlantFMU steps it -- the AD tape of the
// stage residual kept from step to step, the Jacobian taken in one sweep, no
// Jacobian at an iterate that has already converged -- against the F-16 plant
// stepped the way it was up to 0.19.0: a tape recorded on every step with the
// controls, the gust and the start state as constants on it, CppAD's dense
// Jacobian driver, every iterate given a Newton step.
//
// The two trajectories must agree bit for bit while the controls move on every
// step, the gust is switched on, changes on every step and is switched off
// again, and the flight condition wanders across breakpoints of the aero and
// engine tables (so the kept tape has to be recorded again on the way).
//
// The reference is the same vector field wrapped so that it says nothing
// about tape parameters; TapeParameter<S>() then yields constants.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <cppad/cppad.hpp>

#include <Aetherion/Environment/DrydenTurbulence.h>
#include <Aetherion/Examples/F16SteadyFlight/F16Types.h>
#include <Aetherion/FlightDynamics/Trim/TrimBodyRates.h>
#include <Aetherion/FlightDynamics/Trim/TrimSolver.h>
#include <Aetherion/FlightDynamics/Trim/TrimWeight.h>
#include <Aetherion/ODE/RKMK/Core/TapeParameters.h>
#include <Aetherion/RigidBody/BuildInitialState.h>
#include <Aetherion/RigidBody/Config.h>
#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/RigidBody/StateLayout.h>
#include <Aetherion/Serialization/DAVEML/LoadInertiaFromDAVEML.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <numbers>
#include <string>

using namespace Aetherion;

#ifndef DAVEML_F16_AERO_FILE
#define DAVEML_F16_AERO_FILE ""
#endif
#ifndef DAVEML_F16_PROP_FILE
#define DAVEML_F16_PROP_FILE ""
#endif

namespace {

    using F16VF = Examples::F16SteadyFlight::F16VF;

    /// F16VF declaring no tape parameters: a tape per step, all constants.
    struct PlainF16
    {
        F16VF vf;
        template <class S>
        Eigen::Matrix<S, 7, 1> operator()(S t, const ODE::RKMK::Lie::SE3<S>& g, const Eigen::Matrix<S, 7, 1>& x) const
        {
            return vf(t, g, x);
        }
    };

    using KeptStepper = RigidBody::SixDoFStepper<F16VF>;
    using PlainStepper = RigidBody::SixDoFStepper<PlainF16>;

    static_assert(ODE::RKMK::Core::TapeParameterised<F16VF>);
    static_assert(!ODE::RKMK::Core::TapeParameterised<PlainF16>);
    static_assert(KeptStepper::Integrator::kKeepsTape);
    static_assert(!PlainStepper::Integrator::kKeepsTape);

    bool sameState(const RigidBody::StateD& a, const RigidBody::StateD& b)
    {
        using Aetherion::ODE::RKMK::Core::BitIdentical;
        return BitIdentical(a.g.R, b.g.R) && BitIdentical(a.g.p, b.g.p) && BitIdentical(a.nu_B, b.nu_B) && BitIdentical(a.m, b.m);
    }

    std::string siblingFile(const std::string& path, const std::string& name)
    {
        const auto slash = path.find_last_of("/\\");
        return (slash == std::string::npos ? std::string() : path.substr(0, slash + 1)) + name;
    }
}

TEST_CASE("F16 plant: the kept tape and the single-sweep Jacobian reproduce the per-step tape bit for bit",
    "[f16][tape][regression]")
{
    const std::string aeroFile = DAVEML_F16_AERO_FILE;
    const std::string propFile = DAVEML_F16_PROP_FILE;
    REQUIRE_FALSE(aeroFile.empty());
    REQUIRE_FALSE(propFile.empty());

    constexpr double kFt_m = 0.3048;
    constexpr double kDeg = std::numbers::pi / 180.0;
    constexpr double kLat = 36.01917, kLon = -75.67444, kHeading = 45.0, kAlt_ft = 10013.0, kVt_fps = 565.685, kXcg_ft = 1.132;

    const auto ip = Serialization::LoadInertiaFromDAVEML(siblingFile(aeroFile, "F16_inertia.dml"));
    const auto aero = std::make_shared<const Serialization::DAVEMLAeroModel>(aeroFile);
    const auto prop = std::make_shared<const Serialization::DAVEMLPropModel>(propFile);

    const double vt = kVt_fps * kFt_m;
    const double vN = vt * std::cos(kHeading * kDeg), vE = vt * std::sin(kHeading * kDeg);
    FlightDynamics::TrimInputs tin{};
    tin.vt_fps = kVt_fps;
    tin.alt_ft = kAlt_ft;
    tin.weight_lbf = FlightDynamics::LevelFlightTrimWeight_lbf(ip.mass_kg, kLat, kAlt_ft * kFt_m, vN, vE);
    tin.bodyRates = FlightDynamics::LevelFlightBodyRates(kLat, kAlt_ft * kFt_m, vN, vE, kHeading, 0.0, 0.0);
    const FlightDynamics::TrimSolver solver(*aero, *prop, kXcg_ft);
    const FlightDynamics::TrimPoint trim = solver.solve(tin);
    REQUIRE(trim.converged);

    RigidBody::Config cfg{};
    cfg.pose.lat_deg = kLat;
    cfg.pose.lon_deg = kLon;
    cfg.pose.alt_m = kAlt_ft * kFt_m;
    cfg.pose.azimuth_deg = kHeading;
    cfg.pose.zenith_deg = 90.0 - trim.alpha_deg;
    cfg.pose.roll_deg = 0.0;
    cfg.velocityNED.north_mps = vN;
    cfg.velocityNED.east_mps = vE;
    cfg.velocityNED.down_mps = 0.0;
    cfg.bodyRates = FlightDynamics::LevelFlightBodyRates(kLat, kAlt_ft * kFt_m, vN, vE, kHeading, trim.alpha_deg, 0.0);
    cfg.inertialParameters = ip;
    const auto x0 = RigidBody::BuildInitialStateVector(cfg, 0.0);
    using L = RigidBody::StateLayout;
    RigidBody::StateD s{};
    s.g.p = Eigen::Vector3d(x0[L::IDX_P], x0[L::IDX_P + 1], x0[L::IDX_P + 2]);
    s.g.q = Eigen::Quaterniond(x0[L::IDX_Q], x0[L::IDX_Q + 1], x0[L::IDX_Q + 2], x0[L::IDX_Q + 3]).normalized();
    s.g.R = s.g.q.toRotationMatrix();
    s.nu_B << x0[L::IDX_W], x0[L::IDX_W + 1], x0[L::IDX_W + 2], x0[L::IDX_V], x0[L::IDX_V + 1], x0[L::IDX_V + 2];
    s.m = x0[L::IDX_M];

    const F16VF field(ip, FlightDynamics::J2GravityPolicy{},
        FlightDynamics::F16AeroPolicy(aero, trim.el_deg, 0.0, 0.0, kXcg_ft * kFt_m),
        FlightDynamics::F16PropPolicy(prop, trim.pwr_pct));

    // The FMU's configuration ...
    ODE::RKMK::Core::NewtonOptions fast{};
    fast.report_step_norm = false;
    fast.jacobian_single_sweep = true;
    KeptStepper kept(field, fast);
    // ... and the one it had: defaults, and a field that declares nothing.
    PlainStepper plain(PlainF16{ field });

    Environment::DrydenParameters tp{};
    tp.sigma_u_mps = tp.sigma_v_mps = 3.0;
    tp.sigma_w_mps = 3.0;
    tp.wingspan_m = aero->bspanFt() * kFt_m;
    Environment::DrydenTurbulence turbulence(tp, 11);

    const double dt = 0.01;
    const int steps = 240;
    int kept_tapes = 0, plain_tapes = 0, kept_jacobians = 0, plain_jacobians = 0;
    for (int k = 0; k < steps; ++k) {
        const double t = k * dt;
        // Controls move on every step, far enough to change table segments.
        const double el = trim.el_deg + 4.0 * std::sin(2.1 * t);
        const double ail = 6.0 * std::sin(1.7 * t + 1.0);
        const double rdr = 5.0 * std::sin(1.3 * t + 2.0);
        const double pwr = trim.pwr_pct + 25.0 * std::sin(0.9 * t);
        // Calm for the first 20 steps, turbulent for 200, calm again.
        Environment::GustState gust{};
        if (k >= 20 && k < 220) gust = turbulence.step(dt, vt);

        for (F16VF* vf : { &kept.vectorField(), &plain.vectorField().vf }) {
            vf->aero.setControls(el, ail, rdr);
            vf->thrust.pwr_pct = pwr;
            vf->aero.setGust(gust);
            vf->thrust.setGust(gust);
        }

        const auto a = kept.step(t, s, dt);
        const auto b = plain.step(t, s, dt);
        INFO("step " << k);
        REQUIRE(b.converged);
        REQUIRE(a.converged);
        REQUIRE(a.iters == b.iters);
        REQUIRE(sameState(KeptStepper::unpack(a), PlainStepper::unpack(b)));
        kept_tapes += a.tapes_recorded;
        plain_tapes += b.tapes_recorded;
        kept_jacobians += a.jacobian_evals;
        plain_jacobians += b.jacobian_evals;
        s = PlainStepper::unpack(b);
    }

    INFO("tapes recorded: kept " << kept_tapes << ", per step " << plain_tapes << "; Jacobians: " << kept_jacobians
                                 << " against " << plain_jacobians);
    CHECK(plain_tapes >= steps);   // the reference records on every step
    CHECK(kept_tapes >= 3);        // first step, gust on, and at least one table segment
    CHECK(kept_tapes < steps / 2); // and is otherwise kept
    CHECK(kept_jacobians < plain_jacobians);
}

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// test_ConvergenceOrder.cpp
//
// Two distinct properties are checked here, and they must not be confused.
//
// 1. Exactness on constant body twist (symmetric sphere)
// ------------------------------------------------------------------------
// For a torque-free *symmetric* body the Newton-Euler equations give
// omega_dot = 0 and v_dot_B = -omega x v_B.  With v_B = 0 the body twist
// xi = [omega0; 0] is constant, so the exact flow is the one-parameter
// subgroup g(t) = g0 * Exp(t * xi).  Any RKMK method reproduces this
// *exactly* (to rounding), independent of its classical order, because the
// stage system collapses onto the exponential of a constant algebra element.
//
// This makes the symmetric sphere useless as a convergence-order test --
// every step size lands at machine precision and the log-log slope is
// meaningless noise.  It is a valuable *structure-preservation* test
// instead, and is retained as such below.
//
// 2. Convergence order (torque-free asymmetric body)
// ------------------------------------------------------------------------
// To measure order the flow must leave the one-parameter subgroup.  A
// torque-free *asymmetric* body does exactly that: Euler's equations
//
//     omega_dot = -I^-1 (omega x I omega)          (nonlinear, non-constant)
//     v_dot_B   = -omega x v_B
//
// make xi(t) genuinely time-varying, so the dexp^-1 correction and the
// Radau IIA stage coupling are both exercised.  The reference solution is
// computed at a step size 50x smaller than the finest tested step, and is
// cross-validated between the two independent integrators so that the
// measured order is not an artefact of self-comparison.
//
// Error metrics at T:
//   e_rot(h) = ||R_num(T) - R_ref(T)||_F      (Frobenius)
//   e_pos(h) = ||p_num(T) - p_ref(T)||_2      [m]
//
// CSV output (if AETHERION_SOURCE_DIR is defined via CMake):
//   ${AETHERION_SOURCE_DIR}/papers/eucass/data/convergence_order.csv
//   Columns: h, rot_radau5, pos_radau5, rot_rk4, pos_rk4
//
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/KinematicsXiField.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <Aetherion/ODE/RKMK/Integrators/ExplicitRK4_RKMK_ProductSE3.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

using namespace Aetherion;
using namespace Aetherion::RigidBody;
using namespace Aetherion::ODE::RKMK;

// ---------------------------------------------------------------------------
// Type aliases
// ---------------------------------------------------------------------------
using ZeroVF = RigidBody::VectorField<FlightDynamics::ZeroGravityPolicy>;

using RadauStepper = SixDoFStepper<ZeroVF>;

using RK4Stepper = SixDoFStepper<
    ZeroVF,
    7,
    Integrators::ExplicitRK4_RKMK_ProductSE3<KinematicsXiField, ZeroVF, 7>>;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
namespace {

    // Torque-free symmetric sphere: Ixx = Iyy = Izz, CG at the reference point.
    InertialParameters symmetric_sphere()
    {
        InertialParameters ip;
        ip.mass_kg = 1.0;
        ip.Ixx = 0.4; ip.Iyy = 0.4; ip.Izz = 0.4;
        ip.Ixy = 0.0; ip.Iyz = 0.0; ip.Ixz = 0.0;
        ip.xbar_m = 0.0; ip.ybar_m = 0.0; ip.zbar_m = 0.0;
        return ip;
    }

    // Torque-free asymmetric body (brick-like principal inertias, all distinct).
    // The intermediate-axis term omega x I omega is then non-zero for a generic
    // omega, so Euler's equations are genuinely nonlinear.
    InertialParameters asymmetric_body()
    {
        InertialParameters ip;
        ip.mass_kg = 1.0;
        ip.Ixx = 0.10; ip.Iyy = 0.25; ip.Izz = 0.32;
        ip.Ixy = 0.0;  ip.Iyz = 0.0;  ip.Ixz = 0.0;
        ip.xbar_m = 0.0; ip.ybar_m = 0.0; ip.zbar_m = 0.0;
        return ip;
    }

    StateD sphere_initial_state()
    {
        StateD s;
        s.g = Lie::SE3<double>::Identity();
        s.nu_B << 1.0, 0.5, 0.25,
                  0.0, 0.0, 0.0;
        s.m = 1.0;
        return s;
    }

    // Fast tumble plus a non-zero body velocity so that the SE(3) screw
    // coupling J(omega) v -- not just the SO(3) factor -- is exercised.
    StateD asymmetric_initial_state()
    {
        StateD s;
        s.g = Lie::SE3<double>::Identity();
        s.nu_B <<  1.0,  2.0,  0.5,
                  50.0,  5.0, -3.0;
        s.m = 1.0;
        return s;
    }

    // Exact rotation at time T for constant omega: R(T) = Exp_SO3(omega0 * T).
    Eigen::Matrix3d exact_rotation(const Eigen::Vector3d& omega0, double T)
    {
        const Eigen::Vector3d phi   = omega0 * T;
        const double          theta = phi.norm();
        if (theta < 1e-12) return Eigen::Matrix3d::Identity();

        Eigen::Matrix3d A;
        A <<  0.0,    -phi(2),  phi(1),
              phi(2),  0.0,    -phi(0),
             -phi(1),  phi(0),  0.0;

        return Eigen::Matrix3d::Identity()
             + (std::sin(theta) / theta) * A
             + ((1.0 - std::cos(theta)) / (theta * theta)) * A * A;
    }

    // Integrate over [0, T] with fixed step h; return the final state.
    template<class Stepper>
    StateD integrate(Stepper& stepper, const StateD& s0, double T, double h)
    {
        const int N = static_cast<int>(std::llround(T / h));
        StateD    s = s0;
        for (int i = 0; i < N; ++i) {
            auto res = stepper.step(static_cast<double>(i) * h, s, h);
            REQUIRE(res.converged);
            s = Stepper::unpack(res);
        }
        return s;
    }

    double empirical_order(double h1, double e1, double h2, double e2)
    {
        return std::log(e1 / e2) / std::log(h1 / h2);
    }

    // Mean log-log slope over consecutive step-size pairs, ignoring pairs where
    // either error has bottomed out on the round-off floor.
    double mean_order(const std::vector<double>& h,
                      const std::vector<double>& e,
                      double                     floor_val = 1e-13)
    {
        double sum = 0.0;
        int    n   = 0;
        for (std::size_t i = 0; i + 1 < h.size(); ++i) {
            if (e[i] > floor_val && e[i + 1] > floor_val) {
                sum += empirical_order(h[i], e[i], h[i + 1], e[i + 1]);
                ++n;
            }
        }
        return (n > 0) ? sum / static_cast<double>(n) : 0.0;
    }

    // Convergence-study configuration, shared by the order tests and the CSV
    // writer so that the published figure and the assertions cannot drift apart.
    constexpr double            kT     = 2.0;      // integration horizon [s]
    constexpr double            kHRef  = 2.5e-4;   // reference step size  [s]
    const std::array<double, 5> kHVec  { 0.2, 0.1, 0.05, 0.025, 0.0125 };

    Core::NewtonOptions tight_newton()
    {
        Core::NewtonOptions opt;
        opt.abs_tol   = 1e-13;
        opt.rel_tol   = 1e-13;
        opt.max_iters = 50;
        return opt;
    }

    struct ErrorSeries {
        std::vector<double> rot;
        std::vector<double> pos;
    };

    // Reference solution for the asymmetric case, computed once at kHRef.
    const StateD& asymmetric_reference()
    {
        static const StateD ref = [] {
            RadauStepper stepper{ asymmetric_body(), tight_newton() };
            return integrate(stepper, asymmetric_initial_state(), kT, kHRef);
        }();
        return ref;
    }

    template<class Stepper>
    ErrorSeries error_series(Stepper& stepper, const StateD& ref)
    {
        ErrorSeries out;
        for (double h : kHVec) {
            const StateD s = integrate(stepper, asymmetric_initial_state(), kT, h);
            out.rot.push_back((s.g.R - ref.g.R).norm());
            out.pos.push_back((s.g.p - ref.g.p).norm());
        }
        return out;
    }

} // namespace

// ---------------------------------------------------------------------------
// 1. Structure preservation: exactness on a constant body twist
// ---------------------------------------------------------------------------
TEST_CASE("RKMK on SE(3) is exact for a constant body twist (symmetric sphere)",
    "[convergence][exactness][sphere]")
{
    constexpr double      T = 1.0;
    const Eigen::Vector3d omega0{ 1.0, 0.5, 0.25 };
    const Eigen::Matrix3d R_exact = exact_rotation(omega0, T);

    RadauStepper radau{ symmetric_sphere(), tight_newton() };
    RK4Stepper   rk4  { symmetric_sphere() };

    for (double h : { 0.5, 0.25, 0.1, 0.05, 0.025, 0.01 }) {
        const StateD s_radau = integrate(radau, sphere_initial_state(), T, h);
        const StateD s_rk4   = integrate(rk4,   sphere_initial_state(), T, h);

        const double e_radau = (s_radau.g.R - R_exact).norm();
        const double e_rk4   = (s_rk4.g.R   - R_exact).norm();

        INFO("h = " << h << "  e_radau = " << e_radau << "  e_rk4 = " << e_rk4);

        // Both integrators reproduce the one-parameter subgroup to rounding,
        // regardless of step size.  This is the defining property of a
        // Lie-group integrator, not a statement about classical order.
        REQUIRE(e_radau < 1e-13);
        REQUIRE(e_rk4   < 1e-13);
    }
}

// ---------------------------------------------------------------------------
// 2. Reference-solution sanity check
// ---------------------------------------------------------------------------
// The order tests below compare against a Radau IIA solution at kHRef.  If that
// reference were biased towards one method the measured slopes would be
// meaningless, so it is cross-validated here against the *other* integrator run
// at the same fine step.
// ---------------------------------------------------------------------------
TEST_CASE("Convergence reference is method-independent",
    "[convergence][reference]")
{
    RK4Stepper   rk4{ asymmetric_body() };
    const StateD ref_rk4 = integrate(rk4, asymmetric_initial_state(), kT, kHRef);
    const StateD& ref    = asymmetric_reference();

    const double d_rot = (ref_rk4.g.R - ref.g.R).norm();
    const double d_pos = (ref_rk4.g.p - ref.g.p).norm();

    INFO("Radau vs RK4 at h_ref = " << kHRef
         << ":  d_rot = " << d_rot << "  d_pos = " << d_pos);

    // At h_ref the two independent integrators must agree far below the
    // smallest error appearing in the convergence study (~1e-9, see below).
    REQUIRE(d_rot < 1e-11);
    REQUIRE(d_pos < 1e-8);
}

// ---------------------------------------------------------------------------
// 3. Radau IIA RKMK -- expected order 5
// ---------------------------------------------------------------------------
TEST_CASE("Convergence order: Radau IIA RKMK on SE(3) attains order 5",
    "[convergence][radau]")
{
    RadauStepper stepper{ asymmetric_body(), tight_newton() };
    const ErrorSeries e = error_series(stepper, asymmetric_reference());

    const std::vector<double> h(kHVec.begin(), kHVec.end());
    const double p_rot = mean_order(h, e.rot);
    const double p_pos = mean_order(h, e.pos);

    INFO("Radau IIA RKMK mean order: rotation = " << p_rot << ", position = " << p_pos);
    for (std::size_t i = 0; i < h.size(); ++i)
        INFO("  h=" << h[i] << "  e_rot=" << e.rot[i] << "  e_pos=" << e.pos[i]);

    REQUIRE(p_rot > 4.5);
    REQUIRE(p_pos > 4.5);
}

// ---------------------------------------------------------------------------
// 4. Explicit RK4 RKMK -- expected order 4
// ---------------------------------------------------------------------------
TEST_CASE("Convergence order: explicit RK4 RKMK on SE(3) attains order 4",
    "[convergence][rk4]")
{
    RK4Stepper stepper{ asymmetric_body() };
    const ErrorSeries e = error_series(stepper, asymmetric_reference());

    const std::vector<double> h(kHVec.begin(), kHVec.end());
    const double p_rot = mean_order(h, e.rot);
    const double p_pos = mean_order(h, e.pos);

    INFO("Explicit RK4 RKMK mean order: rotation = " << p_rot << ", position = " << p_pos);
    for (std::size_t i = 0; i < h.size(); ++i)
        INFO("  h=" << h[i] << "  e_rot=" << e.rot[i] << "  e_pos=" << e.pos[i]);

    REQUIRE(p_rot > 3.5);
    REQUIRE(p_rot < 4.6);   // must be clearly separated from the order-5 scheme
    REQUIRE(p_pos > 3.5);
}

// ---------------------------------------------------------------------------
// 5. CSV export for the paper figure
// ---------------------------------------------------------------------------
TEST_CASE("Convergence order: write CSV for paper figure",
    "[convergence][csv]")
{
    const StateD& ref = asymmetric_reference();

    ErrorSeries e_radau;
    {
        RadauStepper stepper{ asymmetric_body(), tight_newton() };
        e_radau = error_series(stepper, ref);
    }

    ErrorSeries e_rk4;
    {
        RK4Stepper stepper{ asymmetric_body() };
        e_rk4 = error_series(stepper, ref);
    }

    WARN("=== Convergence data (h, rot_radau5, pos_radau5, rot_rk4, pos_rk4) ===");
    for (std::size_t i = 0; i < kHVec.size(); ++i)
        WARN("  " << kHVec[i]
             << "  " << e_radau.rot[i] << "  " << e_radau.pos[i]
             << "  " << e_rk4.rot[i]   << "  " << e_rk4.pos[i]);

    const std::vector<double> h(kHVec.begin(), kHVec.end());
    WARN("mean order Radau: rot = " << mean_order(h, e_radau.rot)
         << ", pos = " << mean_order(h, e_radau.pos));
    WARN("mean order RK4  : rot = " << mean_order(h, e_rk4.rot)
         << ", pos = " << mean_order(h, e_rk4.pos));

#ifdef AETHERION_SOURCE_DIR
    const std::string csv_path =
        std::string(AETHERION_SOURCE_DIR) + "/papers/eucass/data/convergence_order.csv";

    std::ofstream ofs(csv_path);
    if (ofs.is_open()) {
        ofs << "h,rot_radau5,pos_radau5,rot_rk4,pos_rk4\n";
        ofs << std::scientific;
        ofs.precision(8);
        for (std::size_t i = 0; i < kHVec.size(); ++i)
            ofs << kHVec[i] << "," << e_radau.rot[i] << "," << e_radau.pos[i]
                            << "," << e_rk4.rot[i]   << "," << e_rk4.pos[i] << "\n";
        WARN("CSV written to: " << csv_path);
    } else {
        WARN("Could not write CSV to: " << csv_path
             << " -- ensure papers/eucass/data/ exists.");
    }
#else
    WARN("AETHERION_SOURCE_DIR not defined -- no CSV written.");
#endif
}

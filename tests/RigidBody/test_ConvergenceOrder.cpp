// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// test_ConvergenceOrder.cpp
//
// Empirical convergence-order study for the SE(3) RKMK integrators.
//
// Test case: torque-free symmetric sphere in zero gravity
// -------------------------------------------------------
// Initial state (at the origin, identity pose):
//   omega_B = [1.0, 0.5, 0.25] rad/s
//   v_B     = [0.0, 0.0, 0.0]  m/s
//   m       = 1.0 kg
//
// For a symmetric sphere (Ixx = Iyy = Izz) with no external forces,
// Euler's equations give omega_dot = 0, so the exact solution is:
//
//   omega(t) = omega0                    (constant)
//   R(t)     = Exp_SO3(omega0 * t)       (Rodrigues formula)
//   p(t)     = 0                         (no translation)
//
// Error metric at T = 1 s:
//   e(h) = ||R_num(T) - R_exact(T)||_F   (Frobenius norm)
//
// Step sizes: h in {0.5, 0.25, 0.1, 0.05, 0.025, 0.01} s.
//
// Expected empirical orders:
//   RadauIIA RKMK  (order 5 method): mean log-slope >= 4.0
//   Explicit RK4 RKMK (order 4 method): mean log-slope >= 3.0
//
// CSV output (if AETHERION_SOURCE_DIR is defined via CMake):
//   ${AETHERION_SOURCE_DIR}/papers/eucass/data/convergence_order.csv
//   Columns: h, error_radau5_rkmk, error_rk4_rkmk
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

    InertialParameters symmetric_sphere()
    {
        InertialParameters ip;
        ip.mass_kg = 1.0;
        ip.Ixx = 0.4; ip.Iyy = 0.4; ip.Izz = 0.4;
        ip.Ixy = 0.0; ip.Iyz = 0.0; ip.Ixz = 0.0;
        ip.xbar_m = 0.0; ip.ybar_m = 0.0; ip.zbar_m = 0.0;
        return ip;
    }

    StateD initial_state()
    {
        StateD s;
        s.g    = Lie::SE3<double>::Identity();
        s.nu_B << 1.0, 0.5, 0.25,
                  0.0, 0.0, 0.0;
        s.m = 1.0;
        return s;
    }

    // Exact rotation at time T: R(T) = Exp_SO3(omega0 * T) via Rodrigues.
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

    // Integrate over [0, T] with fixed step h; return the final rotation matrix.
    template<class Stepper>
    Eigen::Matrix3d integrate_rotation(Stepper& stepper, double T, double h)
    {
        const int N = static_cast<int>(std::round(T / h));
        StateD s = initial_state();
        for (int i = 0; i < N; ++i) {
            auto res = stepper.step(static_cast<double>(i) * h, s, h);
            REQUIRE(res.converged);
            s = Stepper::unpack(res);
        }
        return s.g.R;
    }

    double empirical_order(double h1, double e1, double h2, double e2)
    {
        return std::log(e1 / e2) / std::log(h1 / h2);
    }

} // namespace

// ---------------------------------------------------------------------------
// Radau IIA RKMK -- expected order 5
// ---------------------------------------------------------------------------
TEST_CASE("Convergence order: Radau IIA RKMK on SE(3) achieves >= order 4",
    "[convergence][radau]")
{
    constexpr double T = 1.0;
    const Eigen::Vector3d    omega0  { 1.0, 0.5, 0.25 };
    const Eigen::Matrix3d    R_exact = exact_rotation(omega0, T);
    const std::array<double,6> h_vec { 0.5, 0.25, 0.1, 0.05, 0.025, 0.01 };

    Core::NewtonOptions opt;
    opt.abs_tol  = 1e-12;
    opt.rel_tol  = 1e-12;
    opt.max_iters = 30;

    RadauStepper stepper{ symmetric_sphere(), opt };

    std::vector<double> errors;
    for (double h : h_vec) {
        const Eigen::Matrix3d R_num = integrate_rotation(stepper, T, h);
        errors.push_back((R_num - R_exact).norm());
    }

    double sum_order = 0.0;
    int    n_pairs   = 0;
    for (std::size_t i = 0; i + 1 < h_vec.size(); ++i) {
        if (errors[i] > 1e-14 && errors[i + 1] > 1e-14) {
            sum_order += empirical_order(h_vec[i], errors[i], h_vec[i + 1], errors[i + 1]);
            ++n_pairs;
        }
    }
    const double mean_order = n_pairs > 0 ? sum_order / n_pairs : 0.0;

    INFO("Radau IIA RKMK mean empirical order = " << mean_order);
    for (std::size_t i = 0; i < h_vec.size(); ++i)
        INFO("  h=" << h_vec[i] << "  err=" << errors[i]);

    REQUIRE(mean_order >= 4.0);
}

// ---------------------------------------------------------------------------
// Explicit RK4 RKMK -- expected order 4
// ---------------------------------------------------------------------------
TEST_CASE("Convergence order: explicit RK4 RKMK on SE(3) achieves >= order 3",
    "[convergence][rk4]")
{
    constexpr double T = 1.0;
    const Eigen::Vector3d    omega0  { 1.0, 0.5, 0.25 };
    const Eigen::Matrix3d    R_exact = exact_rotation(omega0, T);
    const std::array<double,6> h_vec { 0.5, 0.25, 0.1, 0.05, 0.025, 0.01 };

    RK4Stepper stepper{ symmetric_sphere() };

    std::vector<double> errors;
    for (double h : h_vec) {
        const Eigen::Matrix3d R_num = integrate_rotation(stepper, T, h);
        errors.push_back((R_num - R_exact).norm());
    }

    double sum_order = 0.0;
    int    n_pairs   = 0;
    for (std::size_t i = 0; i + 1 < h_vec.size(); ++i) {
        if (errors[i] > 1e-14 && errors[i + 1] > 1e-14) {
            sum_order += empirical_order(h_vec[i], errors[i], h_vec[i + 1], errors[i + 1]);
            ++n_pairs;
        }
    }
    const double mean_order = n_pairs > 0 ? sum_order / n_pairs : 0.0;

    INFO("Explicit RK4 RKMK mean empirical order = " << mean_order);
    for (std::size_t i = 0; i < h_vec.size(); ++i)
        INFO("  h=" << h_vec[i] << "  err=" << errors[i]);

    REQUIRE(mean_order >= 3.0);
}

// ---------------------------------------------------------------------------
// Write CSV for paper figure (runs after both integrators are validated)
// ---------------------------------------------------------------------------
TEST_CASE("Convergence order: write CSV for paper figure",
    "[convergence][csv]")
{
    constexpr double T = 1.0;
    const Eigen::Vector3d    omega0  { 1.0, 0.5, 0.25 };
    const Eigen::Matrix3d    R_exact = exact_rotation(omega0, T);
    const std::array<double,6> h_vec { 0.5, 0.25, 0.1, 0.05, 0.025, 0.01 };

    // Radau IIA errors
    std::vector<double> err_radau;
    {
        Core::NewtonOptions opt;
        opt.abs_tol = 1e-12; opt.rel_tol = 1e-12; opt.max_iters = 30;
        RadauStepper stepper{ symmetric_sphere(), opt };
        for (double h : h_vec) {
            const Eigen::Matrix3d R_num = integrate_rotation(stepper, T, h);
            err_radau.push_back((R_num - R_exact).norm());
        }
    }

    // Explicit RK4 errors
    std::vector<double> err_rk4;
    {
        RK4Stepper stepper{ symmetric_sphere() };
        for (double h : h_vec) {
            const Eigen::Matrix3d R_num = integrate_rotation(stepper, T, h);
            err_rk4.push_back((R_num - R_exact).norm());
        }
    }

    // Always print so data is visible in test output
    WARN("=== Convergence data (h, error_radau5, error_rk4) ===");
    for (std::size_t i = 0; i < h_vec.size(); ++i)
        WARN("  " << h_vec[i] << "  " << err_radau[i] << "  " << err_rk4[i]);

#ifdef AETHERION_SOURCE_DIR
    const std::string csv_path =
        std::string(AETHERION_SOURCE_DIR) + "/papers/eucass/data/convergence_order.csv";

    std::ofstream ofs(csv_path);
    if (ofs.is_open()) {
        ofs << "h,error_radau5_rkmk,error_rk4_rkmk\n";
        ofs << std::scientific;
        ofs.precision(6);
        for (std::size_t i = 0; i < h_vec.size(); ++i)
            ofs << h_vec[i] << "," << err_radau[i] << "," << err_rk4[i] << "\n";
        WARN("CSV written to: " << csv_path);
    } else {
        WARN("Could not write CSV to: " << csv_path
             << " -- ensure papers/eucass/data/ exists.");
    }
#else
    WARN("AETHERION_SOURCE_DIR not defined -- no CSV written.");
#endif
}

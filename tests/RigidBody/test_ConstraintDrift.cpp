// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// test_ConstraintDrift.cpp
//
// Orthonormality of the attitude representation over a long trajectory.
//
// The SE(3) RKMK integrator updates the pose by g <- g * Exp(Delta), so the
// rotation factor is the product of exact SO(3) elements: the constraint
// R^T R = I is satisfied by construction and can only degrade through
// floating-point rounding, which does not accumulate secularly.
//
// A conventional integrator that treats the quaternion as four unconstrained
// real numbers has no such guarantee.  Its state leaves the unit sphere at a
// rate set by the local truncation error, and the induced "rotation" matrix
// R(q) = (q q^T scaling) is no longer orthonormal.  Renormalising q every step
// masks the symptom but is an ad hoc projection.
//
// Three integrators are compared on the same torque-free tumbling body:
//   (a) SE(3) Radau IIA RKMK              -- this library
//   (b) quaternion RK4, no renormalisation
//   (c) quaternion RK4, renormalised every step
//
// Metric:  d(t) = || R(t)^T R(t) - I ||_F
//
// CSV output (if AETHERION_SOURCE_DIR is defined via CMake):
//   ${AETHERION_SOURCE_DIR}/papers/eucass/data/constraint_drift.csv
//   Columns: t, rkmk_se3, quat_rk4_plain, quat_rk4_renorm
//
#include <catch2/catch_test_macros.hpp>

#include <Aetherion/RigidBody/SixDofStepper.h>
#include <Aetherion/RigidBody/VectorField.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/State.h>
#include <Aetherion/FlightDynamics/Policies/GravityPolicies.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>
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

using ZeroVF       = RigidBody::VectorField<FlightDynamics::ZeroGravityPolicy>;
using RadauStepper = SixDoFStepper<ZeroVF>;

namespace {

    // ---------------------------------------------------------------------
    // Shared problem definition
    // ---------------------------------------------------------------------
    constexpr double kT      = 600.0;   // trajectory length [s]
    constexpr double kH      = 0.1;     // step size [s]
    constexpr int    kSample = 50;      // record every kSample steps

    constexpr double kIxx = 0.10, kIyy = 0.25, kIzz = 0.32;

    const Eigen::Vector3d kOmega0{ 1.0, 2.0, 0.5 };
    const Eigen::Vector3d kV0    { 50.0, 5.0, -3.0 };

    InertialParameters asymmetric_body()
    {
        InertialParameters ip;
        ip.mass_kg = 1.0;
        ip.Ixx = kIxx; ip.Iyy = kIyy; ip.Izz = kIzz;
        ip.Ixy = 0.0;  ip.Iyz = 0.0;  ip.Ixz = 0.0;
        ip.xbar_m = 0.0; ip.ybar_m = 0.0; ip.zbar_m = 0.0;
        return ip;
    }

    StateD initial_state()
    {
        StateD s;
        s.g = Lie::SE3<double>::Identity();
        s.nu_B << kOmega0, kV0;
        s.m = 1.0;
        return s;
    }

    double orthonormality_defect(const Eigen::Matrix3d& R)
    {
        return (R.transpose() * R - Eigen::Matrix3d::Identity()).norm();
    }

    // ---------------------------------------------------------------------
    // Baseline: quaternion RK4 on a flat 13-vector
    //
    //   y = [ qw qx qy qz | px py pz | wx wy wz | vx vy vz ]
    //
    // The quaternion maps body -> inertial.  Nothing in the integrator knows
    // that y[0..3] is constrained to the unit sphere.
    // ---------------------------------------------------------------------
    using Vec13 = Eigen::Matrix<double, 13, 1>;

    Vec13 quaternion_rk4_rhs(const Vec13& y)
    {
        const Eigen::Vector4d q = y.segment<4>(0);
        const Eigen::Vector3d w = y.segment<3>(7);
        const Eigen::Vector3d v = y.segment<3>(10);

        // q_dot = 0.5 * q (x) [0, w]   (Hamilton product, scalar-first)
        Eigen::Vector4d qdot;
        qdot(0) = 0.5 * (-q(1) * w(0) - q(2) * w(1) - q(3) * w(2));
        qdot(1) = 0.5 * ( q(0) * w(0) + q(2) * w(2) - q(3) * w(1));
        qdot(2) = 0.5 * ( q(0) * w(1) - q(1) * w(2) + q(3) * w(0));
        qdot(3) = 0.5 * ( q(0) * w(2) + q(1) * w(1) - q(2) * w(0));

        // p_dot = R(q) v_B   -- built from the raw (possibly non-unit) q
        const Eigen::Quaterniond qq(q(0), q(1), q(2), q(3));
        const Eigen::Matrix3d    R = qq.toRotationMatrix();

        // Euler's equations, torque free:  w_dot = -I^-1 (w x I w)
        const Eigen::Vector3d Iw{ kIxx * w(0), kIyy * w(1), kIzz * w(2) };
        const Eigen::Vector3d cross = w.cross(Iw);
        const Eigen::Vector3d wdot{ -cross(0) / kIxx, -cross(1) / kIyy, -cross(2) / kIzz };

        // Zero external force in the body frame:  v_dot_B = -w x v_B
        const Eigen::Vector3d vdot = -w.cross(v);

        Vec13 dy;
        dy.segment<4>(0)  = qdot;
        dy.segment<3>(4)  = R * v;
        dy.segment<3>(7)  = wdot;
        dy.segment<3>(10) = vdot;
        return dy;
    }

    Vec13 quaternion_rk4_step(const Vec13& y, double h)
    {
        const Vec13 k1 = quaternion_rk4_rhs(y);
        const Vec13 k2 = quaternion_rk4_rhs(y + (0.5 * h) * k1);
        const Vec13 k3 = quaternion_rk4_rhs(y + (0.5 * h) * k2);
        const Vec13 k4 = quaternion_rk4_rhs(y + h * k3);
        return y + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }

    Vec13 quaternion_initial_state()
    {
        Vec13 y = Vec13::Zero();
        y(0) = 1.0;                     // identity quaternion
        y.segment<3>(7)  = kOmega0;
        y.segment<3>(10) = kV0;
        return y;
    }

    Eigen::Matrix3d rotation_from(const Vec13& y)
    {
        const Eigen::Quaterniond q(y(0), y(1), y(2), y(3));
        return q.toRotationMatrix();
    }

    struct DriftTrace {
        std::vector<double> t;
        std::vector<double> rkmk;
        std::vector<double> quat_plain;
        std::vector<double> quat_renorm;
    };

    // Runs all three integrators once and returns the sampled defect traces.
    const DriftTrace& drift_trace()
    {
        static const DriftTrace trace = [] {
            DriftTrace out;

            Core::NewtonOptions opt;
            opt.abs_tol   = 1e-12;
            opt.rel_tol   = 1e-12;
            opt.max_iters = 30;

            RadauStepper stepper{ asymmetric_body(), opt };

            StateD s      = initial_state();
            Vec13  yPlain = quaternion_initial_state();
            Vec13  yRenrm = quaternion_initial_state();

            const int N = static_cast<int>(std::llround(kT / kH));
            for (int k = 0; k < N; ++k) {
                if (k % kSample == 0) {
                    out.t.push_back(static_cast<double>(k) * kH);
                    out.rkmk.push_back(orthonormality_defect(s.g.R));
                    out.quat_plain.push_back(orthonormality_defect(rotation_from(yPlain)));
                    out.quat_renorm.push_back(orthonormality_defect(rotation_from(yRenrm)));
                }

                const auto res = stepper.step(static_cast<double>(k) * kH, s, kH);
                REQUIRE(res.converged);
                s = RadauStepper::unpack(res);

                yPlain = quaternion_rk4_step(yPlain, kH);

                yRenrm = quaternion_rk4_step(yRenrm, kH);
                yRenrm.segment<4>(0).normalize();     // ad hoc projection
            }

            out.t.push_back(kT);
            out.rkmk.push_back(orthonormality_defect(s.g.R));
            out.quat_plain.push_back(orthonormality_defect(rotation_from(yPlain)));
            out.quat_renorm.push_back(orthonormality_defect(rotation_from(yRenrm)));
            return out;
        }();
        return trace;
    }

} // namespace

// ---------------------------------------------------------------------------
// 1. The SE(3) RKMK pose stays on the manifold to rounding
// ---------------------------------------------------------------------------
TEST_CASE("SE(3) RKMK preserves SO(3) orthonormality over a 600 s trajectory",
    "[drift][rkmk]")
{
    const DriftTrace& d = drift_trace();

    double worst = 0.0;
    for (double v : d.rkmk) worst = std::max(worst, v);

    INFO("max ||R^T R - I||_F over " << kT << " s = " << worst);

    // The pose is a product of exact SO(3) factors, so the defect is pure
    // round-off accumulated over the step count -- bounded near machine
    // epsilon rather than growing with the truncation error.
    REQUIRE(worst < 1e-12);
}

// ---------------------------------------------------------------------------
// 2. The unconstrained quaternion baseline leaves the manifold
// ---------------------------------------------------------------------------
TEST_CASE("Unconstrained quaternion RK4 accumulates orthonormality error",
    "[drift][quaternion]")
{
    const DriftTrace& d = drift_trace();

    const double rkmk_final  = d.rkmk.back();
    const double plain_final = d.quat_plain.back();

    INFO("final defect: RKMK = " << rkmk_final << ", quaternion RK4 = " << plain_final);

    // The unnormalised baseline must be orders of magnitude worse, and the
    // growth must be secular rather than bounded round-off.
    REQUIRE(plain_final > 1.0e3 * rkmk_final);
    REQUIRE(plain_final > d.quat_plain[d.quat_plain.size() / 2]);
}

// ---------------------------------------------------------------------------
// 3. Renormalisation also satisfies the constraint -- by projection
// ---------------------------------------------------------------------------
// This is asserted explicitly so that the comparison is not overstated.
// Re-normalising the quaternion every step *does* keep ||R^T R - I||_F at the
// round-off level, so the orthonormality metric alone does not distinguish the
// two approaches.  The difference is in *how* the constraint is met: RKMK never
// leaves the group, whereas renormalisation lets the state leave the unit sphere
// and then projects it back -- an operation applied outside the integrator,
// which perturbs the solution and is not accounted for in the propagation
// Jacobian used by a filter.
// ---------------------------------------------------------------------------
TEST_CASE("Renormalised quaternion RK4 meets the constraint by projection",
    "[drift][quaternion][renorm]")
{
    const DriftTrace& d = drift_trace();

    double worst = 0.0;
    for (double v : d.quat_renorm) worst = std::max(worst, v);

    INFO("max ||R^T R - I||_F with per-step renormalisation = " << worst);
    REQUIRE(worst < 1e-12);
    REQUIRE(worst < 1e-6 * d.quat_plain.back());
}

// ---------------------------------------------------------------------------
// 3. CSV export for the paper figure
// ---------------------------------------------------------------------------
TEST_CASE("Constraint drift: write CSV for paper figure", "[drift][csv]")
{
    const DriftTrace& d = drift_trace();

    WARN("final defects -- RKMK: "        << d.rkmk.back()
         << "  quat RK4 (plain): "        << d.quat_plain.back()
         << "  quat RK4 (renormalised): " << d.quat_renorm.back());

#ifdef AETHERION_SOURCE_DIR
    const std::string csv_path =
        std::string(AETHERION_SOURCE_DIR) + "/papers/eucass/data/constraint_drift.csv";

    std::ofstream ofs(csv_path);
    if (ofs.is_open()) {
        ofs << "t,rkmk_se3,quat_rk4_plain,quat_rk4_renorm\n";
        ofs << std::scientific;
        ofs.precision(8);
        for (std::size_t i = 0; i < d.t.size(); ++i)
            ofs << d.t[i] << "," << d.rkmk[i] << ","
                << d.quat_plain[i] << "," << d.quat_renorm[i] << "\n";
        WARN("CSV written to: " << csv_path);
    } else {
        WARN("Could not write CSV to: " << csv_path
             << " -- ensure papers/eucass/data/ exists.");
    }
#else
    WARN("AETHERION_SOURCE_DIR not defined -- no CSV written.");
#endif
}

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// test_ManifoldFactorial.cpp
//
// Isolates what Munthe-Kaas buys, by holding the Butcher tableau fixed.
//
// The pre-existing drift study compared SE(3) Radau IIA RKMK against an
// explicit quaternion RK4.  That varies three factors at once -- implicit vs
// explicit, order 5 vs 4, and MK vs no-MK -- so the measured gap cannot be
// attributed to Munthe-Kaas.  This file completes the 2x2:
//
//                    | flat 13-vector state |  SE(3) RKMK
//   -----------------+----------------------+---------------
//   explicit RK4     |        (b)           |     (d)
//   implicit Radau5  |        (c)  <-- new  |     (a)
//
// (a) vs (c) differ *only* in the manifold treatment: same tableau, same
// order, same stage-residual tolerance.  That pair is the controlled
// comparison.  (a) vs (d) isolates the tableau at fixed MK; (b) vs (c)
// isolates the tableau at fixed flat state.
//
// A fifth cell, flat Gauss (3-stage, order 6), is included because the
// quaternion norm is a *quadratic* invariant: by the Cooper condition a
// symplectic tableau conserves it exactly with no MK at all.  Reporting it
// keeps the conclusion honest about what the orthonormality metric can and
// cannot discriminate.
//
#include <catch2/catch_test_macros.hpp>

#include "FlatStateIntegrators.h"

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
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using namespace Aetherion;
using namespace Aetherion::RigidBody;
using namespace Aetherion::ODE::RKMK;

namespace Flat = Aetherion::Testing::Flat;

using ZeroVF       = RigidBody::VectorField<FlightDynamics::ZeroGravityPolicy>;
using RadauStepper = SixDoFStepper<ZeroVF>;
using RK4Stepper   = SixDoFStepper<
    ZeroVF,
    7,
    Integrators::ExplicitRK4_RKMK_ProductSE3<KinematicsXiField, ZeroVF, 7>>;

namespace {

    // ---------------------------------------------------------------------
    // Shared problem: torque-free asymmetric body, non-zero body velocity so
    // that the SE(3) screw coupling is exercised, not just the SO(3) factor.
    // Identical to the body used by the drift and convergence studies.
    // ---------------------------------------------------------------------
    constexpr double kIxx = 0.10, kIyy = 0.25, kIzz = 0.32;

    const Eigen::Vector3d kOmega0{ 1.0, 2.0, 0.5 };
    const Eigen::Vector3d kV0    { 50.0, 5.0, -3.0 };

    const Flat::Body kFlatBody{ kIxx, kIyy, kIzz };

    InertialParameters asymmetric_body()
    {
        InertialParameters ip;
        ip.mass_kg = 1.0;
        ip.Ixx = kIxx; ip.Iyy = kIyy; ip.Izz = kIzz;
        ip.Ixy = 0.0;  ip.Iyz = 0.0;  ip.Ixz = 0.0;
        ip.xbar_m = 0.0; ip.ybar_m = 0.0; ip.zbar_m = 0.0;
        return ip;
    }

    StateD se3_initial_state()
    {
        StateD s;
        s.g = Lie::SE3<double>::Identity();
        s.nu_B << kOmega0, kV0;
        s.m = 1.0;
        return s;
    }

    Core::NewtonOptions tight_newton()
    {
        Core::NewtonOptions opt;
        opt.abs_tol   = 1e-13;
        opt.rel_tol   = 1e-13;
        opt.max_iters = 50;
        return opt;
    }

    std::string sci(double v)
    {
        std::ostringstream os;
        os << std::scientific << std::setprecision(2) << v;
        return os.str();
    }

    // =====================================================================
    // Study 1 -- constraint drift over a long trajectory
    // =====================================================================
    constexpr double kDriftT = 600.0;
    constexpr double kDriftH = 0.1;

    constexpr int kDriftSample = 50;   // record every kDriftSample steps

    struct DriftResult {
        double rkmk_radau{ 0.0 };   // (a) MK  + implicit Radau IIA 5
        double rkmk_rk4  { 0.0 };   // (d) MK  + explicit RK4
        double flat_radau{ 0.0 };   // (c) no MK + implicit Radau IIA 5
        double flat_gauss{ 0.0 };   // no MK + implicit Gauss 6 (symplectic)
        double flat_rk4  { 0.0 };   // (b) no MK + explicit RK4
        double flat_renrm{ 0.0 };   // (b) + per-step projection
        int    worst_iters{ 0 };

        // Sampled time histories, in the same cell order, for the figure.
        std::vector<double> t;
        std::vector<double> tr_rkmk_radau, tr_rkmk_rk4, tr_flat_radau;
        std::vector<double> tr_flat_gauss, tr_flat_rk4,  tr_flat_renrm;
    };

    const DriftResult& drift_result()
    {
        static const DriftResult r = [] {
            DriftResult out;

            RadauStepper radau{ asymmetric_body(), tight_newton() };
            RK4Stepper   rk4mk{ asymmetric_body() };

            StateD sA = se3_initial_state();
            StateD sD = se3_initial_state();

            Flat::Vec13 yRadau = Flat::initial_state(kOmega0, kV0);
            Flat::Vec13 yGauss = Flat::initial_state(kOmega0, kV0);
            Flat::Vec13 yRK4   = Flat::initial_state(kOmega0, kV0);
            Flat::Vec13 yRenrm = Flat::initial_state(kOmega0, kV0);

            const auto tabRadau = Flat::radau_iia_5();
            const auto tabGauss = Flat::gauss_6();

            const int N = static_cast<int>(std::llround(kDriftT / kDriftH));
            for (int k = 0; k < N; ++k) {
                const double t = static_cast<double>(k) * kDriftH;

                if (k % kDriftSample == 0) {
                    out.t.push_back(t);
                    out.tr_rkmk_radau.push_back(Flat::orthonormality_defect(sA.g.R));
                    out.tr_rkmk_rk4  .push_back(Flat::orthonormality_defect(sD.g.R));
                    out.tr_flat_radau.push_back(Flat::orthonormality_defect(Flat::rotation_from(yRadau)));
                    out.tr_flat_gauss.push_back(Flat::orthonormality_defect(Flat::rotation_from(yGauss)));
                    out.tr_flat_rk4  .push_back(Flat::orthonormality_defect(Flat::rotation_from(yRK4)));
                    out.tr_flat_renrm.push_back(Flat::orthonormality_defect(Flat::rotation_from(yRenrm)));
                }

                auto ra = radau.step(t, sA, kDriftH);
                REQUIRE(ra.converged);
                sA = RadauStepper::unpack(ra);

                auto rd = rk4mk.step(t, sD, kDriftH);
                sD = RK4Stepper::unpack(rd);

                const auto fr = Flat::implicit_rk_step(tabRadau, kFlatBody, yRadau, kDriftH);
                REQUIRE(fr.converged);
                yRadau = fr.y;
                out.worst_iters = std::max(out.worst_iters, fr.iters);

                const auto fg = Flat::implicit_rk_step(tabGauss, kFlatBody, yGauss, kDriftH);
                REQUIRE(fg.converged);
                yGauss = fg.y;

                yRK4   = Flat::explicit_rk4_step(kFlatBody, yRK4, kDriftH);

                yRenrm = Flat::explicit_rk4_step(kFlatBody, yRenrm, kDriftH);
                yRenrm.segment<4>(0).normalize();
            }

            out.rkmk_radau = Flat::orthonormality_defect(sA.g.R);
            out.rkmk_rk4   = Flat::orthonormality_defect(sD.g.R);
            out.flat_radau = Flat::orthonormality_defect(Flat::rotation_from(yRadau));
            out.flat_gauss = Flat::orthonormality_defect(Flat::rotation_from(yGauss));
            out.flat_rk4   = Flat::orthonormality_defect(Flat::rotation_from(yRK4));
            out.flat_renrm = Flat::orthonormality_defect(Flat::rotation_from(yRenrm));

            out.t.push_back(kDriftT);
            out.tr_rkmk_radau.push_back(out.rkmk_radau);
            out.tr_rkmk_rk4  .push_back(out.rkmk_rk4);
            out.tr_flat_radau.push_back(out.flat_radau);
            out.tr_flat_gauss.push_back(out.flat_gauss);
            out.tr_flat_rk4  .push_back(out.flat_rk4);
            out.tr_flat_renrm.push_back(out.flat_renrm);
            return out;
        }();
        return r;
    }

    // =====================================================================
    // Study 2 -- convergence order, MK vs no-MK at a fixed tableau
    // =====================================================================
    constexpr double            kOrderT    = 2.0;
    constexpr double            kOrderHRef = 2.5e-4;
    const std::array<double, 5> kHVec{ 0.2, 0.1, 0.05, 0.025, 0.0125 };

    double empirical_order(double h1, double e1, double h2, double e2)
    {
        return std::log(e1 / e2) / std::log(h1 / h2);
    }

    double mean_order(const std::vector<double>& h,
                      const std::vector<double>& e,
                      double                     floor_val = 1e-13)
    {
        double sum = 0.0;
        int    n   = 0;
        for (std::size_t i = 0; i + 1 < h.size(); ++i)
            if (e[i] > floor_val && e[i + 1] > floor_val) {
                sum += empirical_order(h[i], e[i], h[i + 1], e[i + 1]);
                ++n;
            }
        return (n > 0) ? sum / static_cast<double>(n) : 0.0;
    }

    struct OrderResult {
        std::vector<double> rot_flat_radau;
        std::vector<double> pos_flat_radau;
        std::vector<double> rot_flat_gauss;
        std::vector<double> pos_flat_gauss;
    };

    // Reference: RKMK Radau IIA at kOrderHRef, i.e. the same reference the
    // published convergence study uses.
    const StateD& reference_state()
    {
        static const StateD ref = [] {
            RadauStepper stepper{ asymmetric_body(), tight_newton() };
            StateD       s = se3_initial_state();
            const int    N = static_cast<int>(std::llround(kOrderT / kOrderHRef));
            for (int i = 0; i < N; ++i) {
                auto res = stepper.step(static_cast<double>(i) * kOrderHRef, s, kOrderHRef);
                REQUIRE(res.converged);
                s = RadauStepper::unpack(res);
            }
            return s;
        }();
        return ref;
    }

    template<int S>
    void flat_error_series(const Core::ButcherTableau<double, S>& tab,
                           std::vector<double>&                   rot,
                           std::vector<double>&                   pos)
    {
        const StateD& ref = reference_state();

        for (double h : kHVec) {
            Flat::Vec13 y = Flat::initial_state(kOmega0, kV0);
            const int   N = static_cast<int>(std::llround(kOrderT / h));
            for (int i = 0; i < N; ++i) {
                const auto r = Flat::implicit_rk_step(tab, kFlatBody, y, h);
                REQUIRE(r.converged);
                y = r.y;
            }
            rot.push_back((Flat::rotation_from(y) - ref.g.R).norm());
            pos.push_back((y.segment<3>(4) - ref.g.p).norm());
        }
    }

    const OrderResult& order_result()
    {
        static const OrderResult r = [] {
            OrderResult out;
            flat_error_series(Flat::radau_iia_5(), out.rot_flat_radau, out.pos_flat_radau);
            flat_error_series(Flat::gauss_6(),     out.rot_flat_gauss, out.pos_flat_gauss);
            return out;
        }();
        return r;
    }

    // =====================================================================
    // Study 3 -- stiff regime: does any single non-MK method give both
    //            manifold membership and stiff accuracy?
    //
    // The flat formulation can reach round-off orthonormality, but only via a
    // symplectic tableau (Study 1, Gauss cell).  Symplectic Runge-Kutta
    // methods are A-stable but not L-stable: the 3-stage Gauss stability
    // function is the (3,3) Pade approximant, with R(infinity) = -1.  A stiff
    // transient is therefore not damped at all -- it alternates in sign and
    // persists indefinitely.  Radau IIA is L-stable, R(infinity) = 0, and
    // stiffly accurate; it annihilates the transient.  So the two properties
    // the flat formulation can offer are mutually exclusive in the choice of
    // tableau, while MK supplies manifold membership for either one.
    //
    // Problem: the same tumbling body with strong linear body-frame damping,
    //   F_B = -c v_B,   c/m = 1e3 1/s,   h = 0.1 s  =>  |h lambda| = 1e2.
    //
    // The metric is analytic, so no reference integration is needed.  The
    // Coriolis term -w x v_B is norm-preserving, hence
    //   d/dt ||v_B||^2 = -2 (c/m) ||v_B||^2   =>   ||v_B(t)|| = ||v_0|| e^{-(c/m) t}
    // exactly.  At T = 600 s the exact value underflows to zero.
    // =====================================================================
    // Representative rate for the long-horizon manifold column, and a sweep in
    // stiffness for the transient-damping column.  The sweep is the part that
    // shows the mechanism: |R(h lambda)| -> 0 for an L-stable tableau and -> 1
    // for a symplectic one, so a single stiffness point can only ever sample a
    // trend.  At h lambda = -100 the 3-stage Gauss amplification factor is
    // still only -0.787, and 6000 steps of that underflows -- which would
    // wrongly suggest Gauss handles the transient.
    constexpr double kStiffRate = 1.0e3;    // c/m [1/s]; time constant 1 ms
    constexpr double kStiffT    = 600.0;
    constexpr double kStiffH    = 0.1;

    constexpr int kSweepSteps = 20;
    const std::array<double, 6> kSweepRates{ 1e1, 1e2, 1e3, 1e4, 1e5, 1e6 };

    // Linear body-frame damping wrench, matching Flat::rhs.
    struct LinearDampingAero final {
        double c_lin{ 0.0 };

        template<class S>
        Spatial::Wrench<S> operator()(const Lie::SE3<S>&,
                                      const Eigen::Matrix<S, 6, 1>& nu_B,
                                      S, S) const
        {
            Spatial::Wrench<S> w{};
            w.f.setZero();
            w.f.template tail<3>() = -S(c_lin) * nu_B.template tail<3>();
            return w;
        }
    };

    using StiffVF = RigidBody::VectorField<FlightDynamics::ZeroGravityPolicy,
                                           LinearDampingAero>;
    using StiffRadauStepper = SixDoFStepper<StiffVF>;
    using StiffRK4Stepper   = SixDoFStepper<
        StiffVF,
        7,
        Integrators::ExplicitRK4_RKMK_ProductSE3<KinematicsXiField, StiffVF, 7>>;

    struct StiffCell {
        double speed{ 0.0 };        // ||v_B(T)||, exact value is 0
        double defect{ 0.0 };       // ||R^T R - I||_F at T
        int    diverged_at{ -1 };   // step index of first non-finite state
    };

    // One row of the stiffness sweep: retained fraction ||v_B(T)|| / ||v_0||
    // after kSweepSteps steps, which for an exact solve is exp(-(c/m) T).
    struct SweepRow {
        double rate{ 0.0 };
        double exact{ 0.0 };
        double mk_radau{ 0.0 };
        double flat_radau{ 0.0 };
        double flat_gauss{ 0.0 };
    };

    struct StiffResult {
        StiffCell             mk_radau, mk_rk4, flat_radau, flat_gauss;
        std::vector<SweepRow> sweep;
    };

    // Amplification factor of the 3-stage Gauss method, i.e. the (3,3) Pade
    // approximant to exp(z).  Quoted alongside the measurement so the reader
    // can see the transient behaviour is the tableau's, not the solver's.
    double gauss3_stability(double z)
    {
        const double n = 1.0 + z / 2.0 + z * z / 10.0 + z * z * z / 120.0;
        const double d = 1.0 - z / 2.0 + z * z / 10.0 - z * z * z / 120.0;
        return n / d;
    }

    const StiffResult& stiff_result()
    {
        static const StiffResult r = [] {
            StiffResult out;

            const double c = kStiffRate * 1.0;    // mass = 1 kg

            StiffVF vf{ asymmetric_body(), FlightDynamics::ZeroGravityPolicy{},
                        LinearDampingAero{ c } };

            StiffRadauStepper mkRadau{ vf, tight_newton() };
            StiffRK4Stepper   mkRK4  { vf };

            const Flat::Body fb{ kIxx, kIyy, kIzz, 1.0, c };
            const auto       tabRadau = Flat::radau_iia_5();
            const auto       tabGauss = Flat::gauss_6();

            StateD sR = se3_initial_state();
            StateD sE = se3_initial_state();

            Flat::Vec13 yRadau = Flat::initial_state(kOmega0, kV0);
            Flat::Vec13 yGauss = Flat::initial_state(kOmega0, kV0);

            const int N = static_cast<int>(std::llround(kStiffT / kStiffH));
            for (int k = 0; k < N; ++k) {
                const double t = static_cast<double>(k) * kStiffH;

                auto ra = mkRadau.step(t, sR, kStiffH);
                REQUIRE(ra.converged);
                sR = StiffRadauStepper::unpack(ra);

                // The explicit tableau is expected to blow up here; keep the
                // first offending step rather than propagating NaNs.
                if (out.mk_rk4.diverged_at < 0) {
                    auto re = mkRK4.step(t, sE, kStiffH);
                    sE = StiffRK4Stepper::unpack(re);
                    if (!sE.g.R.allFinite() || !sE.nu_B.allFinite())
                        out.mk_rk4.diverged_at = k;
                }

                const auto fr = Flat::implicit_rk_step(tabRadau, fb, yRadau, kStiffH);
                REQUIRE(fr.converged);
                yRadau = fr.y;

                const auto fg = Flat::implicit_rk_step(tabGauss, fb, yGauss, kStiffH);
                REQUIRE(fg.converged);
                yGauss = fg.y;
            }

            out.mk_radau.speed    = sR.nu_B.tail<3>().norm();
            out.mk_radau.defect   = Flat::orthonormality_defect(sR.g.R);

            out.flat_radau.speed  = yRadau.segment<3>(10).norm();
            out.flat_radau.defect = Flat::orthonormality_defect(Flat::rotation_from(yRadau));

            out.flat_gauss.speed  = yGauss.segment<3>(10).norm();
            out.flat_gauss.defect = Flat::orthonormality_defect(Flat::rotation_from(yGauss));

            // -------------------------------------------------------------
            // Stiffness sweep at fixed h: transient damping vs |h lambda|
            // -------------------------------------------------------------
            const double v0 = kV0.norm();
            const double Tw = kSweepSteps * kStiffH;

            for (double rate : kSweepRates) {
                SweepRow row;
                row.rate  = rate;
                row.exact = std::exp(-rate * Tw);

                StiffVF vfk{ asymmetric_body(), FlightDynamics::ZeroGravityPolicy{},
                             LinearDampingAero{ rate } };
                StiffRadauStepper mk{ vfk, tight_newton() };

                const Flat::Body fbk{ kIxx, kIyy, kIzz, 1.0, rate };

                StateD      sk = se3_initial_state();
                Flat::Vec13 yr = Flat::initial_state(kOmega0, kV0);
                Flat::Vec13 yg = Flat::initial_state(kOmega0, kV0);

                for (int k = 0; k < kSweepSteps; ++k) {
                    const double t = static_cast<double>(k) * kStiffH;

                    auto rk = mk.step(t, sk, kStiffH);
                    REQUIRE(rk.converged);
                    sk = StiffRadauStepper::unpack(rk);

                    const auto a = Flat::implicit_rk_step(tabRadau, fbk, yr, kStiffH);
                    REQUIRE(a.converged);
                    yr = a.y;

                    const auto b = Flat::implicit_rk_step(tabGauss, fbk, yg, kStiffH);
                    REQUIRE(b.converged);
                    yg = b.y;
                }

                row.mk_radau   = sk.nu_B.tail<3>().norm()     / v0;
                row.flat_radau = yr.segment<3>(10).norm()     / v0;
                row.flat_gauss = yg.segment<3>(10).norm()     / v0;
                out.sweep.push_back(row);
            }

            return out;
        }();
        return r;
    }

} // namespace

// ---------------------------------------------------------------------------
// 0. The tableaux behave as the theory says they do
// ---------------------------------------------------------------------------
TEST_CASE("Cooper condition separates Radau IIA from Gauss", "[factorial][tableau]")
{
    const double dRadau = Flat::quadratic_invariant_defect(Flat::radau_iia_5());
    const double dGauss = Flat::quadratic_invariant_defect(Flat::gauss_6());

    WARN("Cooper defect max|b_i a_ij + b_j a_ji - b_i b_j|:"
         << "  Radau IIA(5) = " << sci(dRadau)
         << "   Gauss(6) = "    << sci(dGauss));

    // Gauss is symplectic: conserves every quadratic invariant exactly.
    REQUIRE(dGauss < 1e-15);
    // Radau IIA is not.
    REQUIRE(dRadau > 1e-3);
}

// ---------------------------------------------------------------------------
// 1. Constraint drift, full 2x2 plus the symplectic cell
// ---------------------------------------------------------------------------
TEST_CASE("Constraint drift: MK isolated at fixed tableau", "[factorial][drift]")
{
    const DriftResult& d = drift_result();

    WARN("||R^T R - I||_F after " << kDriftT << " s at h = " << kDriftH
         << " (" << static_cast<int>(kDriftT / kDriftH) << " steps)\n"
         << "  (a) SE(3) RKMK, implicit Radau IIA(5) : " << sci(d.rkmk_radau) << "\n"
         << "  (d) SE(3) RKMK, explicit RK4          : " << sci(d.rkmk_rk4)   << "\n"
         << "  (c) flat,       implicit Radau IIA(5) : " << sci(d.flat_radau) << "\n"
         << "      flat,       implicit Gauss(6)     : " << sci(d.flat_gauss) << "\n"
         << "  (b) flat,       explicit RK4          : " << sci(d.flat_rk4)   << "\n"
         << "      flat,       explicit RK4 + renorm : " << sci(d.flat_renrm) << "\n"
         << "  max Newton iterations (flat Radau)    : " << d.worst_iters);

    // The controlled comparison: same tableau, same order, same tolerance --
    // only the manifold treatment differs.
    REQUIRE(d.flat_radau > 1.0e3 * d.rkmk_radau);

    // MK holds the constraint for *either* tableau; the flat formulation does
    // not, so manifold preservation is not a consequence of implicitness.
    REQUIRE(d.rkmk_rk4 < 1e-12);
    REQUIRE(d.rkmk_radau < 1e-12);
}

// ---------------------------------------------------------------------------
// 2. Convergence order of the non-MK implicit control
// ---------------------------------------------------------------------------
TEST_CASE("Convergence order: flat implicit controls", "[factorial][convergence]")
{
    const OrderResult& r = order_result();

    const std::vector<double> h(kHVec.begin(), kHVec.end());

    std::ostringstream os;
    os << "flat-state implicit errors vs RKMK reference at T = " << kOrderT << " s\n";
    os << "     h        Radau5 e_R     Radau5 e_p     Gauss6 e_R     Gauss6 e_p\n";
    for (std::size_t i = 0; i < h.size(); ++i)
        os << "  " << h[i] << "\t" << sci(r.rot_flat_radau[i])
           << "\t"        << sci(r.pos_flat_radau[i])
           << "\t"        << sci(r.rot_flat_gauss[i])
           << "\t"        << sci(r.pos_flat_gauss[i]) << "\n";
    os << "  mean order  flat Radau5: attitude "
       << mean_order(h, r.rot_flat_radau) << ", position "
       << mean_order(h, r.pos_flat_radau) << "\n";
    os << "  mean order  flat Gauss6: attitude "
       << mean_order(h, r.rot_flat_gauss) << ", position "
       << mean_order(h, r.pos_flat_gauss);

    WARN(os.str());

    // Both flat implicit controls must attain their nominal order, otherwise
    // the baseline is broken rather than informative.
    REQUIRE(mean_order(h, r.rot_flat_radau) > 4.5);
    REQUIRE(mean_order(h, r.pos_flat_radau) > 4.5);
}

// ---------------------------------------------------------------------------
// 2b. Stiff regime: no single non-MK method gives both properties
// ---------------------------------------------------------------------------
TEST_CASE("Stiff regime: manifold membership and L-stability are exclusive without MK",
    "[factorial][stiff]")
{
    const StiffResult& s = stiff_result();

    std::ostringstream os;

    os << "Transient damping at fixed h = " << kStiffH << " s, after "
       << kSweepSteps << " steps (T = " << kSweepSteps * kStiffH << " s).\n"
       << "Retained fraction ||v_B(T)|| / ||v_0||; exact value is exp(-(c/m) T).\n"
       << "   c/m [1/s]   |h.lam|      exact     MK+Radau5   flat+Radau5  "
          "flat+Gauss6   |R_gauss3|\n";
    for (const auto& r : s.sweep)
        os << "   " << sci(r.rate) << "   " << sci(r.rate * kStiffH)
           << "   " << sci(r.exact)
           << "   " << sci(r.mk_radau)
           << "   " << sci(r.flat_radau)
           << "   " << sci(r.flat_gauss)
           << "   " << sci(std::abs(gauss3_stability(-r.rate * kStiffH))) << "\n";

    os << "\nLong-horizon manifold membership on the same stiff problem\n"
       << "(c/m = " << kStiffRate << " 1/s, h = " << kStiffH
       << " s, T = " << kStiffT << " s):\n"
       << "  MK   + Radau IIA(5)  : ||R^T R - I||_F = " << sci(s.mk_radau.defect)   << "\n"
       << "  MK   + explicit RK4  : diverged at step " << s.mk_rk4.diverged_at      << "\n"
       << "  flat + Radau IIA(5)  : ||R^T R - I||_F = " << sci(s.flat_radau.defect) << "\n"
       << "  flat + Gauss(6)      : ||R^T R - I||_F = " << sci(s.flat_gauss.defect);

    WARN(os.str());

    const SweepRow& stiffest = s.sweep.back();

    // As the problem stiffens at fixed h, the L-stable tableau damps the
    // transient ever harder -- with or without MK, since this is a property of
    // the tableau alone.
    REQUIRE(stiffest.mk_radau   < 1e-6);
    REQUIRE(stiffest.flat_radau < 1e-6);

    // The symplectic tableau does the opposite: |R| -> 1, so the transient is
    // retained almost in full however stiff the problem gets.
    REQUIRE(stiffest.flat_gauss > 0.5);

    // ... yet it is precisely the symplectic tableau that keeps the flat state
    // on the manifold, and the L-stable one that does not.  In the flat
    // formulation the two properties are mutually exclusive.
    REQUIRE(s.flat_gauss.defect < 1e-12);
    REQUIRE(s.flat_radau.defect > 1e-9);

    // MK supplies manifold membership to the L-stable tableau, so the RKMK
    // cell is the only one that has both.
    REQUIRE(s.mk_radau.defect < 1e-12);

    // An explicit tableau cannot substitute: MK preserves its structure, but
    // it has no stiff stability at all.
    REQUIRE(s.mk_rk4.diverged_at > 0);
}

// ---------------------------------------------------------------------------
// 3. CSV export
// ---------------------------------------------------------------------------
TEST_CASE("Manifold factorial: write CSV", "[factorial][csv]")
{
#ifdef AETHERION_SOURCE_DIR
    const DriftResult& d = drift_result();
    const OrderResult& r = order_result();

    const std::string dir = std::string(AETHERION_SOURCE_DIR) + "/papers/eucass/data/";

    std::ofstream f1(dir + "manifold_factorial_drift.csv");
    if (f1.is_open()) {
        f1 << "cell,manifold,tableau,order,defect\n" << std::scientific;
        f1.precision(8);
        f1 << "a,MK,RadauIIA,5," << d.rkmk_radau << "\n"
           << "d,MK,RK4,4,"      << d.rkmk_rk4   << "\n"
           << "c,flat,RadauIIA,5," << d.flat_radau << "\n"
           << "e,flat,Gauss,6,"    << d.flat_gauss << "\n"
           << "b,flat,RK4,4,"      << d.flat_rk4   << "\n"
           << "f,flat-renorm,RK4,4," << d.flat_renrm << "\n";
        WARN("wrote " << dir << "manifold_factorial_drift.csv");
    }

    std::ofstream f3(dir + "manifold_factorial_trace.csv");
    if (f3.is_open()) {
        f3 << "t,mk_radau5,mk_rk4,flat_radau5,flat_gauss6,flat_rk4,flat_rk4_renorm\n"
           << std::scientific;
        f3.precision(8);
        for (std::size_t i = 0; i < d.t.size(); ++i)
            f3 << d.t[i]              << "," << d.tr_rkmk_radau[i] << ","
               << d.tr_rkmk_rk4[i]    << "," << d.tr_flat_radau[i] << ","
               << d.tr_flat_gauss[i]  << "," << d.tr_flat_rk4[i]   << ","
               << d.tr_flat_renrm[i]  << "\n";
        WARN("wrote " << dir << "manifold_factorial_trace.csv");
    }

    std::ofstream f2(dir + "manifold_factorial_order.csv");
    if (f2.is_open()) {
        f2 << "h,rot_flat_radau5,pos_flat_radau5,rot_flat_gauss6,pos_flat_gauss6\n"
           << std::scientific;
        f2.precision(8);
        for (std::size_t i = 0; i < kHVec.size(); ++i)
            f2 << kHVec[i] << "," << r.rot_flat_radau[i] << "," << r.pos_flat_radau[i]
               << ","            << r.rot_flat_gauss[i] << "," << r.pos_flat_gauss[i] << "\n";
        WARN("wrote " << dir << "manifold_factorial_order.csv");
    }
#else
    WARN("AETHERION_SOURCE_DIR not defined -- no CSV written.");
#endif
}

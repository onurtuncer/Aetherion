// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// FlatStateIntegrators.h
//
// Non-Munthe-Kaas baselines for the rigid-body studies, sharing one flat state
// layout so that the *only* difference from the library SE(3) RKMK steppers is
// the treatment of the manifold.
//
// Why this header exists
// ----------------------
// Comparing SE(3) Radau IIA RKMK against an explicit quaternion RK4 varies
// three things at once -- implicit vs explicit, order 5 vs 4, and MK vs no-MK.
// Nothing measured that way can be attributed to Munthe-Kaas.  The controls
// here close that gap: the same Butcher tableaux the library uses, solved to
// the same stage-residual tolerance, applied to an unconstrained 13-vector.
// Holding the tableau fixed and toggling only MK isolates what MK buys.
//
// State layout (the SE(3) state, coordinatised):
//   y = [ qw qx qy qz | px py pz | wx wy wz | vx vy vz ]
// The quaternion maps body -> inertial.  Nothing in these integrators knows
// that y[0..3] is constrained to the unit sphere.
//
// A note on quadratic invariants
// ------------------------------
// ||q||^2 is a quadratic invariant of this vector field: d/dt (q^T q) = 0
// identically, because the quaternion kinematic generator is skew.  By the
// Cooper condition an RK method conserves every quadratic invariant of the ODE
// iff
//   M_ij = b_i a_ij + b_j a_ji - b_i b_j = 0.
// Gauss-Legendre satisfies this; Radau IIA does not.  Both tableaux are
// provided so that the distinction is measured rather than assumed -- see
// quadratic_invariant_defect() below.
//
#pragma once

#include <Aetherion/ODE/RKMK/Core/Tableau.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace Aetherion::Testing::Flat {

    namespace Core = Aetherion::ODE::RKMK::Core;

    using Vec13 = Eigen::Matrix<double, 13, 1>;
    using Mat13 = Eigen::Matrix<double, 13, 13>;

    // -----------------------------------------------------------------------
    // Rigid-body parameters for the torque-free studies
    // -----------------------------------------------------------------------
    // c_lin is a linear damping coefficient [N per m/s] applied as a body-frame
    // force -c_lin * v_B, i.e. the stiff aerodynamic damping the Radau IIA
    // tableau is chosen for.  c_lin = 0 recovers the torque-free case.
    struct Body final {
        double Ixx{ 0.10 };
        double Iyy{ 0.25 };
        double Izz{ 0.32 };
        double mass{ 1.0 };
        double c_lin{ 0.0 };
    };

    // -----------------------------------------------------------------------
    // Vector field on the flat 13-vector
    //
    //   q_dot = 1/2 q (x) [0, w]        (Hamilton product, scalar-first)
    //   p_dot = R(q) v_B                (R built from the raw, possibly
    //                                    non-unit, q -- no projection)
    //   w_dot = -I^-1 (w x I w)         (Euler, torque free)
    //   v_dot = -w x v_B - (c/m) v_B    (linear body-frame damping)
    //
    // This matches RigidBody::VectorField exactly: the spatial Newton-Euler
    // form M dnu = W_ext - ad*(nu) M nu reduces, for M = diag(I, m I_3) and
    // W_ext = [0; -c v_B], to the two lines above.
    // -----------------------------------------------------------------------
    inline Vec13 rhs(const Body& b, const Vec13& y)
    {
        const Eigen::Vector4d q = y.segment<4>(0);
        const Eigen::Vector3d w = y.segment<3>(7);
        const Eigen::Vector3d v = y.segment<3>(10);

        Eigen::Vector4d qdot;
        qdot(0) = 0.5 * (-q(1) * w(0) - q(2) * w(1) - q(3) * w(2));
        qdot(1) = 0.5 * ( q(0) * w(0) + q(2) * w(2) - q(3) * w(1));
        qdot(2) = 0.5 * ( q(0) * w(1) - q(1) * w(2) + q(3) * w(0));
        qdot(3) = 0.5 * ( q(0) * w(2) + q(1) * w(1) - q(2) * w(0));

        const Eigen::Quaterniond qq(q(0), q(1), q(2), q(3));
        const Eigen::Matrix3d    R = qq.toRotationMatrix();

        const Eigen::Vector3d Iw{ b.Ixx * w(0), b.Iyy * w(1), b.Izz * w(2) };
        const Eigen::Vector3d c = w.cross(Iw);
        const Eigen::Vector3d wdot{ -c(0) / b.Ixx, -c(1) / b.Iyy, -c(2) / b.Izz };

        Vec13 dy;
        dy.segment<4>(0)  = qdot;
        dy.segment<3>(4)  = R * v;
        dy.segment<3>(7)  = wdot;
        dy.segment<3>(10) = -w.cross(v) - (b.c_lin / b.mass) * v;
        return dy;
    }

    // Central-difference Jacobian of rhs().  Used only to drive Newton; the
    // root Newton converges to is set by the residual, not by the Jacobian, so
    // the finite-difference approximation costs iterations, never accuracy.
    inline Mat13 jacobian(const Body& b, const Vec13& y)
    {
        Mat13 J;
        for (int j = 0; j < 13; ++j) {
            const double d = 1e-7 * std::max(1.0, std::abs(y(j)));
            Vec13 yp = y, ym = y;
            yp(j) += d;
            ym(j) -= d;
            J.col(j) = (rhs(b, yp) - rhs(b, ym)) / (2.0 * d);
        }
        return J;
    }

    // -----------------------------------------------------------------------
    // Explicit RK4 on the flat state
    // -----------------------------------------------------------------------
    inline Vec13 explicit_rk4_step(const Body& b, const Vec13& y, double h)
    {
        const Vec13 k1 = rhs(b, y);
        const Vec13 k2 = rhs(b, y + (0.5 * h) * k1);
        const Vec13 k3 = rhs(b, y + (0.5 * h) * k2);
        const Vec13 k4 = rhs(b, y + h * k3);
        return y + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }

    // -----------------------------------------------------------------------
    // Implicit RK on the flat state, generic over the Butcher tableau
    //
    // Stage system:   Y_i = y + h sum_j A_ij f(Y_j),   i = 1..s
    // Update:         y1  = y + h sum_i b_i f(Y_i)
    //
    // Solved by Newton on the stacked residual
    //   F(Z)_i = Y_i - y - h sum_j A_ij f(Y_j),
    // with block Jacobian  dF/dZ = I - h (A (x) Df).  This mirrors the stage
    // solve in the library RKMK steppers; only the manifold treatment differs.
    // -----------------------------------------------------------------------
    struct ImplicitResult final {
        Vec13 y{ Vec13::Zero() };
        bool  converged{ false };
        int   iters{ 0 };
    };

    template<int S>
    ImplicitResult implicit_rk_step(const Core::ButcherTableau<double, S>& tab,
                                    const Body&                           b,
                                    const Vec13&                          y,
                                    double                                h,
                                    double                                tol      = 1e-13,
                                    int                                   max_iter = 60)
    {
        constexpr int N = 13 * S;
        using VecN = Eigen::Matrix<double, N, 1>;
        using MatN = Eigen::Matrix<double, N, N>;

        // Initialise every stage at the current state.
        VecN Z;
        for (int i = 0; i < S; ++i) Z.template segment<13>(13 * i) = y;

        ImplicitResult out;

        for (int it = 0; it < max_iter; ++it) {
            // Stage derivatives at the current iterate.
            Eigen::Matrix<double, 13, S> F;
            for (int j = 0; j < S; ++j)
                F.col(j) = rhs(b, Z.template segment<13>(13 * j));

            // Residual.
            VecN res;
            for (int i = 0; i < S; ++i) {
                Vec13 r = Z.template segment<13>(13 * i) - y;
                for (int j = 0; j < S; ++j) r -= h * tab.A(i, j) * F.col(j);
                res.template segment<13>(13 * i) = r;
            }

            // Scaled residual norm: each component is measured against its own
            // magnitude.  A flat absolute tolerance is unusable here because
            // the position grows without bound (|p| ~ 1e3 m after 20 s), so an
            // absolute 1e-13 sits below the round-off floor of the p-block and
            // Newton can never reach it.  Weighting by (1 + |y_i|) keeps the
            // O(1) quaternion block at full precision -- which is what the
            // orthonormality metric depends on -- while asking of the position
            // block only what floating point can deliver.
            double scaled = 0.0;
            for (int i = 0; i < S; ++i)
                for (int c = 0; c < 13; ++c) {
                    const double w = 1.0 + std::abs(Z(13 * i + c));
                    const double e = res(13 * i + c) / w;
                    scaled += e * e;
                }
            scaled = std::sqrt(scaled);

            out.iters = it + 1;
            if (scaled <= tol) {
                out.converged = true;
                break;
            }

            // Block Jacobian  I - h (A (x) Df).
            MatN J = MatN::Identity();
            for (int j = 0; j < S; ++j) {
                const Mat13 Df = jacobian(b, Z.template segment<13>(13 * j));
                for (int i = 0; i < S; ++i)
                    J.template block<13, 13>(13 * i, 13 * j) -= h * tab.A(i, j) * Df;
            }

            Z += J.partialPivLu().solve(VecN(-res));
        }

        // Final update from the converged stages.
        Vec13 y1 = y;
        for (int i = 0; i < S; ++i)
            y1 += h * tab.b(i) * rhs(b, Z.template segment<13>(13 * i));

        out.y = y1;
        return out;
    }

    // -----------------------------------------------------------------------
    // Tableaux
    // -----------------------------------------------------------------------

    // Radau IIA, 3-stage, order 5 -- the tableau the library RKMK stepper uses,
    // so pairing this with the RKMK result isolates Munthe-Kaas exactly.
    inline Core::ButcherTableau<double, 3> radau_iia_5()
    {
        return Core::ButcherTableau<double, 3>::radau_iia_3stage_order5();
    }

    // Gauss-Legendre, 3-stage, order 6 -- same stage count, and hence the same
    // implicit-solve cost, as Radau IIA, but symplectic, so it conserves
    // quadratic invariants exactly.  The strongest non-MK competitor.
    inline Core::ButcherTableau<double, 3> gauss_6()
    {
        using Tab = Core::ButcherTableau<double, 3>;
        const double s15 = std::sqrt(15.0);

        Tab::Vec c;
        c << 0.5 - s15 / 10.0, 0.5, 0.5 + s15 / 10.0;

        Tab::Vec b;
        b << 5.0 / 18.0, 4.0 / 9.0, 5.0 / 18.0;

        Tab::MatA A;
        A << 5.0 / 36.0,              2.0 / 9.0 - s15 / 15.0, 5.0 / 36.0 - s15 / 30.0,
             5.0 / 36.0 + s15 / 24.0, 2.0 / 9.0,              5.0 / 36.0 - s15 / 24.0,
             5.0 / 36.0 + s15 / 30.0, 2.0 / 9.0 + s15 / 15.0, 5.0 / 36.0;

        return Tab{ A, b, c };
    }

    // max_ij | b_i a_ij + b_j a_ji - b_i b_j |.  Zero iff the method conserves
    // every quadratic invariant of the ODE.  Zero for Gauss, non-zero for
    // Radau IIA.
    template<int S>
    double quadratic_invariant_defect(const Core::ButcherTableau<double, S>& tab)
    {
        double worst = 0.0;
        for (int i = 0; i < S; ++i)
            for (int j = 0; j < S; ++j) {
                const double m = tab.b(i) * tab.A(i, j)
                               + tab.b(j) * tab.A(j, i)
                               - tab.b(i) * tab.b(j);
                worst = std::max(worst, std::abs(m));
            }
        return worst;
    }

    // -----------------------------------------------------------------------
    // Shared helpers
    // -----------------------------------------------------------------------
    inline Vec13 initial_state(const Eigen::Vector3d& w0, const Eigen::Vector3d& v0)
    {
        Vec13 y = Vec13::Zero();
        y(0) = 1.0;                     // identity quaternion
        y.segment<3>(7)  = w0;
        y.segment<3>(10) = v0;
        return y;
    }

    inline Eigen::Matrix3d rotation_from(const Vec13& y)
    {
        const Eigen::Quaterniond q(y(0), y(1), y(2), y(3));
        return q.toRotationMatrix();    // no normalisation -- deliberate
    }

    inline double orthonormality_defect(const Eigen::Matrix3d& R)
    {
        return (R.transpose() * R - Eigen::Matrix3d::Identity()).norm();
    }

} // namespace Aetherion::Testing::Flat

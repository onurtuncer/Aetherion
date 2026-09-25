// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// DrydenTurbulence.h
//
// Dryden continuous-turbulence model (MIL-F-8785C, Appendix; MIL-HDBK-1797),
// stepped discretely between integrator sub-steps.
//
// WHAT IT PRODUCES
// ────────────────
// A GustState: three linear gust velocities (u_g, v_g, w_g) in body axes
// [m/s] and the angular velocity of the air mass (p_air, q_air, r_air)
// [rad/s], also in body axes.  An aero policy subtracts the linear gust from
// the body velocity and the angular gust from the body rate before forming
// alpha, beta, TAS and the damping-derivative rates:
//
//     v_rel     = v_B - v_surface - v_wind - v_gust
//     omega_air = omega_B/ECEF   - omega_gust
//
// SPECTRA (MIL-F-8785C form, one-sided in spatial frequency Omega [rad/m])
// ───────────────────────────────────────────────────────────────────────
//     Phi_u(Omega) = sigma_u^2 (2 L_u / pi)              / (1 + (L_u Omega)^2)
//     Phi_v(Omega) = sigma_v^2 (L_v / pi) (1 + 3 (L_v Omega)^2) / (1 + (L_v Omega)^2)^2
//     Phi_w(Omega) = sigma_w^2 (L_w / pi) (1 + 3 (L_w Omega)^2) / (1 + (L_w Omega)^2)^2
//
// Each integrates to sigma^2 over Omega in [0, inf): sigma is the RMS gust.
// Under Taylor's frozen-field hypothesis omega = V Omega, so every filter has
// a corner V / L that is re-evaluated at each step from the current airspeed.
//
// Beware the factor of two between the two specifications: MIL-HDBK-1797
// writes the v and w spectra with 2 L_v and 2 L_w and quotes 2 L_w = h at low
// altitude.  This header uses the MIL-F-8785C definitions throughout: above
// 2000 ft, L_u = L_v = L_w = 1750 ft (533.4 m).
//
// ROTATIONAL GUSTS
// ────────────────
//     p_air = d w_g / d y                    (spanwise gradient of w_g)
//     q_air = -(1/V) d w_g / d t  (lagged)   ( = -d w_g / d x )
//     r_air = +(1/V) d v_g / d t  (lagged)   ( = +d v_g / d x )
//
// with first-order lags of corner pi V / (4 b) on p and q and pi V / (3 b) on
// r, b the wingspan.  p_air is its own first-order process with variance
//
//     var(p_air) = 0.8 sigma_w^2 pi^2 (pi L_w / (4 b))^(1/3) / (16 b L_w),
//
// which is MIL-F-8785C's Phi_p integrated over frequency.  The signs above
// make (p, q, r) the angular velocity of the air, so that a positive spanwise
// gradient of the downward gust (more downward gust on the right wing) leaves
// the aircraft with a negative rate relative to the air on the roll axis,
// which roll damping turns into a right-rolling moment, as the lift
// asymmetry would.  MIL-F-8785C leaves the sign of q and r to the reader;
// this is the one that is physically consistent with the frozen field.
//
// DISCRETISATION
// ──────────────
// Exact zero-order-hold sampling of the linear stochastic filters at the
// step dt: the transition matrix and the process-noise covariance are
// computed from the continuous system (Van Loan's method), so the sample
// statistics are independent of dt.  The white-noise input is unit-intensity
// Gaussian; the output gains are chosen so that the stationary variance of
// each gust component equals sigma^2 exactly.
//
// States: u (1), v and r (3), w and q (3), p (1) = 8.
//
// RANDOM NUMBERS
// ──────────────
// std::mt19937_64 with a caller-supplied seed, converted to Gaussians by a
// Box-Muller transform written out here rather than std::normal_distribution,
// so that a given seed reproduces the same record on every platform.
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>

namespace Aetherion::Environment {

/// Gust state seen by an aero policy, body axes.
struct GustState {
    double u_mps  { 0.0 };  ///< Gust velocity along body x [m/s]
    double v_mps  { 0.0 };  ///< Gust velocity along body y [m/s]
    double w_mps  { 0.0 };  ///< Gust velocity along body z (down) [m/s]
    double p_rad_s{ 0.0 };  ///< Angular velocity of the air about body x [rad/s]
    double q_rad_s{ 0.0 };  ///< Angular velocity of the air about body y [rad/s]
    double r_rad_s{ 0.0 };  ///< Angular velocity of the air about body z [rad/s]

    [[nodiscard]] Eigen::Vector3d linear()  const { return { u_mps, v_mps, w_mps }; }
    [[nodiscard]] Eigen::Vector3d angular() const { return { p_rad_s, q_rad_s, r_rad_s }; }
};

/// Spectral parameters of the Dryden model (MIL-F-8785C scale lengths).
struct DrydenParameters {
    double sigma_u_mps{ 0.0 };   ///< RMS longitudinal gust [m/s]; all sigmas zero = off
    double sigma_v_mps{ 0.0 };   ///< RMS lateral gust [m/s]
    double sigma_w_mps{ 0.0 };   ///< RMS vertical gust [m/s]
    double L_u_m      { 533.4 }; ///< Longitudinal scale length [m] (1750 ft)
    double L_v_m      { 533.4 }; ///< Lateral scale length [m]
    double L_w_m      { 533.4 }; ///< Vertical scale length [m]
    double wingspan_m { 9.144 }; ///< Wingspan b for the rotational gusts [m] (F-16: 30 ft)

    [[nodiscard]] bool isOff() const noexcept
    {
        return sigma_u_mps == 0.0 && sigma_v_mps == 0.0 && sigma_w_mps == 0.0;
    }
};

/// MIL-F-8785C low-altitude (h < 1000 ft) scale lengths and intensities.
///
/// @param alt_agl_m  Height above ground [m]; clamped to [3 m, 304.8 m].
/// @param W20_mps    Mean wind speed at 20 ft (6.1 m) above ground [m/s].
///                   The specification's "light / moderate / severe" are
///                   W20 = 15 / 30 / 45 kt.
/// @param wingspan_m Wingspan for the rotational gusts [m].
///
/// Above 2000 ft the specification's intensities come from a chart of
/// probability of exceedance versus altitude (MIL-F-8785C Figure 7) that is
/// not reproduced here; set the six numbers directly for that regime, with
/// the default 533.4 m scale lengths.
inline DrydenParameters DrydenLowAltitudeParameters(double alt_agl_m, double W20_mps,
                                                     double wingspan_m = 9.144)
{
    constexpr double kFt_m = 0.3048;
    const double h_ft = std::clamp(alt_agl_m, 10.0 * kFt_m, 1000.0 * kFt_m) / kFt_m;
    const double denom = 0.177 + (0.000823 * h_ft);

    DrydenParameters p{};
    p.sigma_w_mps = 0.1 * W20_mps;
    p.sigma_u_mps = p.sigma_w_mps / std::pow(denom, 0.4);
    p.sigma_v_mps = p.sigma_u_mps;
    p.L_w_m       = h_ft * kFt_m;
    p.L_u_m       = (h_ft / std::pow(denom, 1.2)) * kFt_m;
    p.L_v_m       = p.L_u_m;
    p.wingspan_m  = wingspan_m;
    return p;
}

namespace detail {

    /// Matrix exponential by scaling and squaring with a Taylor series.
    /// Adequate for the small, well-conditioned Van Loan blocks used here.
    template<int N>
    Eigen::Matrix<double, N, N> MatrixExponential(const Eigen::Matrix<double, N, N>& M)
    {
        using Mat = Eigen::Matrix<double, N, N>;
        const double norm = M.template lpNorm<Eigen::Infinity>();
        int s = 0;
        if (norm > 0.5) {
            s = static_cast<int>(std::ceil(std::log2(norm / 0.5)));
        }
        const Mat A = M / std::ldexp(1.0, s);

        Mat result = Mat::Identity();
        Mat term   = Mat::Identity();
        for (int k = 1; k <= 18; ++k) {
            term   = (term * A) / static_cast<double>(k);
            result += term;
        }
        for (int i = 0; i < s; ++i) {
            result = result * result;
        }
        return result;
    }

    /// Exact discretisation of  dx = A x dt + B dW  (unit-intensity white
    /// noise): returns Phi = exp(A dt) and Q = integral of exp(As) B B^T
    /// exp(A^T s) ds over [0, dt], by Van Loan's block exponential.
    template<int N>
    void VanLoanDiscretise(const Eigen::Matrix<double, N, N>& A,
                           const Eigen::Matrix<double, N, 1>& B,
                           double dt,
                           Eigen::Matrix<double, N, N>& Phi,
                           Eigen::Matrix<double, N, N>& Q)
    {
        Eigen::Matrix<double, 2 * N, 2 * N> M = Eigen::Matrix<double, 2 * N, 2 * N>::Zero();
        M.template block<N, N>(0, 0) = -A * dt;
        M.template block<N, N>(0, N) = (B * B.transpose()) * dt;
        M.template block<N, N>(N, N) = A.transpose() * dt;
        const auto E = MatrixExponential<2 * N>(M);
        Phi = E.template block<N, N>(N, N).transpose();
        Q   = Phi * E.template block<N, N>(0, N);
        // Symmetrise against round-off.
        Q = 0.5 * (Q + Q.transpose()).eval();
    }

    /// Lower Cholesky factor with a guard against tiny negative pivots.
    template<int N>
    Eigen::Matrix<double, N, N> SafeCholesky(const Eigen::Matrix<double, N, N>& Q)
    {
        Eigen::Matrix<double, N, N> L = Eigen::Matrix<double, N, N>::Zero();
        for (int j = 0; j < N; ++j) {
            double d = Q(j, j);
            for (int k = 0; k < j; ++k) d -= L(j, k) * L(j, k);
            L(j, j) = (d > 0.0) ? std::sqrt(d) : 0.0;
            for (int i = j + 1; i < N; ++i) {
                double v = Q(i, j);
                for (int k = 0; k < j; ++k) v -= L(i, k) * L(j, k);
                L(i, j) = (L(j, j) > 0.0) ? v / L(j, j) : 0.0;
            }
        }
        return L;
    }

} // namespace detail

/// Dryden turbulence filter bank, stepped at a fixed dt between integrator
/// sub-steps.  See the header comment for the model and the conventions.
class DrydenTurbulence
{
public:
    static constexpr std::size_t kNumStates = 8;

    DrydenTurbulence() = default;

    explicit DrydenTurbulence(const DrydenParameters& params, std::uint64_t seed = 1)
        : m_params(params)
    {
        reseed(seed);
    }

    /// Restart the noise stream. Does not clear the filter states.
    void reseed(std::uint64_t seed)
    {
        m_rng.seed(seed);
        m_haveSpare = false;
    }

    /// Zero the filter states (the gust starts from calm).
    void resetStates() noexcept { m_x.fill(0.0); m_haveSpare = false; }

    [[nodiscard]] const DrydenParameters& parameters() const noexcept { return m_params; }
    void setParameters(const DrydenParameters& p) noexcept { m_params = p; }

    /// Filter states, for save/restore.  Layout:
    ///   0: u
    ///   1,2,3: v shaping (2) and r lag
    ///   4,5,6: w shaping (2) and q lag
    ///   7: p
    [[nodiscard]] const std::array<double, kNumStates>& states() const noexcept { return m_x; }
    void setStates(const std::array<double, kNumStates>& x) noexcept { m_x = x; }

    /// Gust corresponding to the current filter states at airspeed V.
    [[nodiscard]] GustState current(double V_mps) const
    {
        const double V = std::max(V_mps, 1.0);
        GustState g{};
        g.u_mps   = m_x[0];
        g.v_mps   = outputSecondOrder(m_params.sigma_v_mps, m_params.L_v_m, V, m_x[1], m_x[2]);
        g.r_rad_s = +m_x[3];
        g.w_mps   = outputSecondOrder(m_params.sigma_w_mps, m_params.L_w_m, V, m_x[4], m_x[5]);
        g.q_rad_s = -m_x[6];
        g.p_rad_s = m_x[7];
        return g;
    }

    /// Advance the filters by dt at airspeed V_mps and return the new gust.
    ///
    /// dt must be the same from call to call for the statistics to be those
    /// of the continuous model (the discretisation is exact for any single
    /// dt, but a varying dt changes the process, not just its realisation).
    GustState step(double dt, double V_mps)
    {
        if (m_params.isOff() || dt <= 0.0) {
            return current(V_mps);
        }
        const double V = std::max(V_mps, 1.0);
        const double b = std::max(m_params.wingspan_m, 0.1);
        constexpr double kPi = std::numbers::pi;

        // ── u: first-order Gauss-Markov, exact ZOH ───────────────────────────
        {
            const double a   = V / m_params.L_u_m;
            const double phi = std::exp(-a * dt);
            m_x[0] = (phi * m_x[0])
                   + (m_params.sigma_u_mps * std::sqrt(std::max(0.0, 1.0 - (phi * phi))) * gaussian());
        }

        // ── p: first-order Gauss-Markov on MIL-F-8785C Phi_p, exact ZOH ──────
        {
            const double a    = kPi * V / (4.0 * b);
            const double Lw   = m_params.L_w_m;
            const double var  = 0.8 * m_params.sigma_w_mps * m_params.sigma_w_mps * kPi * kPi
                              * std::cbrt(kPi * Lw / (4.0 * b)) / (16.0 * b * Lw);
            const double phi  = std::exp(-a * dt);
            m_x[7] = (phi * m_x[7])
                   + (std::sqrt(var) * std::sqrt(std::max(0.0, 1.0 - (phi * phi))) * gaussian());
        }

        // ── v with r lag (corner pi V / 3b), w with q lag (corner pi V / 4b) ─
        stepSecondOrderWithLag(m_params.sigma_v_mps, m_params.L_v_m, V,
                               kPi * V / (3.0 * b), dt, m_x[1], m_x[2], m_x[3], m_cacheV);
        stepSecondOrderWithLag(m_params.sigma_w_mps, m_params.L_w_m, V,
                               kPi * V / (4.0 * b), dt, m_x[4], m_x[5], m_x[6], m_cacheW);

        return current(V);
    }

private:
    // The exact discretisation depends only on (sigma, L, V, corner, dt).  V
    // changes slowly in flight and not at all in a statistics test, so the
    // transition matrix and the noise factor are recomputed only when one of
    // those inputs moves.
    struct LagCache {
        double sigma{ -1.0 }, L{ -1.0 }, V{ -1.0 }, al{ -1.0 }, dt{ -1.0 };
        Eigen::Matrix3d Phi{ Eigen::Matrix3d::Identity() };
        Eigen::Matrix3d Lc { Eigen::Matrix3d::Zero() };
    };
    // Output gain K of the second-order shaping filter: with the canonical
    // states (x1, x2) driven by unit-intensity white noise and the double pole
    // at -a, the stationary covariance is diag(1/(4a^3), 1/(4a)), so the
    // output K (a^2 x1 + sqrt(3) a x2) has variance K^2 a.  K = sigma / sqrt(a)
    // makes it sigma^2.
    static double outputSecondOrder(double sigma, double L, double V, double x1, double x2)
    {
        const double a = V / L;
        const double K = sigma / std::sqrt(a);
        return K * ((a * a * x1) + (std::numbers::sqrt3 * a * x2));
    }

    // Exact ZOH step of the 3-state system: second-order shaping filter
    // (x1, x2) plus the first-order lag xlag on its time derivative divided by V.
    //   x1' = x2
    //   x2' = -a^2 x1 - 2a x2 + xi
    //   xlag' = -al xlag + al * (d/dt output) / V
    // where d/dt output = K(-sqrt3 a^3 x1 + a^2 (1 - 2 sqrt3) x2 + sqrt3 a xi).
    void stepSecondOrderWithLag(double sigma, double L, double V, double al, double dt,
                                double& x1, double& x2, double& xlag, LagCache& cache)
    {
        if (cache.sigma != sigma || cache.L != L || cache.V != V || cache.al != al || cache.dt != dt) {
            const double a  = V / L;
            const double K  = sigma / std::sqrt(a);
            const double s3 = std::numbers::sqrt3;

            Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
            A(0, 1) = 1.0;
            A(1, 0) = -a * a;
            A(1, 1) = -2.0 * a;
            A(2, 0) = -al * K * s3 * a * a * a / V;
            A(2, 1) =  al * K * a * a * (1.0 - (2.0 * s3)) / V;
            A(2, 2) = -al;

            const Eigen::Vector3d B(0.0, 1.0, al * K * s3 * a / V);

            Eigen::Matrix3d Q;
            detail::VanLoanDiscretise<3>(A, B, dt, cache.Phi, Q);
            cache.Lc    = detail::SafeCholesky<3>(Q);
            cache.sigma = sigma; cache.L = L; cache.V = V; cache.al = al; cache.dt = dt;
        }

        const Eigen::Vector3d x(x1, x2, xlag);
        const Eigen::Vector3d n(gaussian(), gaussian(), gaussian());
        const Eigen::Vector3d xn = (cache.Phi * x) + (cache.Lc * n);
        x1 = xn(0); x2 = xn(1); xlag = xn(2);
    }

    // Box-Muller on the top 53 bits of mt19937_64: platform-independent.
    double gaussian()
    {
        if (m_haveSpare) {
            m_haveSpare = false;
            return m_spare;
        }
        double u1 = 0.0;
        do {
            u1 = uniform01();
        } while (u1 <= 0.0);
        const double u2 = uniform01();
        const double r  = std::sqrt(-2.0 * std::log(u1));
        const double th = 2.0 * std::numbers::pi * u2;
        m_spare     = r * std::sin(th);
        m_haveSpare = true;
        return r * std::cos(th);
    }

    double uniform01()
    {
        const std::uint64_t bits = m_rng() >> 11;             // 53 bits
        return static_cast<double>(bits) * (1.0 / 9007199254740992.0);
    }

    DrydenParameters               m_params{};
    std::array<double, kNumStates> m_x{};
    std::mt19937_64                m_rng{ 1 };
    bool                           m_haveSpare{ false };
    double                         m_spare{ 0.0 };
    LagCache                       m_cacheV{};
    LagCache                       m_cacheW{};
};

} // namespace Aetherion::Environment

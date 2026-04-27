// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Aetherion/Spatial/Wrench.h"
#include "Aetherion/ODE/RKMK/Lie/SE3.h"
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Environment/detail/MathWrappers.h>

namespace Aetherion::FlightDynamics {

/// @brief Aerodynamic policy that returns a zero wrench (no aerodynamic forces).
///
/// Models a dragless, liftless body (e.g. vacuum trajectory or baseline case).
/// Satisfies @c AeroPolicy; the compiler optimises the zero addition away entirely.
    struct ZeroAeroPolicy {
        template<class S>
        Spatial::Wrench<S>
            operator()(const ODE::RKMK::Lie::SE3<S>&,
                const Eigen::Matrix<S, 6, 1>&,
                S, S) const
        {
            Spatial::Wrench<S> w{};
            w.f.setZero();
            return w;
        }
    };

    static_assert(AeroPolicy<ZeroAeroPolicy>);

/// @brief Aerodynamic policy for a sphere: pure drag, no lift, no moments.
///
/// Computes the drag force in the body frame using:
/// @f[ \mathbf{F}_\mathrm{drag} = -\tfrac{1}{2}\,\rho(h)\,C_D\,S_\mathrm{ref}\,
///     |\mathbf{v}_B|\,\mathbf{v}_B @f]
/// where @f$\mathbf{v}_B = \nu_B^{3:5}@f$ is the body-frame linear velocity and
/// @f$\rho(h)@f$ is the US 1976 atmospheric density at geocentric altitude
/// @f$h \approx |\mathbf{r}_\mathrm{ECI}| - R_e@f$.
///
/// Satisfies @c AeroPolicy (works for both @c double and @c CppAD::AD<double>).
    struct DragOnlyAeroPolicy {
        double CD   { 0.0 }; ///< Drag coefficient [-].
        double S_ref{ 0.0 }; ///< Reference area [m²].

        DragOnlyAeroPolicy() = default;
        DragOnlyAeroPolicy(double cd, double s_ref) : CD(cd), S_ref(s_ref) {}

        template<class S>
        Spatial::Wrench<S>
            operator()(const ODE::RKMK::Lie::SE3<S>& g,
                const Eigen::Matrix<S, 6, 1>& nu_B,
                S /*mass*/, S /*t*/) const
        {
            using Environment::detail::SquareRoot;

            // ECI-relative body-frame linear velocity (includes Earth rotation)
            const Eigen::Matrix<S, 3, 1> v_B = nu_B.template tail<3>();

            // Earth surface velocity at r_ECI expressed in the body frame:
            //   v_surface_body = R^T * (omega_E x r_ECI)
            // Subtracting this gives the atmosphere-relative (aerodynamic) airspeed.
            constexpr double kOmegaE = Environment::WGS84::kRotationRate_rad_s;
            const Eigen::Matrix<S, 3, 1> omega_E(S(0), S(0), S(kOmegaE));
            const Eigen::Matrix<S, 3, 1> v_surface_body = g.R.transpose() * omega_E.cross(g.p);
            const Eigen::Matrix<S, 3, 1> v_rel = v_B - v_surface_body;

            // Geocentric altitude: alt ≈ |r_ECI| − Re  (spherical approximation)
            const S alt = g.p.norm() - S(Environment::WGS84::kSemiMajorAxis_m);

            // US 1976 atmospheric density at altitude
            const S rho = Environment::US1976Atmosphere(alt).rho;

            // Speed — AD-safe sqrt: avoids undefined derivative at v=0
            const S v2     = v_rel.squaredNorm();
            const S v_safe = SquareRoot(v2 + S(1.0e-30));

            // F_drag = −½ ρ CD S_ref |v_rel| v_rel  (body frame, opposes airspeed)
            const S k = S(0.5) * rho * S(CD) * S(S_ref) * v_safe;

            Spatial::Wrench<S> w{};
            w.f.setZero();
            w.f.template tail<3>() = -k * v_rel;
            return w;
        }
    };

    static_assert(AeroPolicy<DragOnlyAeroPolicy>);

/// @brief Aerodynamic policy for an asymmetric rigid body with drag and
///        rotary damping moments (no lift or cross-coupling terms).
///
/// Computes:
///   - **Drag force**:   @f$ \mathbf{F} = -\tfrac{1}{2}\rho\,C_D\,S\,|v_\text{rel}|\,v_\text{rel} @f$
///   - **Roll  moment**: @f$ L = \tfrac{1}{4}\rho\,|V|\,S\,b^2\,C_{lp}\,p @f$
///   - **Pitch moment**: @f$ M = \tfrac{1}{4}\rho\,|V|\,S\,\bar{c}^2\,C_{mq}\,q @f$
///   - **Yaw   moment**: @f$ N = \tfrac{1}{4}\rho\,|V|\,S\,b^2\,C_{nr}\,r @f$
///
/// All quantities in SI (m, kg, s, rad).  Reference lengths @p b and @p cbar
/// follow the standard non-dimensional rate convention:
///   @f$ \hat{p} = pb/(2V),\; \hat{q} = q\bar{c}/(2V),\; \hat{r} = rb/(2V) @f$
///
/// Uses atmosphere-relative airspeed for both drag and moment scaling,
/// consistent with @c DragOnlyAeroPolicy.  AD-safe for use with CppAD.
///
/// Satisfies @c AeroPolicy.
    struct BrickDampingAeroPolicy {
        double CD  { 0.0 }; ///< Drag coefficient [-].
        double S   { 0.0 }; ///< Reference area [m²].
        double b   { 0.0 }; ///< Span reference length [m] (roll and yaw scaling).
        double cbar{ 0.0 }; ///< Chord reference length [m] (pitch scaling).
        double Clp { 0.0 }; ///< Roll damping derivative @f$C_{l_p}@f$ [-].
        double Cmq { 0.0 }; ///< Pitch damping derivative @f$C_{m_q}@f$ [-].
        double Cnr { 0.0 }; ///< Yaw damping derivative @f$C_{n_r}@f$ [-].

        BrickDampingAeroPolicy() = default;
        BrickDampingAeroPolicy(double cd, double s, double b_,
                               double cbar_, double clp, double cmq, double cnr)
            : CD(cd), S(s), b(b_), cbar(cbar_), Clp(clp), Cmq(cmq), Cnr(cnr) {}

        template<class Sc>
        Spatial::Wrench<Sc>
            operator()(const ODE::RKMK::Lie::SE3<Sc>& g,
                const Eigen::Matrix<Sc, 6, 1>& nu_B,
                Sc /*mass*/, Sc /*t*/) const
        {
            using Environment::detail::SquareRoot;

            // Body angular rates (ECI-relative; Earth-rate correction ~0.015%, neglected)
            const Eigen::Matrix<Sc, 3, 1> omega_B = nu_B.template head<3>();

            // Atmosphere-relative linear velocity (same as DragOnlyAeroPolicy)
            const Eigen::Matrix<Sc, 3, 1> v_B = nu_B.template tail<3>();
            constexpr double kOmegaE = Environment::WGS84::kRotationRate_rad_s;
            const Eigen::Matrix<Sc, 3, 1> omega_E(Sc(0), Sc(0), Sc(kOmegaE));
            const Eigen::Matrix<Sc, 3, 1> v_surface = g.R.transpose() * omega_E.cross(g.p);
            const Eigen::Matrix<Sc, 3, 1> v_rel = v_B - v_surface;

            // Geocentric altitude and US 1976 density
            const Sc alt = g.p.norm() - Sc(Environment::WGS84::kSemiMajorAxis_m);
            const Sc rho = Environment::US1976Atmosphere(alt).rho;

            // True airspeed — AD-safe sqrt
            const Sc v2     = v_rel.squaredNorm();
            const Sc v_safe = SquareRoot(v2 + Sc(1.0e-30));

            // Drag force: −½ ρ CD S |v_rel| v_rel
            const Sc k_drag = Sc(0.5) * rho * Sc(CD) * Sc(S) * v_safe;

            // Damping moments: ¼ ρ |V| S (ref_len²) C_xrate * rate
            const Sc k_mom = Sc(0.25) * rho * v_safe * Sc(S);
            const Sc b2    = Sc(b * b);
            const Sc cb2   = Sc(cbar * cbar);

            Spatial::Wrench<Sc> w{};
            w.f.setZero();
            // Moments (head<3>): [L, M, N]
            w.f(0) = k_mom * b2  * Sc(Clp) * omega_B(0);
            w.f(1) = k_mom * cb2 * Sc(Cmq) * omega_B(1);
            w.f(2) = k_mom * b2  * Sc(Cnr) * omega_B(2);
            // Force (tail<3>)
            w.f.template tail<3>() = -k_drag * v_rel;
            return w;
        }
    };

    static_assert(AeroPolicy<BrickDampingAeroPolicy>);

/// @brief Aerodynamic drag policy for a sphere in a constant steady wind.
///
/// Extends @c DragOnlyAeroPolicy by subtracting the ambient wind velocity
/// (in addition to the Earth surface velocity) from the body-frame velocity
/// before computing drag.  The wind is specified as a constant vector in the
/// ECEF frame; it is rotated to ECI at each call using the current ERA
/// @f$ \theta_{ERA} = \omega_E \, t @f$.
///
/// Drag force:
/// @f[ \mathbf{F} = -\tfrac{1}{2}\rho\,C_D\,S_\text{ref}\,
///     |\mathbf{v}_\text{rel}|\,\mathbf{v}_\text{rel} @f]
/// where
/// @f$ \mathbf{v}_\text{rel} = \mathbf{v}_B
///     - \mathbf{v}_\text{surface,body}
///     - \mathbf{v}_\text{wind,body} @f$.
///
/// Satisfies @c AeroPolicy.
    struct SteadyWindDragPolicy {
        double CD   { 0.0 }; ///< Drag coefficient [-].
        double S_ref{ 0.0 }; ///< Reference area [m²].
        /// Wind velocity in ECEF frame [m/s].
        double v_wind_ecef_x{ 0.0 };
        double v_wind_ecef_y{ 0.0 };
        double v_wind_ecef_z{ 0.0 };

        SteadyWindDragPolicy() = default;
        SteadyWindDragPolicy(double cd, double s_ref,
                             double wx, double wy, double wz)
            : CD(cd), S_ref(s_ref)
            , v_wind_ecef_x(wx), v_wind_ecef_y(wy), v_wind_ecef_z(wz) {}

        template<class S>
        Spatial::Wrench<S>
            operator()(const ODE::RKMK::Lie::SE3<S>& g,
                const Eigen::Matrix<S, 6, 1>& nu_B,
                S /*mass*/, S t) const
        {
            using Environment::detail::SquareRoot;
            using Environment::detail::Sine;
            using Environment::detail::Cosine;

            // ── Earth surface velocity in body frame ──────────────────────
            constexpr double kOmegaE = Environment::WGS84::kRotationRate_rad_s;
            const Eigen::Matrix<S, 3, 1> omega_E(S(0), S(0), S(kOmegaE));
            const Eigen::Matrix<S, 3, 1> v_surface_body =
                g.R.transpose() * omega_E.cross(g.p);

            // ── Steady wind: ECEF → ECI (rotate by +θ_ERA around z) ──────
            const S theta = S(kOmegaE) * t;
            const S ct = Cosine(theta);
            const S st = Sine(theta);
            const Eigen::Matrix<S, 3, 1> v_wind_eci(
                ct * S(v_wind_ecef_x) - st * S(v_wind_ecef_y),
                st * S(v_wind_ecef_x) + ct * S(v_wind_ecef_y),
                S(v_wind_ecef_z));
            const Eigen::Matrix<S, 3, 1> v_wind_body =
                g.R.transpose() * v_wind_eci;

            // ── Atmosphere-relative velocity ──────────────────────────────
            const Eigen::Matrix<S, 3, 1> v_B = nu_B.template tail<3>();
            const Eigen::Matrix<S, 3, 1> v_rel = v_B - v_surface_body - v_wind_body;

            // ── Altitude, density, drag ───────────────────────────────────
            const S alt = g.p.norm() - S(Environment::WGS84::kSemiMajorAxis_m);
            const S rho = Environment::US1976Atmosphere(alt).rho;
            const S v2      = v_rel.squaredNorm();
            const S v_safe  = SquareRoot(v2 + S(1.0e-30));
            const S k       = S(0.5) * rho * S(CD) * S(S_ref) * v_safe;

            Spatial::Wrench<S> w{};
            w.f.setZero();
            w.f.template tail<3>() = -k * v_rel;
            return w;
        }
    };

    static_assert(AeroPolicy<SteadyWindDragPolicy>);

} // namespace Aetherion::FlightDynamics
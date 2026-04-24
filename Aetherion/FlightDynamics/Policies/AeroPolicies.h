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

} // namespace Aetherion::FlightDynamics
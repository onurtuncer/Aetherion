// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Environment/Gravity.h>
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Spatial/Wrench.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

namespace Aetherion::FlightDynamics {

    // ── Zero gravity ─────────────────────────────────────────────────────────

    struct ZeroGravityPolicy {
        template<class S>
        Spatial::Wrench<S>
            operator()(const ODE::RKMK::Lie::SE3<S>&, S) const
        {
            Spatial::Wrench<S> w{};
            w.f.setZero();
            return w;
        }
    };

    static_assert(GravityPolicy<ZeroGravityPolicy>);

    // ── Central (point-mass) gravity ─────────────────────────────────────────

    struct CentralGravityPolicy {
        double mu{ Environment::WGS84::kGM_m3_s2 };

        template<class S>
        Spatial::Wrench<S>
            operator()(const ODE::RKMK::Lie::SE3<S>& g, S mass) const
        {
            const Environment::Vec3<S> r{ g.p(0), g.p(1), g.p(2) };
            const auto g_arr = Environment::CentralGravity(r, S(mu));

            Spatial::Wrench<S> w{};
            w.f.setZero();
            w.f.template tail<3>() =
                g.R.transpose().template cast<S>() *
                Eigen::Matrix<S, 3, 1>{ g_arr[0] * mass,
                g_arr[1] * mass,
                g_arr[2] * mass };
            return w;
        }
    };

    static_assert(GravityPolicy<CentralGravityPolicy>);

    // ── J₂ gravity ───────────────────────────────────────────────────────────
    //
    // All three default values sourced from WGS84.h.

    struct J2GravityPolicy {
        double mu{ Environment::WGS84::kGM_m3_s2 };
        double Re{ Environment::WGS84::kSemiMajorAxis_m };
        double J2{ Environment::WGS84::kJ2 };

        template<class S>
        Spatial::Wrench<S>
            operator()(const ODE::RKMK::Lie::SE3<S>& g, S mass) const
        {
            const Environment::Vec3<S> r{ g.p(0), g.p(1), g.p(2) };
            const auto g_arr = Environment::J2(r, S(mu), S(Re), S(J2));

            Spatial::Wrench<S> w{};
            w.f.setZero();
            w.f.template tail<3>() =
                g.R.transpose().template cast<S>() *
                Eigen::Matrix<S, 3, 1>{ g_arr[0] * mass,
                g_arr[1] * mass,
                g_arr[2] * mass };
            return w;
        }
    };

    static_assert(GravityPolicy<J2GravityPolicy>);

} // namespace Aetherion::FlightDynamics

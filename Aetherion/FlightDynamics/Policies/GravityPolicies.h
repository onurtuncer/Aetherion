// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Environment/Gravity.h>
#include <Aetherion/Spatial/Wrench.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>

namespace Aetherion::FlightDynamics {

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

    struct CentralGravityPolicy {
        double mu{ 3.986004418e14 };

        template<class S>
        Spatial::Wrench<S>
            operator()(const ODE::RKMK::Lie::SE3<S>& g, S mass) const
        {
            const Environment::Vec3<S> r{ g.p(0), g.p(1), g.p(2) };
            const auto g_arr = Environment::CentralGravity(r, S(mu));
            const Eigen::Matrix<S, 3, 1> F_W{ g_arr[0] * mass, g_arr[1] * mass, g_arr[2] * mass };

            Spatial::Wrench<S> w{};
            w.f.setZero();
            // Rotate W -> B:  F_B = R^T * F_W
            w.f.template tail<3>() = g.R.transpose().template cast<S>() * F_W;
            return w;
        }
    };

    static_assert(GravityPolicy<CentralGravityPolicy>);
    
    struct J2GravityPolicy {
        double mu{ 3.986004418e14 };
        double Re{ 6378137.0 };
        double J2{ 1.08262668e-3 };

        template<class S>
        Spatial::Wrench<S>
            operator()(const ODE::RKMK::Lie::SE3<S>& g, S mass) const
        {
            const Environment::Vec3<S> r{ g.p(0), g.p(1), g.p(2) };
            const auto g_arr = Environment::J2(r, S(mu), S(Re), S(J2));
            const Eigen::Matrix<S, 3, 1> F_W{ g_arr[0] * mass, g_arr[1] * mass, g_arr[2] * mass };

            Spatial::Wrench<S> w{};
            w.f.setZero();
            w.f.template tail<3>() = g.R.transpose().template cast<S>() * F_W;
            return w;
        }
    };

    static_assert(GravityPolicy<J2GravityPolicy>);

} // namespace Aetherion::FlightDynamics
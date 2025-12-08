// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace Aetherion::Environment {

    /// Result of the atmosphere call
    template <class Scalar>
    struct Us1976State {
        Scalar T;    // Temperature [K]
        Scalar p;    // Static pressure [Pa]
        Scalar rho;  // Density [kg/m^3]
        Scalar a;    // Speed of sound [m/s]
    };

    namespace detail {

        // ADL-friendly wrappers so this works for double, CppAD::AD<>, etc.
        template <class S>
        inline S Power(const S& x, const S& y) {
            using std::pow;
            return pow(x, y);
        }

        template <class S>
        inline S Exponential(const S& x) {
            using std::exp;
            return exp(x);
        }

        template <class S>
        inline S SquareRoot(const S& x) {
            using std::sqrt;
            return sqrt(x);
        }

    } // namespace detail

    /// U.S. Standard Atmosphere 1976, 0–86 km (geometric altitude).
    /// - Input:  geometric altitude above MSL in meters (approx; internally converted to geopotential).
    /// - Output: temperature T [K], pressure p [Pa], density rho [kg/m^3], speed of sound a [m/s].
    ///
    /// CppAD friendliness:
    /// - Templated on Scalar.
    /// - Only uses elementary functions (no std::isnan, no branching on sign of AD, etc.).
    /// - There *is* piecewise behavior via `if` on altitude; for AD, the tape records the branch
    ///   corresponding to the evaluation altitude. This is fine as long as you don't differentiate
    ///   *through* layer boundaries.
    template <class Scalar>
    inline Us1976State<Scalar> US1976Atmosphere(const Scalar& altitude_m_in)
    {
        using detail::Exponential;
        using detail::Power;
        using detail::SquareRoot;

        // ---- Physical constants (US1976) ---------------------------------------
        const Scalar g0 = Scalar(9.80665);    // [m/s^2]
        const Scalar R = Scalar(287.05287);  // [J/(kg·K)] specific gas constant for air
        const Scalar gamma = Scalar(1.4);       // ratio of specific heats

        // Geopotential reference Earth radius (USSA-76) [m]
        const Scalar Re = Scalar(6356766.0);

        // ---- Clamp altitude to model limits: 0 .. 84.852 km --------------------
        Scalar h = altitude_m_in;
        if (h < Scalar(0)) {
            h = Scalar(0);
        }
        const Scalar h_max = Scalar(84852.0); // 84.852 km
        if (h > h_max) {
            h = h_max;
        }

        // Convert geometric altitude to geopotential altitude H
        // H = Re * h / (Re + h)
        const Scalar H = Re * h / (Re + h);      // [m]
        const Scalar H_km = H * Scalar(1.0e-3);  // [km]

        // ---- US1976 base values per layer (0..7) -------------------------------
        // Geopotential base heights [km]
        static constexpr double Hb_km_arr[8] = {
            0.0,    // 0: 0 km
            11.0,   // 1: 11 km
            20.0,   // 2: 20 km
            32.0,   // 3: 32 km
            47.0,   // 4: 47 km
            51.0,   // 5: 51 km
            71.0,   // 6: 71 km
            84.852  // 7: 84.852 km
        };

        // Base temperatures [K]
        static constexpr double Tb_arr[8] = {
            288.15,   // 0
            216.65,   // 1
            216.65,   // 2
            228.65,   // 3
            270.65,   // 4
            270.65,   // 5
            214.65,   // 6
            186.946   // 7
        };

        // Base pressures [Pa]
        static constexpr double Pb_arr[8] = {
            101325.0,   // 0
            22632.1,    // 1
            5474.89,    // 2
            868.019,    // 3
            110.906,    // 4
            66.9389,    // 5
            3.95642,    // 6
            0.3734      // 7  (top of model I)
        };

        // Temperature lapse rates Lb = dT/dH [K/km]
        static constexpr double Lb_K_per_km_arr[8] = {
            -6.5,   // 0: 0–11 km
            0.0,    // 1: 11–20 km (isothermal)
            1.0,    // 2: 20–32 km
            2.8,    // 3: 32–47 km
            0.0,    // 4: 47–51 km
            -2.8,   // 5: 51–71 km
            -2.0,   // 6: 71–84.852 km
            0.0     // 7: >=84.852 km (not really used here)
        };

        // ---- Find the layer index b such that Hb <= H < H_{b+1} ----------------
        std::size_t b = 0;
        for (; b + 1 < 8; ++b) {
            const Scalar Hb_next = Scalar(Hb_km_arr[b + 1]);
            if (H_km < Hb_next) {
                break;
            }
        }
        if (b >= 7) {
            b = 7; // Just in case, clamp
        }

        const Scalar Hb_km = Scalar(Hb_km_arr[b]);
        const Scalar Hb = Hb_km * Scalar(1000.0);   // [m]
        const Scalar Tb = Scalar(Tb_arr[b]);        // [K]
        const Scalar Pb = Scalar(Pb_arr[b]);        // [Pa]
        const Scalar Lb = Scalar(Lb_K_per_km_arr[b]) * Scalar(1.0e-3); // [K/m]

        // ---- Compute temperature and pressure at altitude H --------------------
        Scalar T;
        Scalar p;

        if (Lb == Scalar(0)) {
            // Isothermal layer
            T = Tb;
            const Scalar exponent = -g0 * (H - Hb) / (R * Tb);
            p = Pb * Exponential(exponent);
        }
        else {
            // Gradient layer (linear temperature profile)
            T = Tb + Lb * (H - Hb);
            const Scalar theta = T / Tb;
            const Scalar exponent = -g0 / (Lb * R);
            p = Pb * Power(theta, exponent);
        }

        // Ideal gas: rho = p / (R T)
        const Scalar rho = p / (R * T);

        // Speed of sound
        const Scalar a = SquareRoot(gamma * R * T);

        Us1976State<Scalar> out{ T, p, rho, a };
        return out;
    }

} // namespace Aetherion::Environment

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include <Aetherion/Environment/detail/MathWrappers.h> 

namespace Aetherion::Environment {

    /// Result of the atmosphere call
    template <class Scalar>
    struct Us1976State {
        Scalar T;    // Temperature [K]
        Scalar p;    // Static pressure [Pa]
        Scalar rho;  // Density [kg/m^3]
        Scalar a;    // Speed of sound [m/s]
    };

    /// Deviations from the standard day, applied to US1976Atmosphere().
    ///
    /// - deltaT_K: uniform temperature offset added to every layer base
    ///   temperature ("ISA + dT" day). The lapse rates are unchanged.
    /// - deltaP_sl_Pa: sea-level pressure minus 101 325 Pa (a QNH offset).
    ///
    /// The pressure profile is re-integrated hydrostatically from the shifted
    /// sea-level pressure through the shifted temperature profile, so
    /// dp/dH = -rho g0 holds inside every layer for any offsets, and density
    /// follows from the gas law. Both members zero reproduces the standard
    /// atmosphere bit for bit.
    struct AtmosphereOffsets {
        double deltaT_K     { 0.0 };
        double deltaP_sl_Pa { 0.0 };

        [[nodiscard]] constexpr bool isStandard() const noexcept
        {
            return deltaT_K == 0.0 && deltaP_sl_Pa == 0.0;
        }
    };

    namespace detail {

        /// Layer tables of the US1976 homosphere (0..84.852 km geopotential).
        struct Us1976Layers {
            static constexpr std::array<double, 8> Hb_km = {
                0.0, 11.0, 20.0, 32.0, 47.0, 51.0, 71.0, 84.852 };
            static constexpr std::array<double, 8> Tb_K = {
                288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946 };
            static constexpr std::array<double, 8> Pb_Pa = {
                101325.0, 22632.1, 5474.89, 868.019, 110.906, 66.9389, 3.95642, 0.3734 };
            static constexpr std::array<double, 8> Lb_K_per_km = {
                -6.5, 0.0, 1.0, 2.8, 0.0, -2.8, -2.0, 0.0 };
            static constexpr double g0    = 9.80665;
            static constexpr double R     = 287.05287;
            static constexpr double gamma = 1.4;
            static constexpr double p_sl  = 101325.0;
        };

        /// Walk the layers from sea level and return the base pressure of each
        /// layer for a temperature profile shifted by deltaT_K and a sea-level
        /// pressure p_sl_Pa. Pure double: the offsets are parameters, never AD
        /// variables.
        inline std::array<double, 8> Us1976BasePressures(double deltaT_K, double p_sl_Pa)
        {
            using L = Us1976Layers;
            std::array<double, 8> p{};
            p[0] = p_sl_Pa;
            for (std::size_t b = 0; b + 1 < 8; ++b) {
                const double Tb  = L::Tb_K[b] + deltaT_K;
                const double Lb  = L::Lb_K_per_km[b] * 1.0e-3;               // [K/m]
                const double dH  = (L::Hb_km[b + 1] - L::Hb_km[b]) * 1000.0; // [m]
                if (Lb == 0.0) {
                    p[b + 1] = p[b] * std::exp(-L::g0 * dH / (L::R * Tb));
                } else {
                    const double Tnext = Tb + Lb * dH;
                    p[b + 1] = p[b] * std::pow(Tnext / Tb, -L::g0 / (Lb * L::R));
                }
            }
            return p;
        }

        /// Base pressures actually used by US1976Atmosphere for given offsets.
        ///
        /// The standard table carries the published (rounded) base pressures.
        /// For a non-standard day each base is the published value scaled by the
        /// ratio of the re-integrated base pressure with offsets to the
        /// re-integrated base pressure without, so the profile stays continuous
        /// with the table and reduces to it exactly when the offsets vanish.
        inline std::array<double, 8> Us1976OffsetBasePressures(const AtmosphereOffsets& off)
        {
            using L = Us1976Layers;
            if (off.isStandard()) {
                return L::Pb_Pa;
            }
            // The atmosphere is evaluated once per aero call inside an implicit
            // integrator's Newton iteration, always with the same offsets; a
            // one-entry memo makes the non-standard day as cheap as the table.
            thread_local AtmosphereOffsets      memo_off{};
            thread_local std::array<double, 8>  memo_p = L::Pb_Pa;
            if (memo_off.deltaT_K == off.deltaT_K && memo_off.deltaP_sl_Pa == off.deltaP_sl_Pa
                && !memo_off.isStandard()) {
                return memo_p;
            }
            const auto p_off = Us1976BasePressures(off.deltaT_K, L::p_sl + off.deltaP_sl_Pa);
            const auto p_std = Us1976BasePressures(0.0, L::p_sl);
            std::array<double, 8> out{};
            for (std::size_t b = 0; b < 8; ++b) {
                out[b] = L::Pb_Pa[b] * (p_off[b] / p_std[b]);
            }
            memo_off = off;
            memo_p   = out;
            return out;
        }

    } // namespace detail

    /// U.S. Standard Atmosphere 1976, 0-86 km (geometric altitude).
    /// - Input:  geometric altitude above MSL in meters (approx; internally converted to geopotential).
    /// - Output: temperature T [K], pressure p [Pa], density rho [kg/m^3], speed of sound a [m/s].
    ///
    /// CppAD friendliness:
    /// - Templated on Scalar.
    /// - Only uses elementary functions (no std::isnan, no branching on sign of AD, etc.).
    /// - There *is* piecewise behavior via `if` on altitude; for AD, the tape records the branch
    ///   corresponding to the evaluation altitude. This is fine as long as you don't differentiate
    ///   *through* layer boundaries.
    ///
    /// The two-argument overload applies AtmosphereOffsets (ISA + dT, sea-level
    /// pressure offset); the one-argument form is the standard day.
    template <class Scalar>
    inline Us1976State<Scalar> US1976Atmosphere(const Scalar& altitude_m_in,
                                                const AtmosphereOffsets& offsets)
    {
        using detail::Exponential;
        using detail::Power;
        using detail::SquareRoot;

        // ---- Physical constants (US1976) ---------------------------------------
        const auto g0 = Scalar(9.80665);    // [m/s^2]
        const auto R = Scalar(287.05287);  // [J/(kg·K)] specific gas constant for air
        const auto gamma = Scalar(1.4);       // ratio of specific heats

        // Geopotential reference Earth radius (USSA-76) [m]
        const auto Re = Scalar(6356766.0);

        // ---- Clamp / extend altitude ────────────────────────────────────────────
        // Below 0: clamp to sea level.
        // 0–84.852 km: standard US1976 layers.
        // Above 84.852 km: isothermal exponential decay using the scale height
        //   H_s = R·T_top / g0 ≈ 5 480 m at the model ceiling (T=186.946 K).
        //   Temperature and speed of sound are held at the 84.852 km values so
        //   that Mach number remains well-defined; only pressure and density decay.
        Scalar h = altitude_m_in;
        if (h < Scalar(0)) {
            h = Scalar(0);
        }
        const auto h_max = Scalar(84852.0); // 84.852 km — top of US1976 homosphere
        auto above_decay = Scalar(1.0);     // multiplier applied to p and rho
        if (h > h_max) {
            // Isothermal scale height at T_top = 186.946 K
            const auto H_scale = Scalar(287.05287 * (186.946 + offsets.deltaT_K) / 9.80665); // ≈5 480 m on a standard day
            above_decay = Exponential(-(h - h_max) / H_scale);
            h = h_max;
        }

        // Convert geometric altitude to geopotential altitude H
        // H = Re * h / (Re + h)
        const Scalar H = Re * h / (Re + h);      // [m]
        const Scalar H_km = H * Scalar(1.0e-3);  // [km]

        // ---- US1976 base values per layer (0..7) -------------------------------
        using Layers = detail::Us1976Layers;
        const auto& Hb_km_arr        = Layers::Hb_km;
        const auto& Tb_arr           = Layers::Tb_K;
        const auto& Lb_K_per_km_arr  = Layers::Lb_K_per_km;
        // Base pressures for this day (the published table when offsets are zero).
        const std::array<double, 8> Pb_arr = detail::Us1976OffsetBasePressures(offsets);
        const double dT = offsets.deltaT_K;

        // ---- Find the layer index b such that Hb <= H < H_{b+1} ----------------
        std::size_t b = 0;
        for (; b + 1 < 8; ++b) {
            const auto Hb_next = Scalar(Hb_km_arr[b + 1]);
            if (H_km < Hb_next) {
                break;
            }
        }
        if (b >= 7) {
            b = 7; // Just in case, clamp
        }

        const auto Hb_km = Scalar(Hb_km_arr[b]);
        const Scalar Hb = Hb_km * Scalar(1000.0);   // [m]
        const auto Tb = Scalar(Tb_arr[b] + dT);   // [K], shifted by deltaT_K
        const auto Pb = Scalar(Pb_arr[b]);        // [Pa]
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

        // Apply above-ceiling decay to pressure (temperature held at ceiling value)
        p = p * above_decay;

        // Ideal gas: rho = p / (R T)
        const Scalar rho = p / (R * T);

        // Speed of sound (based on ceiling temperature above 84.852 km)
        const Scalar a = SquareRoot(gamma * R * T);

        Us1976State<Scalar> out{ T, p, rho, a };
        return out;
    }

    /// Standard-day US1976 atmosphere (no offsets).
    template <class Scalar>
    inline Us1976State<Scalar> US1976Atmosphere(const Scalar& altitude_m_in)
    {
        return US1976Atmosphere(altitude_m_in, AtmosphereOffsets{});
    }

} // namespace Aetherion::Environment

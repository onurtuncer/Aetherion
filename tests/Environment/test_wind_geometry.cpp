// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_wind_geometry.cpp
//
// Unit tests for:
//   GeometricAltitude_m   — flattening-corrected altitude from ECI position
//   LinearWindShear       — from_ned() factory + velocity_ecef()
//   detail::MathWrappers  — ArcTangent, ArcTangent2, ArcSine
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Aetherion/Environment/GeometricAltitude.h>
#include <Aetherion/Environment/WindModels.h>
#include <Aetherion/Environment/detail/MathWrappers.h>
#include <Aetherion/Environment/WGS84.h>

#include <Eigen/Dense>
#include <cmath>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
namespace detail = Aetherion::Environment::detail;

constexpr double kPi = 3.14159265358979323846;
constexpr double kA  = Aetherion::Environment::WGS84::kSemiMajorAxis_m;
constexpr double kF  = Aetherion::Environment::WGS84::kFlattening;

// ─────────────────────────────────────────────────────────────────────────────
// GeometricAltitude_m
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("GeometricAltitude_m: exact at equator", "[GeometricAltitude]")
{
    const double h = 10'000.0;
    Eigen::Vector3d r_eci{ kA + h, 0.0, 0.0 };
    // sin_gc = 0 → r_surface = a → altitude = h exactly
    CHECK_THAT(Aetherion::Environment::GeometricAltitude_m(r_eci),
               WithinAbs(h, 1e-3));
}

TEST_CASE("GeometricAltitude_m: exact at North Pole", "[GeometricAltitude]")
{
    const double h = 10'000.0;
    const double b = kA * (1.0 - kF);  // polar radius
    Eigen::Vector3d r_eci{ 0.0, 0.0, b + h };
    // sin_gc = 1 → r_surface = a*(1-f) = b → altitude = h exactly
    CHECK_THAT(Aetherion::Environment::GeometricAltitude_m(r_eci),
               WithinAbs(h, 1e-3));
}

TEST_CASE("GeometricAltitude_m: mid-latitude gives positive altitude", "[GeometricAltitude]")
{
    // Build ECI position consistently with the formula so the result is exact.
    // Geocentric lat = 36 deg, h_geom = 3052 m
    constexpr double lat_gc = 36.0 * kPi / 180.0;
    constexpr double h_geom = 3052.0;
    const double sin_gc  = std::sin(lat_gc);
    const double r_surf  = kA * (1.0 - kF * sin_gc * sin_gc);
    const double r       = r_surf + h_geom;
    Eigen::Vector3d r_eci{ r * std::cos(lat_gc), 0.0, r * std::sin(lat_gc) };

    CHECK_THAT(Aetherion::Environment::GeometricAltitude_m(r_eci),
               WithinAbs(h_geom, 1.0));
}

// ─────────────────────────────────────────────────────────────────────────────
// LinearWindShear::from_ned
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("LinearWindShear::from_ned: NED-to-ECEF rotation at equator lon=0",
          "[LinearWindShear][from_ned]")
{
    // At lat=0, lon=0:
    //   North ECEF: (-sin(0)*cos(0), -sin(0)*sin(0),  cos(0)) = (0, 0, 1)
    //   East  ECEF: (-sin(0),         cos(0),          0)      = (0, 1, 0)
    const auto w = Aetherion::Environment::LinearWindShear::from_ned(
        0.0, 0.0, 0.0, 0.0, /*lat=*/0.0, /*lon=*/0.0);

    CHECK_THAT(w.Nx, WithinAbs( 0.0, 1e-12));
    CHECK_THAT(w.Ny, WithinAbs( 0.0, 1e-12));
    CHECK_THAT(w.Nz, WithinAbs( 1.0, 1e-12));
    CHECK_THAT(w.Ex, WithinAbs( 0.0, 1e-12));
    CHECK_THAT(w.Ey, WithinAbs( 1.0, 1e-12));
    CHECK_THAT(w.Ez, WithinAbs( 0.0, 1e-12));
}

TEST_CASE("LinearWindShear::from_ned: stores gradient and intercept",
          "[LinearWindShear][from_ned]")
{
    const auto w = Aetherion::Environment::LinearWindShear::from_ned(
        0.003, 0.001, 1.0, -6.096, 0.5, 0.3);

    CHECK_THAT(w.grad_N, WithinAbs(0.003,   1e-12));
    CHECK_THAT(w.grad_E, WithinAbs(0.001,   1e-12));
    CHECK_THAT(w.int_N,  WithinAbs(1.0,     1e-12));
    CHECK_THAT(w.int_E,  WithinAbs(-6.096,  1e-12));
}

// ─────────────────────────────────────────────────────────────────────────────
// LinearWindShear::velocity_ecef
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("LinearWindShear::velocity_ecef: constant wind at equator lon=0",
          "[LinearWindShear][velocity_ecef]")
{
    // At equator lon=0: North→(0,0,1), East→(0,1,0)
    // grad=0, int_N=5, int_E=10 → ECEF = [0, 10, 5]
    const auto w = Aetherion::Environment::LinearWindShear::from_ned(
        0.0, 0.0, 5.0, 10.0, 0.0, 0.0);

    // Any equatorial ECI position; altitude doesn't matter when grad=0
    Eigen::Vector3d r_eci{ kA + 5000.0, 0.0, 0.0 };
    const auto v = w.velocity_ecef(r_eci, 0.0);

    CHECK_THAT(v(0), WithinAbs( 0.0, 1e-9));
    CHECK_THAT(v(1), WithinAbs(10.0, 1e-9));
    CHECK_THAT(v(2), WithinAbs( 5.0, 1e-9));
}

TEST_CASE("LinearWindShear::velocity_ecef: wind shear scales with altitude",
          "[LinearWindShear][velocity_ecef]")
{
    // grad_E = 0.003 m/s/m, int_E = 0 → at h=1000m: vE = 3 m/s
    const auto w = Aetherion::Environment::LinearWindShear::from_ned(
        0.0, 0.003, 0.0, 0.0, 0.0, 0.0);

    Eigen::Vector3d r_eci{ kA + 1000.0, 0.0, 0.0 };
    const auto v = w.velocity_ecef(r_eci, 0.0);

    // At equator lon=0, East→(0,1,0): v_y = 3 m/s
    CHECK_THAT(v(1), WithinAbs(3.0, 1e-6));
}

// ─────────────────────────────────────────────────────────────────────────────
// MathWrappers — ArcTangent, ArcTangent2, ArcSine
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("MathWrappers::ArcTangent(double)", "[MathWrappers]")
{
    CHECK_THAT(detail::ArcTangent(1.0),  WithinAbs(kPi / 4.0, 1e-12));
    CHECK_THAT(detail::ArcTangent(0.0),  WithinAbs(0.0, 1e-12));
    CHECK_THAT(detail::ArcTangent(-1.0), WithinAbs(-kPi / 4.0, 1e-12));
}

TEST_CASE("MathWrappers::ArcTangent2(double, double)", "[MathWrappers]")
{
    CHECK_THAT(detail::ArcTangent2(1.0, 1.0),   WithinAbs( kPi / 4.0, 1e-12));
    CHECK_THAT(detail::ArcTangent2(1.0, -1.0),  WithinAbs( 3.0 * kPi / 4.0, 1e-12));
    CHECK_THAT(detail::ArcTangent2(-1.0, 1.0),  WithinAbs(-kPi / 4.0, 1e-12));
    CHECK_THAT(detail::ArcTangent2(0.0, 1.0),   WithinAbs(0.0, 1e-12));
}

TEST_CASE("MathWrappers::ArcSine(double)", "[MathWrappers]")
{
    CHECK_THAT(detail::ArcSine( 1.0), WithinAbs( kPi / 2.0, 1e-12));
    CHECK_THAT(detail::ArcSine( 0.0), WithinAbs( 0.0, 1e-12));
    CHECK_THAT(detail::ArcSine(-1.0), WithinAbs(-kPi / 2.0, 1e-12));
    CHECK_THAT(detail::ArcSine( 0.5), WithinAbs( kPi / 6.0, 1e-12));
}

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// test_WindModels.cpp
//
// Catch2 tests for:
//   FlightDynamics/WindModels.h  (ZeroWind, ConstantECEFWind,
//                                 PowerLawWindShear, GeodesicCallbackWind)
//   FlightDynamics/Policies/AeroPolicies.h  (WindAwareDragPolicy)
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Dense>
#include <cppad/cppad.hpp>
#include <cmath>

#include <Aetherion/FlightDynamics/WindModels.h>
#include <Aetherion/FlightDynamics/GeodesicCallbackWind.h>
#include <Aetherion/FlightDynamics/Policies/AeroPolicies.h>

// Verify all wind models are registered
static_assert(Aetherion::FlightDynamics::is_wind_model_v<Aetherion::FlightDynamics::ZeroWind>);
static_assert(Aetherion::FlightDynamics::is_wind_model_v<Aetherion::FlightDynamics::ConstantECEFWind>);
static_assert(Aetherion::FlightDynamics::is_wind_model_v<Aetherion::FlightDynamics::PowerLawWindShear>);
static_assert(Aetherion::FlightDynamics::is_wind_model_v<Aetherion::FlightDynamics::GeodesicCallbackWind>);
#include <Aetherion/Environment/WGS84.h>
#include <Aetherion/Environment/Atmosphere.h>

namespace {
    using Vec3d  = Eigen::Vector3d;
    using SE3d   = Aetherion::ODE::RKMK::Lie::SE3<double>;
    using Vec6d  = Eigen::Matrix<double, 6, 1>;
    using Mat3d  = Eigen::Matrix3d;
    using Catch::Matchers::WithinAbs;
    using namespace Aetherion::FlightDynamics;
    using namespace Aetherion::Environment;

    // ECI position at lat=0, lon=0, alt=9144 m with R=Identity
    const double kR = WGS84::kSemiMajorAxis_m + 9144.0;
    const Vec3d  kR_ECI{ kR, 0.0, 0.0 };
    const double kOmegaE = WGS84::kRotationRate_rad_s;
}

// =============================================================================
// ZeroWind
// =============================================================================

TEST_CASE("ZeroWind: always returns zero ECEF wind", "[wind][zero]")
{
    ZeroWind w;
    for (double t : {0.0, 15.0, 30.0}) {
        auto v = w.velocity_ecef(kR_ECI, t);
        CHECK(v.isZero());
    }
}

TEST_CASE("ZeroWind: AD<double> evaluates to zero", "[wind][zero][AD]")
{
    using AD = CppAD::AD<double>;
    ZeroWind w;
    Eigen::Matrix<AD,3,1> r{ AD(kR), AD(0), AD(0) };
    auto v = w.velocity_ecef(r, AD(0.0));
    for (int i = 0; i < 3; ++i)
        CHECK(CppAD::Value(v(i)) == 0.0);
}

// =============================================================================
// ConstantECEFWind
// =============================================================================

TEST_CASE("ConstantECEFWind: returns constant ECEF vector regardless of position/time",
    "[wind][constant]")
{
    ConstantECEFWind w(1.0, 2.0, 3.0);
    for (double t : {0.0, 10.0, 30.0}) {
        auto v = w.velocity_ecef(kR_ECI, t);
        CHECK_THAT(v(0), WithinAbs(1.0, 1e-15));
        CHECK_THAT(v(1), WithinAbs(2.0, 1e-15));
        CHECK_THAT(v(2), WithinAbs(3.0, 1e-15));
    }
}

TEST_CASE("ConstantECEFWind::from_ned: east wind at equator gives +y ECEF",
    "[wind][constant]")
{
    // At lat=0, lon=0: East in NED = +y in ECEF
    auto w = ConstantECEFWind::from_ned(0.0, 10.0, 0.0, 0.0, 0.0);
    auto v = w.velocity_ecef(kR_ECI, 0.0);
    CHECK_THAT(v(0), WithinAbs(0.0,  1e-10));
    CHECK_THAT(v(1), WithinAbs(10.0, 1e-10));
    CHECK_THAT(v(2), WithinAbs(0.0,  1e-10));
}

TEST_CASE("ConstantECEFWind: AD<double> returns same value as double",
    "[wind][constant][AD]")
{
    using AD = CppAD::AD<double>;
    ConstantECEFWind w(5.0, -3.0, 1.5);
    Eigen::Matrix<AD,3,1> r{ AD(kR), AD(0), AD(0) };
    auto v = w.velocity_ecef(r, AD(0.0));
    CHECK_THAT(CppAD::Value(v(0)), WithinAbs(5.0,  1e-14));
    CHECK_THAT(CppAD::Value(v(1)), WithinAbs(-3.0, 1e-14));
    CHECK_THAT(CppAD::Value(v(2)), WithinAbs(1.5,  1e-14));
}

// =============================================================================
// PowerLawWindShear
// =============================================================================

TEST_CASE("PowerLawWindShear: at h_ref returns reference wind", "[wind][shear]")
{
    PowerLawWindShear w(0.0, 21.336, 0.0, kR - WGS84::kSemiMajorAxis_m, 4.0/3.0);
    auto v = w.velocity_ecef(kR_ECI, 0.0);
    CHECK_THAT(v(1), WithinAbs(21.336, 1e-10));
}

TEST_CASE("PowerLawWindShear: scales with altitude power law", "[wind][shear]")
{
    const double v_ref  = 21.336;
    const double h_ref  = 9144.0;
    const double n      = 4.0/3.0;
    PowerLawWindShear w(0.0, v_ref, 0.0, h_ref, n);

    const double h_test = 4966.0;
    const Vec3d  r_test{ WGS84::kSemiMajorAxis_m + h_test, 0.0, 0.0 };
    auto v = w.velocity_ecef(r_test, 0.0);

    const double expected = v_ref * std::pow(h_test / h_ref, n);
    CHECK_THAT(v(1), WithinAbs(expected, expected * 1e-9));
}

TEST_CASE("PowerLawWindShear: wind is zero at sea level (h→0)", "[wind][shear]")
{
    PowerLawWindShear w(0.0, 21.336, 0.0, 9144.0, 4.0/3.0);
    const Vec3d r_sl{ WGS84::kSemiMajorAxis_m + 1.0, 0.0, 0.0 }; // near sea level
    auto v = w.velocity_ecef(r_sl, 0.0);
    CHECK(v.norm() < 0.1);  // very small near sea level
}

TEST_CASE("PowerLawWindShear: AD<double> evaluates without error", "[wind][shear][AD]")
{
    using AD = CppAD::AD<double>;
    PowerLawWindShear w(0.0, 21.336, 0.0, 9144.0, 4.0/3.0);
    Eigen::Matrix<AD,3,1> r{ AD(kR), AD(0), AD(0) };
    auto v = w.velocity_ecef(r, AD(0.0));
    CHECK(CppAD::Value(v(1)) > 0.0);
    CHECK_THAT(CppAD::Value(v(1)), WithinAbs(21.336, 1e-6));
}

// =============================================================================
// GeodesicCallbackWind
// =============================================================================

TEST_CASE("GeodesicCallbackWind: callback receives correct geodetic position",
    "[wind][callback]")
{
    double got_lat = -999, got_lon = -999, got_alt = -999, got_t = -999;

    GeodesicCallbackWind w([&](double lat, double lon, double alt, double t) -> Vec3d {
        got_lat = lat; got_lon = lon; got_alt = alt; got_t = t;
        return Vec3d::Zero();
    });

    // Use t=0 so ECI and ECEF are aligned (lon=0 exactly)
    w.velocity_ecef(kR_ECI, 0.0);

    CHECK_THAT(got_lat, WithinAbs(0.0,    1e-6));   // equatorial
    CHECK_THAT(got_lon, WithinAbs(0.0,    1e-6));   // prime meridian at t=0
    CHECK_THAT(got_alt, WithinAbs(9144.0, 1.0));    // ~30 000 ft
    CHECK_THAT(got_t,   WithinAbs(0.0,    1e-12));
}

TEST_CASE("GeodesicCallbackWind: east wind at equator maps to +y ECEF",
    "[wind][callback]")
{
    GeodesicCallbackWind w([](double, double, double, double) -> Vec3d {
        return Vec3d{ 0.0, 6.096, 0.0 };   // 20 ft/s eastward
    });

    auto v_ecef = w.velocity_ecef(kR_ECI, 0.0);
    CHECK_THAT(v_ecef(0), WithinAbs(0.0,   1e-8));
    CHECK_THAT(v_ecef(1), WithinAbs(6.096, 1e-6));
    CHECK_THAT(v_ecef(2), WithinAbs(0.0,   1e-8));
}

TEST_CASE("GeodesicCallbackWind: AD path uses frozen double cache", "[wind][callback][AD]")
{
    using AD = CppAD::AD<double>;

    double expected_y = 0.0;
    GeodesicCallbackWind w([&](double, double, double, double) -> Vec3d {
        return Vec3d{ 0.0, 6.096, 0.0 };
    });

    // First evaluate with double to populate cache
    auto v_d = w.velocity_ecef(kR_ECI, 0.0);
    expected_y = v_d(1);

    // AD path must return the cached value
    Eigen::Matrix<AD,3,1> r{ AD(kR), AD(0), AD(0) };
    auto v_ad = w.velocity_ecef(r, AD(0.0));
    CHECK_THAT(CppAD::Value(v_ad(1)), WithinAbs(expected_y, 1e-12));
}

// =============================================================================
// WindAwareDragPolicy
// =============================================================================

TEST_CASE("WindAwareDragPolicy<ZeroWind>: identical to DragOnlyAeroPolicy",
    "[wind][policy]")
{
    // v_rel is same as DragOnlyAeroPolicy when wind = 0
    const Vec3d  v_surf_body = Mat3d::Identity().transpose() *
        Vec3d(0, 0, kOmegaE).cross(kR_ECI);

    SE3d g(Mat3d::Identity(), kR_ECI);
    Vec6d nu;
    const Vec3d dv(10.0, -8.0, -20.0);
    nu << 0,0,0, v_surf_body(0)+dv(0), v_surf_body(1)+dv(1), v_surf_body(2)+dv(2);

    WindAwareDragPolicy<ZeroWind>  wp{ 0.1, 0.018241, ZeroWind{} };
    DragOnlyAeroPolicy             dp{ 0.1, 0.018241 };

    auto ww = wp(g, nu, 14.59, 0.0);
    auto wd = dp(g, nu, 14.59, 0.0);

    for (int i = 3; i < 6; ++i)
        CHECK_THAT(ww.f(i), WithinAbs(wd.f(i), std::abs(wd.f(i)) * 1e-10 + 1e-20));
}

TEST_CASE("WindAwareDragPolicy<ConstantECEFWind>: TAS equals wind speed at t=0 for co-rotating sphere",
    "[wind][policy]")
{
    // Co-rotating sphere (v_B = v_surface_body), no falling velocity.
    // Wind = 6.096 m/s east (+y ECEF at equator). TAS must equal wind speed.
    const Vec3d v_surf = Mat3d::Identity().transpose() *
        Vec3d(0, 0, kOmegaE).cross(kR_ECI);

    SE3d g(Mat3d::Identity(), kR_ECI);
    Vec6d nu;
    nu << 0,0,0, v_surf(0), v_surf(1), v_surf(2);

    auto wind = ConstantECEFWind::from_ned(0.0, 6.096, 0.0, 0.0, 0.0);
    WindAwareDragPolicy<ConstantECEFWind> wp{ 0.1, 0.018241, wind };
    auto w = wp(g, nu, 14.59, 0.0);

    // Force magnitude → back-calc TAS
    const double rho = US1976Atmosphere(9144.0).rho;
    const double F   = w.f.tail<3>().norm();
    const double TAS = std::sqrt(2.0 * F / (rho * 0.1 * 0.018241));
    CHECK_THAT(TAS, WithinAbs(6.096, 6.096 * 1e-5));
}

TEST_CASE("WindAwareDragPolicy<PowerLawWindShear>: TAS at h_ref equals reference wind",
    "[wind][policy]")
{
    // Co-rotating sphere at initial altitude. Wind at h_ref must equal v_ref.
    const Vec3d v_surf = Mat3d::Identity().transpose() *
        Vec3d(0, 0, kOmegaE).cross(kR_ECI);

    SE3d g(Mat3d::Identity(), kR_ECI);
    Vec6d nu;
    nu << 0,0,0, v_surf(0), v_surf(1), v_surf(2);

    auto shear = PowerLawWindShear::from_ned(0.0, 21.336, 0.0, 0.0, 0.0, 9144.0, 4.0/3.0);
    WindAwareDragPolicy<PowerLawWindShear> wp{ 0.1, 0.018241, shear };
    auto w = wp(g, nu, 14.59, 0.0);

    const double rho = US1976Atmosphere(9144.0).rho;
    const double F   = w.f.tail<3>().norm();
    const double TAS = std::sqrt(2.0 * F / (rho * 0.1 * 0.018241));
    CHECK_THAT(TAS, WithinAbs(21.336, 21.336 * 1e-4));
}

TEST_CASE("WindAwareDragPolicy<GeodesicCallbackWind>: callback-driven wind",
    "[wind][policy][callback]")
{
    // Callback returns 6.096 m/s east — same as ConstantECEFWind scenario.
    const Vec3d v_surf = Mat3d::Identity().transpose() *
        Vec3d(0, 0, kOmegaE).cross(kR_ECI);

    SE3d g(Mat3d::Identity(), kR_ECI);
    Vec6d nu;
    nu << 0,0,0, v_surf(0), v_surf(1), v_surf(2);

    GeodesicCallbackWind cb([](double, double, double, double) -> Vec3d {
        return Vec3d{ 0.0, 6.096, 0.0 };
    });
    WindAwareDragPolicy<GeodesicCallbackWind> wp{ 0.1, 0.018241, std::move(cb) };
    auto w = wp(g, nu, 14.59, 0.0);

    const double rho = US1976Atmosphere(9144.0).rho;
    const double F   = w.f.tail<3>().norm();
    const double TAS = std::sqrt(2.0 * F / (rho * 0.1 * 0.018241));
    CHECK_THAT(TAS, WithinAbs(6.096, 6.096 * 1e-4));
}

TEST_CASE("WindAwareDragPolicy<ZeroWind>: AD<double> evaluates", "[wind][policy][AD]")
{
    using AD    = CppAD::AD<double>;
    using SE3AD = Aetherion::ODE::RKMK::Lie::SE3<AD>;
    using Vec6AD = Eigen::Matrix<AD,6,1>;
    using Mat3AD = Eigen::Matrix<AD,3,3>;
    using Vec3AD = Eigen::Matrix<AD,3,1>;

    const Vec3AD rECI{ AD(kR), AD(0), AD(0) };
    const Vec3AD oE{ AD(0), AD(0), AD(kOmegaE) };
    const Vec3AD v_surf = Mat3AD::Identity().transpose() * oE.cross(rECI);

    SE3AD g(Mat3AD::Identity(), rECI);
    Vec6AD nu;
    nu << AD(0),AD(0),AD(0), v_surf(0)+AD(5.0), v_surf(1), v_surf(2)+AD(-30.0);

    WindAwareDragPolicy<ZeroWind> wp{ 0.1, 0.018241, ZeroWind{} };
    auto w = wp(g, nu, AD(14.59), AD(0.0));
    CHECK(w.f.tail<3>().norm() != AD(0.0));  // drag is non-zero
}

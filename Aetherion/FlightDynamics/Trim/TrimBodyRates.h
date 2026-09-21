// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TrimBodyRates.h
//
// Initial body rates for a level-flight trim — the companion of TrimWeight.h.
//
// Rationale
// ─────────
// "Body rates = 0" at a trim point is meant relative to the local-level frame:
// the pitch and roll attitude are supposed to stay put.  But the local-level
// frame is not fixed to the Earth.  As the vehicle moves over the curved
// surface the local vertical tips forward with it, at the transport rate
//
//   ω_transport (NED) = [  V_E/(N+h),  −V_N/(M+h),  0 ]
//
// so a vehicle with zero Earth-relative body rate is, seen from the local-level
// frame, pitching nose-up at V/(R+h).  That is 0.0016 °/s at the NASA
// Scenario-11 point and 0.0055 °/s at the Mach 2 Scenario-12 point, where the
// stiffness in α is large enough for it to start a 40 ft phugoid.
//
// LevelFlightBodyRates() returns the Earth-relative body rates that cancel it,
// in the form RigidBody::Config::bodyRates expects; BuildInitialStateVector adds
// the Earth's own rotation on top.  The sum reproduces the initial
// bodyAngularRateWrtEi of NASA TM-2015-218675 reference simulation 05 to 2e-6 °/s
// in pitch and 1e-4 °/s in roll.
//
// The same rates belong in TrimInputs::bodyRates.  F16AeroPolicy evaluates its
// damping derivatives at the body rate relative to the air mass — that is,
// relative to the Earth — so in level flight it sees exactly the transport rate,
// and the trim has to balance the same C_mq·q̂ or the vehicle starts with a
// pitching moment.  The term is tiny (~2e-6) but at Mach 2 enough to start a
// 40 ft phugoid.
//
// The vertical component is deliberately left at zero.  The full NED transport
// rate also has a term −V_E tan φ/(N+h) about the vertical, which keeps the
// *heading* constant — a rhumb line.  Wings-level unaccelerated flight follows
// a great circle instead, along which the heading drifts on its own; and the
// term is singular at the poles, where it would start the Scenario-15 vehicle
// (89.95° N, heading east) with a 1.8 °/s yaw rate and no bank to sustain it.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/FlightDynamics/Trim/TrimWeight.h>
#include <Aetherion/RigidBody/BodyRates.h>

#include <cmath>
#include <numbers>

namespace Aetherion::FlightDynamics {

/// @brief Earth-relative body rates that hold pitch and roll attitude fixed with
///        respect to the local-level frame in level flight.
///
/// @param lat_deg     Geodetic latitude [deg].
/// @param alt_m       Ellipsoidal height above the WGS-84 ellipsoid [m].
/// @param vNorth_mps  Earth-relative north velocity [m/s].
/// @param vEast_mps   Earth-relative east velocity [m/s].
/// @param heading_deg ZYX Euler yaw, body → NED [deg].
/// @param pitch_deg   ZYX Euler pitch, body → NED [deg].
/// @param roll_deg    ZYX Euler roll, body → NED [deg].
/// @return Body rates relative to the Earth-fixed frame, in body axes [rad/s].
inline RigidBody::BodyRates LevelFlightBodyRates(double lat_deg, double alt_m,
                                                 double vNorth_mps, double vEast_mps,
                                                 double heading_deg, double pitch_deg,
                                                 double roll_deg)
{
    constexpr double kDegRad = std::numbers::pi / 180.0;

    const auto [M, N] = WGS84CurvatureRadii(lat_deg * kDegRad);

    // Transport rate in NED, horizontal components only — see the file header
    const double wN =  vEast_mps  / (N + alt_m);
    const double wE = -vNorth_mps / (M + alt_m);

    // NED → body, ZYX Euler sequence
    const double cPsi = std::cos(heading_deg * kDegRad), sPsi = std::sin(heading_deg * kDegRad);
    const double cThe = std::cos(pitch_deg   * kDegRad), sThe = std::sin(pitch_deg   * kDegRad);
    const double cPhi = std::cos(roll_deg    * kDegRad), sPhi = std::sin(roll_deg    * kDegRad);

    const double x1 =  cPsi * wN + sPsi * wE;   // after yaw
    const double y1 = -sPsi * wN + cPsi * wE;

    const double x2 = cThe * x1;                // after pitch (no vertical component)
    const double z2 = sThe * x1;

    return { x2, cPhi * y1 + sPhi * z2, -sPhi * y1 + cPhi * z2 };
}

} // namespace Aetherion::FlightDynamics

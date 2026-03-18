// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "Snapshot1.h"
#include <Aetherion/RigidBody/State.h>

namespace Aetherion::Simulation {

    // ─────────────────────────────────────────────────────────────────────────────
    // MakeSnapshot1
    //
    // Converts a RigidBody::StateD (SE(3) × R^7) together with the simulation
    // time and the Greenwich Sidereal Angle into the flat Snapshot1 struct that
    // matches the NASA TM-2015-218675 Atmos_01 CSV columns.
    //
    // Parameters
    //   t         – simulation time                           [s]
    //   s         – rigid-body state  (s.g = SE3, s.nu_B = [ω_B; v_B])
    //   theta_gst – Earth rotation angle  (ECI → ECEF about +Z)  [rad]
    // ─────────────────────────────────────────────────────────────────────────────
    [[nodiscard]]
    Snapshot1 MakeSnapshot1(
        double                       t,
        const RigidBody::StateD& s,
        double                       theta_gst);

} // namespace Aetherion::Simulation
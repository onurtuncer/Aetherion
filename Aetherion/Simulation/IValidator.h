// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

// -----------------------------------------------------------------------
// NASA TM-2015-218675 required tolerances (Check Case 1)
// -----------------------------------------------------------------------
static constexpr double kNasaTolerancePosition_m = 1.0;    // [m]
static constexpr double kNasaToleranceSpeed_mps = 0.01;   // [m/s]

// True iff current state is within NASA TM-2015-218675 tolerances
[[nodiscard]] bool within_nasa_tolerances() const noexcept
{
    return radius_error_m() < kNasaTolerancePosition_m &&
        speed_error_mps() < kNasaToleranceSpeed_mps;
}
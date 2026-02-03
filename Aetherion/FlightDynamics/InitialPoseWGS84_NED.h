// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::FlightDynamics {

    struct InitialPoseWGS84_NED {


         // --- Geodetic WGS84/Local NED (JSON input convenience) ---
        double lat_deg{ 0.0 };
        double lon_deg{ 0.0 };
        double alt_m{ 0.0 };
        double azimuth_deg{ 0.0 };
        double zenith_deg{ 0.0 };
        double roll_deg{ 0.0 };
    };


} // namespace Aetherion::FlightDynamics
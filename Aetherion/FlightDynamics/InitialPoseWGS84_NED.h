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

        // double t0{ 0.0 }; //TODO maybe delete this here!

         // --- Geodetic WGS84/Local NED (JSON input convenience) ---
        double lat_deg{ 0.0 };
        double lon_deg{ 0.0 };
        double alt_m{ 0.0 };
        double azimuth_deg{ 0.0 };
        double zenith_deg{ 0.0 };
        double roll_deg{ 0.0 };

        // --- Expanded initial conditions used by the dynamics ---
        //   Vec3     pW{ 0,0,0 };        // ECI position
        //   QuatWxyz qWB{ 1,0,0,0 };     // body->ECI quaternion

        //  Vec3 omegaB{ 0,0,0 };
        //  Vec3 vB{ 0,0,0 };
        // double m{ 1.0 };
    };


} // namespace Aetherion::FlightDynamics
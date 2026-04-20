// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::RigidBody {

/// @brief Initial geodetic pose of the rigid body in the WGS-84 / local-NED convention.
///
/// Stores the launch-site position on the WGS-84 ellipsoid together with the
/// initial pointing direction (azimuth / zenith) and roll angle. These values
/// are the JSON-level input convenience representation; they are converted to
/// ECI/NED coordinates before integration begins via
/// Aetherion::Coordinate::MakeLaunchStateECI().
struct GeodeticPoseNED {

    /// @name Position — WGS-84 geodetic coordinates
    ///@{
    double lat_deg{ 0.0 };      ///< Geodetic latitude [deg]; positive North.
    double lon_deg{ 0.0 };      ///< Geodetic longitude [deg]; positive East.
    double alt_m{ 0.0 };        ///< Altitude above the WGS-84 ellipsoid [m].
    ///@}

    /// @name Attitude — NED pointing angles
    ///@{
    double azimuth_deg{ 0.0 };  ///< Azimuth of the body x-axis measured clockwise from North [deg].
    double zenith_deg{ 0.0 };   ///< Zenith angle of the body x-axis from the local vertical [deg]; 0 = straight up.
    double roll_deg{ 0.0 };     ///< Roll about the body x-axis [deg].
    ///@}
};


} // namespace Aetherion::RigidBody

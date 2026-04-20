// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <array>
#include <cmath>

#include "Aetherion/Coordinate/Math.h"

namespace Aetherion::Coordinate {

     // -------------------------------------------------------------------------
    // Output type: local NED azimuth, zenith, roll (all in radians)
    // -------------------------------------------------------------------------
/// @brief Local NED orientation angles (azimuth, zenith, roll).
///
/// All angles in radians.  Frame conventions:
/// - NED is right-handed: x = North, y = East, z = Down.
/// - Body: x forward, y right, z down.
    template <class Scalar>
    struct LocalOrientationNED {
        Scalar azimuth; ///< Angle from North toward East in the horizontal plane [rad, −π … π].
        Scalar zenith;  ///< Angle from local Up (0) toward body forward (π) [rad, 0 … π].
        Scalar roll;    ///< Rotation about body forward axis @f$x_B@f$ [rad].
    };

/// @brief Decompose body attitude into local NED azimuth, zenith, and roll angles.
///
/// Computes @f$R_{NB} = R_{IN}^\top\,R_{IB}@f$ (body → NED DCM) and extracts
/// the standard yaw–pitch–roll Euler angles using a 3-2-1 (ψ, θ, φ) sequence.
///
/// @note AD-friendly — uses only @c +, @c -, @c *, @c /, @c sqrt, @c sin,
///       @c cos, @c atan2, @c acos.
///
/// @param q_body_to_inertial  Quaternion rotating body frame into ECI frame.
/// @param q_ned_to_inertial   Quaternion rotating local NED frame into ECI frame.
/// @return                    @c LocalOrientationNED with azimuth, zenith, roll in radians.
    template <class Scalar>
    inline LocalOrientationNED<Scalar>
        QuaternionToAzZenRollNED(const Quat<Scalar>& q_body_to_inertial,
            const Quat<Scalar>& q_ned_to_inertial)
    {
        using detail::QuaternionToRotationMatrix;
        using detail::ArcTangent2;
        using detail::ArcCos;

        // R_IB: body -> inertial
        const Mat3<Scalar> R_IB = QuaternionToRotationMatrix(q_body_to_inertial);

        // R_IN: NED -> inertial
        const Mat3<Scalar> R_IN = QuaternionToRotationMatrix(q_ned_to_inertial);

        // Compute R_NB: body -> NED
        // v^I = R_IB * v^B
        // v^I = R_IN * v^N => v^N = R_IN^T * v^I
        // => v^N = (R_IN^T * R_IB) * v^B
        Mat3<Scalar> R_NB{}; // row-major

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                Scalar sum = Scalar(0);
                for (int k = 0; k < 3; ++k) {
                    // R_IN^T(r,k) = R_IN(k,r)
                    sum += R_IN[3 * k + r] * R_IB[3 * k + c];
                }
                R_NB[3 * r + c] = sum;
            }
        }

        // ---------------------------------------------------------------------
        // Direction of body forward axis (x_B) expressed in NED.
        // Since R_NB is body -> NED, the forward direction is the first column.
        // ---------------------------------------------------------------------
        const Scalar fN = R_NB[0]; // row 0, col 0
        const Scalar fE = R_NB[3]; // row 1, col 0
        const Scalar fD = R_NB[6]; // row 2, col 0

        // Azimuth: angle in horizontal plane from North toward East
        const Scalar azimuth = ArcTangent2(fE, fN);

        // Zenith: angle from local Up (0,0,-1) toward the direction vector.
        // Up · f = cos(zenith) => (0,0,-1)·(fN,fE,fD) = -fD
        const Scalar cos_zenith = -fD;  // assuming perfectly normalized rotation
        const Scalar zenith = ArcCos(cos_zenith);

        // ---------------------------------------------------------------------
        // Roll: rotation about forward axis x_B.
        //
        // Extract via standard NED 3-2-1 (yaw-pitch-roll) decomposition:
        //   R_BN = R_NB^T, nav->body DCM.
        //
        // For R_BN with NED and yaw-pitch-roll (psi, theta, phi):
        //   theta = asin(-R_BN(2,0))
        //   phi   = atan2(-R_BN(2,1), R_BN(2,2))
        //   psi   = atan2( R_BN(1,0), R_BN(0,0))
        // ---------------------------------------------------------------------
        Mat3<Scalar> R_BN{};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R_BN[3 * r + c] = R_NB[3 * c + r]; // transpose
            }
        }

        const Scalar roll = ArcTangent2(
            -R_BN[3 * 2 + 1], // -R_BN(2,1)
            R_BN[3 * 2 + 2]  //  R_BN(2,2)
        );

        return LocalOrientationNED<Scalar>{ azimuth, zenith, roll };
    }

} // namespace Aetherion::Coordinate

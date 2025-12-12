#pragma once

#include <array>
#include <cmath>

#include "Aetherion/Coordinate/Math.h"

namespace Aetherion::Coordinate {

      // -------------------------------------------------------------------------
    // Output type: local NED azimuth, zenith, roll (all in radians)
    // -------------------------------------------------------------------------
    template <class Scalar>
    struct LocalOrientationNED {
        Scalar azimuth; // angle in horizontal plane, from North toward East
        Scalar zenith;  // angle from local Up toward body forward
        Scalar roll;    // rotation about body x_B (forward) axis
    };

    // -------------------------------------------------------------------------
    // QuaternionToAzZenRollNED
    //
    // Inputs:
    //   q_body_to_inertial : body -> inertial attitude quaternion
    //   q_ned_to_inertial  : NED  -> inertial attitude quaternion
    //
    // Conventions:
    //   - NED is right-handed: x=N, y=E, z=D.
    //   - Body: x forward, y right, z down.
    //
    // Returns:
    //   LocalOrientationNED<Scalar> with:
    //     azimuth : angle from North toward East (rad, [-pi, pi])
    //     zenith  : angle from Up (0) down to direction (pi) (rad, [0, pi])
    //     roll    : body roll about x_B (rad)
    //
    // This is AD-friendly: only uses +, -, *, /, sqrt, sin, cos, atan2, acos.
    // -------------------------------------------------------------------------
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

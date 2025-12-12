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

namespace Aetherion::Coordinate {

    template <class Scalar>
    using Vec3 = std::array<Scalar, 3>;

    template <class Scalar>
    using Quat = std::array<Scalar, 4>; // [w, x, y, z], body->ECI

    template <class Scalar>
    using Mat3 = std::array<Scalar, 9>; // row-major 3x3


    namespace detail {

        template <class S>
        inline S Dot(const Vec3<S>& a, const Vec3<S>& b) {
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        }

        // NED [N,E,D] -> NEU [N,E,U] where U = -D
        template <class S>
        inline Vec3<S> NEDToNEU(const Vec3<S>& v_ned) {
            return Vec3<S>{ v_ned[0], v_ned[1], -v_ned[2] };
        }

        // (optional but often handy)
        template <class S>
        inline Vec3<S> NEUToNED(const Vec3<S>& v_neu) {
            return Vec3<S>{ v_neu[0], v_neu[1], -v_neu[2] };
        }

        template <class S>
        inline S Sine(const S& x) {
            using std::sin;
            return sin(x);
        }

        template <class S>
        inline S Cosine(const S& x) {
            using std::cos;
            return cos(x);
        }

        template <class S>
        inline S ArcTangent2(const S& y, const S& x) {
            using std::atan2;
            return atan2(y, x);
        }

        template <class S>
        inline S ArcCos(const S& x) {
            using std::acos;
            return acos(x);
        }

        template <class S>
        inline S SquareRoot(const S& x) {
            using std::sqrt;
            return sqrt(x);
        }

        template <class S>
        inline Vec3<S> Cross(const Vec3<S>& a, const Vec3<S>& b) {
            return Vec3<S>{
                a[1] * b[2] - a[2] * b[1],
                    a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0]
            };
        }

        template <class S>
        inline Vec3<S> Normalize(const Vec3<S>& v) {
            const S n2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            const S inv_n = S(1) / SquareRoot(n2);
            return Vec3<S>{ v[0] * inv_n, v[1] * inv_n, v[2] * inv_n };
        }

        /// Convert rotation matrix (columns = body axes in some world frame)
        /// to quaternion q_WB (body->world), robust for all rotations.
        template <class S>
        inline Quat<S> RotationMatrixToQuaternion(
            const Vec3<S>& x_b, // column 0
            const Vec3<S>& y_b, // column 1
            const Vec3<S>& z_b  // column 2
        )
        {
            using detail::SquareRoot;

            const S m00 = x_b[0], m01 = y_b[0], m02 = z_b[0];
            const S m10 = x_b[1], m11 = y_b[1], m12 = z_b[1];
            const S m20 = x_b[2], m21 = y_b[2], m22 = z_b[2];

            const S one = S(1);

            const S trace = m00 + m11 + m22;

            S w, x, y, z;

            if (trace > S(0)) {
                // trace is positive: use w as the primary
                const S S4 = SquareRoot(trace + one) * S(2); // 4*w
                w = S4 * S(0.25);
                x = (m21 - m12) / S4;
                y = (m02 - m20) / S4;
                z = (m10 - m01) / S4;
            }
            else if (m00 > m11 && m00 > m22) {
                // m00 is largest on diagonal: use x as the primary
                const S S4 = SquareRoot(one + m00 - m11 - m22) * S(2); // 4*x
                w = (m21 - m12) / S4;
                x = S4 * S(0.25);
                y = (m01 + m10) / S4;
                z = (m02 + m20) / S4;
            }
            else if (m11 > m22) {
                // m11 is largest: use y as the primary
                const S S4 = SquareRoot(one + m11 - m00 - m22) * S(2); // 4*y
                w = (m02 - m20) / S4;
                x = (m01 + m10) / S4;
                y = S4 * S(0.25);
                z = (m12 + m21) / S4;
            }
            else {
                // m22 is largest: use z as the primary
                const S S4 = SquareRoot(one + m22 - m00 - m11) * S(2); // 4*z
                w = (m10 - m01) / S4;
                x = (m02 + m20) / S4;
                y = (m12 + m21) / S4;
                z = S4 * S(0.25);
            }

            // Final normalize to clean up numerical drift
            const S norm2 = w * w + x * x + y * y + z * z;
            const S inv_n = one / SquareRoot(norm2);

            return Quat<S>{ w* inv_n, x* inv_n, y* inv_n, z* inv_n };
        }

        // Quaternion -> 3x3 rotation matrix (row-major), AD-friendly.
      // q_AB: rotates frame B into frame A, v^A = R(q_AB) v^B.
        template <class S>
        inline Mat3<S> QuaternionToRotationMatrix(const Quat<S>& q_in) {
            S w = q_in[0];
            S x = q_in[1];
            S y = q_in[2];
            S z = q_in[3];

            // Normalize to guard against drift (smooth and AD-friendly).
            const S n2 = w * w + x * x + y * y + z * z;
            const S inv_n = S(1) / SquareRoot(n2);
            w *= inv_n;
            x *= inv_n;
            y *= inv_n;
            z *= inv_n;

            const S two = S(2);

            const S xx = x * x;
            const S yy = y * y;
            const S zz = z * z;
            const S xy = x * y;
            const S xz = x * z;
            const S yz = y * z;
            const S wx = w * x;
            const S wy = w * y;
            const S wz = w * z;

            Mat3<S> R{};

            // Row 0
            R[0] = S(1) - two * (yy + zz);
            R[1] = two * (xy - wz);
            R[2] = two * (xz + wy);

            // Row 1
            R[3] = two * (xy + wz);
            R[4] = S(1) - two * (xx + zz);
            R[5] = two * (yz - wx);

            // Row 2
            R[6] = two * (xz - wy);
            R[7] = two * (yz + wx);
            R[8] = S(1) - two * (xx + yy);

            return R;
        }

    } // namespace detail

} // namespace Aetherion::Coordinate
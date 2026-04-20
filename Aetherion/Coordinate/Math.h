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

/// @brief 3-element column vector (CppAD-friendly, no Eigen dependency).
    template <class Scalar>
    using Vec3 = std::array<Scalar, 3>;

/// @brief Unit quaternion @f$[w,\,x,\,y,\,z]@f$ — body → ECI convention.
    template <class Scalar>
    using Quat = std::array<Scalar, 4>; // [w, x, y, z], body->ECI

/// @brief Row-major 3×3 rotation matrix stored as a flat 9-element array.
    template <class Scalar>
    using Mat3 = std::array<Scalar, 9>; // row-major 3x3


    namespace detail {

        /// @brief Dot product of two 3-vectors.
        template <class S>
        inline S Dot(const Vec3<S>& a, const Vec3<S>& b) {
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        }

        /// @brief Convert NED @f$[N,E,D]@f$ to NEU @f$[N,E,U]@f$ (negate Down).
        template <class S>
        inline Vec3<S> NEDToNEU(const Vec3<S>& v_ned) {
            return Vec3<S>{ v_ned[0], v_ned[1], -v_ned[2] };
        }

        /// @brief Convert NEU @f$[N,E,U]@f$ to NED @f$[N,E,D]@f$ (negate Up).
        template <class S>
        inline Vec3<S> NEUToNED(const Vec3<S>& v_neu) {
            return Vec3<S>{ v_neu[0], v_neu[1], -v_neu[2] };
        }

        /// @brief @c std::sin wrapper — enables ADL for CppAD scalars.
        template <class S>
        inline S Sine(const S& x) {
            using std::sin;
            return sin(x);
        }

        /// @brief @c std::cos wrapper — enables ADL for CppAD scalars.
        template <class S>
        inline S Cosine(const S& x) {
            using std::cos;
            return cos(x);
        }

        /// @brief @c std::atan2 wrapper — enables ADL for CppAD scalars.
        template <class S>
        inline S ArcTangent2(const S& y, const S& x) {
            using std::atan2;
            return atan2(y, x);
        }

        /// @brief @c std::acos wrapper — enables ADL for CppAD scalars.
        template <class S>
        inline S ArcCos(const S& x) {
            using std::acos;
            return acos(x);
        }

        /// @brief @c std::sqrt wrapper — enables ADL for CppAD scalars.
        template <class S>
        inline S SquareRoot(const S& x) {
            using std::sqrt;
            return sqrt(x);
        }

        /// @brief Cross product @f$\mathbf{a} \times \mathbf{b}@f$.
        template <class S>
        inline Vec3<S> Cross(const Vec3<S>& a, const Vec3<S>& b) {
            return Vec3<S>{
                a[1] * b[2] - a[2] * b[1],
                    a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0]
            };
        }

        /// @brief Normalise a 3-vector to unit length.
        template <class S>
        inline Vec3<S> Normalize(const Vec3<S>& v) {
            const S n2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            const S inv_n = S(1) / SquareRoot(n2);
            return Vec3<S>{ v[0] * inv_n, v[1] * inv_n, v[2] * inv_n };
        }

        /// @brief Convert a rotation matrix (given as three column vectors) to a unit quaternion.
        ///
        /// Uses the numerically robust Shepperd method to avoid near-zero denominators.
        /// The input columns are the body-frame axes expressed in the world frame,
        /// so the quaternion represents the body → world rotation @f$q_{WB}@f$.
        ///
        /// @param x_b  First column (body x-axis in world frame).
        /// @param y_b  Second column (body y-axis in world frame).
        /// @param z_b  Third column (body z-axis in world frame).
        /// @return     Unit quaternion @f$[w,\,x,\,y,\,z]@f$ (normalised).
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

        /// @brief Convert a unit quaternion to a row-major 3×3 rotation matrix.
        ///
        /// @f$ v^A = R(q_{AB})\, v^B @f$ — i.e. @c q_in rotates vectors from
        /// frame B into frame A.  The quaternion is normalised internally to
        /// guard against accumulated drift (branch-free, CppAD-friendly).
        ///
        /// @param q_in  Quaternion @f$[w,\,x,\,y,\,z]@f$.
        /// @return      Row-major 3×3 rotation matrix.
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

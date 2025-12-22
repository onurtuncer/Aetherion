// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Eigen interop helpers for Lie::SE3-like types.
//
// Provides:
//   - get_q(g): Eigen::Quaternion<Scalar>
//   - get_p(g): Eigen::Matrix<Scalar,3,1>
//   - to_qp(g): (Quaternion, translation) pair
//
// Expectations / supported SE3 APIs:
//   Quaternion extraction (first match wins):
//     - g.q() / g.rotation()
//     - g.q / g.rotation  (data member)
//   Translation extraction (first match wins):
//     - g.p()             -> 3x1 vector-like
//     - g.translation()   -> 3x1 vector-like
//     - g.p(i)            -> indexable (0..2)
//     - g.translation(i)  -> indexable (0..2)
//     - g.p / g.translation (data member) -> 3x1 vector-like
// ------------------------------------------------------------------------------

#pragma once

#include <type_traits>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Aetherion::ODE::RKMK::Lie {

    namespace detail {

        template<class...>
        inline constexpr bool always_false_v = false;

        // Cast any quaternion-like object (with w/x/y/z) into Eigen::Quaternion<Scalar>.
        template<class Scalar, class QLike>
        [[nodiscard]] inline Eigen::Quaternion<Scalar> cast_quat(const QLike& q) {
            return Eigen::Quaternion<Scalar>(
                static_cast<Scalar>(q.w()),
                static_cast<Scalar>(q.x()),
                static_cast<Scalar>(q.y()),
                static_cast<Scalar>(q.z())
            );
        }

        // Cast any vector-like object with operator()(i) or operator[](i) into Eigen::Matrix<Scalar,3,1>.
        template<class Scalar, class VLike>
        [[nodiscard]] inline Eigen::Matrix<Scalar, 3, 1> cast_vec3(const VLike& v) {
            if constexpr (requires { v(0); v(1); v(2); }) {
                return { static_cast<Scalar>(v(0)), static_cast<Scalar>(v(1)), static_cast<Scalar>(v(2)) };
            }
            else if constexpr (requires { v[0]; v[1]; v[2]; }) {
                return { static_cast<Scalar>(v[0]), static_cast<Scalar>(v[1]), static_cast<Scalar>(v[2]) };
            }
            else {
                static_assert(always_false_v<VLike>, "Cannot cast translation vector to Eigen::Vector3.");
            }
        }

        // Must appear BEFORE extract_q_double/extract_p_double
        template<class QLike>
        [[nodiscard]] inline Eigen::Quaterniond cast_quat_double(const QLike& q) {
            return Eigen::Quaterniond(
                static_cast<double>(q.w()),
                static_cast<double>(q.x()),
                static_cast<double>(q.y()),
                static_cast<double>(q.z())
            );
        }

        // Must appear BEFORE extract_q_double/extract_p_double
        template<class VLike>
        [[nodiscard]] inline Eigen::Vector3d cast_vec3_double(const VLike& v) {
            if constexpr (requires { v(0); v(1); v(2); }) {
                return Eigen::Vector3d(
                    static_cast<double>(v(0)),
                    static_cast<double>(v(1)),
                    static_cast<double>(v(2))
                );
            }
            else if constexpr (requires { v[0]; v[1]; v[2]; }) {
                return Eigen::Vector3d(
                    static_cast<double>(v[0]),
                    static_cast<double>(v[1]),
                    static_cast<double>(v[2])
                );
            }
            else {
                static_assert(always_false_v<VLike>, "Cannot cast translation vector to Eigen::Vector3d.");
            }
        }

        template<class G>
        [[nodiscard]] inline Eigen::Quaterniond extract_q_double(const G& g) {
            if constexpr (requires { g.q(); })          return cast_quat_double(g.q());
            else if constexpr (requires { g.rotation(); }) return cast_quat_double(g.rotation());
            else if constexpr (requires { g.q; })       return cast_quat_double(g.q);
            else if constexpr (requires { g.rotation; })   return cast_quat_double(g.rotation);
            else static_assert(always_false_v<G>, "Cannot extract quaternion (need q()/rotation()).");
        }

        template<class G>
        [[nodiscard]] inline Eigen::Vector3d extract_p_double(const G& g) {
            if constexpr (requires { g.p(); })              return cast_vec3_double(g.p());
            else if constexpr (requires { g.translation(); }) return cast_vec3_double(g.translation());
            else if constexpr (requires { g.p(0); g.p(1); g.p(2); }) {
                return { static_cast<double>(g.p(0)), static_cast<double>(g.p(1)), static_cast<double>(g.p(2)) };
            }
            else if constexpr (requires { g.translation(0); g.translation(1); g.translation(2); }) {
                return { static_cast<double>(g.translation(0)), static_cast<double>(g.translation(1)), static_cast<double>(g.translation(2)) };
            }
            else if constexpr (requires { g.p; })         return cast_vec3_double(g.p);
            else if constexpr (requires { g.translation; }) return cast_vec3_double(g.translation);
            else static_assert(always_false_v<G>, "Cannot extract translation (need p()/translation()).");
        }

    } // namespace detail

    // ------------------------------------------------------------
    // get_q
    // ------------------------------------------------------------
    template<class G, class Scalar = double>
    [[nodiscard]] inline Eigen::Quaternion<Scalar> get_q(const G& g) {
        if constexpr (requires { g.q(); }) {
            return detail::cast_quat<Scalar>(g.q());
        }
        else if constexpr (requires { g.rotation(); }) {
            return detail::cast_quat<Scalar>(g.rotation());
        }
        else if constexpr (requires { g.q; }) {
            return detail::cast_quat<Scalar>(g.q);
        }
        else if constexpr (requires { g.rotation; }) {
            return detail::cast_quat<Scalar>(g.rotation);
        }
        else {
            static_assert(detail::always_false_v<G>, "Cannot extract quaternion from SE3-like type (missing q()/rotation()).");
        }
    }

    // ------------------------------------------------------------
    // get_p
    // ------------------------------------------------------------
    template<class G, class Scalar = double>
    [[nodiscard]] inline Eigen::Matrix<Scalar, 3, 1> get_p(const G& g) {
        if constexpr (requires { g.p(); }) {
            return detail::cast_vec3<Scalar>(g.p());
        }
        else if constexpr (requires { g.translation(); }) {
            return detail::cast_vec3<Scalar>(g.translation());
        }
        else if constexpr (requires { g.p(0); g.p(1); g.p(2); }) {
            return {
                static_cast<Scalar>(g.p(0)),
                static_cast<Scalar>(g.p(1)),
                static_cast<Scalar>(g.p(2))
            };
        }
        else if constexpr (requires { g.translation(0); g.translation(1); g.translation(2); }) {
            return {
                static_cast<Scalar>(g.translation(0)),
                static_cast<Scalar>(g.translation(1)),
                static_cast<Scalar>(g.translation(2))
            };
        }
        else if constexpr (requires { g.p; }) {
            return detail::cast_vec3<Scalar>(g.p);
        }
        else if constexpr (requires { g.translation; }) {
            return detail::cast_vec3<Scalar>(g.translation);
        }
        else {
            static_assert(detail::always_false_v<G>, "Cannot extract translation from SE3-like type (missing p()/translation()).");
        }
    }

    // ------------------------------------------------------------
    // Convenience bundle (matches your test usage pattern)
    // ------------------------------------------------------------
    template<class Scalar = double>
    struct QP {
        Eigen::Quaternion<Scalar> q;
        Eigen::Matrix<Scalar, 3, 1> p;
    };

    template<class G, class Scalar = double>
    [[nodiscard]] inline QP<Scalar> to_qp(const G& g) {
        auto q = get_q<G, Scalar>(g);
        q.normalize();
        const auto p = get_p<G, Scalar>(g);
        return { q, p };
    }

} // namespace Aetherion::ODE::RKMK::Lie

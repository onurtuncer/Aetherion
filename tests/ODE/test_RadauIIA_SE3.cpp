// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Catch2 tests for: Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_SE3.h
//
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <Aetherion/ODE/RKMK/Core/StagePack.h>
#include <Aetherion/ODE/RKMK/Lie/SE3.h>
#include <Aetherion/ODE/RKMK/Lie/SE3EigenInterop.h>
#include <Aetherion/ODE/RKMK/Integrators/RadauIIA_RKMK_SE3.h>

namespace {
    using Catch::Approx;

    namespace Core = Aetherion::ODE::RKMK::Core;
    namespace Lie = Aetherion::ODE::RKMK::Lie;
    namespace Int = Aetherion::ODE::RKMK::Integrators;

    template<class...>
    inline constexpr bool always_false_v = false;

    template<class S>
    using Vec6 = Eigen::Matrix<S, 6, 1>;

  
   template<class S>
    [[nodiscard]] Lie::SE3<S> make_se3(const Eigen::Quaternion<S>& q,
        const Eigen::Matrix<S, 3, 1>& p) {
        using G = Lie::SE3<S>;
        if constexpr (requires { G{ q, p }; }) return G{ q, p };
        if constexpr (requires { G(q, p); }) return G(q, p);
        if constexpr (requires { G::FromQuatTranslation(q, p); }) return G::FromQuatTranslation(q, p);
        if constexpr (requires { G::from_quat_translation(q, p); }) return G::from_quat_translation(q, p);
        static_assert(always_false_v<S>, "Cannot construct SE3 from (quat, p).");
    } 

    template<class G>
    [[nodiscard]] G compose(const G& a, const G& b) {
        if constexpr (requires { a* b; }) return a * b;
        if constexpr (requires { a.compose(b); }) return a.compose(b);
        if constexpr (requires { G::compose(a, b); }) return G::compose(a, b);
        static_assert(always_false_v<G>, "SE3 must support composition.");
    }

    template<class S>
    [[nodiscard]] Lie::SE3<S> exp_se3(const Vec6<S>& xi) {
        using G = Lie::SE3<S>;
        if constexpr (requires { G::Exp(xi); }) return G::Exp(xi);
        if constexpr (requires { G::exp(xi); }) return G::exp(xi);
        static_assert(always_false_v<S>, "SE3 must provide Exp/exp.");
    } 

    template<class G>
    [[nodiscard]] Eigen::Quaterniond get_q(const G& g) {
        if constexpr (requires { g.q(); }) {
            const auto q = g.q();
            return Eigen::Quaterniond((double)q.w(), (double)q.x(), (double)q.y(), (double)q.z());
        }
        if constexpr (requires { g.rotation(); }) {
            const auto q = g.rotation();
            return Eigen::Quaterniond((double)q.w(), (double)q.x(), (double)q.y(), (double)q.z());
        }
        if constexpr (requires { g.q; }) {
            const auto q = g.q;
            return Eigen::Quaterniond((double)q.w(), (double)q.x(), (double)q.y(), (double)q.z());
        }
        static_assert(always_false_v<G>, "Cannot extract quaternion.");
    } 


       inline double so3_angle_error(const Eigen::Quaterniond& qa, const Eigen::Quaterniond& qb) {
        // relative rotation q_rel = qa * qb^{-1}
        Eigen::Quaterniond q_rel = qa * qb.conjugate();
        q_rel.normalize();

        // Quaternion double-cover: q and -q represent same rotation
        const double w = std::clamp(std::abs(q_rel.w()), 0.0, 1.0);
        return 2.0 * std::acos(w);
    }

   
    template<class G>
    [[nodiscard]] Eigen::Matrix4d to_T(const G& g) {
        Eigen::Quaterniond q = get_q(g);
        q.normalize();
        const Eigen::Matrix3d R = q.toRotationMatrix();
        const Eigen::Vector3d p = get_p(g);

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = p;
        return T;
    } 

    struct ConstantTwistField final {
        Vec6<double> xi = Vec6<double>::Zero();

        template<class S>
        Vec6<S> operator()(S /*t*/, const Lie::SE3<S>& /*g*/) const {
            return xi.template cast<S>();
        }
    };
}

TEST_CASE("RadauIIA_RKMK_SE3 constant twist matches g0*Exp(h*xi)", "[rkmk][radau][se3]") {
    using SE3d = Lie::SE3<double>;

    // Initial pose
    const Eigen::Vector3d axis(0.3, -0.2, 0.15);
    const double theta = axis.norm();
    Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    if (theta > 0.0) q0 = Eigen::AngleAxisd(theta, axis / theta);

    const Eigen::Vector3d p0(1.0, -2.0, 0.5);
    const SE3d g0 = Lie::SE3<double>(q0, p0);

    ConstantTwistField f;
    f.xi << 0.8, -0.1, 0.25, 1.2, -0.7, 0.4;

    const double t0 = 0.0;
    const double h = 0.1;

    Core::NewtonOptions opt;
    opt.max_iters = 30;
    opt.throw_on_fail = true;

    Int::RadauIIA_RKMK_SE3<ConstantTwistField> stepper(f);
    const auto res = stepper.step(t0, g0, h, opt);

    REQUIRE(res.converged);
    REQUIRE(res.residual_norm == Approx(0.0).margin(1e-10));

    const Vec6<double> hxi = h * f.xi;
    const SE3d g1_ref = g0 * SE3d::Exp(hxi);

    const auto a = Lie::to_qp(res.g1);
    const auto b = Lie::to_qp(g1_ref);

    REQUIRE((a.p - b.p).norm() == Approx(0.0).margin(1e-10));
    REQUIRE(so3_angle_error(a.q, b.q) == Approx(0.0).margin(1e-10));

    // For constant twist, each stage should converge to xi_i = h*xi
    Core::StagePack<double, 3, 0> pack;
    for (int i = 0; i < 3; ++i) {
        const auto xi_i = pack.xi(res.stage_xi, i);
        REQUIRE((xi_i - hxi).norm() == Approx(0.0).margin(1e-10));
    }
}

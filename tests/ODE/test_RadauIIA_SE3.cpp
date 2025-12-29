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

    template<class G>
    [[nodiscard]] Eigen::Vector3d get_p(const G& g) {
        if constexpr (requires { g.p(); }) {
            const auto p = g.p();
            return Eigen::Vector3d((double)p(0), (double)p(1), (double)p(2));
        }
        if constexpr (requires { g.translation(); }) {
            const auto p = g.translation();
            return Eigen::Vector3d((double)p(0), (double)p(1), (double)p(2));
        }
        if constexpr (requires { g.p; }) {
            const auto p = g.p;
            return Eigen::Vector3d((double)p(0), (double)p(1), (double)p(2));
        }
        static_assert(always_false_v<G>, "Cannot extract translation (need p()/translation() or member p).");
    }

   
    [[nodiscard]] Eigen::Quaterniond quat_close_sign(const Eigen::Quaterniond& qa,
        const Eigen::Quaterniond& qb)
    {
        // q and -q represent the same rotation
        return (qa.dot(qb) < 0.0)
            ? Eigen::Quaterniond(-qb.w(), -qb.x(), -qb.y(), -qb.z())
            : qb;
    }

    [[nodiscard]] Core::NewtonOptions tight_newton() {
        Core::NewtonOptions opt;
        opt.max_iters = 40;
        opt.throw_on_fail = true;
        return opt;
    }

    // Field: xi(t,g) = 0
    struct ZeroField final {
        template<class S>
        Vec6<S> operator()(S /*t*/, const Lie::SE3<S>& /*g*/) const {
            return Vec6<S>::Zero();
        }
    };

    // Field: xi(t,g) = [0; v] (pure translation), constant v
    struct PureTranslationConstant final {
        Eigen::Vector3d v = Eigen::Vector3d::Zero();

        template<class S>
        Vec6<S> operator()(S /*t*/, const Lie::SE3<S>& /*g*/) const {
            Vec6<S> xi = Vec6<S>::Zero();
            xi.template segment<3>(3) = v.template cast<S>();
            return xi;
        }
    };

    // Field: xi(t,g) = [0; v0 + a*t] (pure translation), time varying
    struct PureTranslationAffine final {
        Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
        Eigen::Vector3d a = Eigen::Vector3d::Zero();

        template<class S>
        Vec6<S> operator()(S t, const Lie::SE3<S>& /*g*/) const {
            Vec6<S> xi = Vec6<S>::Zero();
            const Eigen::Matrix<S, 3, 1> v = v0.template cast<S>() + a.template cast<S>() * t;
            xi.template segment<3>(3) = v;
            return xi;
        }
    };


       inline double so3_angle_error(const Eigen::Quaterniond& qa, const Eigen::Quaterniond& qb) {
        // relative rotation q_rel = qa * qb^{-1}
        Eigen::Quaterniond q_rel = qa * qb.conjugate();
        q_rel.normalize();

        // Quaternion double-cover: q and -q represent same rotation
        const double w = std::clamp(std::abs(q_rel.w()), 0.0, 1.0);
        return 2.0 * std::acos(w);
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

TEST_CASE("RadauIIA_RKMK_SE3: zero twist returns identity step", "[rkmk][radau][se3]") {
    using SE3d = Lie::SE3<double>;

    const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d    p0(1.0, -2.0, 0.5);
    const SE3d g0(q0, p0);

    ZeroField f;

    const double t0 = 0.0;
    const double h = 0.25;

    Int::RadauIIA_RKMK_SE3<ZeroField> stepper(f);
    const auto res = stepper.step(t0, g0, h, tight_newton());

    REQUIRE(res.converged);
    REQUIRE(res.residual_norm == Approx(0.0).margin(1e-12));

    const auto g1 = Lie::to_qp(res.g1);
    const auto g0qp = Lie::to_qp(g0);

    const auto q1_aligned = quat_close_sign(g0qp.q, g1.q);
    REQUIRE(std::abs(q1_aligned.w() - g0qp.q.w()) == Approx(0.0).margin(1e-12));
    REQUIRE((q1_aligned.vec() - g0qp.q.vec()).norm() == Approx(0.0).margin(1e-12));
    REQUIRE((g1.p - g0qp.p).norm() == Approx(0.0).margin(1e-12));
}

TEST_CASE("RadauIIA_RKMK_SE3: pure translation constant velocity matches p0 + h*v", "[rkmk][radau][se3]") {
    using SE3d = Lie::SE3<double>;

   // const Eigen::Quaterniond q0 = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.2, -0.5, 0.1).normalized());
    const Eigen::Quaterniond q0(Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.2, -0.5, 0.1).normalized()));

    const Eigen::Vector3d    p0(-3.0, 0.2, 5.0);
    const SE3d g0(q0, p0);

    PureTranslationConstant f;
    f.v = Eigen::Vector3d(1.2, -0.7, 0.4);

    const double t0 = 0.0;
    const double h = 0.1;

    Int::RadauIIA_RKMK_SE3<PureTranslationConstant> stepper(f);
    const auto res = stepper.step(t0, g0, h, tight_newton());

    REQUIRE(res.converged);
    REQUIRE(res.residual_norm == Approx(0.0).margin(1e-10));

    const auto g1 = Lie::to_qp(res.g1);

    const Eigen::Matrix3d R0 = q0.normalized().toRotationMatrix();
    const Eigen::Vector3d p_ref = p0 + h * (R0 * f.v);   // <-- key fix

    REQUIRE((g1.p - p_ref).norm() == Approx(0.0).margin(1e-10));

    // Orientation unchanged
    Eigen::Quaterniond q_ref = q0.normalized();
    const auto q1_aligned = quat_close_sign(q_ref, g1.q);
    REQUIRE(std::abs(q1_aligned.w() - q_ref.w()) == Approx(0.0).margin(1e-10));
    REQUIRE((q1_aligned.vec() - q_ref.vec()).norm() == Approx(0.0).margin(1e-10));

}

TEST_CASE("RadauIIA_RKMK_SE3: pure translation with v(t)=v0+a*t matches analytic integral", "[rkmk][radau][se3]") {
    using SE3d = Lie::SE3<double>;

    const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d    p0(0.5, -1.0, 2.0);
    const SE3d g0(q0, p0);

    PureTranslationAffine f;
    f.v0 = Eigen::Vector3d(0.2, -0.1, 0.05);
    f.a = Eigen::Vector3d(1.0, 0.3, -0.4);

    const double t0 = 1.5;
    const double h = 0.2;
    const double t1 = t0 + h;

    Int::RadauIIA_RKMK_SE3<PureTranslationAffine> stepper(f);
    const auto res = stepper.step(t0, g0, h, tight_newton());

    REQUIRE(res.converged);
    REQUIRE(res.residual_norm == Approx(0.0).margin(1e-10));

    const auto g1 = Lie::to_qp(res.g1);

    // p1 = p0 + R0 * ∫(v0 + a t) dt
    const Eigen::Matrix3d R0 = q0.normalized().toRotationMatrix();
    const Eigen::Vector3d integral =
        f.v0 * h + 0.5 * f.a * (t1 * t1 - t0 * t0);

    const Eigen::Vector3d p_ref = p0 + R0 * integral;

    REQUIRE((g1.p - p_ref).norm() == Approx(0.0).margin(5e-9));

    // Still pure translation => orientation unchanged.
    const Eigen::Quaterniond q_ref = q0.normalized();
    const auto q1_aligned = quat_close_sign(q_ref, g1.q);
    REQUIRE(std::abs(q1_aligned.w() - q_ref.w()) == Approx(0.0).margin(1e-12));
    REQUIRE((q1_aligned.vec() - q_ref.vec()).norm() == Approx(0.0).margin(1e-12));
}

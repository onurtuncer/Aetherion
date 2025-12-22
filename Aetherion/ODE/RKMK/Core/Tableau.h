// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD,
// Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------
//
// File: Aetherion/ODE/RKMK/Core/Tableau.h
//
// Generic Butcher tableau for (implicit) Runge–Kutta methods, plus a factory for
// Radau IIA (3-stage, order 5).
//
#pragma once

#include <type_traits>

#include <Eigen/Dense>

#include <Aetherion/ODE/RKMK/Core/Scalar.h>
#include <Aetherion/ODE/RKMK/Core/Concepts.h>

namespace Aetherion::ODE::RKMK::Core {

    template<class S, int Stages>
        requires ADCompatibleScalar<S> && (Stages > 0)
    struct ButcherTableau final {
        using Scalar = S;

        static constexpr int s = Stages;

        using MatA = Eigen::Matrix<Scalar, s, s>;
        using Vec = Eigen::Matrix<Scalar, s, 1>;

        MatA A = MatA::Zero();
        Vec  b = Vec::Zero();
        Vec  c = Vec::Zero();

        constexpr ButcherTableau() = default;

        constexpr ButcherTableau(const MatA& A_in, const Vec& b_in, const Vec& c_in)
            : A(A_in), b(b_in), c(c_in) {
        }

        [[nodiscard]] constexpr int stages() const noexcept { return s; }

        // ------------------------------------------------------------------
        // Radau IIA (3-stage, order 5)
        // ------------------------------------------------------------------
        //
        // c = [(4 - sqrt(6))/10, (4 + sqrt(6))/10, 1]
        // b = [(16 - sqrt(6))/36, (16 + sqrt(6))/36, 1/9]
        // A =
        // [ (88-7*sqrt(6))/360,  (296-169*sqrt(6))/1800, (-2+3*sqrt(6))/225
        //   (296+169*sqrt(6))/1800,(88+7*sqrt(6))/360,   (-2-3*sqrt(6))/225
        //   (16-sqrt(6))/36,      (16+sqrt(6))/36,        1/9 ]
        //
        [[nodiscard]] static ButcherTableau radau_iia_3stage_order5() {
            static_assert(s == 3, "radau_iia_3stage_order5() requires Stages == 3");

            const Scalar s6 = sqrt_s(Scalar(6));

            Vec c;
            c(0) = (Scalar(4) - s6) / Scalar(10);
            c(1) = (Scalar(4) + s6) / Scalar(10);
            c(2) = Scalar(1);

            Vec b;
            b(0) = (Scalar(16) - s6) / Scalar(36);
            b(1) = (Scalar(16) + s6) / Scalar(36);
            b(2) = Scalar(1) / Scalar(9);

            MatA A;
            A(0, 0) = (Scalar(88) - Scalar(7) * s6) / Scalar(360);
            A(0, 1) = (Scalar(296) - Scalar(169) * s6) / Scalar(1800);
            A(0, 2) = (Scalar(-2) + Scalar(3) * s6) / Scalar(225);
             
            A(1, 0) = (Scalar(296) + Scalar(169) * s6) / Scalar(1800);
            A(1, 1) = (Scalar(88) + Scalar(7) * s6) / Scalar(360);
            A(1, 2) = (Scalar(-2) - Scalar(3) * s6) / Scalar(225);

            A(2, 0) = (Scalar(16) - s6) / Scalar(36);
            A(2, 1) = (Scalar(16) + s6) / Scalar(36);
            A(2, 2) = Scalar(1) / Scalar(9);

            return ButcherTableau{ A, b, c };
        }
    };

} // namespace Aetherion::ODE::RKMK::Core

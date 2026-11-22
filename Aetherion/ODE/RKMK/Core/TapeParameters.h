// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// File: Aetherion/ODE/RKMK/Core/TapeParameters.h
//
// What a functor has to say about itself for its AD tape to be kept from one
// Newton solve to the next (ParametricCppADResidualJacobian in Newton.h).
//
// A tape records the functor's arithmetic on the unknowns. Every plain double
// the functor reads on the way -- the start state of the step, the time, the
// step length, a control deflection, a gust -- is a constant of that tape. If
// such a number changes, the tape is the tape of another function.
//
// A functor that wants its tape kept therefore sorts its doubles into two
// sets, and says so through two members:
//
//   template <class Visitor> void forEachTapeParameter(Visitor&& visit) const;
//       calls visit(const double&) once for every double that may change
//       between solves, always in the same order. These become CppAD
//       *dynamic parameters* of the tape: the kept tape is given their new
//       values (ADFun::new_dynamic) instead of being recorded again.
//       The functor must read each of them through TapeParameter<S>(member),
//       on the member itself and not on a copy: the member's address is how
//       the value is found while the tape is being recorded.
//
//   bool tapeConstantsIdentical(const T& other) const;
//       true when everything else the functor would put on a tape -- every
//       double that is NOT visited above, every table, every branch taken on
//       a plain double -- is bit for bit that of `other`. The kept tape is
//       used only if this holds between the functor it was recorded from and
//       the functor of the solve at hand; otherwise it is recorded again.
//
// Comparisons made on a TapeParameter<S> value are comparisons on S: CppAD
// records them for a dynamic parameter as it does for a variable, and
// ADFun::compare_change_number() reports one that went the other way, so the
// re-recording rule of the Newton evaluators covers the parameters too.
//
// Outside a recording made by ParametricCppADResidualJacobian there is no
// binding, and TapeParameter<S>(v) is S(v): a functor written this way puts
// the same operations on any other tape as one that wrote S(v).
// ------------------------------------------------------------------------------
#pragma once

#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <cppad/cppad.hpp>

namespace Aetherion::ODE::RKMK::Core {

    namespace detail {

        /// Addresses of the functor's parameters and the AD dynamic parameters
        /// standing for them, for the recording in progress on this thread.
        struct TapeParameterBinding
        {
            std::vector<const double*> addresses;
            CppAD::vector<CppAD::AD<double>> values;
        };

        inline const TapeParameterBinding*& activeTapeParameterBinding() noexcept
        {
            thread_local const TapeParameterBinding* active = nullptr;
            return active;
        }

        struct DiscardTapeParameter
        {
            void operator()(const double& /*value*/) const noexcept {}
        };

    } // namespace detail

    /// @brief Installs a parameter binding for the lifetime of the object (one recording).
    class TapeParameterScope final
    {
    public:
        explicit TapeParameterScope(const detail::TapeParameterBinding& binding) noexcept
            : m_previous(detail::activeTapeParameterBinding())
        {
            detail::activeTapeParameterBinding() = &binding;
        }
        ~TapeParameterScope() { detail::activeTapeParameterBinding() = m_previous; }

        TapeParameterScope(const TapeParameterScope&) = delete;
        TapeParameterScope& operator=(const TapeParameterScope&) = delete;

    private:
        const detail::TapeParameterBinding* m_previous;
    };

    /// @brief The value of a double the functor holds, as scalar type @p S.
    ///
    /// For @c S = double this is @p value. For @c S = CppAD::AD<double> it is
    /// the dynamic parameter bound to the address of @p value while a kept
    /// tape is being recorded, and the constant @c S(value) otherwise.
    /// @param value  A member of the functor, passed by reference: its address is the key.
    template <class S>
    [[nodiscard]] inline S TapeParameter(const double& value)
    {
        if constexpr (std::is_same_v<S, CppAD::AD<double>>) {
            if (const detail::TapeParameterBinding* b = detail::activeTapeParameterBinding()) {
                for (std::size_t i = 0; i < b->addresses.size(); ++i)
                    if (b->addresses[i] == &value) return b->values[i];
            }
        }
        return S(value);
    }

    /// @brief A functor whose tape may be kept across solves (see the file comment).
    template <class T>
    concept TapeParameterised = requires(const T& a, const T& b) {
        a.forEachTapeParameter(detail::DiscardTapeParameter{});
        {
            a.tapeConstantsIdentical(b)
        } -> std::convertible_to<bool>;
    };

    /// @brief Bit-for-bit equality of two doubles: +0 and -0 differ, a NaN equals itself.
    [[nodiscard]] inline bool BitIdentical(double a, double b) noexcept
    {
        return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
    }

    /// @brief Bit-for-bit equality of two double matrices of the same shape.
    template <class DerivedA, class DerivedB>
    [[nodiscard]] inline bool BitIdentical(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>& b) noexcept
    {
        if (a.rows() != b.rows() || a.cols() != b.cols()) return false;
        for (Eigen::Index j = 0; j < a.cols(); ++j)
            for (Eigen::Index i = 0; i < a.rows(); ++i)
                if (!BitIdentical(a(i, j), b(i, j))) return false;
        return true;
    }

} // namespace Aetherion::ODE::RKMK::Core

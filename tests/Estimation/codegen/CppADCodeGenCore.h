// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// CppADCodeGenCore.h
//
// The source-generation subset of CppADCodeGen 2.5.0 (CG<Base>, CodeHandler,
// LanguageC), in place of <cppad/cg.hpp>, which does not compile with MSVC
// (19.50, /std:c++latest) for three reasons, recorded in
// TODO-sensor-fusion.md:
//
//   1. cg.hpp befriends GreaterThanZero<Base>, NearEqual<Base>, CondExp<Base>,
//      sign<Base>, pow<Base>, ... by unqualified template-id from inside
//      namespace CppAD::cg, while declare_cg.hpp declares them in namespace
//      CppAD. MSVC looks the template-id up in CppAD::cg only (C2672, with or
//      without /permissive). Fixed below by using-declarations of the CppAD
//      templates into CppAD::cg between declare_cg.hpp and cg.hpp.
//   2. model/compiler/{gcc,clang}_compiler.hpp use a dependent base's member
//      without this-> (C2065 'linkerName'), and
//   3. model/threadpool/pthread_pool_c.hpp holds a C source file as one raw
//      string literal beyond MSVC's limit (C2026).
//
// 2 and 3 belong to the model-library / dynamic-library machinery (Linux
// only: it drives gcc or clang and dlopen), which source generation does not
// need, so this header leaves the model/ directory out. The include order
// otherwise follows cppad/cg/cppadcg.hpp of 2.5.0; like it, it must come
// before any CppAD include.
// ------------------------------------------------------------------------------

#pragma once

#ifdef CPPAD_CPPAD_HPP
#error "CppADCodeGenCore.h must be included before cppad/cppad.hpp"
#endif

#include <algorithm>
#include <array>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <forward_list>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iosfwd>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <valarray>
#include <vector>

#ifndef CPPAD_CG_SYSTEM_WIN
#if defined(_WIN32) || defined(_WIN64)
#define CPPAD_CG_SYSTEM_WIN 1
#endif
#endif
#ifndef CPPAD_CG_SYSTEM_LINUX
#if defined(__linux__) || defined(__APPLE__)
#define CPPAD_CG_SYSTEM_LINUX 1
#endif
#endif

// clang-format off
#include <cppad/base_require.hpp>

#include <cppad/cg/cppadcg_assert.hpp>
#include <cppad/cg/exception.hpp>
#include <cppad/cg/operation.hpp>
#include <cppad/cg/declare_cg.hpp>

// (1) Make the friend template-ids of cg.hpp resolvable for MSVC.
namespace CppAD::cg {
    using CppAD::abs_geq;
    using CppAD::CondExp;
    using CppAD::EqualOpSeq;
    using CppAD::GreaterThanOrZero;
    using CppAD::GreaterThanZero;
    using CppAD::LessThanOrZero;
    using CppAD::LessThanZero;
    using CppAD::NearEqual;
    using CppAD::pow;
    using CppAD::sign;
} // namespace CppAD::cg

#include <cppad/cg/model/system/system.hpp>
#include <cppad/cg/model/system/linux_system.hpp>

#include <cppad/cg/smart_containers.hpp>
#include <cppad/cg/ostream_config_restore.hpp>
#include <cppad/cg/array_view.hpp>

#include <cppad/cg/patterns/index/index_pattern.hpp>
#include <cppad/cg/patterns/index/linear_index_pattern.hpp>
#include <cppad/cg/patterns/index/sectioned_index_pattern.hpp>
#include <cppad/cg/patterns/index/plane_2d_index_pattern.hpp>
#include <cppad/cg/patterns/index/random_index_pattern.hpp>
#include <cppad/cg/patterns/index/random_1d_index_pattern.hpp>
#include <cppad/cg/patterns/index/random_2d_index_pattern.hpp>
#include <cppad/cg/patterns/index/index_pattern_impl.hpp>

#include <cppad/cg/debug.hpp>
#include <cppad/cg/argument.hpp>
#include <cppad/cg/operation_node.hpp>
#include <cppad/cg/operation_stack.hpp>
#include <cppad/cg/nodes/index_operation_node.hpp>
#include <cppad/cg/nodes/index_assign_operation_node.hpp>
#include <cppad/cg/nodes/loop_start_operation_node.hpp>
#include <cppad/cg/nodes/loop_end_operation_node.hpp>
#include <cppad/cg/nodes/print_operation_node.hpp>
#include <cppad/cg/cg.hpp>
#include <cppad/cg/default.hpp>
#include <cppad/cg/variable.hpp>
#include <cppad/cg/identical.hpp>
#include <cppad/cg/range.hpp>
#include <cppad/cg/atomic_dependency_locator.hpp>
#include <cppad/cg/variable_name_generator.hpp>
#include <cppad/cg/job_timer.hpp>
#include <cppad/cg/lang/language.hpp>
#include <cppad/cg/lang/lang_stream_stack.hpp>
#include <cppad/cg/scope_path_element.hpp>
#include <cppad/cg/array_id_compresser.hpp>
#include <cppad/cg/patterns/loop_position.hpp>
#include <cppad/cg/arithmetic.hpp>
#include <cppad/cg/arithmetic_assign.hpp>
#include <cppad/cg/math.hpp>
#include <cppad/cg/math_other.hpp>
#include <cppad/cg/nan.hpp>
#include <cppad/cg/cond_exp_op.hpp>
#include <cppad/cg/compare.hpp>
#include <cppad/cg/ordered.hpp>
#include <cppad/cg/unary.hpp>

#include <cppad/cg/code_handler.hpp>
#include <cppad/cg/code_handler_impl.hpp>
#include <cppad/cg/code_handler_vector.hpp>
#include <cppad/cg/code_handler_loops.hpp>

#include <cppad/cg/base_double.hpp>
#include <cppad/cg/base_float.hpp>

#include <cppad/cppad.hpp>
#include <cppad/cg/arithmetic_ad.hpp>
#include <cppad/cg/extra/extra.hpp>

#include <cppad/cg/util.hpp>
#include <cppad/cg/evaluator/evaluator.hpp>
#include <cppad/cg/evaluator/evaluator_ad.hpp>
#include <cppad/cg/evaluator/evaluator_adcg.hpp>
#include <cppad/cg/evaluator/evaluator_cg.hpp>
#include <cppad/cg/operation_path_node.hpp>
#include <cppad/cg/operation_path.hpp>
#include <cppad/cg/solver.hpp>
#include <cppad/cg/collect_variable.hpp>
#include <cppad/cg/graph_mod.hpp>
#include <cppad/cg/operation_node_name_streambuf.hpp>

#include <cppad/cg/custom_position.hpp>
#include <cppad/cg/base_abstract_atomic_fun.hpp>
#include <cppad/cg/abstract_atomic_fun.hpp>
#include <cppad/cg/atomic_fun.hpp>
#include <cppad/cg/atomic_fun_bridge.hpp>

#include <cppad/cg/patterns/independent_node_sorter.hpp>
#include <cppad/cg/patterns/equation_group.hpp>
#include <cppad/cg/patterns/iter_equation_group.hpp>
#include <cppad/cg/patterns/loop_model.hpp>
#include <cppad/cg/patterns/loop_free_model.hpp>
#include <cppad/cg/patterns/equation_pattern.hpp>
#include <cppad/cg/patterns/loop.hpp>
#include <cppad/cg/patterns/dependent_pattern_matcher.hpp>

#include <cppad/cg/lang/c/lang_c_atomic_fun.hpp>
#include <cppad/cg/lang/c/language_c.hpp>
#include <cppad/cg/lang/c/language_c_arrays.hpp>
#include <cppad/cg/lang/c/language_c_index_patterns.hpp>
#include <cppad/cg/lang/c/language_c_double.hpp>
#include <cppad/cg/lang/c/language_c_float.hpp>
#include <cppad/cg/lang/c/language_c_loops.hpp>
#include <cppad/cg/lang/c/lang_c_default_var_name_gen.hpp>
#include <cppad/cg/lang/c/lang_c_default_hessian_var_name_gen.hpp>
#include <cppad/cg/lang/c/lang_c_default_reverse2_var_name_gen.hpp>
#include <cppad/cg/lang/c/lang_c_custom_var_name_gen.hpp>
#include <cppad/cg/lang/c/lang_c_util.hpp>
// clang-format on

// Mark the full headers as included, so that a later <cppad/cg.hpp> (which
// cppad/cg/support/cppadcg_eigen.hpp includes) does not pull model/ back in.
#define CPPADCG_INCLUDED
#define CPPAD_CG_INCLUDED

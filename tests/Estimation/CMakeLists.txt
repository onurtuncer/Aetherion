# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(
  test_estimation
  test_Configuration.cpp
  test_EKF.cpp
  test_LogContract.cpp
  test_MeasurementModels.cpp
  test_ProcessModel.cpp
  test_Replay.cpp
  test_ReplayEdgeCases.cpp
  test_ReplayInputValidation.cpp
  test_ReplayRobustness.cpp)

target_link_libraries(test_estimation PRIVATE Aetherion Catch2::Catch2WithMain)

include(Catch)
catch_discover_tests(test_estimation)

# CppAD code-generation compatibility (AETHERION_WITH_CPPADCG). generate_estimation_cg is a host tool: it tapes the
# estimator maps with CppAD::AD<CppAD::cg::CG<double>> and writes plain C at build time. test_estimation_codegen
# compiles that C with the C compiler and checks it against double and CppAD::AD<double>; it does not link CppADCodeGen.
if(AETHERION_WITH_CPPADCG)
  enable_language(C)

  add_executable(generate_estimation_cg codegen/generate_estimation_cg.cpp)
  target_link_libraries(generate_estimation_cg PRIVATE Aetherion CppADCodeGen::cppadcg)

  set(_aetherion_cg_dir "${CMAKE_CURRENT_BINARY_DIR}/generated_cg")
  add_custom_command(
    OUTPUT "${_aetherion_cg_dir}/estimation_cg.c" "${_aetherion_cg_dir}/estimation_cg.h"
    COMMAND generate_estimation_cg "${_aetherion_cg_dir}"
    DEPENDS generate_estimation_cg
    COMMENT "Generating C for the estimator maps with CppADCodeGen"
    VERBATIM)

  add_executable(test_estimation_codegen codegen/test_CodeGen.cpp "${_aetherion_cg_dir}/estimation_cg.c")
  target_include_directories(test_estimation_codegen PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/codegen"
                                                             "${_aetherion_cg_dir}")
  target_link_libraries(test_estimation_codegen PRIVATE Aetherion Catch2::Catch2WithMain)
  catch_discover_tests(test_estimation_codegen)
  unset(_aetherion_cg_dir)
endif()

if(AETHERION_ENABLE_MUTATION_TESTING)
  include(MutationTesting)
endif()

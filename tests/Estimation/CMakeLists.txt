# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(
  test_estimation
  test_Configuration.cpp
  test_EKF.cpp
  test_GoldenVectors.cpp
  test_LogContract.cpp
  test_MeasurementModels.cpp
  test_ProcessModel.cpp
  test_Replay.cpp
  test_ReplayEdgeCases.cpp
  test_ReplayInputValidation.cpp
  test_ReplayRobustness.cpp)

target_link_libraries(test_estimation PRIVATE Aetherion Catch2::Catch2WithMain)

# Golden vectors of the reference filter (step 8): data, committed, replayed by test_GoldenVectors.cpp here and by
# Hemerion against its embedded filter. generate_golden_vectors rewrites them; it is a maintainer tool, not run by the
# build (see doc/codegen.rst).
set(_aetherion_golden_dir "${PROJECT_SOURCE_DIR}/data/Estimation/GoldenVectors")
target_compile_definitions(test_estimation PRIVATE AETHERION_GOLDEN_VECTORS_DIR="${_aetherion_golden_dir}")
add_executable(generate_golden_vectors golden/generate_golden_vectors.cpp)
target_link_libraries(generate_golden_vectors PRIVATE Aetherion)

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

  # The step-8 generator tool. generate_ekf_c writes the filter functions of one configuration and error convention as
  # plain C plus a header with the dimension, block offsets and configuration hash. Here it is run for kNavBaroGnss16 in
  # both conventions, into the build tree only: generated C is never committed, installed or released from this
  # repository (TODO-sensor-fusion.md, "Decided (2026-11-13)"), and neither is the generator executable.
  add_executable(generate_ekf_c codegen/generate_ekf_c.cpp)
  target_link_libraries(generate_ekf_c PRIVATE Aetherion CppADCodeGen::cppadcg)

  # The version of the vcpkg port at the pinned baseline; CppADCodeGen's headers do not carry one.
  set(_aetherion_cppadcg_version "2.5.0")
  set(_aetherion_ekf_dir "${CMAKE_CURRENT_BINARY_DIR}/generated_ekf")
  set(_aetherion_ekf_banner "${CMAKE_CURRENT_SOURCE_DIR}/codegen/test_banner.txt")
  set(_aetherion_ekf_commit "build-tree-test")
  set(_aetherion_ekf_sources "")
  foreach(_convention ri li)
    add_custom_command(
      OUTPUT "${_aetherion_ekf_dir}/aeth_ekf_${_convention}.c" "${_aetherion_ekf_dir}/aeth_ekf_${_convention}.h"
      COMMAND
        generate_ekf_c --out "${_aetherion_ekf_dir}" --stem "aeth_ekf_${_convention}" --config nav-baro-gnss16
        --convention ${_convention} --banner "${_aetherion_ekf_banner}" --generator-commit "${_aetherion_ekf_commit}"
        --cppadcg-version "${_aetherion_cppadcg_version}"
      DEPENDS generate_ekf_c "${_aetherion_ekf_banner}"
      COMMENT "Generating the ${_convention} filter C for nav-baro-gnss16"
      VERBATIM)
    list(APPEND _aetherion_ekf_sources "${_aetherion_ekf_dir}/aeth_ekf_${_convention}.c")
  endforeach()

  target_sources(test_estimation_codegen PRIVATE codegen/test_EkfCodeGen.cpp ${_aetherion_ekf_sources})
  target_include_directories(test_estimation_codegen PRIVATE "${_aetherion_ekf_dir}")
  target_compile_definitions(
    test_estimation_codegen
    PRIVATE AETHERION_GOLDEN_VECTORS_DIR="${_aetherion_golden_dir}"
            AETHERION_EKF_GENERATED_DIR="${_aetherion_ekf_dir}"
            AETHERION_EKF_TEST_BANNER="${_aetherion_ekf_banner}"
            AETHERION_EKF_TEST_COMMIT="${_aetherion_ekf_commit}"
            AETHERION_CPPADCG_VERSION="${_aetherion_cppadcg_version}")

  catch_discover_tests(test_estimation_codegen)

  # Generated code and the generator stay out of the git index and out of every install rule.
  add_test(NAME estimation_generated_code_not_published
           COMMAND "${CMAKE_COMMAND}" "-DSOURCE_DIR=${PROJECT_SOURCE_DIR}" "-DBINARY_DIR=${PROJECT_BINARY_DIR}" -P
                   "${PROJECT_SOURCE_DIR}/cmake/CheckGeneratedEstimatorCodeNotPublished.cmake")
  unset(_aetherion_cg_dir)
endif()

if(AETHERION_ENABLE_MUTATION_TESTING)
  include(MutationTesting)
endif()

# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------

include(Catch)

# ── FeedThrough FMU (smoke-tests the packaged .fmu via ecos) ─────────────────
#
# generateFMU() builds and packages the FeedThrough.fmu archive (FeedThrough_fmi2 target). test_feedthrough_ecos loads
# that real .fmu archive through ecos's simulation_structure/simulation API -- unlike an in-process fmu4cpp test, this
# exercises the full FMU packaging pipeline (unzip, modelDescription.xml parsing, shared-library load).
#
# generateFMU() is available because cmake/generate_fmu.cmake is include()d unconditionally from the root
# CMakeLists.txt.

generatefmu(
  FeedThrough
  FMI_VERSIONS
  fmi2
  SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/FeedThrough/FeedThrough.cpp")

add_executable(test_feedthrough_ecos FeedThrough/test_feedthrough_ecos.cpp)

target_link_libraries(test_feedthrough_ecos PRIVATE Catch2::Catch2WithMain libecos)

target_compile_definitions(test_feedthrough_ecos
                           PRIVATE FEEDTHROUGH_FMU_PATH="${CMAKE_BINARY_DIR}/models/fmi2/FeedThrough/FeedThrough.fmu")

add_dependencies(test_feedthrough_ecos FeedThrough_fmi2)

# ecos picks the FMU extractor from $SHELL on Windows (vendor/ecos/src/util/unzipper.hpp): `unzip` when SHELL contains
# "bash", otherwise `tar`. PowerShell and cmd leave SHELL unset, so ecos runs `tar`; wherever Git's GNU tar shadows the
# bsdtar in System32 it cannot read a zip and treats the drive letter as an rsh host, loadFmu() returns null and ecos
# dereferences it -- the three tests SEGFAULT (issue #54). Pin SHELL so ecos uses unzip whenever unzip is actually on
# PATH; without unzip the environment is left alone and the bsdtar path keeps working as before. Running the executable
# directly (outside ctest) still needs SHELL set by hand.
set(_ecos_test_properties "")

if(WIN32)
  find_program(
    AETHERION_UNZIP_EXECUTABLE
    unzip
    NO_CMAKE_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH)
  mark_as_advanced(AETHERION_UNZIP_EXECUTABLE)
  if(AETHERION_UNZIP_EXECUTABLE)
    set(_ecos_test_properties PROPERTIES ENVIRONMENT "SHELL=bash")
  else()
    message(STATUS "unzip not on PATH: test_feedthrough_ecos lets ecos fall back to tar (see issue #54)")
  endif()
endif()

catch_discover_tests(
  test_feedthrough_ecos
  TEST_SUFFIX
  "_ecos"
  ${_ecos_test_properties})

# ── Eigen ─────────────────────────────────────────────────────────────────────
add_executable(test_eigen test_eigen.cpp)

target_link_libraries(test_eigen PRIVATE Catch2::Catch2WithMain Eigen3::Eigen)

target_include_directories(test_eigen PRIVATE "${PROJECT_SOURCE_DIR}/vendor/eigen")

catch_discover_tests(test_eigen)

# add_test(NAME eigen_basic_test COMMAND test_eigen)

add_executable(test_nlohmann test_nlohmann_json.cpp)

target_link_libraries(test_nlohmann PRIVATE Catch2::Catch2WithMain nlohmann_json)

catch_discover_tests(test_nlohmann)

add_executable(test_cppad test_cppad.cpp)

target_link_libraries(test_cppad PRIVATE Catch2::Catch2WithMain CppAD::cppad Eigen3::Eigen)

catch_discover_tests(test_cppad)

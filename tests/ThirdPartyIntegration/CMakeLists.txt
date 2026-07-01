# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------

include(Catch)

# ── FeedThrough FMU (smoke-tests the fmu4cpp FMI 2.0 integration) ────────────
#
# Two targets:
#   FeedThrough_fmi2         — shared library (.fmu artifact), built by generateFMU()
#   test_feedthrough_fmi2    — Catch2 in-process test: same model linked directly
#                              against fmu4cpp objects so no DLL loading is needed.
#
# generateFMU() is available because cmake/generate_fmu.cmake is include()d
# unconditionally from the root CMakeLists.txt.

generatefmu(
  FeedThrough
  FMI_VERSIONS
  fmi2
  SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/FeedThrough/FeedThrough.cpp")

add_executable(
  test_feedthrough_fmi2
  FeedThrough/test_feedthrough.cpp
  $<TARGET_OBJECTS:fmu4cpp_base>
  $<TARGET_OBJECTS:fmu4cpp_fmi2>)

target_compile_definitions(test_feedthrough_fmi2 PRIVATE FMI2)

target_include_directories(
  test_feedthrough_fmi2
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/FeedThrough"
          "${CMAKE_SOURCE_DIR}/vendor/fmu4cpp/export/include"
          "${CMAKE_SOURCE_DIR}/vendor/fmu4cpp/export/src")

target_link_libraries(test_feedthrough_fmi2 PRIVATE Catch2::Catch2WithMain)

catch_discover_tests(test_feedthrough_fmi2 TEST_SUFFIX "_fmu4cpp")

# ── Eigen ─────────────────────────────────────────────────────────────────────
add_executable(test_eigen test_eigen.cpp)

target_link_libraries(test_eigen PRIVATE Catch2::Catch2WithMain Eigen3::Eigen)

target_include_directories(test_eigen PRIVATE "${CMAKE_SOURCE_DIR}/vendor/eigen")

catch_discover_tests(test_eigen)

# add_test(NAME eigen_basic_test COMMAND test_eigen)

add_executable(test_nlohmann test_nlohmann_json.cpp)

target_link_libraries(test_nlohmann PRIVATE Catch2::Catch2WithMain nlohmann_json)

catch_discover_tests(test_nlohmann)

add_executable(test_cppad test_cppad.cpp)

target_link_libraries(test_cppad PRIVATE Catch2::Catch2WithMain CppAD::cppad Eigen3::Eigen)

target_include_directories(test_cppad PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

catch_discover_tests(test_cppad)

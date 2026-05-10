# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(
  test_rigidBody
  test_AerodynamicWrench.cpp
  test_GravitationalWrench.cpp
  test_SixDofStepper.cpp
  test_SE3StateInit.cpp
  test_Config.cpp)

target_compile_features(test_rigidBody PRIVATE cxx_std_23)

target_compile_options(test_rigidBody PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/bigobj>)

target_link_libraries(
  test_rigidBody
  PRIVATE Aetherion
          Catch2::Catch2WithMain
          CppAD::cppad
          Eigen3::Eigen)

target_include_directories(test_rigidBody PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

include(Catch)
catch_discover_tests(test_rigidBody)

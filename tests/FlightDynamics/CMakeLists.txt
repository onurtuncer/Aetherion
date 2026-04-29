# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_initial_state
                    test_BuildInitialStateVector.cpp)

target_compile_options(test_initial_state PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_link_libraries(test_initial_state
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad
        Eigen3::Eigen
)

target_include_directories(test_initial_state PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

include(Catch)
catch_discover_tests(test_initial_state)

# --------------------------------------------------------------------------
# Gravity policy tests
# --------------------------------------------------------------------------

add_executable(test_gravity_policies
                    test_GravityPolicies.cpp)

target_link_libraries(test_gravity_policies
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad
        Eigen3::Eigen
)

target_include_directories(test_gravity_policies PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

catch_discover_tests(test_gravity_policies)

# --------------------------------------------------------------------------
# Mass, propulsion, and aero policy tests
# --------------------------------------------------------------------------

add_executable(test_mass_propulsion_aero_policies
                    test_MassPropulsionAeroPolicies.cpp)

target_compile_options(test_mass_propulsion_aero_policies PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_link_libraries(test_mass_propulsion_aero_policies
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad
        Eigen3::Eigen
)

target_include_directories(test_mass_propulsion_aero_policies PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

catch_discover_tests(test_mass_propulsion_aero_policies)

# --------------------------------------------------------------------------
# Wind model and WindAwareDragPolicy tests
# --------------------------------------------------------------------------

add_executable(test_wind_models test_WindModels.cpp)

target_compile_options(test_wind_models PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_link_libraries(test_wind_models
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad
        Eigen3::Eigen
)

target_include_directories(test_wind_models PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

catch_discover_tests(test_wind_models)

# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/Simulation/CMakeLists.txt
#
# AetherionSimulation — post-processing / snapshot layer.
#
# Kept as a separate static library so that the core Aetherion library stays
# independent of simulation-specific concerns. Downstream targets link against
# Aetherion::Simulation instead of (or in addition to) Aetherion::Aetherion.
# ------------------------------------------------------------------------------

set(SPDLOG_USE_STD_FORMAT ON)

add_library(AetherionSimulation STATIC)
add_library(Aetherion::Simulation ALIAS AetherionSimulation)

target_compile_features(AetherionSimulation PUBLIC cxx_std_23)

target_compile_options(AetherionSimulation PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_sources(AetherionSimulation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Application.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ArgumentParser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Config.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Log.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MakeSnapshot1.cpp
)

target_include_directories(AetherionSimulation
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/vendor>

)

target_link_libraries(AetherionSimulation
    PUBLIC
        Aetherion::Aetherion
        Eigen3::Eigen
)

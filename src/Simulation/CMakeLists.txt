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

add_library(AetherionSimulation STATIC)
add_library(Aetherion::Simulation ALIAS AetherionSimulation)

target_compile_features(AetherionSimulation PUBLIC cxx_std_23)

target_sources(AetherionSimulation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/MakeSnapshot1.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/ISimulator.cpp
)

target_include_directories(AetherionSimulation
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
)

target_link_libraries(AetherionSimulation
    PUBLIC
        Aetherion::Aetherion
        Eigen3::Eigen
)
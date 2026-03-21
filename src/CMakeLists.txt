# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/CMakeLists.txt
#
# Defines the Aetherion static (or shared) library target and delegates
# source registration to each module subdirectory via target_sources().
# FMU targets are defined in src/FMU/CMakeLists.txt and are independent
# executables/shared libraries — they do not contribute to the Aetherion target.
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Library target
# ------------------------------------------------------------------------------
if(AETHERION_BUILD_SHARED)
    add_library(Aetherion SHARED)
else()
    add_library(Aetherion STATIC)
endif()

add_library(Aetherion::Aetherion ALIAS Aetherion)

# ------------------------------------------------------------------------------
# Language standard
# ------------------------------------------------------------------------------
target_compile_features(Aetherion PUBLIC cxx_std_23)

# ------------------------------------------------------------------------------
# Include directories
# ------------------------------------------------------------------------------
target_include_directories(Aetherion
    PUBLIC
        # During build: project root so that includes like
        # <FlightDynamics/Serialization/SimulationConfigJson.hpp> resolve.
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/vendor/eigen>
        # During install (if AETHERION_INSTALL=ON):
        $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------------------------------
# Module subdirectories
# Each subdirectory calls target_sources(Aetherion PRIVATE ...) to register
# its own .cpp files. No source globbing — every file is listed explicitly.
# ------------------------------------------------------------------------------
add_subdirectory(Coordinate)
add_subdirectory(FlightDynamics)
add_subdirectory(ODE)
add_subdirectory(Serialization)

# ------------------------------------------------------------------------------
# Simulation targets (optional, independent of the Aetherion library target)
# ------------------------------------------------------------------------------

if(AETHERION_BUILD_SIMULATION)
    add_subdirectory(Simulation)
endif()

# ------------------------------------------------------------------------------
# FMU targets (optional, independent of the Aetherion library target)
# ------------------------------------------------------------------------------
if(AETHERION_BUILD_FMUS)
    add_subdirectory(FMU)
endif()



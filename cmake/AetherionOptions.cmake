# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
# AetherionOptions.cmake
#
# Central registry for all user-facing CMake build options.
# Include this file early in the root CMakeLists.txt, before any
# add_subdirectory() calls, so all options are defined when subdirectories
# are processed.
#
# Usage (root CMakeLists.txt):
#   include(cmake/AetherionOptions.cmake)
#
# Override from command line:
#   cmake -DAETHERION_BUILD_TESTS=OFF -DBUILD_DOCS=ON ..
# ------------------------------------------------------------------------------

include_guard(GLOBAL)

# ==============================================================================
# Build Targets
# ==============================================================================

option(AETHERION_BUILD_TESTS
    "Build the Aetherion unit and integration test suite (requires CTest)"
    ON)

option(AETHERION_BUILD_SIMULATION
    "Build the Aetherion application layer"
    ON)

option(AETHERION_BUILD_EXAMPLES
    "Build the example executables under src/Examples"
    ON)

option(BUILD_DOCS
    "Build Sphinx/Doxygen documentation. When ON, only the doc target \
is configured and all other targets are skipped (early return in root)."
    OFF)

# ==============================================================================
# Library Configuration
# ==============================================================================

option(AETHERION_BUILD_SHARED
    "Build Aetherion as a shared library instead of a static library"
    OFF)

option(AETHERION_INSTALL
    "Generate install rules for the Aetherion library and headers"
    OFF)

# ==============================================================================
# FMU / fmu4cpp Integration
# ==============================================================================

option(AETHERION_BUILD_FMUS
    "Build the FMU targets under src/FMU (DraglessSphere, FeedThrough, \
TabulatedSignal)"
    ON)

# Passed through to the fmu4cpp vendor subdirectory.
option(FMU4CPP_BUILD_TESTS
    "Build fmu4cpp's own internal tests (vendored; should normally be OFF)"
    OFF)

option(FMU4CPP_TESTS
    "Alias guard for fmu4cpp test option (vendored; keep OFF)"
    OFF)

option(FMU4CPP_MSVC_STATIC_CRT
    "Link fmu4cpp against the static MSVC runtime (/MT) instead of the \
dynamic one (/MD). Applies to MSVC builds only."
    ON)

# ==============================================================================
# Developer / CI Options
# ==============================================================================

option(ENABLE_STATIC_ANALYSIS
    "Run cppcheck static analysis as part of the build (requires cppcheck \
to be installed and visible on PATH)"
    OFF)

option(AETHERION_ENABLE_COVERAGE
    "Instrument the build for code coverage reporting (GCC/Clang only). \
Used by the coverage CI workflow."
    OFF)

option(AETHERION_WARNINGS_AS_ERRORS
    "Treat all compiler warnings as errors (-Werror / /WX). \
Recommended for CI; avoid enabling in end-user builds."
    OFF)

# ==============================================================================
# Derived / Enforced Constraints
# ==============================================================================

# fmu4cpp vendored tests must always be suppressed from Aetherion's build.
# Force these regardless of what a parent project or cache may have set.
set(BUILD_TESTING       OFF CACHE BOOL "Disable top-level CTest for vendors" FORCE)
set(FMU4CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FMU4CPP_TESTS       OFF CACHE BOOL "" FORCE)

# Coverage instrumentation requires tests to be built.
if(AETHERION_ENABLE_COVERAGE AND NOT AETHERION_BUILD_TESTS)
    message(WARNING
        "[Aetherion] AETHERION_ENABLE_COVERAGE=ON has no effect when \
AETHERION_BUILD_TESTS=OFF. Enabling tests automatically.")
    set(AETHERION_BUILD_TESTS ON CACHE BOOL "" FORCE)
endif()

# ==============================================================================
# Summary
# ==============================================================================

message(STATUS "--------------------------------------------------------------")
message(STATUS "  Aetherion build configuration")
message(STATUS "--------------------------------------------------------------")
message(STATUS "  Version                  : ${PROJECT_VERSION}")
message(STATUS "  Build type               : ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard             : 23")
message(STATUS "")
message(STATUS "  Build targets")
message(STATUS "    Shared library         : ${AETHERION_BUILD_SHARED}")
message(STATUS "    Simulation layer       : ${AETHERION_BUILD_SIMULATION}")
message(STATUS "    FMUs                   : ${AETHERION_BUILD_FMUS}")
message(STATUS "    Tests                  : ${AETHERION_BUILD_TESTS}")
message(STATUS "    Examples               : ${AETHERION_BUILD_EXAMPLES}")
message(STATUS "    Documentation          : ${BUILD_DOCS}")
message(STATUS "    Install rules          : ${AETHERION_INSTALL}")
message(STATUS "")
message(STATUS "  Developer options")
message(STATUS "    Static analysis        : ${ENABLE_STATIC_ANALYSIS}")
message(STATUS "    Coverage               : ${AETHERION_ENABLE_COVERAGE}")
message(STATUS "    Warnings as errors     : ${AETHERION_WARNINGS_AS_ERRORS}")
message(STATUS "")
message(STATUS "  MSVC static CRT          : ${FMU4CPP_MSVC_STATIC_CRT}")
message(STATUS "--------------------------------------------------------------")
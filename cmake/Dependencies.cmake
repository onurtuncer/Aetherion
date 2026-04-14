# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

if(BUILD_DOCS)
    return()
endif()

# ------------------------------------------------------------------------------
# CppAD
#   Windows -> prebuilt vendor binaries (vendor/cppad/x64-{Debug,Release})
#   Linux   -> FetchContent build from source
# ------------------------------------------------------------------------------
if(WIN32)
    set(CPPAD_VENDOR_ROOT "${CMAKE_SOURCE_DIR}/vendor/cppad")

    add_library(CppAD::cppad STATIC IMPORTED GLOBAL)
    set_target_properties(CppAD::cppad PROPERTIES
        IMPORTED_CONFIGURATIONS "Debug;Release"
        IMPORTED_LOCATION_DEBUG
            "${CPPAD_VENDOR_ROOT}/x64-Debug/lib/cppad_lib.lib"
        IMPORTED_LOCATION_RELEASE
            "${CPPAD_VENDOR_ROOT}/x64-Release/lib/cppad_lib.lib"
        INTERFACE_INCLUDE_DIRECTORIES
            "${CPPAD_VENDOR_ROOT}/x64-Release/include"
    )
else()
    include(FetchContent)
    FetchContent_Declare(
        cppad
        GIT_REPOSITORY https://github.com/coin-or/CppAD.git
        GIT_TAG        20240000.7
        GIT_SHALLOW    TRUE
    )
    set(cppad_static_lib YES CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(cppad)

    # cppad_lib is the real target produced by CppAD's own CMakeLists.
    # Create the namespaced alias that all Aetherion targets link against.
    add_library(CppAD::cppad ALIAS cppad_lib)
endif()

# ------------------------------------------------------------------------------
# Eigen3 (vendored headers)
# ------------------------------------------------------------------------------
set(EIGEN3_VENDOR_DIR "${CMAKE_SOURCE_DIR}/vendor/eigen")

if(EXISTS "${EIGEN3_VENDOR_DIR}/Eigen/Dense")
    message(STATUS "Using vendored Eigen headers in ${EIGEN3_VENDOR_DIR}")

    add_library(eigen3_vendor INTERFACE)
    target_include_directories(eigen3_vendor
        INTERFACE
            "${EIGEN3_VENDOR_DIR}"
    )

    add_library(Eigen3::Eigen ALIAS eigen3_vendor)
else()
    message(FATAL_ERROR
        "Eigen3 not found: vendored headers at "
        "${EIGEN3_VENDOR_DIR}/Eigen/Dense"
    )
endif()

# ------------------------------------------------------------------------------
# nlohmann/json (vendored header-only)
# ------------------------------------------------------------------------------
add_library(nlohmann_json INTERFACE)

target_include_directories(nlohmann_json
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor
)

target_compile_definitions(nlohmann_json
    INTERFACE
        NLOHMANN_JSON_HEADER_ONLY
)

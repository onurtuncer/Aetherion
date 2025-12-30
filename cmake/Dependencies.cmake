# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

if(BUILD_DOCS)
    return()
endif()

# Path to the installed CppAD binaries in the repo
set(CPPAD_VENDOR_ROOT "${CMAKE_SOURCE_DIR}/vendor/cppad")

# Imported CppAD library target with Debug/Release locations
add_library(CppAD::cppad STATIC IMPORTED GLOBAL)

set_target_properties(CppAD::cppad PROPERTIES
    IMPORTED_CONFIGURATIONS "Debug;Release"

    IMPORTED_LOCATION_DEBUG
        "${CPPAD_VENDOR_ROOT}/x64-Debug/lib/cppad_lib.lib"

    IMPORTED_LOCATION_RELEASE
        "${CPPAD_VENDOR_ROOT}/x64-Release/lib/cppad_lib.lib"

    # Headers — same for both configs
    INTERFACE_INCLUDE_DIRECTORIES
        "${CPPAD_VENDOR_ROOT}/x64-Release/include"
)

set(EIGEN3_VENDOR_DIR "${CMAKE_SOURCE_DIR}/vendor/eigen")

if (EXISTS "${EIGEN3_VENDOR_DIR}/Eigen/Dense")
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

add_library(nlohmann_json INTERFACE)

target_include_directories(nlohmann_json
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor
)

target_compile_definitions(nlohmann_json
    INTERFACE
        NLOHMANN_JSON_HEADER_ONLY
)


#set(ECOS_BUILD_CLI OFF)     # Set to ON for building ecos command-line-interface
#set(ECOS_BUILD_CLIB OFF)    # Set to ON for building C API
#set(ECOS_WITH_PROXYFMU OFF) # Set to ON for remoting
#
#FetchContent_Declare(
#        ecos
#        GIT_REPOSITORY https://github.com/Ecos-platform/ecos.git
#        GIT_TAG master
#)
#
#FetchContent_MakeAvailable(ecos)



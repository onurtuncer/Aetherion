# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.26)

project(Aetherion VERSION 1.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

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

   # Fallback to vendored header-only Eigen
    set(EIGEN3_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/eigen")

    if (EXISTS "${EIGEN3_VENDOR_DIR}/Eigen/Dense")
        message(STATUS "Using vendored Eigen headers in ${EIGEN3_VENDOR_DIR}")

        # Create a normal interface target
        add_library(eigen3_vendor INTERFACE)
        target_include_directories(eigen3_vendor
            INTERFACE
                "${EIGEN3_VENDOR_DIR}"
        )

        # Provide the standard Eigen3::Eigen alias
        add_library(Eigen3::Eigen ALIAS eigen3_vendor)
    else()
        message(FATAL_ERROR
            "Eigen3 not found: vendored headers at "
            "${EIGEN3_VENDOR_DIR}/Eigen/Dense"
        )
    endif()

include(Dependencies)

# libode as a vendored submodule
set(LIBODE_SUBMODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/libode")

if (EXISTS "${LIBODE_SUBMODULE_DIR}/CMakeLists.txt")
    message(STATUS "Using vendored libode in ${LIBODE_SUBMODULE_DIR}")

    set(ODE_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(ODE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(ODE_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
    set(ODE_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)

    add_subdirectory("${LIBODE_SUBMODULE_DIR}" EXCLUDE_FROM_ALL)

    set(LIBODE_INCLUDE_DIR "${LIBODE_SUBMODULE_DIR}/include" CACHE PATH "libode include root")
else()
    message(FATAL_ERROR
        "libode submodule not found at ${LIBODE_SUBMODULE_DIR}. "
        "Did you run: git submodule update --init --recursive ?"
    )
endif()

find_package(fmu4cpp REQUIRED)



add_executable(aetherion
    src/main.cpp
    # add other .cpp files here
)

target_link_libraries(aetherion
    PRIVATE
        fmu4cpp::fmu4cpp
        ode::ode        
)

if (BUILD_DOCS)
  add_subdirectory(doc)
endif()

if (ENABLE_STATIC_ANALYSIS)
  include(cppcheck)
endif()

if(AETHERION_BUILD_TESTS)
   
	add_subdirectory(tests)
endif()


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

# --- Eigen as submodule ---
block(SCOPE_FOR VARIABLES)
    # Disable Eigen tests
    set(EIGEN_BUILD_TESTING OFF)
    set(EIGEN_TESTING OFF)          # for older Eigen versions
    set(BUILD_TESTING OFF)          # prevent CTest integration inside Eigen

    add_subdirectory(vendor/eigen)
endblock()

include(Dependencies)

find_package(fmu4cpp REQUIRED)
find_package(libode REQUIRED)

add_executable(aetherion
    src/main.cpp
    # add other .cpp files here
)

target_link_libraries(aetherion
    PRIVATE
        fmu4cpp::fmu4cpp
        libode::libode         # header-only, INTERFACE
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


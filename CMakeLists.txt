# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.26)

project(Aetherion VERSION 1.0 LANGUAGES CXX)

# ---- Hack for CppAD on Windows: it *requires* PkgConfig just to probe Eigen ----
if (WIN32)
    # Point PKG_CONFIG_EXECUTABLE at cmake.exe as a dummy.
    # FindPkgConfig only checks that the program exists; CppAD's Eigen
    # support is optional, so failed pkg-config calls will just mean
    # "no Eigen", which is fine for us.
    set(PKG_CONFIG_EXECUTABLE "${CMAKE_COMMAND}" CACHE FILEPATH "Dummy pkg-config for CppAD" FORCE)

    # Be explicit that we don't want Eigen support in CppAD anyway
    set(include_eigen OFF CACHE BOOL "Disable Eigen support in CppAD" FORCE)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

include(Dependencies)

# Enable testing option
option(BUILD_TESTING "Build unit tests" ON)

find_package(fmu4cpp REQUIRED)
find_package(libode REQUIRED)
find_package(CppAD REQUIRED)


target_link_libraries(aetherion
    PRIVATE
        fmu4cpp::fmu4cpp
        libode          # header-only, INTERFACE
)


# Only include tests if enabled
if(BUILD_TESTING)
    include(CTest)        # Defines BUILD_TESTING, adds test target
    add_subdirectory(tests)
endif()
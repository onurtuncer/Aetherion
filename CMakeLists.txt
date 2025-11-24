# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.26)

project(Aetherion VERSION 1.0 LANGUAGES CXX)

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
# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.26)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" projectVersion)
project(Aetherion VERSION ${projectVersion} LANGUAGES CXX)

option(FMU4CPP_BUILD_TESTS "Build internal tests" ON)
option(FMU4CPP_MSVC_STATIC_CRT "Use static MSVC runtime library" ON)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

if (BUILD_DOCS)
  add_subdirectory(doc)
  return()
endif()

include("cmake/generate_fmu.cmake")

# --- fmu4cpp vendor integration ---
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(FMU4CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FMU4CPP_TESTS OFF CACHE BOOL "" FORCE)

add_subdirectory(vendor/fmu4cpp/export EXCLUDE_FROM_ALL)

#add_subdirectory(${CMAKE_SOURCE_DIR}/vendor/fmu4cpp/export)

include(Dependencies)

add_subdirectory(src/FMU)

if(NOT BUILD_DOCS)

add_library(Aetherion STATIC)

target_sources(Aetherion PRIVATE
  #  src/Coordinate/ECEFToGeodetic.cpp
   src/Coordinate/InertialToLocal.cpp
   src/ODE/RKMK/Lie/SE3.cpp
   src/FlightDynamics/Serialization/JsonConfigNlohmannAdapter.cpp
   src/FlightDynamics/Serialization/InertialParametersJson.cpp
   src/FlightDynamics/Serialization/InitialPoseWGS84_NEDJson.cpp
   src/FlightDynamics/Serialization/AerodynamicParametersJson.cpp
   src/FlightDynamics/Serialization/SimulationParametersJson.cpp
   src/FlightDynamics/Serialization/FlightSimulationConfigJson.cpp
)

target_include_directories(Aetherion
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
        ${CMAKE_SOURCE_DIR}/vendor/eigen
    #    $<INSTALL_INTERFACE:include>
)

target_compile_features(Aetherion PUBLIC cxx_std_23)

if (ENABLE_STATIC_ANALYSIS)
  include(cppcheck)
endif()

if(AETHERION_BUILD_TESTS)
   
	add_subdirectory(tests)
endif()

endif() # Build docs


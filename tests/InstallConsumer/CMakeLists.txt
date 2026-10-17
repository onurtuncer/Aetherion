# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
#
# A consumer of an installed Aetherion SDK, built against the installation alone: no source tree, no build tree, no
# vcpkg. CI installs Aetherion, moves the installation to another directory, and builds and runs this project against it
# (pre-DOI audit F07: find_package(Aetherion) failed for every consumer, a public JSON header could not be compiled, and
# the CppAD library could be a dangling link). It also builds an FMU with generateFMU(), which referred to targets that
# exist only in Aetherion's own build tree (second pre-DOI audit, N05); CI then loads and steps it under FMPy.
#
# cmake -S tests/InstallConsumer -B consumer-build -DCMAKE_PREFIX_PATH=<moved prefix> cmake --build consumer-build &&
# consumer-build/aetherion_consumer && python tests/InstallConsumer/check_fmu.py
# consumer-build/models/fmi2/ConsumerAtmosphere.fmu
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.25)
project(AetherionInstallConsumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Aetherion REQUIRED)

add_executable(aetherion_consumer main.cpp)
target_link_libraries(aetherion_consumer PRIVATE Aetherion::Aetherion)
if(MSVC)
  target_compile_options(aetherion_consumer PRIVATE /utf-8 /bigobj)
endif()

# An FMU from the installation alone: fmu4cpp runtime, description generator and model template all come from it.
generatefmu(
  ConsumerAtmosphere
  FMI_VERSIONS
  fmi2
  SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/ConsumerAtmosphereFMU.cpp"
  LINK_TARGETS
  Aetherion::Aetherion
  DESTINATION
  "${CMAKE_BINARY_DIR}/models")
if(MSVC)
  target_compile_options(ConsumerAtmosphere_fmi2 PRIVATE /utf-8 /bigobj)
endif()

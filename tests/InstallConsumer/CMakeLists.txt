# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
#
# A consumer of an installed Aetherion SDK, built against the installation alone: no source tree, no build tree, no
# vcpkg. CI installs Aetherion, moves the installation to another directory, and builds and runs this project against it
# (pre-DOI audit F07: find_package(Aetherion) failed for every consumer, a public JSON header could not be compiled, and
# the CppAD library could be a dangling link).
#
# cmake -S tests/InstallConsumer -B consumer-build -DCMAKE_PREFIX_PATH=<moved prefix> cmake --build consumer-build &&
# consumer-build/aetherion_consumer
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

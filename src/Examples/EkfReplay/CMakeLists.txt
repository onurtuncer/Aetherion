# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/Examples/EkfReplay/CMakeLists.txt Replays the reference invariant EKF over Hemerion's co-simulation logs (sensor
# fusion step 6).

add_executable(EkfReplay ${CMAKE_CURRENT_SOURCE_DIR}/EkfReplay.cpp)

target_compile_features(EkfReplay PRIVATE cxx_std_23)

target_compile_options(EkfReplay PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>)

target_link_libraries(EkfReplay PRIVATE Aetherion::Aetherion Aetherion::Simulation)

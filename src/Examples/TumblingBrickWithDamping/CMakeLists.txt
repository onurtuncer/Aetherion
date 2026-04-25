# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/Examples/TumblingBrickWithDamping/CMakeLists.txt
#
# Builds the TumblingBrickWithDamping.exe example:
#   NASA TM-2015-218675 Atmospheric Scenario 3.
# ------------------------------------------------------------------------------

add_executable(TumblingBrickWithDamping
    ${CMAKE_CURRENT_SOURCE_DIR}/TumblingBrickWithDamping.cpp
)

target_compile_features(TumblingBrickWithDamping PRIVATE cxx_std_23)

# MSVC: force UTF-8 source encoding; /bigobj for CppAD-heavy TU section count
target_compile_options(TumblingBrickWithDamping PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>
)

target_link_libraries(TumblingBrickWithDamping
    PRIVATE
        Aetherion::Aetherion
        Aetherion::Simulation
)

add_custom_command(TARGET TumblingBrickWithDamping POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/data/Atmos_03_TumblingBrickWithDamping/nasa_2015_scenario3_tumbling_brick_with_damping.json"
        "$<TARGET_FILE_DIR:TumblingBrickWithDamping>/nasa_2015_scenario3_tumbling_brick_with_damping.json"
    COMMENT "Copying NASA Atmos-03 config to build directory"
)

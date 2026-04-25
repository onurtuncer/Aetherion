# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/Examples/TumblingBrickNoDamping/CMakeLists.txt
#
# Builds the TumblingBrickNoDamping.exe example:
#   NASA TM-2015-218675 Atmospheric Scenario 2.
#
# Links against:
#   Aetherion::Aetherion    — core physics (SE(3) ODE, coordinate transforms,
#                              FlightDynamics policies, RigidBody types)
#   Aetherion::Simulation   — Application, ISimulator, Snapshot1, Log (spdlog)
#   Eigen3::Eigen           — linear algebra (transitive via Aetherion)
# ------------------------------------------------------------------------------

add_executable(TumblingBrickNoDamping
    ${CMAKE_CURRENT_SOURCE_DIR}/TumblingBrickNoDamping.cpp
)

target_compile_features(TumblingBrickNoDamping PRIVATE cxx_std_23)

# MSVC: force UTF-8 source encoding; /bigobj for CppAD-heavy TU section count
target_compile_options(TumblingBrickNoDamping PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>
)

target_link_libraries(TumblingBrickNoDamping
    PRIVATE
        Aetherion::Aetherion
        Aetherion::Simulation
)

# ──────────────────────────────────────────────────────────────────────────────
# Copy the JSON config and reference data alongside the executable so users
# can run directly from the build directory:
#   TumblingBrickNoDamping.exe \
#       --inputFileName nasa_2015_scenario2_tumbling_brick_no_damping.json
# ──────────────────────────────────────────────────────────────────────────────
add_custom_command(TARGET TumblingBrickNoDamping POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/data/Atmos_02_TumblingBrickNoDamping/nasa_2015_scenario2_tumbling_brick_no_damping.json"
        "$<TARGET_FILE_DIR:TumblingBrickNoDamping>/nasa_2015_scenario2_tumbling_brick_no_damping.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/data/Atmos_02_TumblingBrickNoDamping/Atmos_02_TumblingBrickNoDamping/Atmos_02_sim_01.csv"
        "$<TARGET_FILE_DIR:TumblingBrickNoDamping>/Atmos_02_sim_01.csv"
    COMMENT "Copying NASA Atmos-02 config and reference data to build directory"
)

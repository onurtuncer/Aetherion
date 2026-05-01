# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/Examples/DraglessSphere/CMakeLists.txt
#
# Builds the DraglessSphere.exe example: NASA TM-2015-218675 Atmos Scenario 1.
#
# Links against:
#   Aetherion::Aetherion    — core physics (SE(3) ODE, coordinate transforms,
#                              FlightDynamics policies, RigidBody types)
#   Aetherion::Simulation   — Application, ISimulator, Snapshot1, Log (spdlog)
#   Eigen3::Eigen           — linear algebra (transitive via Aetherion)
# ------------------------------------------------------------------------------

add_executable(DraglessSphere
    ${CMAKE_CURRENT_SOURCE_DIR}/DraglessSphere.cpp
)

target_compile_features(DraglessSphere PRIVATE cxx_std_23)

# MSVC: force UTF-8 source encoding; /bigobj for ISimulator::snapshot2() inline body
target_compile_options(DraglessSphere PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>
)

target_link_libraries(DraglessSphere
    PRIVATE
        Aetherion::Aetherion
        Aetherion::Simulation
)

# ──────────────────────────────────────────────────────────────────────────────
# Install the reference JSON config alongside the executable so users can run
#   DraglessSphere.exe --inputFileName nasa_atmos_01_dragless_sphere.json
# directly from the build directory.
# ──────────────────────────────────────────────────────────────────────────────
add_custom_command(TARGET DraglessSphere POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/data/Atmos_01_DroppedSphere/nasa_2015_scenario1_dragless_sphere.json"
        "$<TARGET_FILE_DIR:DraglessSphere>/nasa_2015_scenario1_dragless_sphere.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/data/Atmos_01_DroppedSphere/Atmos_01_sim_01_si_units.csv"
        "$<TARGET_FILE_DIR:DraglessSphere>/Atmos_01_sim_01_si_units.csv"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/scripts/compare_sim_validation.py"
        "$<TARGET_FILE_DIR:DraglessSphere>/compare_sim_validation.py"
    COMMENT "Copying NASA Atmos-01 config, validation data and comparison script to build directory"
)
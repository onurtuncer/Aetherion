# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/Examples/SphereWithAtmosphericDrag/CMakeLists.txt
#
# Builds the SphereWithAtmosphericDrag.exe example:
#   NASA TM-2015-218675 Atmospheric Scenario 6.
#
# Links against:
#   Aetherion::Aetherion    — core physics (SE(3) ODE, coordinate transforms,
#                              FlightDynamics policies, RigidBody types)
#   Aetherion::Simulation   — Application, ISimulator, Snapshot1, Log (spdlog)
#   Eigen3::Eigen           — linear algebra (transitive via Aetherion)
# ------------------------------------------------------------------------------

add_executable(SphereWithAtmosphericDrag
    ${CMAKE_CURRENT_SOURCE_DIR}/SphereWithAtmosphericDrag.cpp
)

target_compile_features(SphereWithAtmosphericDrag PRIVATE cxx_std_23)

# MSVC: force UTF-8 source encoding; /bigobj for CppAD-heavy TU section count
target_compile_options(SphereWithAtmosphericDrag PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>
)

target_link_libraries(SphereWithAtmosphericDrag
    PRIVATE
        Aetherion::Aetherion
        Aetherion::Simulation
)

# ──────────────────────────────────────────────────────────────────────────────
# Copy the JSON config and validation script alongside the executable so users
# can run directly from the build directory:
#   SphereWithAtmosphericDrag.exe --inputFileName nasa_2015_scenario6_sphere_with_drag.json
#   python compare_sim_validation.py
# ──────────────────────────────────────────────────────────────────────────────
add_custom_command(TARGET SphereWithAtmosphericDrag POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/data/Atmos_06_SphereWithDrag/nasa_2015_scenario6_sphere_with_drag.json"
        "$<TARGET_FILE_DIR:SphereWithAtmosphericDrag>/nasa_2015_scenario6_sphere_with_drag.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/scripts/compare_sim_validation.py"
        "$<TARGET_FILE_DIR:SphereWithAtmosphericDrag>/compare_sim_validation.py"
    COMMENT "Copying NASA Atmos-06 config and validation script to build directory"
)

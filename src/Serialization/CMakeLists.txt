# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/Serialization/CMakeLists.txt
#
# Registers Serialization submodule sources into the Aetherion library target.
#
# All serializers use the nlohmann JSON adapter declared in
# JsonConfigNlohmannAdapter.cpp. That file must be listed first so the
# translation unit is available to the linker before the type-specific
# serializers that depend on it.
# ------------------------------------------------------------------------------

target_sources(Aetherion PRIVATE
    # Adapter (foundation � keep first)
    ${CMAKE_CURRENT_SOURCE_DIR}/JsonConfigNlohmannAdapter.cpp

    # Config loaders
    ${CMAKE_CURRENT_SOURCE_DIR}/LoadConfig.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/LoadConfigFromString.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ConfigJson.cpp

    # Physical parameter serializers
    ${CMAKE_CURRENT_SOURCE_DIR}/InertialParametersJson.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AerodynamicParametersJson.cpp

    # State / pose serializers
    ${CMAKE_CURRENT_SOURCE_DIR}/GeodeticPoseNEDJson.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/VelocityNEDJson.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BodyRatesJson.cpp
)

# DAVE-ML reader (pugixml-based, minimal feature set)
add_subdirectory(DAVEML)
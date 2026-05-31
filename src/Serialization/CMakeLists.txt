# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/Serialization/CMakeLists.txt
#
# Registers Serialization submodule sources into the Aetherion library target.
# ------------------------------------------------------------------------------

target_sources(
  Aetherion
  PRIVATE # Config loaders (top-level, not JSON-specific)
          ${CMAKE_CURRENT_SOURCE_DIR}/LoadConfig.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/LoadConfigFromString.cpp
          # JSON adapter + type serializers
          ${CMAKE_CURRENT_SOURCE_DIR}/Json/ConfigNlohmannAdapter.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/Json/Config.cpp
          # Physical parameter serializers
          ${CMAKE_CURRENT_SOURCE_DIR}/Json/InertialParameters.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/Json/AerodynamicParameters.cpp
          # State / pose serializers
          ${CMAKE_CURRENT_SOURCE_DIR}/Json/GeodeticPoseNED.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/Json/VelocityNED.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/Json/BodyRates.cpp)

# DAVE-ML reader (pugixml-based, minimal feature set)
add_subdirectory(DAVEML)

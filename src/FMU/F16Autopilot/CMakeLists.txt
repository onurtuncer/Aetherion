# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/FMU/F16Autopilot/CMakeLists.txt
#
# Builds the F16Autopilot FMI 2.0 Co-Simulation FMU.
#
# The FMU wraps the NASA LaRC F16_control.dml LQR autopilot (Bruce Jackson, NASA TM-2015-218675) evaluated via
# DAVEMLControlModel. F16_control.dml is staged into the FMU archive under resources/.
# ------------------------------------------------------------------------------

set(F16_DATA_DIR "${PROJECT_SOURCE_DIR}/data/F16_S119_source")

set(F16AP_STAGED_RES "${CMAKE_BINARY_DIR}/f16autopilot_resources")
file(MAKE_DIRECTORY "${F16AP_STAGED_RES}")
file(COPY "${F16_DATA_DIR}/F16_control.dml" DESTINATION "${F16AP_STAGED_RES}")

generatefmu(
  F16Autopilot
  FMI_VERSIONS
  fmi2
  SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/F16AutopilotFMU.cpp"
  INCLUDE_DIRS
  "${PROJECT_SOURCE_DIR}" # <Aetherion/...> headers
  "${PROJECT_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml" # DAVEMLReader → pugixml
  LINK_TARGETS
  Aetherion::Aetherion
  RESOURCE_FOLDER
  "${F16AP_STAGED_RES}"
  COMPILE_DEFINITIONS
  $<$<CXX_COMPILER_ID:MSVC>:_CRT_SECURE_NO_WARNINGS>)

if(MSVC)
  target_compile_options(F16Autopilot_fmi2 PRIVATE /utf-8 /bigobj)
endif()

if(AETHERION_INSTALL)
  install(FILES "${CMAKE_BINARY_DIR}/models/fmi2/F16Autopilot/F16Autopilot.fmu"
          DESTINATION "${CMAKE_INSTALL_DATADIR}/Aetherion/fmu" COMPONENT Runtime)
endif()

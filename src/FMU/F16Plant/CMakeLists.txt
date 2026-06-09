# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/FMU/F16Plant/CMakeLists.txt
#
# Builds the AethF16Plant FMI 2.0 Co-Simulation FMU.
#
# The DAVE-ML data files are copied into the FMU archive under resources/ by the RESOURCE_FOLDER argument.  At runtime
# the FMU reads them from fmu4cpp::fmu_base::resourceLocation().
# ------------------------------------------------------------------------------

set(F16_DATA_DIR "${CMAKE_SOURCE_DIR}/data/F16_S119_source")

generatefmu(
  F16Plant
  FMI_VERSIONS
  fmi2
  SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/F16PlantFMU.cpp"
  INCLUDE_DIRS
  "${CMAKE_SOURCE_DIR}" # <Aetherion/...> headers
  "${CMAKE_SOURCE_DIR}/vendor/eigen" # <Eigen/...>
  "${CMAKE_SOURCE_DIR}/vendor/CppAD/include" # <cppad/...>
  "${CMAKE_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml" # DAVEMLReader → pugixml
  LINK_TARGETS
  Aetherion::Aetherion
  CppAD::cppad
  RESOURCE_FOLDER
  "${F16_DATA_DIR}"
  COMPILE_DEFINITIONS
  $<$<CXX_COMPILER_ID:MSVC>:_CRT_SECURE_NO_WARNINGS>)

# MSVC-specific compile options not expressible as compile definitions. The generated target name follows the pattern
# <modelIdentifier>_<fmiVersion>.
if(MSVC)
  target_compile_options(F16Plant_fmi2 PRIVATE /utf-8 /bigobj)
endif()

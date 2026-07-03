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

# Stage only the three DAVE-ML files actually loaded at runtime. The source directory also contains control, gnc, and
# other files that are not used by F16PlantFMU and must not bloat the FMU archive.
set(F16_STAGED_RES "${CMAKE_BINARY_DIR}/f16plant_resources")
file(MAKE_DIRECTORY "${F16_STAGED_RES}")
foreach(_dml IN ITEMS F16_aero.dml F16_prop.dml F16_inertia.dml)
  file(COPY "${F16_DATA_DIR}/${_dml}" DESTINATION "${F16_STAGED_RES}")
endforeach()

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
  "${F16_STAGED_RES}"
  COMPILE_DEFINITIONS
  $<$<CXX_COMPILER_ID:MSVC>:_CRT_SECURE_NO_WARNINGS>)

# MSVC-specific compile options not expressible as compile definitions. The generated target name follows the pattern
# <modelIdentifier>_<fmiVersion>.
if(MSVC)
  target_compile_options(F16Plant_fmi2 PRIVATE /utf-8 /bigobj)
endif()

# Install the packaged FMU so it ends up in the WiX MSI and can be found by downstream co-simulation frameworks after
# cmake --install.
if(AETHERION_INSTALL)
  install(FILES "${CMAKE_BINARY_DIR}/models/fmi2/F16Plant/F16Plant.fmu"
          DESTINATION "${CMAKE_INSTALL_DATADIR}/Aetherion/fmu" COMPONENT Runtime)
endif()

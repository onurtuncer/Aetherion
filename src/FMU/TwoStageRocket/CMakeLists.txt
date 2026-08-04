# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/FMU/TwoStageRocket/CMakeLists.txt
#
# Builds the TwoStageRocket FMI 2.0 Co-Simulation FMU.
#
# The DAVE-ML data files are copied into the FMU archive under resources/ by the RESOURCE_FOLDER argument. At runtime
# the FMU reads them from fmu4cpp::fmu_base::resourceLocation(). RocketStageModel.cpp is not part of the Aetherion
# library (it is only otherwise compiled into the TwoStageRocket example executable), so it is listed directly here.
# ------------------------------------------------------------------------------

set(TWOSTAGE_DATA_DIR "${PROJECT_SOURCE_DIR}/data/TwoStageRocket_S119_source")

set(TWOSTAGE_STAGED_RES "${CMAKE_BINARY_DIR}/twostagerocket_resources")
file(MAKE_DIRECTORY "${TWOSTAGE_STAGED_RES}")
foreach(_dml IN ITEMS twostage_inertia.dml twostage_prop.dml twostage_aero.dml)
  file(COPY "${TWOSTAGE_DATA_DIR}/${_dml}" DESTINATION "${TWOSTAGE_STAGED_RES}")
endforeach()

generatefmu(
  TwoStageRocket
  FMI_VERSIONS
  fmi2
  SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/TwoStageRocketFMU.cpp"
  "${PROJECT_SOURCE_DIR}/src/Examples/TwoStageRocket/RocketStageModel.cpp"
  INCLUDE_DIRS
  "${PROJECT_SOURCE_DIR}" # <Aetherion/...> headers
  "${PROJECT_SOURCE_DIR}/vendor/eigen" # <Eigen/...>
  "${PROJECT_SOURCE_DIR}/vendor/CppAD/include" # <cppad/...>
  "${PROJECT_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml" # DAVEMLReader → pugixml
  LINK_TARGETS
  Aetherion::Aetherion
  CppAD::cppad
  RESOURCE_FOLDER
  "${TWOSTAGE_STAGED_RES}"
  COMPILE_DEFINITIONS
  $<$<CXX_COMPILER_ID:MSVC>:_CRT_SECURE_NO_WARNINGS>)

# MSVC-specific compile options not expressible as compile definitions. The generated target name follows the pattern
# <modelIdentifier>_<fmiVersion>.
if(MSVC)
  target_compile_options(TwoStageRocket_fmi2 PRIVATE /utf-8 /bigobj)
endif()

# Install the packaged FMU so it ends up in the WiX MSI and can be found by downstream co-simulation frameworks after
# cmake --install.
if(AETHERION_INSTALL)
  install(FILES "${CMAKE_BINARY_DIR}/models/fmi2/TwoStageRocket/TwoStageRocket.fmu"
          DESTINATION "${CMAKE_INSTALL_DATADIR}/Aetherion/fmu" COMPONENT Runtime)
endif()

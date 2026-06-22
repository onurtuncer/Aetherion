# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
# Packaging.cmake
#
# CPack configuration for producing a Windows MSI installer via the WiX Toolset (v3, candle/light) generator. Included
# from the root CMakeLists.txt when AETHERION_INSTALL=ON.
# ------------------------------------------------------------------------------

set(CPACK_PACKAGE_NAME "Aetherion")
set(CPACK_PACKAGE_VENDOR "Onur Tuncer, PhD - Istanbul Technical University")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "Aetherion")
set(CPACK_PACKAGE_CONTACT "https://github.com/onurtuncer/Aetherion")

# WiX requires the license as plain text or RTF; the repo's LICENSE has no extension, so stage a renamed copy for CPack
# to pick up.
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_BINARY_DIR}/LICENSE.txt")
configure_file("${CMAKE_SOURCE_DIR}/LICENSE" "${CPACK_RESOURCE_FILE_LICENSE}" COPYONLY)

if(WIN32)
  set(CPACK_GENERATOR "WIX")
  set(CPACK_WIX_PROGRAM_MENU_FOLDER "Aetherion")

  # Fixed upgrade GUID so later versions upgrade this install in place instead of installing side-by-side. Must never
  # change once released.
  set(CPACK_WIX_UPGRADE_GUID "B13AB45B-155E-4B79-A4D0-4103A71DE9A5")
endif()

include(CPack)

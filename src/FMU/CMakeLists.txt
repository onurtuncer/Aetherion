# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------

generatefmu(
  FeedThrough
  FMI_VERSIONS
  fmi2
  SOURCES
  FeedThrough/FeedThrough.cpp)

add_subdirectory(F16Plant)

# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/ODE/CMakeLists.txt
#
# Registers ODE module sources into the Aetherion library target.
#
# Structure:
#   ODE/
#   └── RKMK/              — Runge-Kutta Munthe-Kaas integrators
#       └── Lie/           — Lie group geometry (SE3, SO3, …)
#           └── SE3.cpp    — Special Euclidean group SE(3) implementation
#
# As new integrators or Lie group implementations are added (e.g. SO3.cpp,
# RK4.cpp), append them here under their respective subdirectory comment.
# ------------------------------------------------------------------------------

# --- RKMK / Lie group geometry ------------------------------------------------
target_sources(Aetherion PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/RKMK/Lie/SE3.cpp
)

# --- Future: additional integrators / Lie groups ------------------------------
# target_sources(Aetherion PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/RKMK/Lie/SO3.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/RKMK/RK4/RK4.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/RadauIIA/RadauIIA.cpp
# )
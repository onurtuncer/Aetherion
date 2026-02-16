# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_json
                  #  test_JsonConfig.cpp 
                    test_InitialPoseWGS84_NEDJson.cpp
                    test_InertialParametersJson.cpp 
                    test_SimulationParametersJson.cpp
                    test_AerodynamicParametersJson.cpp 
                    test_FlightSimulationConfigJson.cpp)

target_link_libraries(test_json
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
)

#target_include_directories(test_json PRIVATE "${CMAKE_SOURCE_DIR}")
                                               
add_test(NAME json_tests COMMAND test_json)
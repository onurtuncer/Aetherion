# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_json
                    test_GeodeticPoseNEDJson.cpp
                    test_InertialParametersJson.cpp 
                    test_RotationRateAboutBodyAxesJson.cpp 
                    test_SimulationParametersJson.cpp
                    test_AerodynamicParametersJson.cpp 
                    test_ConfigJson.cpp
                    test_VelocityNEDJson.cpp
                   )

target_link_libraries(test_json
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(test_json)
                                               
add_executable(test_load_file test_LoadSimulationConfigFromFile.cpp)

target_link_libraries(test_load_file 
    PRIVATE
        Aetherion
        Catch2::Catch2)

include(Catch)
catch_discover_tests(test_load_file TEST_SPEC "~[external_file]")

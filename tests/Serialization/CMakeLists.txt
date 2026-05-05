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
                    test_BodyRatesJson.cpp              
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
                                               
add_executable(test_load_file "test_LoadConfig.cpp")

target_link_libraries(test_load_file
    PRIVATE
        Aetherion
        Catch2::Catch2)

catch_discover_tests(test_load_file TEST_SPEC "~[external_file]")

add_executable(test_load_config_from_string "test_LoadConfigFromString.cpp")

target_link_libraries(test_load_config_from_string
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain)

catch_discover_tests(test_load_config_from_string)

# DAVE-ML reader smoke tests — path to F-16 inertia file injected at configure time
set(DAVEML_F16_INERTIA "${CMAKE_SOURCE_DIR}/data/F16_S119_source/F16_inertia.dml")

add_executable(test_daveml_reader test_DAVEMLReader.cpp)

target_compile_definitions(test_daveml_reader PRIVATE
    "DAVEML_TEST_FILE=\"${DAVEML_F16_INERTIA}\"")

target_compile_options(test_daveml_reader PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_include_directories(test_daveml_reader PRIVATE
    "${CMAKE_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml")

target_link_libraries(test_daveml_reader
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain)

catch_discover_tests(test_daveml_reader)

# DAVE-ML aero model smoke tests
set(DAVEML_F16_AERO "${CMAKE_SOURCE_DIR}/data/F16_S119_source/F16_aero.dml")

add_executable(test_daveml_aero_model test_DAVEMLAeroModel.cpp)

target_compile_definitions(test_daveml_aero_model PRIVATE
    "DAVEML_AERO_FILE=\"${DAVEML_F16_AERO}\"")

target_compile_options(test_daveml_aero_model PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_include_directories(test_daveml_aero_model PRIVATE
    "${CMAKE_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml"
    "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

target_link_libraries(test_daveml_aero_model
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad)

catch_discover_tests(test_daveml_aero_model)

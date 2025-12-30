# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_eigen
    test_eigen.cpp
)

target_link_libraries(test_eigen
    PRIVATE
        Catch2::Catch2WithMain
        Eigen3::Eigen       
)

target_include_directories(test_eigen PRIVATE "${CMAKE_SOURCE_DIR}/vendor/eigen")

add_test(NAME eigen_basic_test COMMAND test_eigen)

add_executable(test_nlohmann
    test_nlohmann_json.cpp
)

target_link_libraries(test_nlohmann
    PRIVATE
        Catch2::Catch2WithMain
        nlohmann_json      
)

# target_include_directories(test_eigen PRIVATE "${CMAKE_SOURCE_DIR}/vendor/nlohmann")

add_test(NAME nlohmann_json_test COMMAND test_nlohmann)

add_executable(test_cppad
    test_cppad.cpp
)

target_link_libraries(test_cppad
    PRIVATE
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

target_include_directories(test_cppad PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

add_test(NAME cppad_basic_test COMMAND test_cppad)
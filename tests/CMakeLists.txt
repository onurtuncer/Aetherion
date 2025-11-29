# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

FetchContent_Declare(
    catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.2
)
FetchContent_MakeAvailable(catch2)

# Tell CMake where to look for the installed package
#list(APPEND CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/vendor/catch2)
#
#find_package(Catch2 3 CONFIG REQUIRED)


include(CTest)

add_executable(test_libode
    test_libode.cpp
)

target_link_libraries(test_libode
    PRIVATE
        Catch2::Catch2WithMain
        ode::ode
)

target_include_directories(test_libode
    PRIVATE
        "${LIBODE_INCLUDE_DIR}"
)

add_test(NAME libode_basic_test COMMAND test_libode)

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



# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# -----------------------
# Catch2 via FetchContent
# -----------------------

FetchContent_Declare(
    catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.2
)
FetchContent_MakeAvailable(catch2)

# -----------------------
# Test executable
# -----------------------
add_executable(test_libode
    test_libode.cpp
)

target_link_libraries(test_libode
    PRIVATE
        Catch2::Catch2WithMain
        libode::libode
)

# if you have libode_SOURCE_DIR from FetchContent_GetProperties(libode)
#target_include_directories(test_libode
#    PRIVATE
#        "${libode_SOURCE_DIR}/include"
#)

add_test(NAME libode_basic_test COMMAND test_libode)
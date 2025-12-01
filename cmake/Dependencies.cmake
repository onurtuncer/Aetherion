# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

if(BUILD_DOCS)
    return()
endif()



include(FetchContent)

#FetchContent_Declare(
#    fmu4cpp
#    GIT_REPOSITORY https://github.com/Ecos-platform/fmu4cpp.git
#    GIT_TAG        master
#)
#
#FetchContent_MakeAvailable(fmu4cpp)


#if(COMMAND generate_fmu)
#    message(STATUS "generate_fmu() is available after include")
#else()
#    message(FATAL_ERROR "generate_fmu() is NOT available after include")
#endif()

#set(ECOS_BUILD_CLI OFF)     # Set to ON for building ecos command-line-interface
#set(ECOS_BUILD_CLIB OFF)    # Set to ON for building C API
#set(ECOS_WITH_PROXYFMU OFF) # Set to ON for remoting
#
#FetchContent_Declare(
#        ecos
#        GIT_REPOSITORY https://github.com/Ecos-platform/ecos.git
#        GIT_TAG master
#)
#
#FetchContent_MakeAvailable(ecos)



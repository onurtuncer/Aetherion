// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
// FeedThrough.cpp
//
// Translation-unit for the FeedThrough FMU artifact (shared library).
// The model implementation lives in FeedThrough.hpp so the same code is
// shared with the Catch2 integration test (test_feedthrough.cpp).
// model_identifier() is provided by the CMake-generated
// model_identifier_FeedThrough_fmi2.cpp when building via generateFMU().
// ------------------------------------------------------------------------------

#include "FeedThrough.hpp"

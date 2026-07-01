// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
// FeedThrough.hpp
//
// Simple pass-through FMU: each output equals the corresponding input.
// Used as a smoke test for the fmu4cpp FMI 2.0 Co-Simulation integration.
// Header-only so the same translation unit can be shared by the FMU shared
// library build and the Catch2 in-process integration test.
// ------------------------------------------------------------------------------
#pragma once

#include <fmu4cpp/fmu_base.hpp>
#include <fmu4cpp/model_info.hpp>
#include <string>

using namespace fmu4cpp;

class FeedThrough : public fmu_base {
public:
    FMU4CPP_CTOR(FeedThrough)
    {
        register_integer("integerIn", &integerIn_)
            .setCausality(causality_t::INPUT)
            .setVariability(variability_t::DISCRETE);

        register_real("realIn", &realIn_)
            .setCausality(causality_t::INPUT)
            .setVariability(variability_t::DISCRETE);

        register_boolean("booleanIn", &booleanIn_)
            .setCausality(causality_t::INPUT)
            .setVariability(variability_t::DISCRETE);

        register_string("stringIn", &stringIn_)
            .setCausality(causality_t::INPUT)
            .setVariability(variability_t::DISCRETE);

        register_integer("integerOut", &integerOut_)
            .setCausality(causality_t::OUTPUT)
            .setVariability(variability_t::DISCRETE);

        register_real("realOut", &realOut_)
            .setCausality(causality_t::OUTPUT)
            .setVariability(variability_t::DISCRETE);

        register_boolean("booleanOut", &booleanOut_)
            .setCausality(causality_t::OUTPUT)
            .setVariability(variability_t::DISCRETE);

        register_string("stringOut", &stringOut_)
            .setCausality(causality_t::OUTPUT)
            .setVariability(variability_t::DISCRETE);

        FeedThrough::reset();
    }

    bool do_step(double /*dt*/) override
    {
        integerOut_ = integerIn_;
        realOut_    = realIn_;
        booleanOut_ = booleanIn_;
        stringOut_  = stringIn_;
        return true;
    }

    void reset() override
    {
        integerIn_  = 0;
        realIn_     = 0.0;
        booleanIn_  = false;
        stringIn_   = "empty";

        integerOut_ = 0;
        realOut_    = 0.0;
        booleanOut_ = false;
        stringOut_  = "empty";
    }

private:
    int         integerIn_{};
    double      realIn_{};
    bool        booleanIn_{};
    std::string stringIn_;

    int         integerOut_{};
    double      realOut_{};
    bool        booleanOut_{};
    std::string stringOut_;
};

inline model_info fmu4cpp::get_model_info()
{
    model_info info;
    info.modelName   = "FeedThrough";
    info.description = "Simple pass-through FMU (output = input) for fmu4cpp integration testing";
    return info;
}

FMU4CPP_INSTANTIATE(FeedThrough);

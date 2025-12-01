// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
// Project: Aetherion
// Feed-through FMU example using fmu4cpp
// ------------------------------------------------------------------------------

#include <fmu4cpp/fmu_base.hpp>
#include <string>

using namespace fmu4cpp;

class FeedThrough : public fmu_base {
public:
    // Let the macro handle constructor signature + base-class wiring
    FMU4CPP_CTOR(FeedThrough)
    {
        // Inputs
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

        // Outputs – simple feed-through of the inputs
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

    // New API: do_step takes only dt
    bool do_step(double dt) override
    {
        (void)dt; // unused for pure feed-through model

        // Feed-through: copy inputs to outputs
        integerOut_ = integerIn_;
        realOut_ = realIn_;
        booleanOut_ = booleanIn_;
        stringOut_ = stringIn_;

        // Optional logging
       // debugLog(fmi2OK, "FeedThrough::do_step called");

        return true;
    }

    void reset() override
    {
        integerIn_ = 0;
        realIn_ = 0.0;
        booleanIn_ = false;
        stringIn_ = "empty";

        integerOut_ = 0;
        realOut_ = 0.0;
        booleanOut_ = false;
        stringOut_ = "empty";
    }

private:
    // Inputs
    int         integerIn_{};
    double      realIn_{};
    bool        booleanIn_{};
    std::string stringIn_;

    // Outputs
    int         integerOut_{};
    double      realOut_{};
    bool        booleanOut_{};
    std::string stringOut_;
};

// Model metadata (FMI modelDescription.xml content)
model_info fmu4cpp::get_model_info()
{
    model_info info;
    info.modelName = "FeedThrough";
    info.description = "A simple feed-through FMU (in = out)";
    return info;
}

// Boilerplate to expose the model to fmu4cpp runtime
FMU4CPP_INSTANTIATE(FeedThrough);

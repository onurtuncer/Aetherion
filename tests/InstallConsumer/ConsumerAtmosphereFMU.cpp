// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// An FMU built by generateFMU() in a project that has only the installed SDK
// (second pre-DOI audit, finding N05: generateFMU() referred to targets that
// exist only in Aetherion's own build tree). It uses Aetherion as well as
// fmu4cpp: out.rho_kg_m3 is the US1976 density at in.alt_m.
// ------------------------------------------------------------------------------

#include <fmu4cpp/fmu_base.hpp>
#include <fmu4cpp/model_info.hpp>

#include <Aetherion/Environment/Atmosphere.h>

using namespace fmu4cpp;

class ConsumerAtmosphere : public fmu_base
{
public:
    FMU4CPP_CTOR(ConsumerAtmosphere)
    {
        register_real("in.alt_m", &alt_m_)
            .setCausality(causality_t::INPUT)
            .setVariability(variability_t::CONTINUOUS);
        register_real("out.rho_kg_m3", &rho_kg_m3_)
            .setCausality(causality_t::OUTPUT)
            .setVariability(variability_t::CONTINUOUS)
            .setInitial(initial_t::CALCULATED);
        ConsumerAtmosphere::reset();
    }

    bool do_step(double /*dt*/) override
    {
        rho_kg_m3_ = Aetherion::Environment::US1976Atmosphere(alt_m_).rho;
        return true;
    }

    void reset() override
    {
        alt_m_ = 0.0;
        rho_kg_m3_ = Aetherion::Environment::US1976Atmosphere(0.0).rho;
    }

private:
    double alt_m_{};
    double rho_kg_m3_{};
};

model_info fmu4cpp::get_model_info()
{
    model_info info;
    info.modelName = "ConsumerAtmosphere";
    info.description = "US1976 density at an input altitude; built against an installed Aetherion SDK";
    return info;
}

FMU4CPP_INSTANTIATE(ConsumerAtmosphere);

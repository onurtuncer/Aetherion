// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <Aetherion/Examples/TwoStageRocket/RocketStageModel.h>

#include <algorithm>
#include <stdexcept>
#include <unordered_map>

namespace Aetherion::Examples::TwoStageRocket {

RocketStageModel::RocketStageModel(
    std::shared_ptr<Serialization::DAVEMLAeroModel> inertiaDml,
    std::shared_ptr<Serialization::DAVEMLAeroModel> propDml)
    : m_inertiaDml(std::move(inertiaDml))
    , m_propDml   (std::move(propDml))
{
    if (!m_inertiaDml) throw std::invalid_argument("RocketStageModel: inertiaDml is null");
    if (!m_propDml)    throw std::invalid_argument("RocketStageModel: propDml is null");
}

// ── propulsion ────────────────────────────────────────────────────────────────

RocketPropulsionResult RocketStageModel::propulsion(double t_sim) const
{
    const bool s1Firing = !m_fuel.staged &&
                           (m_fuel.stg1FuelUsed_kg < kStg1MaxFuel_kg);
    const bool s2Firing =  m_fuel.staged &&
                           (m_fuel.stg2FuelUsed_kg < kStg2MaxFuel_kg) &&
                           (t_sim >= stg2IgnitionTime_s);

    std::unordered_map<std::string, double> vars;
    vars["stg1firing"] = s1Firing ? 1.0 : 0.0;
    vars["stg2firing"] = s2Firing ? 1.0 : 0.0;

    const auto result = m_propDml->evaluateRaw<double>(std::move(vars));

    auto get = [&](const char* id) -> double {
        auto it = result.find(id);
        return (it != result.end()) ? it->second : 0.0;
    };

    RocketPropulsionResult out;
    out.thrust_N = get("thrust");  // bodyThrustForce_X [N]
    out.mdot_kgs = get("mdot");    // propellant consumption rate [kg/s]
    return out;
}

// ── inertialParameters ────────────────────────────────────────────────────────

RigidBody::InertialParameters RocketStageModel::inertialParameters() const
{
    std::unordered_map<std::string, double> vars;
    vars["stagedFlag"]   = m_fuel.staged ? 1.0 : 0.0;
    vars["stg1fuelUsed"] = m_fuel.stg1FuelUsed_kg;
    vars["stg2fuelUsed"] = m_fuel.stg2FuelUsed_kg;

    const auto result = m_inertiaDml->evaluateRaw<double>(std::move(vars));

    auto get = [&](const char* id) -> double {
        auto it = result.find(id);
        return (it != result.end()) ? it->second : 0.0;
    };

    RigidBody::InertialParameters ip;
    ip.mass_kg = get("XMASS");
    ip.Ixx     = get("XIXX");
    ip.Iyy     = get("XIYY");
    ip.Izz     = get("XIZZ");  // = XIYY for this axisymmetric vehicle
    ip.Ixz     = get("XIZX");  // zero
    ip.Ixy     = get("XIXY");  // zero
    ip.Iyz     = get("XIYZ");  // zero
    // DXCG [m, sign=+FWD]: CG is DXCG forward of MRC in body +x.
    // InertialParameters.xbar_m = x-offset of CG from body origin (MRC)
    // → positive when CG is forward of MRC, consistent with VectorField coupling.
    ip.xbar_m  = get("DXCG");
    ip.ybar_m  = 0.0;          // DYCG = 0 (axisymmetric)
    ip.zbar_m  = 0.0;          // DZCG = 0 (axisymmetric)
    return ip;
}

// ── advance ───────────────────────────────────────────────────────────────────

bool RocketStageModel::advance(double dt_s, double mdot_kgs)
{
    if (m_fuel.staged) {
        m_fuel.stg2FuelUsed_kg = std::min(
            m_fuel.stg2FuelUsed_kg + mdot_kgs * dt_s,
            kStg2MaxFuel_kg);
        return false;
    }

    m_fuel.stg1FuelUsed_kg += mdot_kgs * dt_s;

    if (m_fuel.stg1FuelUsed_kg >= kStg1MaxFuel_kg) {
        m_fuel.stg1FuelUsed_kg = kStg1MaxFuel_kg;
        m_fuel.staged          = true;
        m_fuel.stg2FuelUsed_kg = 0.0;
        return true;  // caller must drop kStg1DryMass_kg from state
    }
    return false;
}

} // namespace Aetherion::Examples::TwoStageRocket

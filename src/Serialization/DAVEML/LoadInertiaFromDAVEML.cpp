// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <Aetherion/Serialization/DAVEML/LoadInertiaFromDAVEML.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLReader.h>

namespace Aetherion::Serialization {

RigidBody::InertialParameters LoadInertiaFromDAVEML(
    const std::string& path,
    const std::unordered_map<std::string, double>& inputs)
{
    DAVEMLReader reader(path);

    // Set any user-supplied input overrides (e.g. CG position)
    for (const auto& [id, val] : inputs)
        reader.setInput(id, val);

    // Helper: return SI value if varID exists, else 0.0
    auto getSI = [&](const char* id) -> double {
        return reader.hasVar(id) ? reader.getValueSI(id) : 0.0;
    };

    RigidBody::InertialParameters ip;

    ip.mass_kg = getSI("XMASS");

    ip.Ixx = getSI("XIXX");
    ip.Iyy = getSI("XIYY");
    ip.Izz = getSI("XIZZ");
    ip.Ixz = getSI("XIZX");   // DAVE-ML sign: XIZX is the XZ cross-product
    ip.Ixy = getSI("XIXY");
    ip.Iyz = getSI("XIYZ");

    // CG offsets (DAVE-ML sign conventions vary by file;
    // DXCG is positive FORWARD, DYCG positive RIGHT, DZCG positive DOWN)
    ip.xbar_m = getSI("DXCG");
    ip.ybar_m = getSI("DYCG");
    ip.zbar_m = getSI("DZCG");

    return ip;
}

} // namespace Aetherion::Serialization

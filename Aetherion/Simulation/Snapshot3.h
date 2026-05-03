// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Snapshot3.h  [PLACEHOLDER]
//
// Currently mirrors the Snapshot2 (31-column NASA) schema.
// Extend this struct when the scenario assigned to SnapshotFormat::Three
// is designed.  Steps:
//   1. Add / remove fields in Snapshot3.
//   2. Update Snapshot3CsvTraits::kColumnNames to match.
//   3. Update Snapshot3_WriteCsvRow to serialise every field.
//   4. Update SnapshotTraits<SnapshotFormat::Three>::column_count.
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>
#include <iomanip>
#include <limits>
#include <ostream>
#include <array>
#include <string_view>

namespace Aetherion::Simulation {

/// @brief Placeholder snapshot — mirrors Snapshot2 (31 cols).
/// @todo Extend for the scenario assigned to SnapshotFormat::Three.
struct Snapshot3
{
    double time{ 0.0 };
    Eigen::Vector3d gePosition_m{};
    Eigen::Vector3d feVelocity_m_s{};
    double altitudeMsl_m          { 0.0 };
    double longitude_rad          { 0.0 };
    double latitude_rad           { 0.0 };
    double localGravity_m_s2      { 0.0 };
    double eulerAngle_rad_Yaw     { 0.0 };
    double eulerAngle_rad_Pitch   { 0.0 };
    double eulerAngle_rad_Roll    { 0.0 };
    double bodyAngularRateWrtEi_rad_s_Roll { 0.0 };
    double bodyAngularRateWrtEi_rad_s_Pitch{ 0.0 };
    double bodyAngularRateWrtEi_rad_s_Yaw  { 0.0 };
    double altitudeRateWrtMsl_m_s { 0.0 };
    double speedOfSound_m_s       { 0.0 };
    double airDensity_kg_m3       { 0.0 };
    double ambientPressure_Pa     { 0.0 };
    double ambientTemperature_K   { 0.0 };
    double aero_bodyForce_N_X     { 0.0 };
    double aero_bodyForce_N_Y     { 0.0 };
    double aero_bodyForce_N_Z     { 0.0 };
    double aero_bodyMoment_Nm_L   { 0.0 };
    double aero_bodyMoment_Nm_M   { 0.0 };
    double aero_bodyMoment_Nm_N   { 0.0 };
    double mach                   { 0.0 };
    double dynamicPressure_Pa     { 0.0 };
    double trueAirspeed_m_s       { 0.0 };
};

struct Snapshot3CsvTraits
{
    static constexpr std::array kColumnNames = {
        std::string_view{"time"},
        std::string_view{"gePosition_m_X"},   std::string_view{"gePosition_m_Y"},   std::string_view{"gePosition_m_Z"},
        std::string_view{"feVelocity_m_s_X"}, std::string_view{"feVelocity_m_s_Y"}, std::string_view{"feVelocity_m_s_Z"},
        std::string_view{"altitudeMsl_m"},    std::string_view{"longitude_rad"},     std::string_view{"latitude_rad"},
        std::string_view{"localGravity_m_s2"},
        std::string_view{"eulerAngle_rad_Yaw"}, std::string_view{"eulerAngle_rad_Pitch"}, std::string_view{"eulerAngle_rad_Roll"},
        std::string_view{"bodyAngularRateWrtEi_rad_s_Roll"},
        std::string_view{"bodyAngularRateWrtEi_rad_s_Pitch"},
        std::string_view{"bodyAngularRateWrtEi_rad_s_Yaw"},
        std::string_view{"altitudeRateWrtMsl_m_s"},
        std::string_view{"speedOfSound_m_s"},    std::string_view{"airDensity_kg_m3"},
        std::string_view{"ambientPressure_Pa"},  std::string_view{"ambientTemperature_K"},
        std::string_view{"aero_bodyForce_N_X"},  std::string_view{"aero_bodyForce_N_Y"},  std::string_view{"aero_bodyForce_N_Z"},
        std::string_view{"aero_bodyMoment_Nm_L"},std::string_view{"aero_bodyMoment_Nm_M"},std::string_view{"aero_bodyMoment_Nm_N"},
        std::string_view{"mach"}, std::string_view{"dynamicPressure_Pa"}, std::string_view{"trueAirspeed_m_s"},
    };
    static constexpr std::size_t kColumnCount = kColumnNames.size();
};

inline void Snapshot3_WriteCsvHeader(std::ostream& os)
{
    for (std::size_t i = 0; i < Snapshot3CsvTraits::kColumnCount; ++i)
    { if (i) os << ','; os << Snapshot3CsvTraits::kColumnNames[i]; }
    os << '\n';
}

inline void Snapshot3_WriteCsvRow(std::ostream& os, const Snapshot3& s)
{
    os << std::scientific << std::setprecision(std::numeric_limits<double>::max_digits10);
    const auto w = [&](double v){ os << ',' << v; };
    os << s.time;
    w(s.gePosition_m.x());     w(s.gePosition_m.y());     w(s.gePosition_m.z());
    w(s.feVelocity_m_s.x());   w(s.feVelocity_m_s.y());   w(s.feVelocity_m_s.z());
    w(s.altitudeMsl_m);        w(s.longitude_rad);         w(s.latitude_rad);
    w(s.localGravity_m_s2);
    w(s.eulerAngle_rad_Yaw);   w(s.eulerAngle_rad_Pitch);  w(s.eulerAngle_rad_Roll);
    w(s.bodyAngularRateWrtEi_rad_s_Roll);
    w(s.bodyAngularRateWrtEi_rad_s_Pitch);
    w(s.bodyAngularRateWrtEi_rad_s_Yaw);
    w(s.altitudeRateWrtMsl_m_s);
    w(s.speedOfSound_m_s);     w(s.airDensity_kg_m3);
    w(s.ambientPressure_Pa);   w(s.ambientTemperature_K);
    w(s.aero_bodyForce_N_X);   w(s.aero_bodyForce_N_Y);   w(s.aero_bodyForce_N_Z);
    w(s.aero_bodyMoment_Nm_L); w(s.aero_bodyMoment_Nm_M); w(s.aero_bodyMoment_Nm_N);
    w(s.mach); w(s.dynamicPressure_Pa); w(s.trueAirspeed_m_s);
    os << '\n';
}

} // namespace Aetherion::Simulation

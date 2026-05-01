// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Snapshot2.h
//
// NASA TM-2015-218675-compatible observable state snapshot: exactly the 31
// columns present in every Atmos_0N_sim_0M_si_units.csv reference file.
//
// Differences from Snapshot1 (38 columns):
//   REMOVED  v_eci_X/Y/Z             (Aetherion-internal, not in NASA reference)
//   REMOVED  q_body_to_eci_W/X/Y/Z   (Aetherion-internal, not in NASA reference)
//   SAME     all 31 NASA columns including aero_bodyForce_N_* and
//            aero_bodyMoment_Nm_* which are now populated via MakeSnapshot2.
//
// Use Snapshot2 when you want a 1-to-1 column match with the NASA reference
// CSVs, e.g. to run compare_sim_validation.py without the "only N common
// columns" message.
//
// The companion MakeSnapshot2.h converts a RigidBody::StateD + gravity policy
// + aero policy into a fully populated Snapshot2.
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>
#include <iomanip>
#include <limits>
#include <ostream>
#include <array>
#include <string_view>

namespace Aetherion::Simulation {

/// @brief NASA TM-2015-218675 compatible state snapshot — 31 columns.
///
/// Field names and units match the NASA SI-units reference CSV columns
/// exactly, enabling direct column-by-column comparison with
/// ``Atmos_0N_sim_0M_si_units.csv``.
struct Snapshot2
{
    /// @name Time
    ///@{
    double time{ 0.0 };
    ///@}

    /// @name Kinematics
    ///@{
    Eigen::Vector3d gePosition_m{};      ///< ECEF position [m].
    Eigen::Vector3d feVelocity_m_s{};    ///< Earth-relative (NED) velocity [m/s].

    double altitudeMsl_m  { 0.0 };       ///< Geodetic altitude [m].
    double longitude_rad  { 0.0 };       ///< Geodetic longitude [rad].
    double latitude_rad   { 0.0 };       ///< Geodetic latitude [rad].
    double localGravity_m_s2{ 0.0 };     ///< Gravity magnitude [m/s²].
    ///@}

    /// @name Attitude
    ///@{
    double eulerAngle_rad_Yaw  { 0.0 };  ///< ZYX Euler yaw   [rad].
    double eulerAngle_rad_Pitch{ 0.0 };  ///< ZYX Euler pitch [rad].
    double eulerAngle_rad_Roll { 0.0 };  ///< ZYX Euler roll  [rad].
    ///@}

    /// @name Angular rates and kinematics
    ///@{
    double bodyAngularRateWrtEi_rad_s_Roll { 0.0 }; ///< Body roll  rate wrt ECI [rad/s].
    double bodyAngularRateWrtEi_rad_s_Pitch{ 0.0 }; ///< Body pitch rate wrt ECI [rad/s].
    double bodyAngularRateWrtEi_rad_s_Yaw  { 0.0 }; ///< Body yaw   rate wrt ECI [rad/s].
    double altitudeRateWrtMsl_m_s          { 0.0 }; ///< dh/dt [m/s] (= −v_NED_down).
    ///@}

    /// @name Atmosphere (US 1976 Standard)
    ///@{
    double speedOfSound_m_s   { 0.0 };   ///< Speed of sound [m/s].
    double airDensity_kg_m3   { 0.0 };   ///< Air density [kg/m³].
    double ambientPressure_Pa { 0.0 };   ///< Static pressure [Pa].
    double ambientTemperature_K{ 0.0 };  ///< Static temperature [K].
    ///@}

    /// @name Aerodynamics
    ///@{
    double aero_bodyForce_N_X { 0.0 };   ///< Aero force, body x [N].
    double aero_bodyForce_N_Y { 0.0 };   ///< Aero force, body y [N].
    double aero_bodyForce_N_Z { 0.0 };   ///< Aero force, body z [N].
    double aero_bodyMoment_Nm_L{ 0.0 };  ///< Roll  moment [N·m].
    double aero_bodyMoment_Nm_M{ 0.0 };  ///< Pitch moment [N·m].
    double aero_bodyMoment_Nm_N{ 0.0 };  ///< Yaw   moment [N·m].
    ///@}

    /// @name Air data
    ///@{
    double mach               { 0.0 };   ///< Mach number [-].
    double dynamicPressure_Pa { 0.0 };   ///< ½ρV² [Pa].
    double trueAirspeed_m_s   { 0.0 };   ///< Atmosphere-relative speed [m/s].
    ///@}
};

// ─────────────────────────────────────────────────────────────────────────────
// CSV traits — compile-time column descriptor (31 columns)
// ─────────────────────────────────────────────────────────────────────────────

struct Snapshot2CsvTraits
{
    static constexpr std::array kColumnNames = {
        std::string_view{ "time" },
        std::string_view{ "gePosition_m_X" },
        std::string_view{ "gePosition_m_Y" },
        std::string_view{ "gePosition_m_Z" },
        std::string_view{ "feVelocity_m_s_X" },
        std::string_view{ "feVelocity_m_s_Y" },
        std::string_view{ "feVelocity_m_s_Z" },
        std::string_view{ "altitudeMsl_m" },
        std::string_view{ "longitude_rad" },
        std::string_view{ "latitude_rad" },
        std::string_view{ "localGravity_m_s2" },
        std::string_view{ "eulerAngle_rad_Yaw" },
        std::string_view{ "eulerAngle_rad_Pitch" },
        std::string_view{ "eulerAngle_rad_Roll" },
        std::string_view{ "bodyAngularRateWrtEi_rad_s_Roll" },
        std::string_view{ "bodyAngularRateWrtEi_rad_s_Pitch" },
        std::string_view{ "bodyAngularRateWrtEi_rad_s_Yaw" },
        std::string_view{ "altitudeRateWrtMsl_m_s" },
        std::string_view{ "speedOfSound_m_s" },
        std::string_view{ "airDensity_kg_m3" },
        std::string_view{ "ambientPressure_Pa" },
        std::string_view{ "ambientTemperature_K" },
        std::string_view{ "aero_bodyForce_N_X" },
        std::string_view{ "aero_bodyForce_N_Y" },
        std::string_view{ "aero_bodyForce_N_Z" },
        std::string_view{ "aero_bodyMoment_Nm_L" },
        std::string_view{ "aero_bodyMoment_Nm_M" },
        std::string_view{ "aero_bodyMoment_Nm_N" },
        std::string_view{ "mach" },
        std::string_view{ "dynamicPressure_Pa" },
        std::string_view{ "trueAirspeed_m_s" },
    };

    static constexpr std::size_t kColumnCount = kColumnNames.size();
};

// ─────────────────────────────────────────────────────────────────────────────
// CSV I/O
// ─────────────────────────────────────────────────────────────────────────────

inline void Snapshot2_WriteCsvHeader(std::ostream& os)
{
    constexpr auto& cols = Snapshot2CsvTraits::kColumnNames;
    for (std::size_t i = 0; i < cols.size(); ++i) {
        if (i > 0) os << ',';
        os << cols[i];
    }
    os << '\n';
}

inline void Snapshot2_WriteCsvRow(std::ostream& os, const Snapshot2& s)
{
    static_assert(Snapshot2CsvTraits::kColumnCount == 31,
        "Snapshot2_WriteCsvRow: column count mismatch.");

    os << std::scientific
       << std::setprecision(std::numeric_limits<double>::max_digits10);

    const auto sep = [&os](double v) { os << ',' << v; };

    os << s.time;
    sep(s.gePosition_m.x());      sep(s.gePosition_m.y());      sep(s.gePosition_m.z());
    sep(s.feVelocity_m_s.x());    sep(s.feVelocity_m_s.y());    sep(s.feVelocity_m_s.z());
    sep(s.altitudeMsl_m);
    sep(s.longitude_rad);
    sep(s.latitude_rad);
    sep(s.localGravity_m_s2);
    sep(s.eulerAngle_rad_Yaw);    sep(s.eulerAngle_rad_Pitch);  sep(s.eulerAngle_rad_Roll);
    sep(s.bodyAngularRateWrtEi_rad_s_Roll);
    sep(s.bodyAngularRateWrtEi_rad_s_Pitch);
    sep(s.bodyAngularRateWrtEi_rad_s_Yaw);
    sep(s.altitudeRateWrtMsl_m_s);
    sep(s.speedOfSound_m_s);      sep(s.airDensity_kg_m3);
    sep(s.ambientPressure_Pa);    sep(s.ambientTemperature_K);
    sep(s.aero_bodyForce_N_X);    sep(s.aero_bodyForce_N_Y);    sep(s.aero_bodyForce_N_Z);
    sep(s.aero_bodyMoment_Nm_L);  sep(s.aero_bodyMoment_Nm_M);  sep(s.aero_bodyMoment_Nm_N);
    sep(s.mach);
    sep(s.dynamicPressure_Pa);
    sep(s.trueAirspeed_m_s);
    os << '\n';
}

} // namespace Aetherion::Simulation

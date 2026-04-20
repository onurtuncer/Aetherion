// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>   // for Eigen::Quaterniond

//TODO [Onur] - separate .cpp and .h to prevent header leaking Eigen and iomanip to all Snapshot1 users.
//  The .cpp can include this header and then include Eigen and iomanip itself.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <limits>
#include <ostream>
#include <array>
#include <string_view>

namespace Aetherion::Simulation {

/// @brief Full observable state of the rigid body at a single simulation time instant.
///
/// Field names and units match exactly the columns of the NASA TM-2015-218675
/// Atmos_01 check-case CSV, making it straightforward to compare simulation output
/// against the reference data. Populated by MakeSnapshot1 and serialised to CSV
/// by Snapshot1_WriteCsvRow().
struct Snapshot1
{
    /// @name Time
    ///@{
    double time{ 0.0 };                      ///< Simulation time [s].
    ///@}

    /// @name Kinematics
    ///@{
    Eigen::Vector3d gePosition_m{};          ///< ECI (geocentric) position [m].
    Eigen::Vector3d feVelocity_m_s{};        ///< Earth-relative (NED) velocity [m/s].
    Eigen::Vector3d v_eci{};                 ///< ECI inertial velocity [m/s] (used internally).

    double altitudeMsl_m{ 0.0 };             ///< Geometric altitude above mean sea level [m].
    double longitude_rad{ 0.0 };             ///< Geodetic longitude [rad].
    double latitude_rad{ 0.0 };              ///< Geodetic latitude [rad].

    double localGravity_m_s2{ 0.0 };         ///< Gravitational acceleration magnitude at current position [m/s²].
    ///@}

    /// @name Attitude
    ///@{
    double eulerAngle_rad_Yaw{ 0.0 };        ///< ZYX Euler yaw angle [rad].
    double eulerAngle_rad_Pitch{ 0.0 };      ///< ZYX Euler pitch angle [rad].
    double eulerAngle_rad_Roll{ 0.0 };       ///< ZYX Euler roll angle [rad].

    Eigen::Quaterniond q_body_to_eci{};      ///< Unit quaternion representing the body → ECI rotation.
    ///@}

    /// @name Angular rates
    ///@{
    double bodyAngularRateWrtEi_rad_s_Roll{ 0.0 };   ///< Body roll rate with respect to ECI frame [rad/s].
    double bodyAngularRateWrtEi_rad_s_Pitch{ 0.0 };  ///< Body pitch rate with respect to ECI frame [rad/s].
    double bodyAngularRateWrtEi_rad_s_Yaw{ 0.0 };    ///< Body yaw rate with respect to ECI frame [rad/s].

    double altitudeRateWrtMsl_m_s{ 0.0 };   ///< Altitude rate dh/dt [m/s] (equals −v_NED_down).
    ///@}

    /// @name Atmosphere (US 1976 Standard)
    ///@{
    double speedOfSound_m_s{ 0.0 };          ///< Speed of sound [m/s].
    double airDensity_kg_m3{ 0.0 };          ///< Air density [kg/m³].
    double ambientPressure_Pa{ 0.0 };        ///< Static pressure [Pa].
    double ambientTemperature_K{ 0.0 };      ///< Static temperature [K].
    ///@}

    /// @name Aerodynamics (zero for the dragless-sphere case)
    ///@{
    double aero_bodyForce_N_X{ 0.0 };        ///< Aerodynamic force in body x-axis [N].
    double aero_bodyForce_N_Y{ 0.0 };        ///< Aerodynamic force in body y-axis [N].
    double aero_bodyForce_N_Z{ 0.0 };        ///< Aerodynamic force in body z-axis [N].
    double aero_bodyMoment_Nm_L{ 0.0 };      ///< Aerodynamic roll moment [N·m].
    double aero_bodyMoment_Nm_M{ 0.0 };      ///< Aerodynamic pitch moment [N·m].
    double aero_bodyMoment_Nm_N{ 0.0 };      ///< Aerodynamic yaw moment [N·m].
    ///@}

    /// @name Air data
    ///@{
    double mach{ 0.0 };                      ///< Mach number [-].
    double dynamicPressure_Pa{ 0.0 };        ///< Dynamic pressure: ½ρV² [Pa].
    double trueAirspeed_m_s{ 0.0 };          ///< True airspeed [m/s].
    ///@}
};

/// @brief Compile-time descriptor mapping Snapshot1 fields to their CSV column names.
///
/// The column order defined here drives both the header writer (Snapshot1_WriteCsvHeader)
/// and the row writer (Snapshot1_WriteCsvRow), guaranteeing that the two can never diverge.
struct Snapshot1CsvTraits
{
    /// @brief Ordered list of CSV column names; each entry corresponds to one scalar output field of Snapshot1.
    static constexpr std::array kColumnNames = {
        // time
        std::string_view{ "time" },

        // kinematics — gePosition_m (Eigen::Vector3d)
        std::string_view{ "gePosition_m_X" },
        std::string_view{ "gePosition_m_Y" },
        std::string_view{ "gePosition_m_Z" },

        // kinematics — feVelocity_m_s (Eigen::Vector3d)
        std::string_view{ "feVelocity_m_s_X" },
        std::string_view{ "feVelocity_m_s_Y" },
        std::string_view{ "feVelocity_m_s_Z" },

        // kinematics — v_eci (Eigen::Vector3d, internal)
        std::string_view{ "v_eci_X" },
        std::string_view{ "v_eci_Y" },
        std::string_view{ "v_eci_Z" },

        // kinematics — scalars
        std::string_view{ "altitudeMsl_m" },
        std::string_view{ "longitude_rad" },
        std::string_view{ "latitude_rad" },
        std::string_view{ "localGravity_m_s2" },

        // attitude — Euler
        std::string_view{ "eulerAngle_rad_Yaw" },
        std::string_view{ "eulerAngle_rad_Pitch" },
        std::string_view{ "eulerAngle_rad_Roll" },

        // attitude — quaternion (Eigen::Quaterniond, W-first)
        std::string_view{ "q_body_to_eci_W" },
        std::string_view{ "q_body_to_eci_X" },
        std::string_view{ "q_body_to_eci_Y" },
        std::string_view{ "q_body_to_eci_Z" },

        // body rates
        std::string_view{ "bodyAngularRateWrtEi_rad_s_Roll" },
        std::string_view{ "bodyAngularRateWrtEi_rad_s_Pitch" },
        std::string_view{ "bodyAngularRateWrtEi_rad_s_Yaw" },
        std::string_view{ "altitudeRateWrtMsl_m_s" },

        // atmosphere
        std::string_view{ "speedOfSound_m_s" },
        std::string_view{ "airDensity_kg_m3" },
        std::string_view{ "ambientPressure_Pa" },
        std::string_view{ "ambientTemperature_K" },

        // aerodynamics
        std::string_view{ "aero_bodyForce_N_X" },
        std::string_view{ "aero_bodyForce_N_Y" },
        std::string_view{ "aero_bodyForce_N_Z" },
        std::string_view{ "aero_bodyMoment_Nm_L" },
        std::string_view{ "aero_bodyMoment_Nm_M" },
        std::string_view{ "aero_bodyMoment_Nm_N" },

        // air-data
        std::string_view{ "mach" },
        std::string_view{ "dynamicPressure_Pa" },
        std::string_view{ "trueAirspeed_m_s" },
    };

    /// @brief Total number of CSV columns; usable in static_asserts and array-sizing downstream.
    static constexpr std::size_t kColumnCount = kColumnNames.size();
};

/// @brief Writes the CSV column-header row to @p os, driven by Snapshot1CsvTraits::kColumnNames.
///
/// The column order is guaranteed to match Snapshot1_WriteCsvRow() because both use the
/// same static descriptor.
/// @param os Output stream to write to (e.g. an open std::ofstream).
inline void Snapshot1_WriteCsvHeader(std::ostream& os)
{
    constexpr auto& cols = Snapshot1CsvTraits::kColumnNames;
    for (std::size_t i = 0; i < cols.size(); ++i)
    {
        if (i > 0) os << ',';
        os << cols[i];
    }
    os << '\n';
}

/// @brief Writes one CSV data row from a Snapshot1 to @p os.
///
/// Uses `std::numeric_limits<double>::max_digits10` (17 significant digits) so that
/// every double value round-trips through text without precision loss.
/// @param os Output stream to write to.
/// @param s  Snapshot1 whose fields will be written in the column order defined by Snapshot1CsvTraits.
inline void Snapshot1_WriteCsvRow(std::ostream& os, const Snapshot1& s)
{
    static_assert(Snapshot1CsvTraits::kColumnCount == 38,
        "Snapshot1_WriteCsvRow: column count mismatch.");

    // std::numeric_limits<double>::max_digits10 == 17 guarantees that any
    // double written with this precision reads back as the identical bit
    // pattern.  The previous value of 15 was insufficient: large position
    // values such as 6387280.999951441 were being rounded to 6387281.0
    // (a loss of ~0.05 m) when parsed by downstream tools.
    os << std::scientific << std::setprecision(std::numeric_limits<double>::max_digits10);

    // Explicit double — avoids MSVC rejecting Eigen expression templates via auto lambda
    const auto sep = [&os](double v) { os << ',' << v; };

    // --- time ---------------------------------------------------------------
    os << s.time;

    // --- kinematics — gePosition_m ------------------------------------------
    sep(s.gePosition_m.x());
    sep(s.gePosition_m.y());
    sep(s.gePosition_m.z());

    // --- kinematics — feVelocity_m_s ----------------------------------------
    sep(s.feVelocity_m_s.x());
    sep(s.feVelocity_m_s.y());
    sep(s.feVelocity_m_s.z());

    // --- kinematics — v_eci -------------------------------------------------
    sep(s.v_eci.x());
    sep(s.v_eci.y());
    sep(s.v_eci.z());

    // --- kinematics — scalars -----------------------------------------------
    sep(s.altitudeMsl_m);
    sep(s.longitude_rad);
    sep(s.latitude_rad);
    sep(s.localGravity_m_s2);

    // --- attitude — Euler ---------------------------------------------------
    sep(s.eulerAngle_rad_Yaw);
    sep(s.eulerAngle_rad_Pitch);
    sep(s.eulerAngle_rad_Roll);

    // --- attitude — quaternion (W-first, matches header) --------------------
    sep(s.q_body_to_eci.w());
    sep(s.q_body_to_eci.x());
    sep(s.q_body_to_eci.y());
    sep(s.q_body_to_eci.z());

    // --- body rates ---------------------------------------------------------
    sep(s.bodyAngularRateWrtEi_rad_s_Roll);
    sep(s.bodyAngularRateWrtEi_rad_s_Pitch);
    sep(s.bodyAngularRateWrtEi_rad_s_Yaw);
    sep(s.altitudeRateWrtMsl_m_s);

    // --- atmosphere ---------------------------------------------------------
    sep(s.speedOfSound_m_s);
    sep(s.airDensity_kg_m3);
    sep(s.ambientPressure_Pa);
    sep(s.ambientTemperature_K);

    // --- aerodynamics -------------------------------------------------------
    sep(s.aero_bodyForce_N_X);
    sep(s.aero_bodyForce_N_Y);
    sep(s.aero_bodyForce_N_Z);
    sep(s.aero_bodyMoment_Nm_L);
    sep(s.aero_bodyMoment_Nm_M);
    sep(s.aero_bodyMoment_Nm_N);

    // --- air-data -----------------------------------------------------------
    sep(s.mach);
    sep(s.dynamicPressure_Pa);
    sep(s.trueAirspeed_m_s);

    os << '\n';
}

} // Aetherion::Simulation

// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

 // -----------------------------------------------------------------------
 // Snapshot — full observable state at a single time.
 //
 // Field names and units match exactly the columns of the NASA
 // TM-2015-218675 Atmos_01 check-case CSV:
 //
 //   time
 //   gePosition_m_X/Y/Z          — ECI (geocentric) position          [m]
 //   feVelocity_m_s_X/Y/Z        — Earth-relative (NED) velocity      [m/s]
 //   altitudeMsl_m               — geometric altitude above MSL        [m]
 //   longitude_rad               — geodetic longitude                  [rad]
 //   latitude_rad                — geodetic latitude                   [rad]
 //   localGravity_m_s2           — local gravitational acceleration    [m/s²]
 //   eulerAngle_rad_Yaw          — yaw     ZYX Euler               [rad]
 //   eulerAngle_rad_Pitch        — pitch   ZYX Euler               [rad]
 //   eulerAngle_rad_Roll         — roll    ZYX Euler               [rad]
 //   bodyAngularRateWrtEi_rad_s_Roll/Pitch/Yaw — ω_B wrt ECI         [rad/s]
 //   altitudeRateWrtMsl_m_s      — dh/dt (down-component of v_ned)    [m/s]
 //   speedOfSound_m_s            — US1976 speed of sound              [m/s]
 //   airDensity_kg_m3            — US1976 air density                  [kg/m³]
 //   ambientPressure_Pa          — US1976 static pressure              [Pa]
 //   ambientTemperature_K        — US1976 temperature                  [K]
 //   aero_bodyForce_N_X/Y/Z      — aerodynamic force in body frame     [N]
 //   aero_bodyMoment_Nm_L/M/N    — aerodynamic moment in body frame    [N·m]
 //   mach                        — Mach number                         [-]
 //   dynamicPressure_Pa          —                                     [Pa]
 //   trueAirspeed_m_s            — true airspeed                       [m/s]
 //
 // -----------------------------------------------------------------------

//TODO [Onur] - separate .cpp and .h to prevent header leaking Eigen and iomanip to all Snapshot1 users. 
//  The .cpp can include this header and then include Eigen and iomanip itself.

#include <Eigen/Core>
#include <Eigen/Geometry>   // provides Eigen::Quaterniond
#include <iomanip>
#include <ostream>
#include <array>
#include <string_view>

namespace Aetherion::Simulation {

    struct Snapshot1
    {
        // --- time --------------------------------------------------------------
        double time{ 0.0 };                     // [s]  (alias: t for compat.)

        // --- kinematics --------------------------------------------------------
        Eigen::Vector3d gePosition_m{};          // ECI position [m]
        Eigen::Vector3d feVelocity_m_s{};        // Earth-frame (NED) velocity [m/s]
        Eigen::Vector3d v_eci{};                 // ECI velocity (internal use) [m/s]

        double altitudeMsl_m{ 0.0 };             // geometric altitude above MSL [m]
        double longitude_rad{ 0.0 };             // geodetic longitude [rad]
        double latitude_rad{ 0.0 };              // geodetic latitude  [rad]

        double localGravity_m_s2{ 0.0 };         // |g| at current position [m/s²]

        // -- attitude ----------------------------------------------------------
        double eulerAngle_rad_Yaw{ 0.0 };        //  ZYX Euler [rad]
        double eulerAngle_rad_Pitch{ 0.0 };      //  ZYX Euler [rad]
        double eulerAngle_rad_Roll{ 0.0 };       //  ZYX Euler [rad]

        Eigen::Quaterniond q_body_to_eci{};      // body→ECI quaternion

        // -- body rates wrt ECI ------------------------------------------------
        double bodyAngularRateWrtEi_rad_s_Roll{ 0.0 };   // wx [rad/s]
        double bodyAngularRateWrtEi_rad_s_Pitch{ 0.0 };  // wy [rad/s]
        double bodyAngularRateWrtEi_rad_s_Yaw{ 0.0 };    // wz [rad/s]

        double altitudeRateWrtMsl_m_s{ 0.0 };   // dh/dt [m/s]  (−v_ned_down)

        // -- atmosphere (US1976) -----------------------------------------------
        double speedOfSound_m_s{ 0.0 };          // [m/s]
        double airDensity_kg_m3{ 0.0 };          // [kg/m^3]
        double ambientPressure_Pa{ 0.0 };        // [Pa]
        double ambientTemperature_K{ 0.0 };      // [K]

        // --- aerodynamics (zero for dragless sphere) ---------------------------
        double aero_bodyForce_N_X{ 0.0 };        // [N]
        double aero_bodyForce_N_Y{ 0.0 };        // [N]
        double aero_bodyForce_N_Z{ 0.0 };        // [N]
        double aero_bodyMoment_Nm_L{ 0.0 };      // [N·m]
        double aero_bodyMoment_Nm_M{ 0.0 };      // [N·m]
        double aero_bodyMoment_Nm_N{ 0.0 };      // [N·m]

        // --- air-data -----------------------------------------------------------
        double mach{ 0.0 };                      // Mach number [-]
        double dynamicPressure_Pa{ 0.0 };        //  1/2 * rho * v^2  [Pa]
        double trueAirspeed_m_s{ 0.0 };          // TAS [m/s]
    };


    // ---------------------------------------------------------------------------
    // Static field descriptor for Snapshot1
    // Each entry is the CSV column name exactly as it will appear in the header.
    // Expanding Eigen vector/quaternion members into their scalar components.
    // ---------------------------------------------------------------------------
    struct Snapshot1CsvTraits
    {
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

        // Total column count — usable in static_asserts and array sizing downstream
        static constexpr std::size_t kColumnCount = kColumnNames.size();
    };

    // ---------------------------------------------------------------------------
    // Write the header row — driven entirely by the static descriptor above.
    // The same loop will drive WriteCsvRow() so column order can never diverge.
    // ---------------------------------------------------------------------------
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

    inline void Snapshot1_WriteCsvRow(std::ostream& os, const Snapshot1& s)
    {
        static_assert(Snapshot1CsvTraits::kColumnCount == 38,
            "Snapshot1_WriteCsvRow: column count mismatch.");

        os << std::scientific << std::setprecision(15);

        // Explicit double — avoids MSVC rejecting Eigen expression templates via auto lambda
        const auto sep = [&os](double v) { os << ',' << v; };

        os << s.time;

        sep(s.gePosition_m.x());
      
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
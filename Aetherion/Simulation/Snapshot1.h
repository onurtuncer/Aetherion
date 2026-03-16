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
 //   dynamicPressure_Pa          —                               [Pa]
 //   trueAirspeed_m_s            — true airspeed                       [m/s]
 //
 // -----------------------------------------------------------------------
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
    double bodyAngularRateWrtEi_rad_s_Roll{ 0.0 };   // ωx [rad/s]
    double bodyAngularRateWrtEi_rad_s_Pitch{ 0.0 };  // ωy [rad/s]
    double bodyAngularRateWrtEi_rad_s_Yaw{ 0.0 };    // ωz [rad/s]

    double altitudeRateWrtMsl_m_s{ 0.0 };   // dh/dt [m/s]  (−v_ned_down)

    // -- atmosphere (US1976) -----------------------------------------------
    double speedOfSound_m_s{ 0.0 };          // [m/s]
    double airDensity_kg_m3{ 0.0 };          // [kg/m³]
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
    double dynamicPressure_Pa{ 0.0 };        // ½ρv²  [Pa]
    double trueAirspeed_m_s{ 0.0 };          // TAS [m/s]
};

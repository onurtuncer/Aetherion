"""
convert_atmos02_to_si.py
Converts Atmos_02_sim_01.csv (US customary) to Atmos_02_sim_01_si_units.csv (SI).
"""
import pandas as pd
import numpy as np
from pathlib import Path

data_dir = Path(__file__).parent.parent / "data" / "Atmos_02_TumblingBrickNoDamping"
src = data_dir / "Atmos_02_sim_01.csv"
dst = data_dir / "Atmos_02_sim_01_si_units.csv"

df = pd.read_csv(src)

ft2m            = 0.3048
fts2ms          = 0.3048
deg2rad         = np.pi / 180.0
ftmin2ms        = 0.3048 / 60.0
slug_ft3_kg_m3  = 515.378818
lbf_ft2_Pa      = 47.8803
lbf_N           = 4.44822
ftlbf_Nm        = 1.35582
nmi_h_ms        = 1852.0 / 3600.0
rankine_kelvin  = 5.0 / 9.0

out = pd.DataFrame()
out["time"]                              = df["time"]
out["gePosition_m_X"]                    = df["gePosition_ft_X"]             * ft2m
out["gePosition_m_Y"]                    = df["gePosition_ft_Y"]             * ft2m
out["gePosition_m_Z"]                    = df["gePosition_ft_Z"]             * ft2m
out["feVelocity_m_s_X"]                  = df["feVelocity_ft_s_X"]           * fts2ms
out["feVelocity_m_s_Y"]                  = df["feVelocity_ft_s_Y"]           * fts2ms
out["feVelocity_m_s_Z"]                  = df["feVelocity_ft_s_Z"]           * fts2ms
out["altitudeMsl_m"]                     = df["altitudeMsl_ft"]              * ft2m
out["longitude_rad"]                     = df["longitude_deg"]               * deg2rad
out["latitude_rad"]                      = df["latitude_deg"]                * deg2rad
out["localGravity_m_s2"]                 = df["localGravity_ft_s2"]          * fts2ms
out["eulerAngle_rad_Yaw"]                = df["eulerAngle_deg_Yaw"]          * deg2rad
out["eulerAngle_rad_Pitch"]              = df["eulerAngle_deg_Pitch"]        * deg2rad
out["eulerAngle_rad_Roll"]               = df["eulerAngle_deg_Roll"]         * deg2rad
out["bodyAngularRateWrtEi_rad_s_Roll"]   = df["bodyAngularRateWrtEi_deg_s_Roll"]  * deg2rad
out["bodyAngularRateWrtEi_rad_s_Pitch"]  = df["bodyAngularRateWrtEi_deg_s_Pitch"] * deg2rad
out["bodyAngularRateWrtEi_rad_s_Yaw"]    = df["bodyAngularRateWrtEi_deg_s_Yaw"]   * deg2rad
out["altitudeRateWrtMsl_m_s"]            = df["altitudeRateWrtMsl_ft_min"]   * ftmin2ms
out["speedOfSound_m_s"]                  = df["speedOfSound_ft_s"]           * fts2ms
out["airDensity_kg_m3"]                  = df["airDensity_slug_ft3"]         * slug_ft3_kg_m3
out["ambientPressure_Pa"]                = df["ambientPressure_lbf_ft2"]     * lbf_ft2_Pa
out["ambientTemperature_K"]              = df["ambientTemperature_dgR"]      * rankine_kelvin
out["aero_bodyForce_N_X"]                = df["aero_bodyForce_lbf_X"]        * lbf_N
out["aero_bodyForce_N_Y"]                = df["aero_bodyForce_lbf_Y"]        * lbf_N
out["aero_bodyForce_N_Z"]                = df["aero_bodyForce_lbf_Z"]        * lbf_N
out["aero_bodyMoment_Nm_L"]              = df["aero_bodyMoment_ftlbf_L"]     * ftlbf_Nm
out["aero_bodyMoment_Nm_M"]              = df["aero_bodyMoment_ftlbf_M"]     * ftlbf_Nm
out["aero_bodyMoment_Nm_N"]              = df["aero_bodyMoment_ftlbf_N"]     * ftlbf_Nm
out["mach"]                              = df["mach"]
out["dynamicPressure_Pa"]                = df["dynamicPressure_lbf_ft2"]     * lbf_ft2_Pa
out["trueAirspeed_m_s"]                  = df["trueAirspeed_nmi_h"]          * nmi_h_ms

out.to_csv(dst, index=False)
print(f"Written {len(out)} rows x {len(out.columns)} columns -> {dst}")

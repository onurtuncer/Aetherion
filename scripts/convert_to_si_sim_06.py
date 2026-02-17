"""
convert_to_si_sim_06.py

Converts Atmos_01_sim_06.csv from Imperial / mixed aerospace units
to fully consistent SI units.

Default input  : ../data/Atmos_01_sim_06.csv
Default output : ../data/Atmos_01_sim_06_si_units.csv

Tailored specifically to the column structure of Atmos_01_sim_06.csv.

Author: Prof. Dr. Onur Tuncer
Affiliation: Istanbul Technical University
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

# Resolve paths relative to this script (robust vs working directory)
SCRIPT_DIR = Path(__file__).resolve().parent
DATA_DIR = SCRIPT_DIR.parent / "data/Atmos_01_DroppedSphere"
DEFAULT_INPUT = DATA_DIR / "Atmos_01_sim_06.csv"

# === Conversion constants ===
FT_TO_M = 0.3048
FT_S_TO_M_S = 0.3048
FT_S2_TO_M_S2 = 0.3048
FT_MIN_TO_M_S = 0.3048 / 60.0
SLUGFT3_TO_KGM3 = 515.378818
LBF_FT2_TO_PA = 47.88025898
LBF_TO_N = 4.4482216152605
FTLBF_TO_NM = 1.3558179483314
RANKINE_TO_K = 5.0 / 9.0
DEG_TO_RAD = np.pi / 180.0
KNOT_TO_MPS = 0.514444  # nmi/h (knots)


def default_output_path(input_path: Path) -> Path:
    return input_path.with_name(input_path.stem + "_si_units" + input_path.suffix)


def convert(df: pd.DataFrame) -> pd.DataFrame:
    """
    Columns in Atmos_01_sim_06.csv:
      time
      eiPosition_ft_{X,Y,Z}
      gePosition_ft_{X,Y,Z}
      eiVelocity_ft_s_{X,Y,Z}
      feVelocity_ft_s_{X,Y,Z}
      altitudeMsl_ft
      longitude_deg, latitude_deg
      localGravity_ft_s2
      eulerAngle_deg_{Yaw,Pitch,Roll}
      bodyAngularRateWrtEi_deg_s_{Roll,Pitch,Yaw}
      altitudeRateWrtMsl_ft_min
      speedOfSound_ft_s
      airDensity_slug_ft3
      ambientPressure_lbf_ft2
      dynamicPressure_lbf_ft2
      ambientTemperature_dgR
      aero_bodyForce_lbf_{X,Y,Z}
      aero_bodyMoment_ftlbf_{L,M,N}
      mach (dimensionless)
      trueAirspeed_nmi_h
    """
    df_si = df.copy()

    # Positions (ft → m)
    df_si[["eiPosition_ft_X", "eiPosition_ft_Y", "eiPosition_ft_Z"]] *= FT_TO_M
    df_si[["gePosition_ft_X", "gePosition_ft_Y", "gePosition_ft_Z"]] *= FT_TO_M

    # Velocities (ft/s → m/s)
    df_si[["eiVelocity_ft_s_X", "eiVelocity_ft_s_Y", "eiVelocity_ft_s_Z"]] *= FT_S_TO_M_S
    df_si[["feVelocity_ft_s_X", "feVelocity_ft_s_Y", "feVelocity_ft_s_Z"]] *= FT_S_TO_M_S

    # Altitude (ft → m)
    df_si["altitudeMsl_ft"] *= FT_TO_M

    # Gravity (ft/s² → m/s²)
    df_si["localGravity_ft_s2"] *= FT_S2_TO_M_S2

    # Altitude rate (ft/min → m/s)
    df_si["altitudeRateWrtMsl_ft_min"] *= FT_MIN_TO_M_S

    # Speed of sound (ft/s → m/s)
    df_si["speedOfSound_ft_s"] *= FT_S_TO_M_S

    # Density (slug/ft³ → kg/m³)
    df_si["airDensity_slug_ft3"] *= SLUGFT3_TO_KGM3

    # Pressure (lbf/ft² → Pa)
    df_si[["ambientPressure_lbf_ft2", "dynamicPressure_lbf_ft2"]] *= LBF_FT2_TO_PA

    # Temperature (°R → K)
    df_si["ambientTemperature_dgR"] *= RANKINE_TO_K

    # Forces (lbf → N)
    df_si[["aero_bodyForce_lbf_X", "aero_bodyForce_lbf_Y", "aero_bodyForce_lbf_Z"]] *= LBF_TO_N

    # Moments (ft·lbf → N·m)
    df_si[["aero_bodyMoment_ftlbf_L", "aero_bodyMoment_ftlbf_M", "aero_bodyMoment_ftlbf_N"]] *= FTLBF_TO_NM

    # Angles (deg → rad)
    df_si[
        ["longitude_deg", "latitude_deg", "eulerAngle_deg_Yaw", "eulerAngle_deg_Pitch", "eulerAngle_deg_Roll"]
    ] *= DEG_TO_RAD

    # Angular rates (deg/s → rad/s)
    df_si[
        ["bodyAngularRateWrtEi_deg_s_Roll", "bodyAngularRateWrtEi_deg_s_Pitch", "bodyAngularRateWrtEi_deg_s_Yaw"]
    ] *= DEG_TO_RAD

    # True airspeed (nmi/h → m/s)
    df_si["trueAirspeed_nmi_h"] *= KNOT_TO_MPS

    # mach is dimensionless -> unchanged
    return df_si


def rename_columns(df: pd.DataFrame) -> pd.DataFrame:
    rename_dict = {
        "eiPosition_ft_X": "eiPosition_m_X",
        "eiPosition_ft_Y": "eiPosition_m_Y",
        "eiPosition_ft_Z": "eiPosition_m_Z",
        "gePosition_ft_X": "gePosition_m_X",
        "gePosition_ft_Y": "gePosition_m_Y",
        "gePosition_ft_Z": "gePosition_m_Z",
        "eiVelocity_ft_s_X": "eiVelocity_m_s_X",
        "eiVelocity_ft_s_Y": "eiVelocity_m_s_Y",
        "eiVelocity_ft_s_Z": "eiVelocity_m_s_Z",
        "feVelocity_ft_s_X": "feVelocity_m_s_X",
        "feVelocity_ft_s_Y": "feVelocity_m_s_Y",
        "feVelocity_ft_s_Z": "feVelocity_m_s_Z",
        "altitudeMsl_ft": "altitudeMsl_m",
        "longitude_deg": "longitude_rad",
        "latitude_deg": "latitude_rad",
        "localGravity_ft_s2": "localGravity_m_s2",
        "altitudeRateWrtMsl_ft_min": "altitudeRateWrtMsl_m_s",
        "speedOfSound_ft_s": "speedOfSound_m_s",
        "airDensity_slug_ft3": "airDensity_kg_m3",
        "ambientPressure_lbf_ft2": "ambientPressure_Pa",
        "dynamicPressure_lbf_ft2": "dynamicPressure_Pa",
        "ambientTemperature_dgR": "ambientTemperature_K",
        "aero_bodyForce_lbf_X": "aero_bodyForce_N_X",
        "aero_bodyForce_lbf_Y": "aero_bodyForce_N_Y",
        "aero_bodyForce_lbf_Z": "aero_bodyForce_N_Z",
        "aero_bodyMoment_ftlbf_L": "aero_bodyMoment_Nm_L",
        "aero_bodyMoment_ftlbf_M": "aero_bodyMoment_Nm_M",
        "aero_bodyMoment_ftlbf_N": "aero_bodyMoment_Nm_N",
        "eulerAngle_deg_Yaw": "eulerAngle_rad_Yaw",
        "eulerAngle_deg_Pitch": "eulerAngle_rad_Pitch",
        "eulerAngle_deg_Roll": "eulerAngle_rad_Roll",
        "bodyAngularRateWrtEi_deg_s_Roll": "bodyAngularRateWrtEi_rad_s_Roll",
        "bodyAngularRateWrtEi_deg_s_Pitch": "bodyAngularRateWrtEi_rad_s_Pitch",
        "bodyAngularRateWrtEi_deg_s_Yaw": "bodyAngularRateWrtEi_rad_s_Yaw",
        "trueAirspeed_nmi_h": "trueAirspeed_m_s",
    }
    return df.rename(columns=rename_dict)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Convert Atmos_01_sim_06.csv from Imperial/mixed units to SI units (tailored columns)."
    )
    p.add_argument(
        "-i",
        "--input",
        type=Path,
        default=DEFAULT_INPUT,
        help="Input CSV path. Default: ../data/Atmos_01_sim_06.csv (relative to this script).",
    )
    p.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output CSV path. Default: <input_stem>_si_units.csv in the same directory as input.",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    input_path: Path = args.input
    output_path: Path = args.output if args.output is not None else default_output_path(input_path)

    if not input_path.exists():
        raise FileNotFoundError(f"Input CSV not found: {input_path}")

    df = pd.read_csv(input_path)

    df_si = convert(df)

    df_si = rename_columns(df_si)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    df_si.to_csv(output_path, index=False)

    print(f"Input : {input_path}")
    print(f"Output: {output_path}")
    print("Done.")


if __name__ == "__main__":
    main()

"""
convert_to_si.py

Converts Atmos_01_sim_01.csv from Imperial / mixed aerospace units
to fully consistent SI units.

Default input  : ../data/Atmos_01_sim_01.csv
Default output : ../data/Atmos_01_sim_01_si_units.csv

All conversions are explicitly defined and tailored to the known
column structure of the input file.

Author: Prof. Dr. Onur Tuncer
Affiliation: Istanbul Technical University
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

# Get path to this script
SCRIPT_DIR = Path(__file__).resolve().parent
DATA_DIR = SCRIPT_DIR.parent / "data/Atmos_01_DroppedSphere"

# Defaults
DEFAULT_INPUT = DATA_DIR / "Atmos_01_sim_01.csv"

# === Conversion constants ===
FT_TO_M = 0.3048
FT_S_TO_M_S = 0.3048
FT_S2_TO_M_S2 = 0.3048
FT_MIN_TO_M_S = 0.3048 / 60.0
SLUGFT3_TO_KGM3 = 515.378818
LBF_TO_N = 4.4482216152605
LBF_FT2_TO_PA = 47.88025898
FTLBF_TO_NM = 1.3558179483314
RANKINE_TO_K = 5.0 / 9.0
DEG_TO_RAD = np.pi / 180.0
KNOT_TO_MPS = 0.514444  # nmi/h


def default_output_path(input_path: Path) -> Path:
    """Return output path with '_si_units' suffix, same directory and extension."""
    return input_path.with_name(input_path.stem + "_si_units" + input_path.suffix)


def convert(df: pd.DataFrame) -> pd.DataFrame:
    df_si = df.copy()

    # Position (ft → m)
    df_si[["gePosition_ft_X", "gePosition_ft_Y", "gePosition_ft_Z"]] *= FT_TO_M

    # Velocity (ft/s → m/s)
    df_si[
        ["feVelocity_ft_s_X", "feVelocity_ft_s_Y", "feVelocity_ft_s_Z", "speedOfSound_ft_s"]
    ] *= FT_S_TO_M_S

    # Gravity (ft/s² → m/s²)
    df_si["localGravity_ft_s2"] *= FT_S2_TO_M_S2

    # Altitude (ft → m)
    df_si["altitudeMsl_ft"] *= FT_TO_M

    # Altitude rate (ft/min → m/s)
    df_si["altitudeRateWrtMsl_ft_min"] *= FT_MIN_TO_M_S

    # Air density (slug/ft³ → kg/m³)
    df_si["airDensity_slug_ft3"] *= SLUGFT3_TO_KGM3

    # Pressure (lbf/ft² → Pa)
    df_si[["ambientPressure_lbf_ft2", "dynamicPressure_lbf_ft2"]] *= LBF_FT2_TO_PA

    # Forces (lbf → N)
    df_si[["aero_bodyForce_lbf_X", "aero_bodyForce_lbf_Y", "aero_bodyForce_lbf_Z"]] *= LBF_TO_N

    # Moments (ft·lbf → N·m)
    df_si[["aero_bodyMoment_ftlbf_L", "aero_bodyMoment_ftlbf_M", "aero_bodyMoment_ftlbf_N"]] *= (
        FTLBF_TO_NM
    )

    # Temperature (°R → K)
    df_si["ambientTemperature_dgR"] *= RANKINE_TO_K

    # Angles (deg → rad)
    df_si[
        [
            "longitude_deg",
            "latitude_deg",
            "eulerAngle_deg_Yaw",
            "eulerAngle_deg_Pitch",
            "eulerAngle_deg_Roll",
        ]
    ] *= DEG_TO_RAD

    # Angular rates (deg/s → rad/s)
    df_si[
        [
            "bodyAngularRateWrtEi_deg_s_Roll",
            "bodyAngularRateWrtEi_deg_s_Pitch",
            "bodyAngularRateWrtEi_deg_s_Yaw",
        ]
    ] *= DEG_TO_RAD

    # True airspeed (nmi/h → m/s)
    df_si["trueAirspeed_nmi_h"] *= KNOT_TO_MPS

    return df_si


def rename_columns(df: pd.DataFrame) -> pd.DataFrame:
    rename_dict = {
        "gePosition_ft_X": "gePosition_m_X",
        "gePosition_ft_Y": "gePosition_m_Y",
        "gePosition_ft_Z": "gePosition_m_Z",
        "feVelocity_ft_s_X": "feVelocity_m_s_X",
        "feVelocity_ft_s_Y": "feVelocity_m_s_Y",
        "feVelocity_ft_s_Z": "feVelocity_m_s_Z",
        "altitudeMsl_ft": "altitudeMsl_m",
        "localGravity_ft_s2": "localGravity_m_s2",
        "altitudeRateWrtMsl_ft_min": "altitudeRateWrtMsl_m_s",
        "speedOfSound_ft_s": "speedOfSound_m_s",
        "airDensity_slug_ft3": "airDensity_kg_m3",
        "ambientPressure_lbf_ft2": "ambientPressure_Pa",
        "dynamicPressure_lbf_ft2": "dynamicPressure_Pa",
        "aero_bodyForce_lbf_X": "aero_bodyForce_N_X",
        "aero_bodyForce_lbf_Y": "aero_bodyForce_N_Y",
        "aero_bodyForce_lbf_Z": "aero_bodyForce_N_Z",
        "aero_bodyMoment_ftlbf_L": "aero_bodyMoment_Nm_L",
        "aero_bodyMoment_ftlbf_M": "aero_bodyMoment_Nm_M",
        "aero_bodyMoment_ftlbf_N": "aero_bodyMoment_Nm_N",
        "ambientTemperature_dgR": "ambientTemperature_K",
        "longitude_deg": "longitude_rad",
        "latitude_deg": "latitude_rad",
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
        description="Convert Atmos_01_sim_01.csv from Imperial/mixed units to SI units (tailored columns)."
    )
    p.add_argument(
        "-i",
        "--input",
        type=Path,
        default=DEFAULT_INPUT,
        help="Input CSV path. Default: ../data/Atmos_01_sim_01.csv (relative to this script).",
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

    # Ensure output directory exists (in case user provides a new path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    df_si.to_csv(output_path, index=False)

    print(f"Input : {input_path}")
    print(f"Output: {output_path}")
    print("Done.")


if __name__ == "__main__":
    main()

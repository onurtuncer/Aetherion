"""
Author: Prof. Dr. Onur Tuncer
Affiliation: Istanbul Technical University
"""

import argparse
import json
import math
from copy import deepcopy
from pathlib import Path

import yaml  # PyYAML

# --- Unit conversions ---
FT_TO_M = 0.3048
FT2_TO_M2 = FT_TO_M ** 2
LBM_TO_KG = 0.45359237
SLUG_TO_KG = 14.5939029372
DEG_TO_RAD = math.pi / 180.0  
KNOT_TO_MPS = 0.514444

def inertia_factor(inertia_units: str) -> float:
    """
    Convert inertia to kg*m^2.
    Supported:
      - "lbm_ft2": (lbm*ft^2) -> kg*m^2
      - "slug_ft2": (slug*ft^2) -> kg*m^2
    """
    if inertia_units == "lbm_ft2":
        return LBM_TO_KG * FT2_TO_M2
    if inertia_units == "slug_ft2":
        return SLUG_TO_KG * FT2_TO_M2
    raise ValueError(f"Unsupported inertia units: {inertia_units}")


def load_config(path: Path) -> dict:
    ext = path.suffix.lower()
    text = path.read_text(encoding="utf-8")
    if ext in [".yaml", ".yml"]:
        return yaml.safe_load(text)
    if ext == ".json":
        return json.loads(text)
    raise ValueError(f"Unsupported input format: {ext} (use .yaml/.yml or .json)")


def save_config(path: Path, data: dict) -> None:
    ext = path.suffix.lower()
    if ext == ".json":
        path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        return
    if ext in [".yaml", ".yml"]:
        path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
        return
    raise ValueError(f"Unsupported output format: {ext} (use .json or .yaml/.yml)")


def default_output_path(input_path: Path, output_format: str) -> Path:
    ext = ".json" if output_format == "json" else ".yaml"
    return input_path.with_name(input_path.stem + "_si" + ext)


def convert_imperial_to_si(cfg: dict, inertia_units: str) -> dict:
    """
    Convert an input config (imperial fields) to SI fields while preserving
    the output schema you showed.

    Imperial keys expected (neat + explicit):
      initialPose.alt_ft
      initialVelocityNED.vn_fps, ve_fps, vd_fps   (ft/s)
      initialRotationAboutBodyAxes.*_deg_s        (kept as deg/s)
      inertialParameters.mass_lbm
      inertialParameters.Ixx..Ixz                (lbm*ft^2 or slug*ft^2)
      inertialParameters.xbar_ft, ybar_ft, zbar_ft
      aerodynamicParameters.S_ft2

    Everything else that is already SI-named (alt_m, vn_mps, mass_kg, etc.)
    is left untouched.
    """
    out = deepcopy(cfg)

    # --- initialPose ---
    pose = out.get("initialPose", {})
    if "alt_ft" in pose and "alt_m" not in pose:
        pose["alt_m"] = pose.pop("alt_ft") * FT_TO_M
    out["initialPose"] = pose

    # --- initialVelocityNED ---
    v = out.get("initialVelocityNED", {})
    if "vn_fps" in v and "vn_mps" not in v:
        v["vn_mps"] = v.pop("vn_fps") * FT_TO_M
    if "ve_fps" in v and "ve_mps" not in v:
        v["ve_mps"] = v.pop("ve_fps") * FT_TO_M
    if "vd_fps" in v and "vd_mps" not in v:
        v["vd_mps"] = v.pop("vd_fps") * FT_TO_M
    out["initialVelocityNED"] = v

    # --- inertialParameters ---
    ip = out.get("inertialParameters", {})
    if "mass_lbm" in ip and "mass_kg" not in ip:
        ip["mass_kg"] = ip.pop("mass_lbm") * LBM_TO_KG

    fac_I = inertia_factor(inertia_units)
    for k in ("Ixx", "Iyy", "Izz", "Ixy", "Iyz", "Ixz"):
        if k in ip:
            ip[k] = ip[k] * fac_I

    if "xbar_ft" in ip and "xbar_m" not in ip:
        ip["xbar_m"] = ip.pop("xbar_ft") * FT_TO_M
    if "ybar_ft" in ip and "ybar_m" not in ip:
        ip["ybar_m"] = ip.pop("ybar_ft") * FT_TO_M
    if "zbar_ft" in ip and "zbar_m" not in ip:
        ip["zbar_m"] = ip.pop("zbar_ft") * FT_TO_M

    out["inertialParameters"] = ip

    # --- aerodynamicParameters ---
    ap = out.get("aerodynamicParameters", {})
    if "S_ft2" in ap and "S" not in ap:
        ap["S"] = ap.pop("S_ft2") * FT2_TO_M2
    out["aerodynamicParameters"] = ap

    return out


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Convert an Imperial YAML/JSON simulation config into SI (JSON/YAML)."
    )
    p.add_argument("-i", "--input", type=Path, required=True, help="Input .yaml/.yml or .json (Imperial fields).")
    p.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output path (.json or .yaml). Default: <input_stem>_si.(json|yaml)",
    )
    p.add_argument(
        "--out-format",
        choices=["json", "yaml"],
        default="json",
        help="Output format if --output is not provided. Default: json",
    )
    p.add_argument(
        "--inertia-units",
        choices=["lbm_ft2", "slug_ft2"],
        default="lbm_ft2",
        help="Interpret inertia tensor units as lbm*ft^2 or slug*ft^2. Default: lbm_ft2",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config(args.input)

    cfg_si = convert_imperial_to_si(cfg, inertia_units=args.inertia_units)

    out_path = args.output if args.output is not None else default_output_path(args.input, args.out_format)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    save_config(out_path, cfg_si)

    print(f"Input : {args.input}")
    print(f"Output: {out_path}")
    print(f"Inertia interpreted as: {args.inertia_units}")
    print("Done.")


if __name__ == "__main__":
    main()

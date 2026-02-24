"""
Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
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


def convert_imperial_to_si(cfg: dict) -> dict:
    """
    Convert an imperial-ish config to an SI config WITH SI-INDICATED OUTPUT KEYS.

    Output schema (as requested):
      initialPose: lat_degree, lon_degree, azimuth_degree, zenith_degree, roll_degree, alt_m
      initialVelocityNED: vn_mps, ve_mps, vd_mps
      initialRotationAboutBodyAxes: p_rad_s, q_rad_s, r_rad_s
      inertialParameters: mass_kg, Ixx/Iyy/Izz/Ixy/Iyz/Ixz (SI kg*m^2), xbar_m/ybar_m/zbar_m
      aerodynamicParameters: S (SI m^2)

    Default assumptions for *generic* keys:
      alt is ft, vn/ve/vd are ft/s, mass is slug, S is ft^2, angles are degrees, p/q/r are deg/s,
      inertia Ixx..Ixz are slug*ft^2.
    """
    out = deepcopy(cfg)

    # -------------------------
    # initialPose (rename to *_degree, alt_m)
    # -------------------------
    pose_in = out.get("initialPose", {}) or {}
    pose_out = {}

    # latitude/longitude degrees
    if "lat_degree" in pose_in:
        pose_out["lat_degree"] = float(pose_in["lat_degree"])
    elif "lat_deg" in pose_in:
        pose_out["lat_degree"] = float(pose_in["lat_deg"])
    elif "lat" in pose_in:
        pose_out["lat_degree"] = float(pose_in["lat"])
    else:
        pose_out["lat_degree"] = 0.0

    if "lon_degree" in pose_in:
        pose_out["lon_degree"] = float(pose_in["lon_degree"])
    elif "lon_deg" in pose_in:
        pose_out["lon_degree"] = float(pose_in["lon_deg"])
    elif "lon" in pose_in:
        pose_out["lon_degree"] = float(pose_in["lon"])
    else:
        pose_out["lon_degree"] = 0.0

    # attitude angles in degrees (just rename consistently)
    def angle_degree(name: str) -> float:
        if f"{name}_degree" in pose_in:
            return float(pose_in[f"{name}_degree"])
        if f"{name}_deg" in pose_in:
            return float(pose_in[f"{name}_deg"])
        if name in pose_in:
            return float(pose_in[name])
        return 0.0

    pose_out["azimuth_degree"] = angle_degree("azimuth")
    pose_out["zenith_degree"]  = angle_degree("zenith")
    pose_out["roll_degree"]    = angle_degree("roll")

    # altitude -> alt_m (accept alt_m, alt_ft, or alt assumed ft)
    if "alt_m" in pose_in:
        pose_out["alt_m"] = float(pose_in["alt_m"])
    elif "alt_ft" in pose_in:
        pose_out["alt_m"] = float(pose_in["alt_ft"]) * FT_TO_M
    elif "alt" in pose_in:
        pose_out["alt_m"] = float(pose_in["alt"]) * FT_TO_M
    else:
        pose_out["alt_m"] = 0.0

    out["initialPose"] = pose_out

    # -------------------------
    # initialVelocityNED -> *_mps
    # -------------------------
    v_in = out.get("initialVelocityNED", {}) or {}
    v_out = {}

    def to_mps(component: str) -> float:
        # priority: already SI, then fps, then kts, then generic assumed fps
        if f"{component}_mps" in v_in:
            return float(v_in[f"{component}_mps"])
        if f"{component}_fps" in v_in:
            return float(v_in[f"{component}_fps"]) * FT_TO_M
        if f"{component}_kts" in v_in:
            return float(v_in[f"{component}_kts"]) * KNOT_TO_MPS
        if component in v_in:  # generic vn/ve/vd assumed fps
            return float(v_in[component]) * FT_TO_M
        return 0.0

    v_out["vn_mps"] = to_mps("vn")
    v_out["ve_mps"] = to_mps("ve")
    v_out["vd_mps"] = to_mps("vd")

    out["initialVelocityNED"] = v_out

    # -------------------------
    # initialRotationAboutBodyAxes -> rad/s
    # -------------------------
    rot_in = out.get("initialRotationAboutBodyAxes", {}) or {}
    rot_out = {}

    def to_rad_s(component: str) -> float:
        # priority: already rad/s, then deg/s, then generic assumed deg/s
        if f"{component}_rad_s" in rot_in:
            return float(rot_in[f"{component}_rad_s"])
        if f"{component}_deg_s" in rot_in:
            return float(rot_in[f"{component}_deg_s"]) * DEG_TO_RAD
        if component in rot_in:  # generic p/q/r assumed deg/s
            return float(rot_in[component]) * DEG_TO_RAD
        return 0.0

    rot_out["p_rad_s"] = to_rad_s("p")
    rot_out["q_rad_s"] = to_rad_s("q")
    rot_out["r_rad_s"] = to_rad_s("r")

    out["initialRotationAboutBodyAxes"] = rot_out

    # -------------------------
    # inertialParameters -> mass_kg, inertia SI kg*m^2, cg in m
    # -------------------------
    ip_in = out.get("inertialParameters", {}) or {}
    ip_out = {}

    # mass: slug -> kg (accept mass_kg, mass_slug, or mass assumed slug)
    if "mass_kg" in ip_in:
        ip_out["mass_kg"] = float(ip_in["mass_kg"])
    elif "mass_slug" in ip_in:
        ip_out["mass_kg"] = float(ip_in["mass_slug"]) * SLUG_TO_KG
    elif "mass" in ip_in:
        ip_out["mass_kg"] = float(ip_in["mass"]) * SLUG_TO_KG
    else:
        ip_out["mass_kg"] = 0.0

    # inertia: accept either Ixx..Ixz (assumed slug*ft^2) OR Ixx_slug_ft2.. (also assumed slug*ft^2)
    facI = SLUG_TO_KG * FT2_TO_M2

    def inertia_component(name: str) -> float:
        # output should be SI for key "Ixx" etc
        if name in ip_in:
            return float(ip_in[name]) * facI
        k2 = f"{name}_slug_ft2"
        if k2 in ip_in:
            return float(ip_in[k2]) * facI
        return 0.0

    for k in ("Ixx", "Iyy", "Izz", "Ixy", "Iyz", "Ixz"):
        ip_out[k] = inertia_component(k)

    # CG: accept *_m, *_ft, or generic assumed ft
    def to_m(name: str) -> float:
        if f"{name}_m" in ip_in:
            return float(ip_in[f"{name}_m"])
        if f"{name}_ft" in ip_in:
            return float(ip_in[f"{name}_ft"]) * FT_TO_M
        if name in ip_in:
            return float(ip_in[name]) * FT_TO_M
        return 0.0

    ip_out["xbar_m"] = to_m("xbar")
    ip_out["ybar_m"] = to_m("ybar")
    ip_out["zbar_m"] = to_m("zbar")

    out["inertialParameters"] = ip_out

    # -------------------------
    # aerodynamicParameters -> S in m^2
    # -------------------------
    ap_in = out.get("aerodynamicParameters", {}) or {}
    ap_out = deepcopy(ap_in)

    # area: accept S (assumed ft^2) or S_ft2; if S already SI and you want to keep it,
    # then provide S_m2 in input.
    if "S_m2" in ap_in:
        ap_out["S"] = float(ap_in["S_m2"])
        ap_out.pop("S_m2", None)
    elif "S_ft2" in ap_in:
        ap_out["S"] = float(ap_in["S_ft2"]) * FT2_TO_M2
        ap_out.pop("S_ft2", None)
    elif "S" in ap_in:
        ap_out["S"] = float(ap_in["S"]) * FT2_TO_M2  # assume ft^2
    else:
        ap_out["S"] = 0.0

    out["aerodynamicParameters"] = ap_out

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
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config(args.input)

    cfg_si = convert_imperial_to_si(cfg)

    out_path = args.output if args.output is not None else default_output_path(args.input, args.out_format)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    save_config(out_path, cfg_si)

    print(f"Input : {args.input}")
    print(f"Output: {out_path}")
    print("Done.")


if __name__ == "__main__":
    main()

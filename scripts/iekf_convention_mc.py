#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""
Left- vs right-invariant EKF on SE2(3) x R^7: Monte Carlo prototype.

Throwaway evidence for the open error-convention decision (TODO-sensor-fusion.md,
doc/sensor_fusion.rst). One filter, the convention is a switch; both conventions
see the same truth, the same IMU/GNSS/baro samples and the same initial error
(paired runs), so every difference is the convention's.

Model (matches the step 2 configuration kNavBaroGnss16):
  state      X = (R, v, p) in SE2(3), b_g, b_a (body), b_p (baro, m)
  frame      launch-centred inertial (LCI), NED axes at t0; spherical-Earth
             point-mass gravity with its gradient (Earth centre below origin)
  updates    GNSS position, GNSS velocity (both in LCI), barometric height
  reset      none (standard IEKF practice) -- the only place, to first
             order, where the two conventions differ

Not modelled (deliberately; irrelevant to the comparison): ECEF/geodetic
mapping, Earth rotation in the measurement mapping, latency, lever arm.

Usage:
  python scripts/iekf_convention_mc.py                 # all scenarios, 20 runs
  python scripts/iekf_convention_mc.py --runs 50 --scenarios large_heading
  python scripts/iekf_convention_mc.py --out results/iekf_mc --no-plots
"""

from __future__ import annotations

import argparse
import multiprocessing as mp
import os
import sys
import time
import warnings
from dataclasses import dataclass, replace

import numpy as np

# ── Constants ----------------------------------------------------------------

MU = 3.986004418e14          # m^3/s^2
RE = 6_371_000.0             # m, spherical Earth
H0 = 3000.0                  # m, initial height above sphere at the LCI origin
CENTRE = np.array([0.0, 0.0, RE + H0])  # Earth centre in LCI (NED: +z down)
G0 = MU / (RE + H0) ** 2
DEG = np.pi / 180.0

I3 = np.eye(3)
NX = 16  # phi 0:3, nu 3:6, rho 6:9, b_g 9:12, b_a 12:15, b_p 15
PHI, NU, RHO, BG, BA, BP = slice(0, 3), slice(3, 6), slice(6, 9), slice(9, 12), slice(12, 15), 15


# ── Lie group helpers ----------------------------------------------------------

def skew(w):
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def vee(W):
    return np.array([W[2, 1], W[0, 2], W[1, 0]])


def so3_exp(w):
    th = np.sqrt(w @ w)
    K = skew(w)
    if th < 1e-8:
        return I3 + K + 0.5 * K @ K
    return I3 + (np.sin(th) / th) * K + ((1.0 - np.cos(th)) / th**2) * K @ K


def so3_log(R):
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(c)
    if th < 1e-8:
        return vee(0.5 * (R - R.T))
    if np.pi - th < 1e-6:  # near pi: axis from the symmetric part
        A = 0.5 * (R + I3)
        ax = A[:, np.argmax(np.diag(A))]
        ax = ax / np.linalg.norm(ax)
        return th * ax
    return (th / (2.0 * np.sin(th))) * vee(R - R.T)


def so3_jl(w):
    th = np.sqrt(w @ w)
    K = skew(w)
    if th < 1e-8:
        return I3 + 0.5 * K
    return I3 + ((1.0 - np.cos(th)) / th**2) * K + ((th - np.sin(th)) / th**3) * K @ K


def se23_exp(xi):
    phi, nu, rho = xi[0:3], xi[3:6], xi[6:9]
    J = so3_jl(phi)
    return so3_exp(phi), J @ nu, J @ rho


def se23_log(R, v, p):
    phi = so3_log(R)
    Jinv = np.linalg.inv(so3_jl(phi))
    return np.concatenate([phi, Jinv @ v, Jinv @ p])


def adjoint(R, v, p):
    Ad = np.zeros((9, 9))
    Ad[0:3, 0:3] = R
    Ad[3:6, 0:3] = skew(v) @ R
    Ad[3:6, 3:6] = R
    Ad[6:9, 0:3] = skew(p) @ R
    Ad[6:9, 6:9] = R
    return Ad


# ── Environment ------------------------------------------------------------------

def gravity(p):
    d = p - CENTRE
    r = np.sqrt(d @ d)
    return -MU * d / r**3


def gravity_gradient(p):
    d = p - CENTRE
    r = np.sqrt(d @ d)
    u = d / r
    return -(MU / r**3) * (I3 - 3.0 * np.outer(u, u))


def height(p):
    d = p - CENTRE
    return np.sqrt(d @ d) - RE


def height_grad(p):
    d = p - CENTRE
    return d / np.sqrt(d @ d)


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# ── Scenario -------------------------------------------------------------------------

@dataclass(frozen=True)
class Scenario:
    name: str
    duration_s: float = 300.0
    imu_hz: float = 50.0
    gnss_hz: float = 5.0
    baro_hz: float = 10.0
    speed_mps: float = 250.0
    profile: str = "turns"            # "turns" or "straight"
    start_offset_m: tuple = (0.0, 0.0, 0.0)
    yaw_err_deg: float | None = None  # fixed magnitude (random sign); None -> drawn from sigma
    yaw_sigma_deg: float = 2.0
    tilt_sigma_deg: float = 1.0
    vel_sigma_mps: float = 0.5
    pos_sigma_m: float = 5.0
    gnss_outage_s: tuple | None = None
    cov_dtype: str = "float64"
    # sensors (MEMS class)
    gyro_nd: float = 5e-5             # rad/s/sqrt(Hz)
    accel_nd: float = 7e-4            # m/s^2/sqrt(Hz)
    gyro_rw: float = 2e-5             # rad/s/sqrt(s)
    accel_rw: float = 2e-4            # m/s^2/sqrt(s)
    baro_rw: float = 0.05             # m/sqrt(s)
    gyro_bias0: float = 2e-3          # rad/s, 1 sigma
    accel_bias0: float = 0.03         # m/s^2, 1 sigma
    baro_bias0: float = 10.0          # m, 1 sigma
    gnss_h_sigma: float = 1.5         # m
    gnss_v_sigma: float = 3.0         # m
    gnss_vel_sigma: float = 0.1       # m/s
    baro_sigma: float = 0.5           # m


SCENARIOS = {
    "nominal": Scenario("nominal"),
    "large_heading": Scenario("large_heading", yaw_err_deg=45.0, yaw_sigma_deg=45.0),
    "huge_heading": Scenario("huge_heading", yaw_err_deg=120.0, yaw_sigma_deg=120.0),
    "outage": Scenario("outage", gnss_outage_s=(100.0, 160.0), yaw_sigma_deg=5.0),
    "far_weak_yaw_f64": Scenario("far_weak_yaw_f64", profile="straight", start_offset_m=(150e3, 50e3, 0.0),
                                 yaw_sigma_deg=10.0),
    "far_weak_yaw_f32": Scenario("far_weak_yaw_f32", profile="straight", start_offset_m=(150e3, 50e3, 0.0),
                                 yaw_sigma_deg=10.0, cov_dtype="float32"),
    "nominal_f32": Scenario("nominal_f32", cov_dtype="float32"),
}


# ── Truth and sensors --------------------------------------------------------------

def make_truth(sc: Scenario, rng):
    """Truth from desired attitude/velocity, integrated with the same discrete
    scheme the filter uses, so IMU noise and biases are the only model error."""
    dt = 1.0 / sc.imu_hz
    n = int(round(sc.duration_s * sc.imu_hz))
    t = np.arange(n + 1) * dt

    # Heading rate: S-turns (bank up to ~35 deg) or straight; light "turbulence"
    # as a sum of incommensurate sinusoids on roll/pitch/yaw.
    if sc.profile == "turns":
        psi_dot = 4.0 * DEG * np.sin(2 * np.pi * t / 50.0) * (t > 30.0)
    else:
        psi_dot = np.zeros_like(t)
    psi = np.cumsum(psi_dot) * dt + rng.uniform(0, 2 * np.pi)
    phs = rng.uniform(0, 2 * np.pi, 6)
    turb = lambda a, f, k: a * DEG * np.sin(2 * np.pi * f * t + phs[k])
    bank = np.arctan(sc.speed_mps * psi_dot / G0) + turb(1.0, 0.7, 0) + turb(0.5, 1.9, 1)
    pitch = turb(0.5, 0.5, 2) + turb(0.3, 1.3, 3)
    yaw_j = turb(0.3, 0.9, 4) + turb(0.2, 2.3, 5)

    v_des = sc.speed_mps * np.stack([np.cos(psi), np.sin(psi), np.zeros_like(psi)], axis=1)

    R = np.empty((n + 1, 3, 3))
    v = np.empty((n + 1, 3))
    p = np.empty((n + 1, 3))
    w_b = np.empty((n, 3))
    f_b = np.empty((n, 3))
    R[0] = rot_z(psi[0] + yaw_j[0]) @ rot_y(pitch[0]) @ rot_x(bank[0])
    v[0] = v_des[0]
    p[0] = np.array(sc.start_offset_m, dtype=float)
    for k in range(n):
        R_next = rot_z(psi[k + 1] + yaw_j[k + 1]) @ rot_y(pitch[k + 1]) @ rot_x(bank[k + 1])
        w_b[k] = so3_log(R[k].T @ R_next) / dt
        g = gravity(p[k])
        a_w = (v_des[k + 1] - v[k]) / dt
        f_b[k] = R[k].T @ (a_w - g)
        # the filter's scheme, exactly
        p[k + 1] = p[k] + v[k] * dt + 0.5 * a_w * dt * dt
        v[k + 1] = v[k] + a_w * dt
        R[k + 1] = R[k] @ so3_exp(w_b[k] * dt)
    return t, R, v, p, w_b, f_b


def make_sensors(sc: Scenario, t, w_b, f_b, p, v, rng):
    dt = 1.0 / sc.imu_hz
    n = len(w_b)
    bg = np.empty((n + 1, 3))
    ba = np.empty((n + 1, 3))
    bp = np.empty(n + 1)
    bg[0] = rng.normal(0, sc.gyro_bias0, 3)
    ba[0] = rng.normal(0, sc.accel_bias0, 3)
    bp[0] = rng.normal(0, sc.baro_bias0)
    sq = np.sqrt(dt)
    for k in range(n):
        bg[k + 1] = bg[k] + rng.normal(0, sc.gyro_rw * sq, 3)
        ba[k + 1] = ba[k] + rng.normal(0, sc.accel_rw * sq, 3)
        bp[k + 1] = bp[k] + rng.normal(0, sc.baro_rw * sq)
    gyro = w_b + bg[:-1] + rng.normal(0, sc.gyro_nd / sq, (n, 3))
    accel = f_b + ba[:-1] + rng.normal(0, sc.accel_nd / sq, (n, 3))

    gnss_every = int(round(sc.imu_hz / sc.gnss_hz))
    baro_every = int(round(sc.imu_hz / sc.baro_hz))
    gnss_sig = np.array([sc.gnss_h_sigma, sc.gnss_h_sigma, sc.gnss_v_sigma])
    gnss = {}
    baro = {}
    for k in range(gnss_every, n + 1, gnss_every):
        if sc.gnss_outage_s and sc.gnss_outage_s[0] <= t[k] < sc.gnss_outage_s[1]:
            continue
        gnss[k] = (p[k] + rng.normal(0, 1, 3) * gnss_sig, v[k] + rng.normal(0, sc.gnss_vel_sigma, 3))
    for k in range(baro_every, n + 1, baro_every):
        baro[k] = height(p[k]) + bp[k] + rng.normal(0, sc.baro_sigma)
    return gyro, accel, gnss, baro, bg, bp, ba


# ── Filter ---------------------------------------------------------------------------

def error_jacobians(conv, R, v, p, w, a):
    """Continuous-time error dynamics xi_dot = F xi + Gc n for the estimate
    (R, v, p) and bias-corrected IMU readings (w, a). Gravity gradient included."""
    g = gravity(p)
    G = gravity_gradient(p)
    F = np.zeros((NX, NX))
    Gc = np.zeros((NX, 13))
    if conv == "RI":
        Ad = adjoint(R, v, p)
        F[NU, PHI] = skew(g) - G @ skew(p)
        F[NU, RHO] = G
        F[RHO, NU] = I3
        F[0:9, BG] = -Ad[:, 0:3]
        F[0:9, BA] = -Ad[:, 3:6]
        Gc[0:9, 0:3] = Ad[:, 0:3]
        Gc[0:9, 3:6] = Ad[:, 3:6]
    else:
        Ww = skew(w)
        F[PHI, PHI] = -Ww
        F[NU, PHI] = -skew(a)
        F[NU, NU] = -Ww
        F[NU, RHO] = R.T @ G @ R
        F[RHO, NU] = I3
        F[RHO, RHO] = -Ww
        F[PHI, BG] = -I3
        F[NU, BA] = -I3
        Gc[PHI, 0:3] = I3
        Gc[NU, 3:6] = I3
    Gc[BG, 6:9] = I3
    Gc[BA, 9:12] = I3
    Gc[BP, 12] = 1.0
    return F, Gc


class InvariantEKF:
    """IEKF on SE2(3) x R^7; `conv` is "LI" or "RI"."""

    def __init__(self, conv, sc: Scenario, R, v, p, P_phys):
        self.conv, self.sc = conv, sc
        self.cdt = np.float32 if sc.cov_dtype == "float32" else np.float64
        self.R, self.v, self.p = R.copy(), v.copy(), p.copy()
        self.bg, self.ba, self.bp = np.zeros(3), np.zeros(3), 0.0
        # P0 given in physical coordinates (world attitude error, dv, dp, biases);
        # mapped to the convention's coordinates so both start equivalent.
        J = np.eye(NX)
        if conv == "RI":
            J[NU, PHI] = skew(v)
            J[RHO, PHI] = skew(p)
        else:
            J[PHI, PHI] = R.T
            J[NU, NU] = R.T
            J[RHO, RHO] = R.T
        self.P = (J @ P_phys @ J.T).astype(self.cdt)
        sc_ = sc
        self.Qc = np.diag(np.r_[[sc_.gyro_nd**2] * 3, [sc_.accel_nd**2] * 3,
                                [sc_.gyro_rw**2] * 3, [sc_.accel_rw**2] * 3, sc_.baro_rw**2])

    def propagate(self, gyro, accel, dt):
        w = gyro - self.bg
        a = accel - self.ba
        R, v, p = self.R, self.v, self.p
        g = gravity(p)
        F0, Gc0 = error_jacobians(self.conv, R, v, p, w, a)

        a_w = R @ a + g
        self.p = p + v * dt + 0.5 * a_w * dt * dt
        self.v = v + a_w * dt
        self.R = R @ so3_exp(w * dt)

        # Trapezoidal F: the RI bias coupling -Ad_Xhat varies within a step at
        # a rate ~ |p_hat| |w|, so a start-of-step F is only first-order
        # accurate for RI (LI carries that variation inside A_L).
        F1, Gc1 = error_jacobians(self.conv, self.R, self.v, self.p, w, a)
        F, Gc = 0.5 * (F0 + F1), 0.5 * (Gc0 + Gc1)
        Fd = F * dt
        Phi = (np.eye(NX) + Fd + 0.5 * Fd @ Fd).astype(self.cdt)
        Qd = (Gc @ self.Qc @ Gc.T * dt).astype(self.cdt)
        P = Phi @ self.P @ Phi.T + Qd
        self.P = (0.5 * (P + P.T)).astype(self.cdt)

    def _update(self, z, H, Rn):
        c = self.cdt
        P = self.P
        H = H.astype(c)
        S = H @ P @ H.T + Rn.astype(c)
        K = np.linalg.solve(S, H @ P).T
        IKH = np.eye(NX, dtype=c) - K @ H
        P = IKH @ P @ IKH.T + K @ Rn.astype(c) @ K.T
        self.P = (0.5 * (P + P.T)).astype(c)
        d = (K @ z.astype(c)).astype(np.float64)

        dR, dv, dp = se23_exp(d[0:9])
        if self.conv == "RI":   # X <- Exp(d) X
            self.R, self.v, self.p = dR @ self.R, dR @ self.v + dv, dR @ self.p + dp
        else:                   # X <- X Exp(d)
            self.R, self.v, self.p = self.R @ dR, self.R @ dv + self.v, self.R @ dp + self.p
        self.bg = self.bg + d[BG]
        self.ba = self.ba + d[BA]
        self.bp = self.bp + d[BP]

    def update_gnss(self, y_p, y_v, sig_p, sig_v):
        R, v, p = self.R, self.v, self.p
        Sp = np.diag(sig_p**2)
        Sv = np.eye(3) * sig_v**2
        H = np.zeros((6, NX))
        if self.conv == "RI":
            z = np.r_[y_p - p, y_v - v]
            H[0:3, PHI] = -skew(p)
            H[0:3, RHO] = I3
            H[3:6, PHI] = -skew(v)
            H[3:6, NU] = I3
            Rn = np.zeros((6, 6))
            Rn[0:3, 0:3] = Sp
            Rn[3:6, 3:6] = Sv
        else:
            z = np.r_[R.T @ (y_p - p), R.T @ (y_v - v)]
            H[0:3, RHO] = I3
            H[3:6, NU] = I3
            Rn = np.zeros((6, 6))
            Rn[0:3, 0:3] = R.T @ Sp @ R
            Rn[3:6, 3:6] = R.T @ Sv @ R
        self._update(z, H, Rn)

    def update_baro(self, y, sig):
        R, p = self.R, self.p
        gh = height_grad(p)
        H = np.zeros((1, NX))
        if self.conv == "RI":
            H[0, PHI] = -gh @ skew(p)
            H[0, RHO] = gh
        else:
            H[0, RHO] = gh @ R
        H[0, BP] = 1.0
        z = np.array([y - (height(p) + self.bp)])
        self._update(z, H, np.array([[sig**2]]))

    def error(self, R, v, p, bg, ba, bp):
        """True error in this filter's coordinates (for NEES)."""
        if self.conv == "RI":
            E = R @ self.R.T
            xi = se23_log(E, v - E @ self.v, p - E @ self.p)
        else:
            Rt = self.R.T
            xi = se23_log(Rt @ R, Rt @ (v - self.v), Rt @ (p - self.p))
        return np.r_[xi, bg - self.bg, ba - self.ba, bp - self.bp]


# ── One paired run ---------------------------------------------------------------------

def run_pair(args):
    sc, seed = args
    rng = np.random.default_rng(seed)
    t, Rt, vt, pt, w_b, f_b = make_truth(sc, rng)
    gyro, accel, gnss, baro, bg, bp, ba = make_sensors(sc, t, w_b, f_b, pt, vt, rng)

    # initial error, physical coordinates; R_true = Exp(dth) R_hat
    yaw = (sc.yaw_err_deg * DEG * rng.choice([-1.0, 1.0]) if sc.yaw_err_deg is not None
           else rng.normal(0, sc.yaw_sigma_deg * DEG))
    dth = np.r_[rng.normal(0, sc.tilt_sigma_deg * DEG, 2), yaw]
    dv = rng.normal(0, sc.vel_sigma_mps, 3)
    dp = rng.normal(0, sc.pos_sigma_m, 3)
    R0 = so3_exp(-dth) @ Rt[0]
    v0, p0 = vt[0] - dv, pt[0] - dp
    P_phys = np.diag(np.r_[[(sc.tilt_sigma_deg * DEG) ** 2] * 2, (sc.yaw_sigma_deg * DEG) ** 2,
                           [sc.vel_sigma_mps**2] * 3, [sc.pos_sigma_m**2] * 3,
                           [sc.gyro_bias0**2] * 3, [sc.accel_bias0**2] * 3, sc.baro_bias0**2])

    dt = 1.0 / sc.imu_hz
    gsig = np.array([sc.gnss_h_sigma, sc.gnss_h_sigma, sc.gnss_v_sigma])
    log_every = int(round(sc.imu_hz / sc.gnss_hz))
    out = {}
    for conv in ("LI", "RI"):
        f = InvariantEKF(conv, sc, R0, v0, p0, P_phys)
        rec = {k: [] for k in ("t", "nees", "yaw", "tilt", "pos", "vel", "bad")}
        failed = False
        for k in range(len(w_b)):
            f.propagate(gyro[k], accel[k], dt)
            kk = k + 1
            try:
                if kk in gnss:
                    f.update_gnss(gnss[kk][0], gnss[kk][1], gsig, sc.gnss_vel_sigma)
                if kk in baro:
                    f.update_baro(baro[kk], sc.baro_sigma)
            except np.linalg.LinAlgError:
                failed = True
            if kk % log_every:
                continue
            e = f.error(Rt[kk], vt[kk], pt[kk], bg[kk], ba[kk], bp[kk])
            P64 = f.P.astype(np.float64)
            try:
                np.linalg.cholesky(P64)
                nees = float(e @ np.linalg.solve(P64, e))
                pd = True
            except np.linalg.LinAlgError:
                nees, pd = np.nan, False
            ew = so3_log(Rt[kk] @ f.R.T)  # world-frame attitude error
            rec["t"].append(t[kk])
            rec["nees"].append(nees)
            rec["yaw"].append(ew[2] / DEG)
            rec["tilt"].append(np.hypot(ew[0], ew[1]) / DEG)
            rec["pos"].append(np.linalg.norm(pt[kk] - f.p))
            rec["vel"].append(np.linalg.norm(vt[kk] - f.v))
            rec["bad"].append((not pd) or failed or not np.all(np.isfinite(f.p)))
            if not np.all(np.isfinite(f.p)):
                break
        out[conv] = {k: np.array(v_) for k, v_ in rec.items()}
    return out


# ── Analysis --------------------------------------------------------------------------

def summarise(name, sc, results):
    def stack(conv, key):
        L = min(len(r[conv][key]) for r in results)
        return np.stack([r[conv][key][:L] for r in results])

    t = results[0]["LI"]["t"][: min(len(r["LI"]["t"]) for r in results)]
    late = t >= t[-1] / 3.0
    rows = {}
    for conv in ("LI", "RI"):
        nees = stack(conv, "nees")
        yaw = stack(conv, "yaw")
        pos = stack(conv, "pos")
        bad = stack(conv, "bad")
        finite = np.isfinite(nees)
        anees = np.nanmean(np.where(finite[:, late], nees[:, late], np.nan)) / NX
        # convergence: first time |yaw| < 1 deg and stays below to the end
        conv_t = []
        for y in yaw:
            ok = np.abs(y) < 1.0
            idx = np.where(~ok)[0]
            last_bad = idx[-1] + 1 if len(idx) else 0
            conv_t.append(t[last_bad] if last_bad < len(t) else np.inf)
        rows[conv] = dict(
            anees=anees,
            yaw_rms_end=np.sqrt(np.mean(yaw[:, -1] ** 2)),
            pos_rms_end=np.sqrt(np.mean(pos[:, -1] ** 2)),
            pos_max=np.max(pos),
            conv_med=np.median(conv_t),
            conv_never=int(np.sum(np.isinf(conv_t))),
            runs_bad=int(np.sum(bad.any(axis=1))),
        )
        if sc.gnss_outage_s:
            w = (t >= sc.gnss_outage_s[1]) & (t < sc.gnss_outage_s[1] + 20.0)
            rows[conv]["reacq_anees"] = np.nanmean(nees[:, w]) / NX
            rows[conv]["outage_pos_max"] = np.max(pos[:, (t >= sc.gnss_outage_s[0]) & (t <= sc.gnss_outage_s[1])])
    dyaw = np.abs(stack("LI", "yaw") - stack("RI", "yaw"))
    rows["paired"] = dict(max_dyaw=np.nanmax(dyaw), mean_dyaw=np.nanmean(dyaw))
    return t, rows


def print_table(all_rows):
    hdr = f"{'scenario':20s} {'conv':4s} {'ANEES':>7s} {'yaw_rms_end':>11s} {'pos_rms_end':>11s} " \
          f"{'pos_max':>9s} {'yaw<1deg_med_s':>14s} {'never':>5s} {'bad':>4s}  extra"
    print(hdr)
    print("-" * len(hdr))
    for name, rows in all_rows.items():
        for conv in ("LI", "RI"):
            r = rows[conv]
            extra = ""
            if "reacq_anees" in r:
                extra = f"reacq_ANEES={r['reacq_anees']:.2f} outage_pos_max={r['outage_pos_max']:.0f} m"
            print(f"{name:20s} {conv:4s} {r['anees']:7.2f} {r['yaw_rms_end']:11.3f} {r['pos_rms_end']:11.2f} "
                  f"{r['pos_max']:9.1f} {r['conv_med']:14.1f} {r['conv_never']:5d} {r['runs_bad']:4d}  {extra}")
        pr = rows["paired"]
        print(f"{'':20s} LI-RI paired |dyaw|: mean {pr['mean_dyaw']:.2e} deg, max {pr['max_dyaw']:.2e} deg")


def plot(all_series, out_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available; skipping plots")
        return
    n = len(all_series)
    fig, axes = plt.subplots(n, 2, figsize=(11, 2.6 * n), squeeze=False)
    for i, (name, (t, res, sc)) in enumerate(all_series.items()):
        for conv, col in (("LI", "C0"), ("RI", "C3")):
            L = len(t)
            yaw = np.stack([r[conv]["yaw"][:L] for r in res])
            nees = np.stack([r[conv]["nees"][:L] for r in res])
            axes[i, 0].semilogy(t, np.sqrt(np.mean(yaw**2, axis=0)), col, label=conv)
            with np.errstate(all="ignore"), warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN columns (failed float32 runs)
                anees_t = np.nanmean(nees, axis=0) / NX
            axes[i, 1].semilogy(t, anees_t, col, label=conv)
        axes[i, 0].set_ylabel(f"{name}\nyaw RMS [deg]", fontsize=8)
        axes[i, 1].set_ylabel("ANEES", fontsize=8)
        axes[i, 1].axhline(1.0, color="k", lw=0.6, ls="--")
        if sc.gnss_outage_s:
            for ax in axes[i]:
                ax.axvspan(*sc.gnss_outage_s, color="0.85")
        axes[i, 0].legend(fontsize=7)
    for ax in axes[-1]:
        ax.set_xlabel("time [s]")
    fig.tight_layout()
    path = os.path.join(out_dir, "iekf_convention_mc.png")
    fig.savefig(path, dpi=120)
    print(f"plot: {path}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=20)
    ap.add_argument("--scenarios", nargs="*", default=list(SCENARIOS))
    ap.add_argument("--duration", type=float, default=None, help="override scenario duration [s]")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", default="results/iekf_convention_mc")
    ap.add_argument("--no-plots", action="store_true")
    a = ap.parse_args(argv)

    os.makedirs(a.out, exist_ok=True)
    all_rows, all_series = {}, {}
    with mp.Pool(a.jobs) as pool:
        for name in a.scenarios:
            sc = SCENARIOS[name]
            if a.duration:
                sc = replace(sc, duration_s=a.duration)
                if sc.gnss_outage_s and sc.gnss_outage_s[1] > a.duration:
                    sc = replace(sc, gnss_outage_s=None)
            t0 = time.time()
            res = pool.map(run_pair, [(sc, a.seed * 100003 + i) for i in range(a.runs)])
            t, rows = summarise(name, sc, res)
            all_rows[name] = rows
            all_series[name] = (t, res, sc)
            print(f"[{name}] {a.runs} paired runs in {time.time() - t0:.0f} s", file=sys.stderr)
    print()
    print_table(all_rows)
    if not a.no_plots:
        plot(all_series, a.out)


if __name__ == "__main__":
    main()

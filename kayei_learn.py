"""kayEi groundwork: measure what feed-forward alignment needs.

  python kayei_learn.py latency   # goniometer (and Z) step -> when does the image respond?
  python kayei_learn.py wobble    # 2 turns at 30 and 90 deg/s (at 2 scan positions if the
                                  # scan stage is up): fit the fiber line model, repeatability
  python kayei_learn.py all

Model (fiber = line offset e at the chuck, tilted t vs the rotation axis, rotation angle th):
  image angle(th)      = a0 + A1 cos th + A2 sin th           (same at every scan position)
  image offset(th, s)  = b0 + (B1 + s*A1') cos th + (B2 + s*A2') sin th
Raw data -> kayei_data/*.csv, summary plot -> kayei_data/<name>.png."""
import csv
import os
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pipython import GCSDevice, pitools

import fiber_align as fa
from hydra import Hydra

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "kayei_data")
SCAN_SN = "0165500183"
EXPOSURE_MS = fa.EXPOSURE_MS
GAINS = (1.0, 4.0, 6.0, 9.0)   # tried in order until the fiber is detected (illumination varies)
GONIO_STEP = 0.3     # deg
Z_STEP = 0.2         # mm (~10 px)
WOBBLE_SPEEDS = (30.0, 90.0)
SCAN_POSITIONS = (5.0, 40.0)


def pi_stage(sn, name):
    try:
        pi = GCSDevice()
        pi.ConnectUSB(sn)
        if not pi.qFRF()["1"]:
            pitools.startup(pi, refmodes="FNL")
        return pi
    except Exception as e:
        print(f"{name} stage not available ({str(e).strip().splitlines()[-1]}); parts needing it are skipped")
        return None


class Rig(fa.Rig):
    """camera + Hydra like fa.Rig, PI Z and scan stages optional; fresh frames only."""

    def __init__(self):
        import neoapi
        self.cam = neoapi.Cam()
        self.cam.Connect(fa.CAM_IP)
        self.cam.GetFeature("ExposureAuto").SetString("Off")
        self.cam.f.ExposureTime.Set(EXPOSURE_MS * 1000)
        for gain in GAINS:   # keep the 20 ms frame; compensate dim illumination with gain
            self.cam.f.Gain.Set(gain)
            for _ in range(3):
                img = self.cam.GetImage(2000)
            if fa.measure(img.GetNPArray().squeeze()) is not None:
                break
        print(f"camera gain {gain:g}")
        self.h = Hydra(fa.HYDRA_IP)
        self.z = pi_stage(fa.Z_SN, "Z")
        self.scan = pi_stage(SCAN_SN, "scan")

    def fresh(self):
        """(arrival time, measurement) of a frame exposed after the call"""
        self.cam.ClearImages()
        img = self.cam.GetImage(2000)
        t = time.perf_counter()
        return t, fa.measure(img.GetNPArray().squeeze())

    def scan_to(self, pos):
        self.scan.MOV("1", pos); pitools.waitontarget(self.scan); time.sleep(0.2)

    def close(self):
        for pi in (self.scan, self.z):
            if pi:
                pi.CloseConnection()
        self.h.close()


def save_csv(name, header, rows):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    return path


# ---------------------------------------------------------------- latency
def step_response(rig, do_step, key, n_before=15, n_after=60):
    """Record key ('angle' or 'x') around a step command. Returns rows (t rel. to the command, value)."""
    rows = []
    for _ in range(n_before):
        t, m = rig.fresh()
        rows.append((t, m[key]))
    t_cmd = time.perf_counter()
    do_step()
    for _ in range(n_after):
        t, m = rig.fresh()
        rows.append((t, m[key]))
    return [(t - t_cmd, v) for t, v in rows]


def analyse_step(rows, n_before):
    t = np.array([r[0] for r in rows]); v = np.array([r[1] for r in rows])
    base, noise = v[:n_before].mean(), max(v[:n_before].std(), 1e-6)
    final = v[-10:].mean()
    moved = np.where((t > 0) & (np.abs(v - base) > max(6 * noise, 0.05 * abs(final - base))))[0]
    t_first = t[moved[0]] if len(moved) else np.nan
    done = np.where((t > 0) & (np.abs(v - final) < max(6 * noise, 0.1 * abs(final - base))))[0]
    # first index after which the value stays settled
    t_settle = np.nan
    for i in done:
        if np.all(np.abs(v[i:] - final) < max(6 * noise, 0.1 * abs(final - base))):
            t_settle = t[i]; break
    return dict(t_first=t_first, t_settle=t_settle, step=final - base, noise=noise)


def run_latency(rig):
    results, allrows = [], []
    g0 = rig.gonio()
    for i, sign in enumerate((+1, -1, +1, -1)):
        rows = step_response(rig, lambda: rig.h.move_abs(fa.GONIO, g0 + (sign > 0) * GONIO_STEP), "angle")
        rig.h.wait(fa.GONIO)
        r = analyse_step(rows, 15); r["axis"] = "gonio"; results.append(r)
        allrows += [("gonio", i, t, v) for t, v in rows]
        print(f"gonio step {sign * GONIO_STEP:+.1f}: image responds after {r['t_first'] * 1000:5.0f} ms, "
              f"settled after {r['t_settle'] * 1000:5.0f} ms, image step {r['step']:+.3f} deg, noise {r['noise']:.4f}")
    rig.hydra_to(fa.GONIO, g0)
    z0 = rig.zpos() if rig.z else None
    for i, sign in enumerate((+1, -1, +1, -1) if rig.z else ()):
        rows = step_response(rig, lambda: rig.z.MOV("1", z0 + (sign > 0) * Z_STEP), "x")
        pitools.waitontarget(rig.z)
        r = analyse_step(rows, 15); r["axis"] = "z"; results.append(r)
        allrows += [("z", i, t, v) for t, v in rows]
        print(f"Z step {sign * Z_STEP:+.1f} mm: image responds after {r['t_first'] * 1000:5.0f} ms, "
              f"settled after {r['t_settle'] * 1000:5.0f} ms, image step {r['step']:+.1f} px, noise {r['noise']:.3f}")
    if rig.z:
        rig.z_to(z0)
    save_csv("latency.csv", ["axis", "rep", "t_s", "value"], allrows)
    # frame period
    ts = [t for ax, i, t, v in allrows if ax == "gonio" and i == 0]
    print(f"frame period {np.mean(np.diff(ts)) * 1000:.1f} ms")
    fig, axs = plt.subplots(1, 2, figsize=(12, 4))
    for ax, name, unit in zip(axs, ("gonio", "z"), ("image angle [deg]", "image offset [px]")):
        for i in range(4):
            r = [(t, v) for a, k, t, v in allrows if a == name and k == i]
            ax.plot([t * 1000 for t, v in r], [v for t, v in r], ".-", lw=0.8, ms=3, label=f"step {i}")
        ax.axvline(0, color="k", lw=0.8); ax.set_xlabel("ms after command"); ax.set_ylabel(unit)
        ax.set_title(f"{name} step response"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "latency.png"), dpi=110)
    for name in ("gonio", "z"):
        rs = [r for r in results if r["axis"] == name]
        if rs:
            print(f"{name}: first response {np.nanmean([r['t_first'] for r in rs]) * 1000:.0f} ms, "
                  f"settled {np.nanmean([r['t_settle'] for r in rs]) * 1000:.0f} ms (mean of 4)")


# ---------------------------------------------------------------- wobble
def turns(rig, speed, n_turns=2):
    """Rotate n_turns at speed, log (t, theta, angle, x) with fresh frames. theta from the encoder,
    read right after the frame arrives."""
    h, rows = rig.h, []
    th0 = h.position(fa.ROT)
    v0 = h.velocity(fa.ROT)
    h.set_velocity(fa.ROT, speed)
    h.move_abs(fa.ROT, th0 + 360 * n_turns + 20)
    try:
        while True:
            t, m = rig.fresh()
            th = h.position(fa.ROT)
            if m:
                rows.append((t, th - th0, m["angle"], m["x"], m["width"]))
            if th - th0 >= 360 * n_turns:
                break
    finally:
        h.stop(fa.ROT)
        time.sleep(0.3)
        h.set_velocity(fa.ROT, v0)
    rig.hydra_to(fa.ROT, th0 + 360 * (n_turns + 1))  # park at the same angle modulo 360
    return rows


def fit_sine(th_deg, y):
    """y = c0 + c1 cos th + c2 sin th; returns coefficients, residual rms, amplitude, phase"""
    th = np.radians(th_deg)
    A = np.c_[np.ones_like(th), np.cos(th), np.sin(th)]
    c = np.linalg.lstsq(A, y, rcond=None)[0]
    res = y - A @ c
    return c, res.std(), np.hypot(c[1], c[2]), np.degrees(np.arctan2(c[2], c[1]))


def run_wobble(rig):
    scans = SCAN_POSITIONS if rig.scan else (None,)
    allrows, fits = [], {}
    for s in scans:
        if s is not None:
            rig.scan_to(s)
        for v in WOBBLE_SPEEDS:
            rows = turns(rig, v)
            allrows += [(s if s is not None else -1, v, *r) for r in rows]
            th = np.array([r[1] for r in rows]); ang = np.array([r[2] for r in rows]); x = np.array([r[3] for r in rows])
            fits[(s, v)] = {}
            for key, y in (("angle", ang), ("x", x)):
                c, rms, amp, ph = fit_sine(th, y)
                # repeatability: fit turn 1, evaluate on turn 2
                t1, t2 = th < 360, th >= 360
                c1 = fit_sine(th[t1], y[t1])[0]
                A2 = np.c_[np.ones(t2.sum()), np.cos(np.radians(th[t2])), np.sin(np.radians(th[t2]))]
                rep = (y[t2] - A2 @ c1).std()
                fits[(s, v)][key] = dict(c=c, rms=rms, amp=amp, phase=ph, rep=rep, n=len(th))
                unit = "deg" if key == "angle" else "px"
                print(f"scan {s} speed {v:3.0f}: {key:5s} amp {amp:7.3f} {unit} phase {ph:+7.1f} deg "
                      f"fit rms {rms:.4f}  turn1->turn2 rms {rep:.4f}  ({len(th)} samples)")
    save_csv("wobble.csv", ["scan_mm", "speed", "t_s", "theta_deg", "angle_deg", "x_px", "width_px"], allrows)

    # phase vs speed -> delay between the frame and the encoder read (phase lag grows with speed)
    for s in scans:
        for key in ("angle", "x"):
            ph = [fits[(s, v)][key]["phase"] for v in WOBBLE_SPEEDS]
            dph = (ph[1] - ph[0] + 180) % 360 - 180
            lag = dph / (WOBBLE_SPEEDS[1] - WOBBLE_SPEEDS[0])  # deg per (deg/s) = s
            print(f"scan {s} {key}: phase shift {dph:+.2f} deg between {WOBBLE_SPEEDS[0]:.0f} and "
                  f"{WOBBLE_SPEEDS[1]:.0f} deg/s -> frame-to-encoder delay {lag * 1000:+.0f} ms")
    if rig.scan:
        # tilt from the offset amplitude change along the fiber vs tilt from the image angle
        v = WOBBLE_SPEEDS[0]
        cA, cB = fits[(scans[0], v)]["x"]["c"], fits[(scans[1], v)]["x"]["c"]
        ds = scans[1] - scans[0]
        px_per_mm_per_deg = np.tan(np.radians(1)) * 1000 / (1000 / abs(fa_cal_px_per_mm()))
        ca = fits[(scans[0], v)]["angle"]["c"]
        print(f"offset amplitude change over {ds:.0f} mm scan: cos {cB[1] - cA[1]:+.2f} px, sin {cB[2] - cA[2]:+.2f} px;"
              f" predicted from the image angle: cos {ca[1] * ds * px_per_mm_per_deg:+.2f}, sin {ca[2] * ds * px_per_mm_per_deg:+.2f}")

    fig, axs = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for (s, v), f in fits.items():
        sel = [r for r in allrows if r[0] == (s if s is not None else -1) and r[1] == v]
        th = np.array([r[3] for r in sel])
        for ax, key, col in ((axs[0], "angle", 4), (axs[1], "x", 5)):
            y = np.array([r[col] for r in sel])
            c = f[key]["c"]
            A = np.c_[np.ones_like(th), np.cos(np.radians(th)), np.sin(np.radians(th))]
            ax.plot(th % 360, y - A @ c, ".", ms=2, label=f"scan {s} {v:.0f} deg/s residual")
    axs[0].set_ylabel("angle residual [deg]"); axs[1].set_ylabel("offset residual [px]")
    axs[1].set_xlabel("rotation angle [deg]")
    for ax in axs:
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
    axs[0].set_title("what is left after the sine model (repeatable shape -> learnable)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "wobble.png"), dpi=110)


def fa_cal_px_per_mm():
    return 49.5  # px per mm of Z (image offset calibration, ~20 um/px)


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    rig = Rig()
    try:
        if what in ("latency", "all"):
            run_latency(rig)
        if what in ("wobble", "all"):
            run_wobble(rig)
    finally:
        rig.close()


if __name__ == "__main__":
    main()

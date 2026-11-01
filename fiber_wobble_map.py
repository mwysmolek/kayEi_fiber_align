"""The uncontrolled fiber motion, measured and shown in 2D, 3D and 4D (3D + time).

Goniometer and Z fixed (aligned once at the middle scan position); at several scan
positions the rotation makes 2 full turns at 60 deg/s while every frame is measured.
From the sine fit at each scan position s:
    x(th, s) = b0(s) + B1(s) cos th + B2(s) sin th
the fiber axis in the ROTATING frame is (ex, ey)(s) = (B1, -B2)(s) in px -> mm: the camera
sees the in-plane component, the out-of-plane component (towards the camera = focus) is the
quadrature part. So one camera + one turn gives the full 3D centre line of the fiber
relative to the rotation axis. Outputs (kayei_data/ and docs/figures/):
  wobble_map.csv                 raw frames (scan, theta, angle, x)
  wobble_2d.png                  angle and offset vs rotation angle, one curve per scan position
  wobble_3d.png                  surfaces offset(theta, s), angle(theta, s) + the reconstructed 3D fiber axis
  wobble_4d.mp4                  the 3D fiber axis rotating (3D + time)
Run: python fiber_wobble_map.py"""
import csv
import os
import time

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pipython import pitools

import fiber_align as fa
import kayei
from kayei_learn import Rig, fit_sine

HERE = os.path.dirname(os.path.abspath(__file__))
SCANS = (5.0, 15.0, 25.0, 35.0, 45.0)
SPEED = 60.0
PX_PER_MM = fa.PX_PER_MM
UM = 1000 / PX_PER_MM


def turns(rig, s, n=2):
    h = rig.h
    h.rezero_turn(fa.ROT); time.sleep(0.1)
    clk = kayei.RotClock(SPEED); th0 = h.position(fa.ROT); clk.sync(th0)
    v0 = h.velocity(fa.ROT); h.set_velocity(fa.ROT, SPEED); h.move_abs(fa.ROT, th0 + 360 * n + 30)
    out, t_sync = [], time.time(); t_start = t_sync
    try:
        while True:
            t, m = rig.fresh(); now = time.time()
            if now - t_sync > 0.25:
                tq = time.time(); thq = h.position(fa.ROT); clk.sync(thq, (tq + time.time()) / 2); t_sync = time.time()
                if now - t_start > 3.0 and thq - th0 < 5.0:
                    raise RuntimeError(f"rotation not moving (status {h.status(fa.ROT):#x}, powered {h.powered(fa.ROT)}, error {h.query('ge').strip()})")
            thf = clk.at(now - kayei.FRAME_DELAY)
            if m:
                out.append((s, thf % 360, m["angle"], m["x"], m["width"]))
            if thf - th0 >= 360 * n:
                break
    finally:
        h.stop(fa.ROT); time.sleep(0.3); h.set_velocity(fa.ROT, v0)
    return out


def main():
    rig = Rig()
    rows = []
    try:
        rig.scan.MOV("1", 25.0); pitools.waitontarget(rig.scan)
        m0, cal = fa.calibrate(rig)
        fa.align(rig, cal, log=lambda s: None)
        g_al, z_al = rig.gonio(), rig.zpos()
        print(f"aligned at scan 25 mm: gonio {g_al:+.4f} z {z_al:.4f}")
        for s in SCANS:
            rig.scan.MOV("1", s); pitools.waitontarget(rig.scan); time.sleep(0.3)
            r = turns(rig, s)
            rows += r
            print(f"scan {s:4.1f}: {len(r)} frames")
        rig.scan.MOV("1", 25.0); pitools.waitontarget(rig.scan)
    finally:
        rig.close()
    with open(os.path.join(HERE, "kayei_data", "wobble_map.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["scan_mm", "theta_deg", "angle_deg", "x_px", "width_px"]); w.writerows(rows)
    analyse(rows)


def analyse(rows):
    rows = np.array(rows, float)
    fits = {}
    for s in SCANS:
        sel = rows[rows[:, 0] == s]
        th, a, x = sel[:, 1], sel[:, 2], sel[:, 3]
        ca = fit_sine(th, a); cx = fit_sine(th, x)
        fits[s] = dict(th=th, a=a, x=x, ca=ca[0], cx=cx[0], ra=ca[1], rx=cx[1])
        print(f"scan {s:4.1f}: angle {ca[2]:.3f} deg @ {ca[3]:+6.1f} deg (resid {ca[1]:.4f}) | offset {cx[2]*UM:6.0f} um @ {cx[3]:+6.1f} deg (resid {cx[1]*UM:.0f} um)")
    S = np.array(SCANS)
    ex = np.array([fits[s]["cx"][1] for s in SCANS]) * UM / 1000    # mm, in-plane (camera sees it)
    ey = -np.array([fits[s]["cx"][2] for s in SCANS]) * UM / 1000   # mm, out-of-plane (towards camera)
    x0 = np.array([fits[s]["cx"][0] for s in SCANS]) * UM / 1000    # mm, mean offset (goniometer-corrected tilt residual)
    # straight-line fit of the axis: tilt and eccentricity at the chuck (s = 0)
    (tx, ex0), (ty, ey0) = np.polyfit(S, ex, 1), np.polyfit(S, ey, 1)
    print(f"fiber axis vs rotation axis: eccentricity at s=0 ({ex0*1000:+.0f}, {ey0*1000:+.0f}) um, tilt ({np.degrees(np.arctan(tx)):+.3f}, {np.degrees(np.arctan(ty)):+.3f}) deg "
          f"= {np.degrees(np.hypot(tx, ty)):.3f} deg total; straight-line residual {np.std(ex - (tx*S+ex0))*1000:.0f} / {np.std(ey - (ty*S+ey0))*1000:.0f} um (bend)")
    np.savetxt(os.path.join(HERE, "kayei_data", "wobble_map_axis.csv"), np.c_[S, ex, ey, x0], delimiter=",", header="scan_mm,ex_mm_inplane,ey_mm_outofplane,mean_offset_mm", comments="")

    # ---- 2D
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.5))
    cols = plt.cm.viridis(np.linspace(0, 1, len(SCANS)))
    for c, s in zip(cols, SCANS):
        f = fits[s]; o = np.argsort(f["th"])
        axs[0].plot(f["th"][o], f["a"][o], ".", ms=2, color=c, label=f"scan {s:.0f} mm")
        axs[1].plot(f["th"][o], f["x"][o] * UM, ".", ms=2, color=c, label=f"scan {s:.0f} mm")
    axs[0].set_ylabel("fiber angle in image [deg]"); axs[1].set_ylabel("fiber centre offset [um]")
    for ax in axs:
        ax.set_xlabel("rotation angle [deg]"); ax.grid(alpha=0.3); ax.legend(fontsize=8, markerscale=5)
    axs[0].set_title("No control: what the camera sees over one turn"); axs[1].set_title("... at five positions along the fiber")
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "docs", "figures", "wobble_2d.png"), dpi=120)

    # ---- 3D
    TH = np.linspace(0, 360, 73)
    fig = plt.figure(figsize=(16, 5))
    for i, (key, label, scale) in enumerate((("cx", "offset [um]", UM), ("ca", "angle [deg]", 1.0))):
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        TT, SS = np.meshgrid(TH, S)
        Z = np.array([[(fits[s][key][0] + fits[s][key][1] * np.cos(np.radians(t)) + fits[s][key][2] * np.sin(np.radians(t))) * scale for t in TH] for s in SCANS])
        ax.plot_surface(TT, SS, Z, cmap="coolwarm", alpha=0.85, linewidth=0)
        for s in SCANS:
            f = fits[s]; ax.scatter(f["th"], np.full_like(f["th"], s), (f["a"] if key == "ca" else f["x"] * UM), s=1, c="k", alpha=0.3)
        ax.set_xlabel("rotation angle [deg]"); ax.set_ylabel("scan position [mm]"); ax.set_zlabel(label); ax.set_title(f"{label.split(' [')[0]} (theta, s)")
    ax = fig.add_subplot(1, 3, 3, projection="3d")
    ax.plot([0, 0], [0, 0], [0, 50], "k--", lw=1, label="rotation axis")
    ax.plot(ex * 1000, ey * 1000, S, "o-", color="C3", lw=2, label="fiber axis (measured)")
    sf = np.linspace(0, 50, 2); ax.plot((tx * sf + ex0) * 1000, (ty * sf + ey0) * 1000, sf, ":", color="gray", label="straight-line fit")
    ax.set_xlabel("in-plane [um]"); ax.set_ylabel("towards camera [um]"); ax.set_zlabel("along fiber [mm]"); ax.legend(fontsize=8)
    ax.set_title("Fiber centre line vs rotation axis (rotating frame)")
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "docs", "figures", "wobble_3d.png"), dpi=120)

    # ---- 4D: the fiber axis rotating, seen from the camera side and in 3D
    out = os.path.join(HERE, "docs", "figures", "wobble_4d.mp4")
    W, H, FPS, N = 1280, 540, 25, 150
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
    lim = max(np.abs(np.r_[ex, ey]).max() * 1000 * 1.3, 50)
    for k in range(N):
        th = 360 * k / N; c, sn = np.cos(np.radians(th)), np.sin(np.radians(th))
        X = (ex * c - ey * sn) * 1000; Y = (ex * sn + ey * c) * 1000     # lab frame, um
        fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
        ax = fig.add_subplot(1, 2, 1, projection="3d")
        ax.plot([0, 0], [0, 0], [0, 50], "k--", lw=1)
        ax.plot(X, Y, S, "o-", color="C3", lw=2)
        for s0 in np.linspace(0, 50, 6):   # swept surface (cone of positions over the turn)
            r = np.hypot(np.interp(s0, S, ex), np.interp(s0, S, ey)) * 1000; ph = np.linspace(0, 2 * np.pi, 60)
            ax.plot(r * np.cos(ph), r * np.sin(ph), s0, color="gray", lw=0.5, alpha=0.5)
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(0, 50)
        ax.set_xlabel("camera x [um]"); ax.set_ylabel("towards camera [um]"); ax.set_zlabel("along fiber [mm]")
        ax.set_title(f"fiber axis, rotation {th:5.1f} deg")
        ax2 = fig.add_subplot(1, 2, 2)
        ax2.plot(S, X, "o-", color="C3", label="what the camera sees (in-plane offset)")
        ax2.plot(S, Y, "s--", color="C0", label="towards the camera (focus)")
        ax2.set_ylim(-lim, lim); ax2.set_xlabel("along fiber [mm]"); ax2.set_ylabel("offset from rotation axis [um]"); ax2.grid(alpha=0.3); ax2.legend(fontsize=8, loc="upper right")
        ax2.set_title("lateral position of the fiber axis along its length")
        fig.tight_layout(); fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        vw.write(cv2.cvtColor(cv2.resize(img, (W, H)), cv2.COLOR_RGB2BGR)); plt.close(fig)
    vw.release()
    print("wrote wobble_2d.png, wobble_3d.png, wobble_4d.mp4")


if __name__ == "__main__":
    if os.environ.get("REPLOT"):
        analyse([tuple(map(float, r)) for r in csv.reader(open(os.path.join(HERE, "kayei_data", "wobble_map.csv"))) if r[0] != "scan_mm"])
    else:
        main()

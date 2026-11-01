"""Align the fiber at several rotation angles: goniometer makes the fiber straight in the image
(angle 0), camera Z stage centres it. Camera is mounted rotated 90 deg, so the fiber appears
VERTICAL in the image; "angle" = deviation of the fiber axis from the image vertical.
Run:  python fiber_align.py --calib        # measure + calibrate gonio/Z -> image, no rotation
      python fiber_align.py [n_angles]     # align at n_angles rotations over 360 deg (default 6)
Writes fiber_align.csv and fiber_align_<angle>.png per rotation."""
import csv
import sys
import time

import cv2
import neoapi
import numpy as np
from pipython import GCSDevice, pitools

from hydra import Hydra

HYDRA_IP, CAM_IP, Z_SN = "172.20.5.201", "172.20.5.20", "0165500187"
GONIO, ROT = 1, 2
TOL_ANGLE = 0.02   # deg, fiber angle in the image
TOL_PX = 1.0       # px, fiber centre vs image centre
MAX_ITER = 8
BAND = 30          # rows per band for edge finding
ROBUST = True      # sigma-clip the bands (False = plain least squares over all bands)
POLARITY = "auto"  # "bright": fiber brighter than background (glowing / bright rims), "dark": silhouette
                   # (opaque fiber in front of a backlight), "auto": per band, whichever contrast is larger
EXPOSURE_MS = 4.0  # 2026-10-31 evening illumination: the fiber glows (bright on grey), saturates above ~4 ms;
                   # with the earlier dark-fiber-on-bright-background light 20 ms was needed (dim rims)


def measure(frame):
    """Fit both fiber edges over the image height. Returns dict with angle (deg, + = bottom of the
    fiber further right), centre x at the middle row, width (px), contrast, or None if no fiber."""
    h, w = frame.shape
    blur = cv2.GaussianBlur(frame, (0, 0), 1.5).astype(np.float32)
    ys, left, right = [], [], []
    for y0 in range(0, h - BAND + 1, BAND):
        col = blur[y0:y0 + BAND].mean(axis=0)
        bg = np.median(col)
        if POLARITY == "dark" or (POLARITY == "auto" and bg - col.min() > col.max() - bg):
            col = 2 * bg - col          # silhouette: mirror the profile so the fiber is "bright"
        pk = col.max()
        if pk - bg < 40:
            continue
        # outermost columns clearly above background = the fiber's outer edges; the fiber is a
        # transparent cylinder (bright rims, dim core, glints), so never threshold at the peak
        above = np.flatnonzero(col > bg + 0.3 * (pk - bg))
        l, r = int(above[0]), int(above[-1])
        if l < 2 or r > w - 3:
            continue
        # each edge at half of ITS OWN height (local rim max vs background), sub-pixel
        t = bg + 0.5 * (col[l:l + 8].max() - bg)
        while l < r and col[l] < t:
            l += 1
        while l > 1 and col[l - 1] >= t:
            l -= 1
        t2 = bg + 0.5 * (col[r - 7:r + 1].max() - bg)
        while r > l and col[r] < t2:
            r -= 1
        while r < w - 2 and col[r + 1] >= t2:
            r += 1
        if l <= 1 or r >= w - 2 or r - l < 3:  # touching the image border / not a fiber
            continue
        xl = l - 1 + (t - col[l - 1]) / (col[l] - col[l - 1])
        xr = r + (col[r] - t2) / (col[r] - col[r + 1])
        ys.append(y0 + BAND / 2); left.append(xl); right.append(xr)
    if len(ys) < 8:
        return None
    ys, left, right = map(np.array, (ys, left, right))
    mid = (left + right) / 2
    # robust straight line: bands whose centre or width disagree with the rest (an object
    # crossing the fiber, a glint merging with an edge) are dropped by sigma clipping
    keep = np.ones(len(ys), bool)
    wmed = np.median(right - left)
    if ROBUST:
        keep &= np.abs((right - left) - wmed) < max(0.25 * wmed, 3.0)
    for _ in range(4 if ROBUST else 0):
        if keep.sum() < 8:
            return None
        slope, icpt = np.polyfit(ys[keep], mid[keep], 1)
        r = mid - (slope * ys + icpt)
        s_r = max(r[keep].std(), 0.3)
        new_keep = keep & (np.abs(r) < 3.5 * s_r)
        if new_keep.sum() == keep.sum():
            break
        keep = new_keep
    ys, left, right, mid = ys[keep], left[keep], right[keep], mid[keep]
    slope, icpt = np.polyfit(ys, mid, 1)
    resid = mid - (slope * ys + icpt)
    return {"angle": np.degrees(np.arctan(slope)), "x": slope * h / 2 + icpt - w / 2,
            "width": float(np.median(right - left)), "rms": float(resid.std()), "n": len(ys),
            "slope": slope, "icpt": icpt}  # centre line: x = slope * y + icpt (for overlays)


class Rig:
    def __init__(self):
        self.cam = neoapi.Cam()
        self.cam.Connect(CAM_IP)
        self.cam.GetFeature("ExposureAuto").SetString("Off")
        self.h = Hydra(HYDRA_IP)
        self.z = GCSDevice()
        self.z.ConnectUSB(Z_SN)
        if not self.z.qFRF()["1"]:
            pitools.startup(self.z, refmodes="FNL")

    def frame(self):
        img = None
        for _ in range(3):  # drop frames exposed while moving
            img = self.cam.GetImage(2000)
        return img.GetNPArray().squeeze().copy()

    def look(self):
        f = self.frame()
        return measure(f), f

    def hydra_to(self, ax, pos):
        self.h.move_abs(ax, pos); time.sleep(0.05); self.h.wait(ax); time.sleep(0.2)

    def z_to(self, pos):
        self.z.MOV("1", pos); pitools.waitontarget(self.z); time.sleep(0.2)

    def gonio(self):
        return self.h.position(GONIO)

    def zpos(self):
        return self.z.qPOS()["1"]

    def close(self):
        self.z.CloseConnection(); self.h.close()


def calibrate(rig, dg=1.0, dz=0.5):
    """Image response to the goniometer and Z (finite differences around the current position)."""
    g0, z0 = rig.gonio(), rig.zpos()
    m0, _ = rig.look()
    rig.hydra_to(GONIO, g0 + dg); mg, _ = rig.look()
    rig.hydra_to(GONIO, g0)
    rig.z_to(z0 + dz); mz, _ = rig.look()
    rig.z_to(z0)
    cal = {"a_per_g": (mg["angle"] - m0["angle"]) / dg, "x_per_g": (mg["x"] - m0["x"]) / dg,
           "a_per_z": (mz["angle"] - m0["angle"]) / dz, "x_per_z": (mz["x"] - m0["x"]) / dz}
    return m0, cal


def align(rig, cal, log=print):
    """Iterate: goniometer removes the angle, then Z removes the offset (incl. the shift the tilt caused)."""
    m = None
    for it in range(MAX_ITER):
        m, f = rig.look()
        if m is None:
            raise RuntimeError("fiber not found in image")
        log(f"   it{it}: angle {m['angle']:+8.4f} deg  x {m['x']:+8.2f} px  width {m['width']:5.1f} px  "
            f"gonio {rig.gonio():+8.4f}  z {rig.zpos():8.4f}")
        if abs(m["angle"]) < TOL_ANGLE and abs(m["x"]) < TOL_PX:
            return m, f, it
        if abs(m["angle"]) >= TOL_ANGLE:
            g = rig.gonio() - m["angle"] / cal["a_per_g"]
            if abs(g) > 5.1:
                raise RuntimeError(f"goniometer would need {g:+.3f} deg (range +/-5.16)")
            rig.hydra_to(GONIO, g)
            m, f = rig.look()
        z = rig.zpos() - m["x"] / cal["x_per_z"]
        if not 0.2 < z < 24.8:
            raise RuntimeError(f"Z would need {z:.3f} mm (range 0..25)")
        rig.z_to(z)
    m, f = rig.look()
    log(f"   not converged after {MAX_ITER} iterations: angle {m['angle']:+.4f} deg, x {m['x']:+.2f} px")
    return m, f, MAX_ITER


def main():
    rig = Rig()
    try:
        m0, cal = calibrate(rig)
        print(f"now: angle {m0['angle']:+.4f} deg, x {m0['x']:+.2f} px, width {m0['width']:.1f} px, "
              f"edge-fit rms {m0['rms']:.2f} px over {m0['n']} bands  (gonio {rig.gonio():+.4f}, z {rig.zpos():.4f})")
        print("calibration: " + ", ".join(f"{k} {v:+.4f}" for k, v in cal.items()))
        if "--calib" in sys.argv:
            return
        n = int(sys.argv[1]) if len(sys.argv) > 1 else 6
        rows = []
        for k in range(n):
            a = 360.0 * k / n
            rig.hydra_to(ROT, a)
            print(f"rotation {a:6.1f} deg")
            t = time.time()
            m, f, its = align(rig, cal)
            rows.append((a, rig.gonio(), rig.zpos(), m["angle"], m["x"], m["width"], its, time.time() - t))
            view = cv2.cvtColor(f, cv2.COLOR_GRAY2BGR)
            cv2.line(view, (f.shape[1] // 2, 0), (f.shape[1] // 2, f.shape[0]), (0, 0, 255), 1)
            cv2.imwrite(f"fiber_align_{int(a):03d}.png", view)
        rig.hydra_to(ROT, 0.0)
        with open("fiber_align.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["rot_deg", "gonio_deg", "z_mm", "angle_deg", "x_px", "width_px", "iterations", "time_s"])
            w.writerows(rows)
        print("\n rot    gonio      z        angle     x px  width  it  time")
        for r in rows:
            print(f"{r[0]:5.0f}  {r[1]:+8.4f}  {r[2]:8.4f}  {r[3]:+8.4f}  {r[4]:+6.2f}  {r[5]:5.1f}  {r[6]:2d}  {r[7]:4.1f}s")
    finally:
        rig.close()


if __name__ == "__main__":
    main()

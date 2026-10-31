"""Fiber diameter from the image, with the Z stage as the length standard.
Z is stepped over +/-2 mm; the fiber centre shift in px vs Z gives px/mm (linear fit with
uncertainty); the fiber width (outermost edges, fiber_align.measure) is converted with it.
Repeated at three scan positions and two exposures. Writes kayei_data/fiber_diameter.csv
and docs/figures/fiber_diameter.png.  Run: python fiber_diameter.py"""
import csv
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pipython import GCSDevice, pitools

import fiber_align as fa
from kayei_learn import Rig, SCAN_SN

HERE = os.path.dirname(os.path.abspath(__file__))
Z_STEPS = np.arange(-2.0, 2.01, 0.25)
SCANS = (5.0, 25.0, 45.0)
EXPOSURES = (20.0, 12.0)


def main():
    rig = Rig()
    z0 = rig.zpos()
    rows, results = [], []
    try:
        for s in SCANS:
            rig.scan.MOV("1", s); pitools.waitontarget(rig.scan); time.sleep(0.3)
            # re-centre the fiber at this scan position (the fiber is bent)
            _, m = rig.fresh()
            rig.z_to(rig.zpos() - m["x"] / -49.5)
            zc = rig.zpos()
            for exp in EXPOSURES:
                rig.cam.f.ExposureTime.Set(exp * 1000); time.sleep(0.2)
                zs, xs, ws = [], [], []
                for dz in Z_STEPS:
                    rig.z_to(zc + dz)
                    for _ in range(5):
                        _, m = rig.fresh()
                        if m:
                            zs.append(zc + dz); xs.append(m["x"]); ws.append(m["width"])
                            rows.append((s, exp, zc + dz, m["x"], m["width"], m["angle"], m["rms"]))
                zs, xs, ws = map(np.array, (zs, xs, ws))
                if len(zs) < 10:
                    print(f"scan {s:4.1f} mm, {exp:3.0f} ms: fiber not detected ({len(zs)} frames) - skipped")
                    continue
                (k, b), cov = np.polyfit(zs, xs, 1, cov=True)
                k_err = np.sqrt(cov[0, 0])
                resid = xs - (k * zs + b)
                w_px, w_sd = ws.mean(), ws.std()
                d_mm = w_px / abs(k)
                d_err = d_mm * np.hypot(w_sd / w_px, k_err / abs(k))
                results.append(dict(scan=s, exp=exp, k=k, k_err=k_err, resid=resid.std(), w_px=w_px, w_sd=w_sd, d=d_mm, d_err=d_err, n=len(zs)))
                print(f"scan {s:4.1f} mm, {exp:3.0f} ms: {abs(k):.3f} +/- {k_err:.3f} px/mm (fit resid {resid.std():.2f} px), "
                      f"width {w_px:.2f} +/- {w_sd:.2f} px -> diameter {d_mm:.4f} +/- {d_err:.4f} mm  ({len(zs)} frames)")
                rig.z_to(zc)
        rig.cam.f.ExposureTime.Set(20000)
        rig.scan.MOV("1", 25.0); pitools.waitontarget(rig.scan)
    finally:
        rig.z_to(z0)
        rig.close()

    os.makedirs(os.path.join(HERE, "kayei_data"), exist_ok=True)
    with open(os.path.join(HERE, "kayei_data", "fiber_diameter.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["scan_mm", "exposure_ms", "z_mm", "x_px", "width_px", "angle_deg", "fit_rms_px"]); w.writerows(rows)
    ks = np.array([r["k"] for r in results]); ds = np.array([r["d"] for r in results])
    print(f"\nimage scale: {abs(ks).mean():.3f} px/mm (spread {abs(ks).std():.3f}) = {1000 / abs(ks).mean():.2f} um/px")
    print(f"fiber diameter: {ds.mean():.4f} mm (spread over scan positions / exposures {ds.std():.4f} mm); caliper 1.1 mm")

    fig, axs = plt.subplots(1, 3, figsize=(14, 4.2))
    for r in results:
        sel = [row for row in rows if row[0] == r["scan"] and row[1] == r["exp"]]
        axs[0].plot([q[2] for q in sel], [q[3] for q in sel], ".", ms=3, label=f"scan {r['scan']:.0f}, {r['exp']:.0f} ms")
    axs[0].set_xlabel("Z [mm]"); axs[0].set_ylabel("fiber centre in image [px]"); axs[0].set_title("Z steps -> image scale"); axs[0].grid(alpha=0.3); axs[0].legend(fontsize=7)
    for exp, mk in zip(EXPOSURES, ("o", "s")):
        rr = [r for r in results if r["exp"] == exp]
        axs[1].errorbar([r["scan"] for r in rr], [r["d"] * 1000 for r in rr], yerr=[r["d_err"] * 1000 for r in rr], fmt=mk + "-", capsize=3, label=f"{exp:.0f} ms exposure")
    axs[1].axhline(1100, color="gray", ls="--", label="caliper 1.1 mm"); axs[1].set_xlabel("scan position [mm]"); axs[1].set_ylabel("diameter [um]"); axs[1].set_title("Fiber diameter along the fiber"); axs[1].grid(alpha=0.3); axs[1].legend(fontsize=8)
    axs[2].bar([f"{r['scan']:.0f}/{r['exp']:.0f}" for r in results], [abs(r["k"]) for r in results], yerr=[r["k_err"] for r in results], capsize=3, color="#2a78d6")
    axs[2].set_ylabel("px per mm"); axs[2].set_xlabel("scan mm / exposure ms"); axs[2].set_title("Image scale per measurement"); axs[2].grid(alpha=0.3, axis="y")
    axs[2].set_ylim(abs(ks).min() - 1, abs(ks).max() + 1)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "docs", "figures", "fiber_diameter.png"), dpi=120)


if __name__ == "__main__":
    main()

"""Scan the camera Z stage (PI 187, VT-80 25 mm) over its range, grab a frame at each step and
score how strongly a horizontal edge (the fiber) shows up. Saves find_fiber_sheet.png (contact
sheet of all steps) and find_fiber.csv; prints the best Z. Camera must be at 172.20.5.20.
Run:  python find_fiber.py [z_from z_to step]   (default 1 24 1)"""
import csv
import sys
import time

import cv2
import neoapi
import numpy as np
from pipython import GCSDevice, pitools

CAM_IP = "172.20.5.20"
Z_SN = "0165500187"


def grab(cam, n=3):
    img = None
    for _ in range(n):  # drop frames taken while moving / exposure settling
        img = cam.GetImage(2000)
    return img.GetNPArray().squeeze().copy()


def edge_score(frame):
    """Strength of horizontal edges: gradient of the row-mean profile (fiber runs horizontally)."""
    rows = cv2.GaussianBlur(frame, (0, 0), 2).mean(axis=1)
    g = np.abs(np.diff(rows))
    return g.max(), int(g.argmax()), cv2.Laplacian(frame, cv2.CV_64F).var()


def main():
    z_from, z_to, step = map(float, sys.argv[1:4]) if len(sys.argv) >= 4 else (1.0, 24.0, 1.0)
    cam = neoapi.Cam()
    cam.Connect(CAM_IP)
    cam.GetFeature("ExposureAuto").SetString("Continuous")
    results, thumbs = [], []
    with GCSDevice() as pi:
        pi.ConnectUSB(Z_SN)
        if not pi.qFRF()["1"]:
            pitools.startup(pi, refmodes="FNL")
        for z in np.arange(z_from, z_to + 1e-9, step):
            pi.MOV("1", float(z)); pitools.waitontarget(pi)
            time.sleep(0.3)
            frame = grab(cam)
            edge, row, sharp = edge_score(frame)
            results.append((float(z), edge, row, sharp, frame.mean()))
            print(f"z={z:6.2f}  edge={edge:6.2f} at row {row:4d}  sharp={sharp:8.1f}  mean={frame.mean():5.1f}")
            t = cv2.cvtColor(cv2.resize(frame, (360, 270)), cv2.COLOR_GRAY2BGR)
            cv2.putText(t, f"z={z:.2f} e={edge:.1f}", (5, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            thumbs.append(t)
        best = max(results, key=lambda r: r[1])
        print(f"best edge at z={best[0]:.2f} (row {best[2]}) - stage left there")
        pi.MOV("1", best[0]); pitools.waitontarget(pi)
    cam.GetFeature("ExposureAuto").SetString("Off")

    with open("find_fiber.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["z_mm", "edge", "edge_row", "sharpness", "mean"])
        w.writerows(results)
    cols = 6
    thumbs += [np.zeros_like(thumbs[0])] * (-len(thumbs) % cols)
    sheet = np.vstack([np.hstack(thumbs[i:i + cols]) for i in range(0, len(thumbs), cols)])
    cv2.imwrite("find_fiber_sheet.png", sheet)


if __name__ == "__main__":
    main()

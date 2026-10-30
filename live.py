"""Live view of the Baumer camera.
Keys:  a = toggle auto exposure   + / - = exposure x1.5 / /1.5 (manual)
       z = zoom centre (x1/x2/x4)  r = reset focus peak
       s = save snapshot           q or Esc = quit
Focus: sharpness = variance of the Laplacian in the centre box (higher = sharper);
       turn the focus until it reaches the peak, the bar shows current vs. peak.
Run:   C:\\dev\\hydra\\.venv\\Scripts\\python C:\\dev\\hydra\\live.py
"""
import time

import cv2
import neoapi

IP = "172.20.5.20"
WIN = "Baumer VCXG-15M  (a=auto exp, +/- exposure, z=zoom, r=reset peak, s=snapshot, q=quit)"
ZOOMS = (1, 2, 4)


def sharpness(gray):
    return cv2.Laplacian(gray, cv2.CV_64F).var()

cam = neoapi.Cam()
cam.Connect(IP)
cam.f.AcquisitionFrameRateEnable.Set(True)
cam.f.AcquisitionFrameRate.Set(cam.f.AcquisitionFrameRate.GetMax())
auto = True
cam.GetFeature("ExposureAuto").SetString("Continuous")

cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, 960, 720)
t_last, fps = time.time(), 0.0
zoom, peak, sharp_avg = 0, 0.0, 0.0
while True:
    img = cam.GetImage(2000)
    if img.IsEmpty():
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
        continue
    frame = img.GetNPArray().squeeze()
    now = time.time()
    fps = 0.9 * fps + 0.1 / max(now - t_last, 1e-6)
    t_last = now

    exp_ms = cam.f.ExposureTime.Get() / 1000
    h, w = frame.shape
    z = ZOOMS[zoom]
    y0, x0 = h // 2 - h // (2 * z), w // 2 - w // (2 * z)
    crop = frame[y0:y0 + h // z, x0:x0 + w // z]
    # focus measured on the centre box (half the view size), smoothed against noise
    ch, cw = crop.shape
    box = (cw // 4, ch // 4, cw // 2, ch // 2)
    s = sharpness(crop[box[1]:box[1] + box[3], box[0]:box[0] + box[2]])
    sharp_avg = s if sharp_avg == 0 else 0.7 * sharp_avg + 0.3 * s
    peak = max(peak, sharp_avg)

    view = cv2.cvtColor(cv2.resize(crop, (w, h), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
    bx, by, bw, bh = (v * z for v in box)
    cv2.rectangle(view, (bx, by), (bx + bw, by + bh), (0, 200, 255), 2)
    lines = [f"{fps:4.1f} fps   exposure {exp_ms:7.1f} ms ({'auto' if auto else 'manual'})"
             f"   mean {frame.mean():5.1f}   max {frame.max()}   zoom x{z}",
             f"sharpness {sharp_avg:9.1f}   peak {peak:9.1f}   ({100 * sharp_avg / max(peak, 1e-9):5.1f} % of peak)"]
    for i, text in enumerate(lines):
        cv2.putText(view, text, (15, 40 + 45 * i), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5)
        cv2.putText(view, text, (15, 40 + 45 * i), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    bar = int((w - 30) * sharp_avg / max(peak, 1e-9))
    cv2.rectangle(view, (15, h - 40), (w - 15, h - 15), (80, 80, 80), 2)
    cv2.rectangle(view, (15, h - 40), (15 + bar, h - 15), (0, 255, 0), -1)
    cv2.imshow(WIN, view)

    key = cv2.waitKey(1) & 0xFF
    if key in (ord("q"), 27) or cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
        break
    if key == ord("a"):
        auto = not auto
        cam.GetFeature("ExposureAuto").SetString("Continuous" if auto else "Off")
    elif key in (ord("+"), ord("="), ord("-")) and not auto:
        f = 1.5 if key != ord("-") else 1 / 1.5
        e = cam.f.ExposureTime
        e.Set(min(max(e.Get() * f, e.GetMin()), e.GetMax()))
    elif key == ord("z"):
        zoom = (zoom + 1) % len(ZOOMS)
        peak = sharp_avg = 0.0  # different area -> different sharpness scale
    elif key == ord("r"):
        peak = sharp_avg = 0.0
    elif key == ord("s"):
        name = time.strftime("snapshot_%Y%m%d_%H%M%S.png")
        cv2.imwrite(name, frame)
        print("saved", name)

cam.GetFeature("ExposureAuto").SetString("Off")
cv2.destroyAllWindows()

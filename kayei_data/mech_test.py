import sys, time; sys.path.insert(0, r"C:\dev\hydra")
import numpy as np, fiber_align as fa
from kayei_learn import Rig
rig = Rig(); h = rig.h
A_G = -1.015
def fitlag(t, y, amp, period, t_off=0.0):
    best = min(((np.sqrt(np.mean((y - amp * np.sin(2 * np.pi * (t - lag) / period)) ** 2)), lag) for lag in np.arange(-0.05, 0.4, 0.002)))
    return best[1], best[0]
try:
    g0 = rig.gonio(); amp = 0.44
    _, m0 = rig.fresh(); a0 = m0["angle"]
    print("A: camera-measured fiber angle while the goniometer runs a sine (rotation stopped)")
    for period in (7.2, 3.6, 1.8, 0.9):
        cam, t0, last = [], time.time(), -1
        while time.time() - t0 < 2 * period + 0.5:
            t = time.time() - t0
            if t - last >= 0.02:
                h.move_abs(1, g0 + amp * np.sin(2 * np.pi * t / period)); last = t
            tf, m = rig.fresh()
            if m: cam.append((time.time() - t0, (m["angle"] - a0) / A_G))   # fiber angle expressed in gonio deg
        t = np.array([c[0] for c in cam]); y = np.array([c[1] for c in cam]); sel = t > period / 2
        lag, rms = fitlag(t[sel], y[sel], amp, period)
        print(f"  period {period:3.1f} s (peak {amp*2*np.pi/period:.2f} deg/s): fiber amplitude {(y[sel].max()-y[sel].min())/2:.3f} of {amp}, lag {lag*1000:4.0f} ms, rms vs shifted sine {rms:.4f} deg")
        h.move_abs(1, g0); h.wait(1); time.sleep(0.5)
    print("B: Hydra goniometer readout under the sine stream while the rotation spins at 200 deg/s")
    v0 = h.velocity(2); th0 = h.position(2)
    for spin in (0.0, 200.0):
        if spin:
            h.set_velocity(2, spin); h.move_abs(2, th0 + 2000)
            time.sleep(0.3)
        period = 1.8; reads, t0, last = [], time.time(), -1
        while time.time() - t0 < 2 * period:
            t = time.time() - t0
            if t - last >= 0.02:
                h.move_abs(1, g0 + amp * np.sin(2 * np.pi * t / period)); last = t
            reads.append((time.time() - t0, h.position(1) - g0))
        if spin:
            h.stop(2); time.sleep(0.3); h.set_velocity(2, v0)
        t = np.array([r[0] for r in reads]); y = np.array([r[1] for r in reads]); sel = t > period / 2
        lag, rms = fitlag(t[sel], y[sel], amp, period)
        print(f"  spin {spin:3.0f}: gonio readout amplitude {(y[sel].max()-y[sel].min())/2:.3f}, lag {lag*1000:4.0f} ms, rms {rms:.4f} deg, {len(reads)/(2*period):.0f} reads/s")
        h.move_abs(1, g0); h.wait(1)
    rig.hydra_to(2, h.position(2) + (-h.position(2)) % 360)
finally:
    rig.close()

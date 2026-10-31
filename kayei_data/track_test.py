"""Actuator tracking test without the rotation: stream a sine at 50 Hz to the goniometer
(encoder read on a second socket) and to Z (camera x as sensor); report amplitude ratio and lag."""
import sys, time, threading
sys.path.insert(0, r"C:\dev\hydra")
import numpy as np
import fiber_align as fa, kayei
from kayei_learn import Rig
from hydra import Hydra
from kayei_learn import fit_sine

def stream(fn_cmd, A, f, secs, rate=50.0):
    """send fn_cmd(A*sin(2 pi f t)) at rate; returns list of (t, cmd)"""
    out, dt, t0 = [], 1.0 / rate, time.perf_counter(); tn = t0; worst = 0
    while True:
        now = time.perf_counter()
        if now - t0 > secs: break
        u = A * np.sin(2 * np.pi * f * (now - t0)); fn_cmd(u); out.append((now, u))
        tn += dt; lag = tn - now
        if lag > 0: time.sleep(lag)
        else: worst = max(worst, -lag); tn = now
    fn_cmd(0.0)
    return out, worst

def fit(t, y, f):
    """y = c + a sin(wt) + b cos(wt) -> amplitude, phase lag (ms, positive = late)"""
    w = 2 * np.pi * f; A = np.c_[np.ones_like(t), np.sin(w * t), np.cos(w * t)]
    c = np.linalg.lstsq(A, y, rcond=None)[0]
    amp = np.hypot(c[1], c[2]); ph = np.arctan2(c[2], c[1])   # y ~ amp sin(wt + ph)
    return amp, -ph / w * 1000, (y - A @ c).std()

rig = Rig(); h = rig.h; hq = Hydra(fa.HYDRA_IP)
try:
    print("gonio accel", h.query("1 getnaccel"), "vel", h.velocity(1), "| Z VEL", rig.z.qVEL("1")["1"], "ACC", rig.z.qACC("1")["1"], "DEC", rig.z.qDEC("1")["1"])
    g0 = h.position(1); z0 = rig.zpos()
    print(f"gonio {g0:+.4f} z {z0:.4f}")
    # ---- goniometer, encoder as sensor (second socket polled in a thread)
    for A, f in ((0.1, 1.0), (1.26, 0.278), (1.26, 0.556), (1.26, 1.0)):
        log, stop = [], threading.Event()
        def poll():
            while not stop.is_set():
                t1 = time.perf_counter(); p = hq.position(1); log.append(((t1 + time.perf_counter()) / 2, p))
        th = threading.Thread(target=poll, daemon=True); th.start()
        cmds, worst = stream(lambda u: h.move_abs(1, g0 + u), A, f, max(6.0, 3 / f))
        time.sleep(0.5); stop.set(); th.join()
        tc, uc = np.array(cmds).T; tl, pl = np.array(log).T
        t0 = tc[0]; sel = tl - t0 > 1 / f   # skip the first period
        amp, lag, rest = fit(tl[sel] - t0, pl[sel] - g0, f)
        ac, lc, _ = fit(tc - t0, uc, f)
        print(f"gonio A {A:.2f} deg f {f:.3f} Hz (peak rate {A*2*np.pi*f:.2f} deg/s): encoder amp {amp:.4f} ({amp/A*100:.1f}%), lag {lag - lc:+.0f} ms, non-sine {rest:.4f} deg; {len(pl)} enc samples, worst tick slip {worst*1000:.0f} ms")
    h.move_abs(1, g0); time.sleep(1)
    # ---- Z, camera as sensor (x in px -> mm with 20.46 um/px); sign from a static step
    rig.z_to(z0 + 0.2); time.sleep(0.5); _, m1 = rig.fresh(); rig.z_to(z0); time.sleep(0.5); _, m0 = rig.fresh()
    kx = (m1["x"] - m0["x"]) / 0.2   # px per mm
    print(f"camera: {kx:+.1f} px/mm of Z")
    for A, f in ((0.1, 1.0), (1.2, 0.278), (1.2, 0.556), (1.2, 1.0)):
        log, stop = [], threading.Event()
        def poll():
            while not stop.is_set():
                t, m = rig.fresh()
                if m: log.append((t, m["x"]))
        th = threading.Thread(target=poll, daemon=True); th.start()
        cmds, worst = stream(lambda u: rig.z.MOV("1", z0 + u), A, f, max(6.0, 3 / f))
        time.sleep(0.5); stop.set(); th.join()
        tc, uc = np.array(cmds).T; tl, xl = np.array(log).T
        t0 = tc[0]; sel = tl - t0 > 1 / f
        amp, lag, rest = fit(tl[sel] - t0, (xl[sel] - m0["x"]) / kx, f)
        ac, lc, _ = fit(tc - t0, uc, f)
        print(f"Z     A {A:.2f} mm  f {f:.3f} Hz (peak rate {A*2*np.pi*f:.2f} mm/s): camera amp {amp:.4f} ({amp/A*100:.1f}%), lag {lag - lc:+.0f} ms (incl. camera), non-sine {rest*1000:.0f} um; {len(xl)} frames, worst tick slip {worst*1000:.0f} ms")
    rig.z_to(z0)
finally:
    hq.close(); rig.close()

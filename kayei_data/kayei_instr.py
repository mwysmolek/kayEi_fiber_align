"""instrumented kayEi gonio loop: per frame log t, theta_est, g_cmd, angle; every 4th frame also the Hydra gonio readout"""
import sys, time, csv
sys.path.insert(0, r"C:\dev\hydra")
import numpy as np
import fiber_align as fa, kayei
from kayei_learn import Rig
A_PER_G = -1.015
rig = Rig(); h = rig.h
try:
    for _ in range(4):
        _, m = rig.fresh()
        if abs(m["angle"]) < 0.01: break
        rig.hydra_to(fa.GONIO, rig.gonio() - m["angle"] / A_PER_G)
    g_al = rig.gonio()
    rig.hydra_to(fa.ROT, h.position(fa.ROT) + (-h.position(fa.ROT)) % 360)   # start at a multiple of 360

    def turns(speed, n, control=None, tag=""):
        clk = kayei.RotClock(speed); th0 = h.position(fa.ROT); clk.sync(th0)
        v0 = h.velocity(fa.ROT); h.set_velocity(fa.ROT, speed); h.move_abs(fa.ROT, th0 + 360 * n + 30)
        out, t_sync, k = [], time.time(), 0
        try:
            while True:
                t, m = rig.fresh(); now = time.time(); k += 1
                if now - t_sync > 0.25:
                    tq = time.time(); thq = h.position(fa.ROT); clk.sync(thq, (tq + time.time()) / 2); t_sync = time.time()
                th_frame = clk.at(now - kayei.FRAME_DELAY) - th0
                g_cmd = control(clk.at(now) - th0, now) if control else None
                g_rd = h.position(fa.GONIO) if k % 4 == 0 else None
                if m: out.append((now, th_frame, m["angle"], g_cmd, g_rd))
                if th_frame >= 360 * n: break
        finally:
            h.stop(fa.ROT); time.sleep(0.3); h.set_velocity(fa.ROT, v0)
        rig.hydra_to(fa.ROT, th0 + 360 * (n + 1))
        return out

    data = turns(90.0, 2)
    th = np.array([d[1] for d in data]); a = np.array([d[2] for d in data])
    model = kayei.WobbleModel().fit(th, np.ones_like(th), a, np.zeros_like(th))
    print("learned:", model.describe())
    rows = []
    for speed, lead in ((200.0, 0.0), (200.0, 0.03), (200.0, 0.06), (100.0, 0.03)):
        state = {"g": g_al}
        def control(th_now, now, lead=lead, speed=speed):
            g = g_al - model.predict(th_now + speed * lead, 1.0)[0] / A_PER_G
            if abs(g - state["g"]) > 0.0005:
                h.move_abs(fa.GONIO, g); state["g"] = g
            return state["g"]
        data = turns(speed, 3, control)
        t = np.array([d[0] for d in data]); th = np.array([d[1] for d in data]); a = np.array([d[2] for d in data])
        gc = np.array([d[3] for d in data]); ev = th >= 360
        print(f"speed {speed:.0f} lead {lead:.2f}: camera angle error rms {a[ev].std():.4f} max {np.abs(a[ev]).max():.4f} deg")
        # where is the lag? compare the Hydra gonio readout with the command stream, and the fiber with the readout
        rd = [(d[0], d[4]) for d in data if d[4] is not None and d[1] >= 360]
        tr = np.array([r[0] for r in rd]); gr = np.array([r[1] for r in rd])
        def lag_of(ts, ys, ref_t, ref_y):   # lag that best aligns ys(ts) with ref_y(ref_t)
            best = min(((np.sqrt(np.mean((ys - np.interp(ts - lag, ref_t, ref_y)) ** 2)), lag) for lag in np.arange(-0.1, 0.4, 0.004)))
            return best
        r1 = lag_of(tr, gr, t, gc)
        fiber_g = g_al - a / A_PER_G + (gc - g_al)   # gonio angle implied by the camera: command + residual error... (fiber tilt = a_g*(g - g_al) + wob) -> not separable; use ideal instead
        ideal_t = t; ideal_g = g_al - model.predict(th, 1.0)[0] / A_PER_G   # gonio that would make the camera see 0 at the frame's angle
        r2 = lag_of(tr, gr, ideal_t, ideal_g)
        r3 = lag_of(t[ev], gc[ev], ideal_t, ideal_g)
        print(f"   Hydra gonio readout vs command stream: lag {r1[1]*1000:+4.0f} ms (rms {r1[0]:.4f}) | readout vs ideal-for-frame: {r2[1]*1000:+4.0f} ms | command vs ideal: {r3[1]*1000:+4.0f} ms")
        rows += [(speed, lead, *d) for d in data]
        rig.hydra_to(fa.GONIO, g_al)
    with open(r"C:\dev\hydra\kayei_data\gonio_instr.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["speed", "lead", "t", "theta", "angle", "g_cmd", "g_read"]); w.writerows(rows)
finally:
    rig.close()

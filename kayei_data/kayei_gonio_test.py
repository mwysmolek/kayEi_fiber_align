"""kayEi on the goniometer axis only (no PI stages): learn 2 turns, then predict at several
rotation speeds and lead times; report the angle error the camera still sees."""
import sys, time, csv
sys.path.insert(0, r"C:\dev\hydra")
import numpy as np
import fiber_align as fa, kayei
from kayei_learn import Rig

SPEEDS = (100.0, 200.0)
LEADS = (0.06, 0.10, 0.14)
A_PER_G = -1.015  # from yesterday's calibrations (-1.01..-1.02)

rig = Rig()
h = rig.h
try:
    # align angle only
    for _ in range(4):
        _, m = rig.fresh()
        if abs(m["angle"]) < 0.01: break
        rig.hydra_to(fa.GONIO, rig.gonio() - m["angle"] / A_PER_G)
    g_al = rig.gonio()
    print(f"aligned: gonio {g_al:+.4f}, angle {m['angle']:+.4f}")

    def turns(speed, n, control=None):
        """rotate n turns at speed; control(theta_now, t_now) -> gonio target or None. returns samples (th_frame, angle)"""
        clk = kayei.RotClock(speed)
        th0 = h.position(fa.ROT); clk.sync(th0)
        v0 = h.velocity(fa.ROT); h.set_velocity(fa.ROT, speed)
        h.move_abs(fa.ROT, th0 + 360 * n + 30)
        out, t_sync = [], time.time()
        try:
            while True:
                t, m = rig.fresh()
                now = time.time()
                if now - t_sync > 0.25:
                    clk.sync(h.position(fa.ROT)); t_sync = time.time()
                th_frame = clk.at(now - kayei.FRAME_DELAY) - th0
                if m: out.append((th_frame, m["angle"]))
                if control: control(clk.at(now) - th0, now)
                if th_frame >= 360 * n: break
        finally:
            h.stop(fa.ROT); time.sleep(0.3); h.set_velocity(fa.ROT, v0)
        return out

    # learn
    data = turns(90.0, 2)
    th = np.array([d[0] for d in data]); a = np.array([d[1] for d in data])
    model = kayei.WobbleModel().fit(th, np.ones_like(th), a, np.zeros_like(th))
    print("learned:", model.describe())
    rows = []
    for speed in SPEEDS:
        for lead in LEADS:
            res = kayei.Residual()
            last = {"g": g_al, "t": 0}
            def control(th_now, now, lead=lead, res=res):
                th_cmd = th_now + speed * lead
                ang = model.predict(th_cmd, 1.0)[0]
                g = g_al - ang / A_PER_G + res.get(th_cmd)[0]
                if abs(g - last["g"]) > 0.0005:
                    h.move_abs(fa.GONIO, g); last["g"] = g
            data = turns(speed, 3, control)   # 3 turns: first = settle, evaluate last 2
            th = np.array([d[0] for d in data]); a = np.array([d[1] for d in data])
            ev = th >= 360
            print(f"speed {speed:4.0f} lead {lead:.2f}: angle error rms {a[ev].std():.4f} mean {a[ev].mean():+.4f} max {np.abs(a[ev]).max():.4f} deg  ({ev.sum()} samples, {len(th)/ (360*3/speed):.0f} fps)")
            rows += [(speed, lead, t_, a_) for t_, a_ in data]
            rig.hydra_to(fa.GONIO, g_al)
    with open(r"C:\dev\hydra\kayei_data\gonio_ff_test.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["speed", "lead", "theta", "angle"]); w.writerows(rows)
finally:
    rig.close()

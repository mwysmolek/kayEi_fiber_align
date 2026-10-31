import sys, time; sys.path.insert(0, r"C:\dev\hydra")
import numpy as np, fiber_align as fa, kayei
from kayei_learn import Rig, fit_sine
rig = Rig(); h = rig.h
try:
    for speed in (30.0, 90.0, 200.0, 360.0):
        clk = kayei.RotClock(speed); th0 = h.position(fa.ROT); clk.sync(th0)
        v0 = h.velocity(fa.ROT); h.set_velocity(fa.ROT, speed); h.move_abs(fa.ROT, th0 + 720 + 40)
        enc, est, out, t_sync = [], [], [], 0
        try:
            while True:
                t, m = rig.fresh(); now = time.time()
                if now - t_sync > 0.25:
                    tq = time.time(); thq = h.position(fa.ROT); tq2 = time.time()
                    if t_sync: enc.append(thq); est.append(clk.at((tq + tq2) / 2))
                    clk.sync(thq, (tq + tq2) / 2); t_sync = time.time()
                thf = clk.at(now - kayei.FRAME_DELAY) - th0
                if m and thf > 40: out.append((thf, m["angle"], m["x"]))
                if thf >= 760: break
        finally:
            h.stop(fa.ROT); time.sleep(0.3); h.set_velocity(fa.ROT, v0)
        th = np.array([o[0] for o in out]); a = np.array([o[1] for o in out]); x = np.array([o[2] for o in out])
        ca = fit_sine(th, a); cx = fit_sine(th, x)
        d = np.array(enc) - np.array(est)
        print(f"{speed:4.0f} deg/s: angle amp {ca[2]:.3f} phase {ca[3]:+7.2f} rms {ca[1]:.4f} | x amp {cx[2]:.2f} phase {cx[3]:+7.2f} rms {cx[1]:.3f} | clock vs encoder: mean {d.mean():+.3f} max {np.abs(d).max():.3f} deg ({len(out)} frames)")
        rig.hydra_to(fa.ROT, h.position(fa.ROT) + (-h.position(fa.ROT)) % 360)
finally:
    rig.close()

"""kayEi goniometer-only with the fixed-rate Tracker thread: lead sweep at several rotation speeds."""
import sys, time, csv
sys.path.insert(0, r"C:\dev\hydra")
import numpy as np
import fiber_align as fa, kayei
from kayei_learn import Rig
A_PER_G, X_PER_Z, X_PER_G = -1.015, -49.5, -70.0
rig = Rig(); h = rig.h
try:
    for _ in range(4):
        _, m = rig.fresh()
        if abs(m["angle"]) < 0.01: break
        rig.hydra_to(fa.GONIO, rig.gonio() - m["angle"] / A_PER_G)
    g_al = rig.gonio()
    rig.hydra_to(fa.ROT, h.position(fa.ROT) + (-h.position(fa.ROT)) % 360)

    def turns(speed, n, tracker_cfg=None):
        """rotate n turns; tracker_cfg = (predictor, lead, rate) or None. returns (th_frame, angle) samples"""
        h.rezero_turn(fa.ROT); time.sleep(0.1)
        clk = kayei.RotClock(speed); th0 = h.position(fa.ROT); clk.sync(th0)
        v0 = h.velocity(fa.ROT); h.set_velocity(fa.ROT, speed)
        tr = None
        if tracker_cfg:
            pred, lead, rate = tracker_cfg
            pred.model_th0 = th0
            tr = kayei.Tracker(pred, clk, lambda t: 1.0, fa.HYDRA_IP, None, rate=rate, gonio_lead=lead, z_lead=lead)
        h.move_abs(fa.ROT, th0 + 360 * n + 30)
        if tr: tr.start()
        out, t_sync = [], time.time(); t_start = time.time()
        try:
            while True:
                t, m = rig.fresh(); now = time.time()
                if now - t_sync > 0.25:
                    tq = time.time(); thq = h.position(fa.ROT); clk.sync(thq, (tq + time.time()) / 2); t_sync = time.time()
                    if thq - th0 < 10 and now - t_start > 2.0:
                        raise RuntimeError(f"rotation not moving (at {thq:.1f}, Hydra error {h.query('ge')})")
                th_frame = clk.at(now - kayei.FRAME_DELAY) - th0
                if m: out.append((th_frame, m["angle"]))
                if tr and tr.error: raise tr.error
                if th_frame >= 360 * n: break
        finally:
            if tr: tr.stop()
            h.stop(fa.ROT); time.sleep(0.3); h.set_velocity(fa.ROT, v0)
        rig.hydra_to(fa.ROT, th0 + 360 * (n + 1))
        return out, tr

    class Pred:
        """predictor whose model is in angles relative to the run start (all runs start at the same angle mod 360)"""
        def __init__(self, model): self.model, self.model_th0 = model, 0.0
        def command(self, th_g, th_z, s):
            return g_al - self.model.predict(th_g - self.model_th0, s)[0] / A_PER_G, 0.0

    data, _ = turns(90.0, 2)
    th = np.array([d[0] for d in data]); a = np.array([d[1] for d in data])
    model = kayei.WobbleModel().fit(th, np.ones_like(th), a, np.zeros_like(th))
    print("learned:", model.describe())
    pred = Pred(model)
    rows = []
    for speed, lead, rate in ((100, 0.015, 50), (100, 0.03, 50), (200, 0.0, 50), (200, 0.015, 50), (200, 0.03, 50), (200, 0.045, 50),
                              (200, 0.03, 100), (360, 0.015, 50), (360, 0.03, 50), (360, 0.045, 50), (360, 0.03, 100)):
        data, tr = turns(float(speed), 3, (pred, lead, rate))
        th = np.array([d[0] for d in data]); a = np.array([d[1] for d in data]); ev = th >= 360
        print(f"speed {speed:3d} lead {lead * 1000:3.0f} ms rate {rate:3d} Hz: angle error rms {a[ev].std():.4f} mean {a[ev].mean():+.4f} max {np.abs(a[ev]).max():.4f} deg "
              f"| tracker {tr.n_tick / (len(th) / 50):.0f} ticks/s, {tr.n_cmd} cmds, worst gap {tr.period_max * 1000:.0f} ms")
        rows += [(speed, lead, rate, t_, a_) for t_, a_ in data]
        rig.hydra_to(fa.GONIO, g_al)
    with open(r"C:\dev\hydra\kayei_data\gonio_tracker.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["speed", "lead", "rate", "theta", "angle"]); w.writerows(rows)
finally:
    rig.close()

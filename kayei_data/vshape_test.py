"""Plain target streaming vs velocity-shaped streaming (speed set each tick to the trajectory's
speed, target 'lead' ahead), camera as the only sensor (no controller polling)."""
import sys, time, threading
sys.path.insert(0, r"C:\dev\hydra")
import numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
import fiber_align as fa
from kayei_learn import Rig

RATE = 50.0
def run(set_target, set_vel, A, f, secs, shaped, lead, vmin, vmax, margin=1.15):
    cmds, dt, t0 = [], 1.0 / RATE, time.perf_counter(); tn = t0; w = 2 * np.pi * f
    while True:
        now = time.perf_counter(); tt = now - t0
        if tt > secs: break
        if shaped:
            v = min(max(abs(A * w * np.cos(w * (tt + lead))) * margin, vmin), vmax); set_vel(v)
            u = A * np.sin(w * (tt + lead))
        else:
            u = A * np.sin(w * tt)
        set_target(u); cmds.append((now, A * np.sin(w * tt)))   # log the un-led trajectory
        tn += dt; lag = tn - now
        if lag > 0: time.sleep(lag)
        else: tn = now
    set_vel(vmax); set_target(0.0)
    return np.array(cmds)

def fit(t, y, f):
    w = 2 * np.pi * f; M = np.c_[np.ones_like(t), np.sin(w * t), np.cos(w * t)]
    c = np.linalg.lstsq(M, y, rcond=None)[0]
    return np.hypot(c[1], c[2]), -np.arctan2(c[2], c[1]) / w * 1000, (y - M @ c).std(), M @ c

rig = Rig(); h = rig.h
try:
    g0 = h.position(1); z0 = rig.zpos(); v0 = h.velocity(1)
    h.move_abs(1, g0 + 0.2); time.sleep(1.0); _, m1 = rig.fresh(); h.move_abs(1, g0); time.sleep(1.0); _, m0 = rig.fresh()
    a_g = (m1["angle"] - m0["angle"]) / 0.2
    rig.z_to(z0 + 0.2); time.sleep(0.5); _, m1 = rig.fresh(); rig.z_to(z0); time.sleep(0.5); _, m0 = rig.fresh()
    kx = (m1["x"] - m0["x"]) / 0.2
    print(f"camera: {a_g:+.3f} deg/deg gonio, {kx:+.1f} px/mm Z")
    fig, axs = plt.subplots(4, 3, figsize=(20, 13)); row = 0; allrows = []
    cases = [("gonio", 1.26, 0.278), ("gonio", 1.26, 0.556), ("Z", 1.2, 0.278), ("Z", 1.2, 0.556)]
    for axis, A, f in cases:
        for col, (shaped, lead_k) in enumerate(((False, 1.0), (True, 1.0), (True, 2.0))):
            if axis == "gonio":
                st, sv, key, scale, ref, vmin, vmax, lead = (lambda u: h.move_abs(1, g0 + u)), (lambda v: h.set_velocity(1, v)), "angle", a_g, m0["angle"], 0.02, 4.0, 0.06
            else:
                st, sv, key, scale, ref, vmin, vmax, lead = (lambda u: rig.z.MOV("1", z0 + u)), (lambda v: rig.z.VEL("1", v)), "x", kx, m0["x"], 0.05, 10.0, 0.04
            log, stop = [], threading.Event()
            def poll():
                while not stop.is_set():
                    t, m = rig.fresh()
                    if m: log.append((t, m[key]))
            th = threading.Thread(target=poll, daemon=True); th.start()
            lead *= lead_k; secs = max(7.0, 3 / f)
            cmds = run(st, sv, A, f, secs, shaped, lead, vmin, vmax)
            time.sleep(0.8); stop.set(); th.join()
            t0 = cmds[0, 0]; tk, yk = np.array(log).T; sk = (tk - t0 > 1 / f) & (tk - t0 < secs - 0.05)
            allrows += [(axis, A, f, int(shaped), lead, t - t0, (yy - ref) / scale) for t, yy in zip(tk, yk)]
            y = (yk - ref) / scale
            amp, lag, rest, _ = fit(tk[sk] - t0, y[sk], f)
            err = y[sk] - A * np.sin(2 * np.pi * f * (tk[sk] - t0))
            unit = "deg" if axis == "gonio" else "mm"
            print(f"{axis:5s} A {A:.2f} f {f:.3f} {'SHAPED' if shaped else 'plain '} lead {lead*1000:3.0f} ms: amp {amp/A*100:5.1f}% lag {lag:+4.0f} ms (incl. camera ~30) non-sine {rest:.4f} {unit} | raw tracking error rms {err.std():.4f} {unit}", flush=True)
            ax = axs[row, col]; ax.plot(cmds[:, 0] - t0, cmds[:, 1], "-", lw=1, label="trajectory"); ax.plot(tk - t0, y, ".", ms=2, label="camera")
            ax.set_xlim(1 / f, 1 / f + 2.5 / f); ax.grid(alpha=0.3); ax.legend(fontsize=8); ax.set_title(f"{axis} A {A} f {f} {'velocity-shaped' if shaped else 'plain targets'}: non-sine {rest:.4f} {unit}", fontsize=9)
            time.sleep(0.8)
        row += 1
    fig.tight_layout(); fig.savefig(r"C:\dev\hydra\kayei_data\vshape_test.png", dpi=90)
    h.set_velocity(1, v0); rig.z.VEL("1", 10.0); h.move_abs(1, g0); rig.z_to(z0); time.sleep(1.5)
    _, m = rig.fresh(); print(f"back: camera angle {(m['angle']-m0['angle']):+.4f} deg, x {(m['x']-m0['x'])/kx*1000:+.0f} um; gonio vel {h.velocity(1)} Z VEL {rig.z.qVEL('1')['1']}")
finally:
    rig.close()

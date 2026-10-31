"""Tracking vs acceleration setting: goniometer (encoder + camera angle) and Z (camera x)."""
import sys, time, threading
sys.path.insert(0, r"C:\dev\hydra")
import numpy as np
import fiber_align as fa
from kayei_learn import Rig
from hydra import Hydra

def stream(fn_cmd, A, f, secs, rate=50.0):
    out, dt, t0 = [], 1.0 / rate, time.perf_counter(); tn = t0
    while True:
        now = time.perf_counter()
        if now - t0 > secs: break
        u = A * np.sin(2 * np.pi * f * (now - t0)); fn_cmd(u); out.append((now, u))
        tn += dt; lag = tn - now
        if lag > 0: time.sleep(lag)
        else: tn = now
    fn_cmd(0.0); return out

def fit(t, y, f):
    w = 2 * np.pi * f; A = np.c_[np.ones_like(t), np.sin(w * t), np.cos(w * t)]
    c = np.linalg.lstsq(A, y, rcond=None)[0]
    return np.hypot(c[1], c[2]), -np.arctan2(c[2], c[1]) / w * 1000, (y - A @ c).std()

def camera_poll(rig, key, log, stop):
    while not stop.is_set():
        t, m = rig.fresh()
        if m: log.append((t, m[key]))

rig = Rig(); h = rig.h; hq = Hydra(fa.HYDRA_IP)
try:
    g0 = h.position(1); z0 = rig.zpos()
    _, m0 = rig.fresh(); a_g = None
    # camera scale for gonio angle and Z: static steps
    h.move_abs(1, g0 + 0.2); time.sleep(1.0); _, m1 = rig.fresh(); h.move_abs(1, g0); time.sleep(1.0); _, m0 = rig.fresh()
    a_g = (m1["angle"] - m0["angle"]) / 0.2
    rig.z_to(z0 + 0.2); time.sleep(0.5); _, m1 = rig.fresh(); rig.z_to(z0); time.sleep(0.5); _, m0 = rig.fresh()
    kx = (m1["x"] - m0["x"]) / 0.2
    print(f"camera: {a_g:+.3f} deg/deg gonio, {kx:+.1f} px/mm Z")
    for acc in (50, 150, 400):
        h.send(f"{acc} 1 setnaccel"); rb = h.query("1 getnaccel"); 
        for A, f in ((0.1, 1.0), (1.26, 0.278), (1.26, 0.556)):
            elog, clog, stop = [], [], threading.Event()
            def epoll():
                while not stop.is_set():
                    t1 = time.perf_counter(); p = hq.position(1); elog.append(((t1 + time.perf_counter()) / 2, p))
            ts = [threading.Thread(target=epoll, daemon=True), threading.Thread(target=camera_poll, args=(rig, "angle", clog, stop), daemon=True)]
            [t.start() for t in ts]
            cmds = stream(lambda u: h.move_abs(1, g0 + u), A, f, max(6.0, 3 / f))
            time.sleep(0.6); stop.set(); [t.join() for t in ts]
            tc, uc = np.array(cmds).T; t0 = tc[0]; _, lc, _ = fit(tc - t0, uc, f)
            te, pe = np.array(elog).T; se = te - t0 > 1 / f; ea, el, er = fit(te[se] - t0, pe[se] - g0, f)
            tk, ak = np.array(clog).T; sk = tk - t0 > 1 / f; ca, cl, cr = fit(tk[sk] - t0, (ak[sk] - m0["angle"]) / a_g, f)
            print(f"gonio accel {rb.strip():>4s} A {A:.2f} f {f:.3f}: encoder {ea/A*100:5.1f}% lag {el-lc:+4.0f} ms non-sine {er:.4f} | camera {ca/A*100:5.1f}% lag {cl-lc:+4.0f} ms non-sine {cr:.4f} deg", flush=True)
        time.sleep(1.0); print(f"   gonio back at {h.position(1)-g0:+.4f} (encoder), camera angle {rig.fresh()[1]['angle']-m0['angle']:+.4f} deg", flush=True)
    h.send("50 1 setnaccel")
    for acc in (100, 300, 1000):
        rig.z.ACC("1", acc); rig.z.DEC("1", acc); rb = rig.z.qACC("1")["1"]
        for A, f in ((0.1, 1.0), (1.2, 0.278), (1.2, 0.556)):
            clog, stop = [], threading.Event()
            t = threading.Thread(target=camera_poll, args=(rig, "x", clog, stop), daemon=True); t.start()
            cmds = stream(lambda u: rig.z.MOV("1", z0 + u), A, f, max(6.0, 3 / f))
            time.sleep(0.6); stop.set(); t.join()
            tc, uc = np.array(cmds).T; t0 = tc[0]; _, lc, _ = fit(tc - t0, uc, f)
            tk, xk = np.array(clog).T; sk = tk - t0 > 1 / f; ca, cl, cr = fit(tk[sk] - t0, (xk[sk] - m0["x"]) / kx, f)
            print(f"Z ACC {rb:6.0f} A {A:.2f} f {f:.3f}: camera {ca/A*100:5.1f}% lag {cl-lc:+4.0f} ms non-sine {cr*1000:4.0f} um", flush=True)
        time.sleep(0.8); print(f"   Z back: camera x {(rig.fresh()[1]['x']-m0['x'])/kx*1000:+.0f} um, qPOS {rig.zpos()-z0:+.4f}", flush=True)
    rig.z.ACC("1", 100); rig.z.DEC("1", 100)
finally:
    hq.close(); rig.close()

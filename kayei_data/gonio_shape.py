"""Goniometer: how does the profile generator execute a 50 Hz target stream? Time series of
command vs encoder (accel 50, A 1.26 deg, f 0.278 Hz) at stream rates 25/50/100 Hz, plus a probe
whether setnvel changes the speed of a move in progress."""
import sys, time, threading
sys.path.insert(0, r"C:\dev\hydra")
import numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
import fiber_align as fa
from hydra import Hydra
from kayei_learn import Rig
rig = Rig(); h = rig.h; hq = Hydra(fa.HYDRA_IP)
rig.z.ACC("1", 100); rig.z.DEC("1", 100); print("Z ACC", rig.z.qACC("1")["1"])
h.send("50 1 setnaccel"); print("gonio accel", h.query("1 getnaccel").strip())
try:
    g0 = h.position(1)
    fig, axs = plt.subplots(3, 1, figsize=(14, 10))
    for ax, rate in zip(axs, (25.0, 50.0, 100.0)):
        A, f = 1.26, 0.278
        elog, stop = [], threading.Event()
        def epoll():
            while not stop.is_set():
                t1 = time.perf_counter(); p = hq.position(1); elog.append(((t1 + time.perf_counter()) / 2, p))
        th = threading.Thread(target=epoll, daemon=True); th.start()
        cmds, dt, t0 = [], 1.0 / rate, time.perf_counter(); tn = t0
        while time.perf_counter() - t0 < 8.0:
            now = time.perf_counter(); u = A * np.sin(2 * np.pi * f * (now - t0)); h.move_abs(1, g0 + u); cmds.append((now, u))
            tn += dt; lag = tn - now
            if lag > 0: time.sleep(lag)
            else: tn = now
        h.move_abs(1, g0); time.sleep(0.6); stop.set(); th.join()
        tc, uc = np.array(cmds).T; te, pe = np.array(elog).T
        ax.plot(tc - t0, uc, "-", lw=1, label=f"command ({rate:.0f} Hz)"); ax.plot(te - t0, pe - g0, ".", ms=2, label="encoder")
        ax.set_xlim(2.0, 4.5); ax.set_ylim(-1.4, 1.4); ax.grid(alpha=0.3); ax.legend(); ax.set_ylabel("deg")
        ve = np.gradient(pe, te); print(f"rate {rate:.0f} Hz: encoder speed max {np.abs(ve).max():.2f} deg/s, time at zero speed {(np.abs(ve) < 0.05).mean()*100:.0f}%, {len(pe)} samples ({len(pe)/(te[-1]-te[0]):.0f}/s)", flush=True)
        time.sleep(1.0)
    fig.tight_layout(); fig.savefig(r"C:\dev\hydra\kayei_data\gonio_shape.png", dpi=100)
    # velocity probe: long move, change nvel mid-way
    v0 = h.velocity(1); h.set_velocity(1, 1.0); h.move_abs(1, g0 + 3.0); t0 = time.perf_counter(); log = []
    while time.perf_counter() - t0 < 1.0: log.append((time.perf_counter() - t0, hq.position(1) - g0))
    h.set_velocity(1, 3.0)
    while time.perf_counter() - t0 < 2.0: log.append((time.perf_counter() - t0, hq.position(1) - g0))
    h.stop(1); time.sleep(0.5); h.set_velocity(1, v0); h.move_abs(1, g0)
    t, p = np.array(log).T; v = np.gradient(p, t)
    print(f"velocity probe: speed in first second {np.median(v[(t>0.3)&(t<1.0)]):.2f} deg/s, after setnvel 3 {np.median(v[(t>1.3)&(t<2.0)]):.2f} deg/s")
    time.sleep(2); print("gonio back at", h.position(1) - g0)
finally:
    hq.close(); rig.close()

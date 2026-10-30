"""All 4 axes at once: each axis runs out-and-back strokes at increasing speeds in its own
thread (own connection). Logs position vs time to all_axes_log.csv and plots all_axes.png.
Needs: Hydra homed (hydra.py --home); PI stages are referenced here (FNL) if not yet."""
import csv
import threading
import time

import matplotlib.pyplot as plt
from pipython import GCSDevice, pitools

from hydra import Hydra

IP = "172.20.5.201"
# name: (kind, address, stroke (a, b), speeds)
AXES = {
    "PI 183 VT-80 50mm [mm]": ("pi", "0165500183", (1.0, 46.0), (5, 10, 15, 20)),
    "PI 187 VT-80 25mm [mm]": ("pi", "0165500187", (1.0, 21.0), (2, 5, 10, 20)),
    "goniometer [deg]": ("hydra", 1, (-4.0, 4.0), (1, 2, 3, 4)),
    "rotation [deg]": ("hydra", 2, (0.0, 360.0), (30, 90, 180, 360)),
}

log = []  # (t, axis name, speed, target, pos)
lock = threading.Lock()
start = threading.Barrier(len(AXES))
abort = threading.Event()
t0 = 0.0


def record(name, speed, target, pos):
    with lock:
        log.append((time.perf_counter() - t0, name, speed, target, pos))


def run_pi(name, sn, stroke, speeds):
    with GCSDevice() as pi:
        pi.ConnectUSB(sn)
        if not pi.qFRF()["1"]:
            pitools.startup(pi, refmodes="FNL")
        v0 = pi.qVEL()["1"]
        pi.MOV("1", stroke[0]); pitools.waitontarget(pi)
        start.wait()
        try:
            for v in speeds:
                pi.VEL("1", v)
                for target in (stroke[1], stroke[0]):
                    pi.MOV("1", target)
                    while not pi.qONT()["1"]:
                        if abort.is_set():
                            pi.STP(noraise=True); return
                        record(name, v, target, pi.qPOS()["1"])
                    record(name, v, target, pi.qPOS()["1"])
        finally:
            pi.VEL("1", v0)


def run_hydra(name, ax, stroke, speeds):
    h = Hydra(IP)
    try:
        v0 = h.velocity(ax)
        h.move_abs(ax, stroke[0]); time.sleep(0.05); h.wait(ax)
        start.wait()
        try:
            for v in speeds:
                h.set_velocity(ax, v)
                for target in (stroke[1], stroke[0]):
                    h.move_abs(ax, target)
                    time.sleep(0.05)
                    while h.is_moving(ax):
                        if abort.is_set():
                            h.stop(ax); return
                        record(name, v, target, h.position(ax))
                    record(name, v, target, h.position(ax))
        finally:
            h.set_velocity(ax, v0)
    finally:
        h.close()


def worker(name, kind, addr, stroke, speeds):
    try:
        (run_pi if kind == "pi" else run_hydra)(name, addr, stroke, speeds)
    except Exception as e:
        abort.set()
        start.abort()
        print(f"{name}: ERROR {e!r} -> stopping all axes")


def main():
    global t0
    t0 = time.perf_counter()
    threads = [threading.Thread(target=worker, args=(n, *cfg)) for n, cfg in AXES.items()]
    for th in threads:
        th.start()
    try:
        for th in threads:
            while th.is_alive():
                th.join(0.2)
    except KeyboardInterrupt:
        abort.set()
        for th in threads:
            th.join()

    with open("all_axes_log.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "axis", "speed_set", "target", "pos"])
        w.writerows(log)
    fig, axs = plt.subplots(len(AXES), 1, figsize=(13, 10), sharex=True)
    for a, name in zip(axs, AXES):
        rows = [r for r in log if r[1] == name]
        a.plot([r[0] for r in rows], [r[4] for r in rows], lw=1.2, color="#2a78d6")
        a.set_ylabel(name)
        a.grid(alpha=0.3)
        for v in AXES[name][3]:
            ts = [r[0] for r in rows if r[2] == v]
            if ts:
                a.text((ts[0] + ts[-1]) / 2, a.get_ylim()[1], f"{v}/s", ha="center", va="top", fontsize=8)
        print(f"{name}: {len(rows)} samples, {rows[0][0]:.1f}..{rows[-1][0]:.1f} s, "
              f"final {rows[-1][4]:+.5f} (target {rows[-1][3]:+.3f})")
    axs[-1].set_xlabel("time [s]")
    fig.suptitle("All 4 axes simultaneously, out-and-back at increasing speeds")
    fig.tight_layout()
    fig.savefig("all_axes.png", dpi=110)


if __name__ == "__main__":
    main()

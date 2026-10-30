"""Rotation stage test at max speed: 10 turns out and back, then 45 deg steps
0..360 and back, holding at each stop; finally re-check the encoder index.
Writes spin_log.csv (t, phase, target, pos) and spin_steps.png."""
import csv
import threading
import time

import numpy as np

from hydra import Hydra

IP = "172.20.5.201"
ROT = 2
TURNS = 10
HOLD = 0.5

log = []
tag = {"phase": "", "target": 0.0}
running = True


def logger():
    mon = Hydra(IP)
    t0 = time.perf_counter()
    while running:
        p = mon.position(ROT)
        log.append((time.perf_counter() - t0, tag["phase"], tag["target"], p))
    mon.close()


def go(h, phase, target, hold=HOLD):
    tag.update(phase=phase, target=target)
    h.move_abs(ROT, target)
    time.sleep(0.05)
    h.wait(ROT, timeout=60)
    time.sleep(hold)


h = Hydra(IP)
print("start", h.position(ROT), "speed", h.velocity(ROT))
th = threading.Thread(target=logger)
th.start()
try:
    t = time.perf_counter()
    go(h, "spin", 360 * TURNS, hold=1.0)
    print(f"+{TURNS} turns: {time.perf_counter() - t:.2f}s at {h.position(ROT):+.5f}")
    t = time.perf_counter()
    go(h, "spin_back", 0, hold=1.0)
    print(f"back to 0: {time.perf_counter() - t:.2f}s at {h.position(ROT):+.5f}")
    for a in list(range(45, 361, 45)) + list(range(315, -1, -45)):
        go(h, "steps", float(a))
    # index check: park at -30, search index in + direction (same as homing)
    go(h, "index", -30.0, hold=0.2)
    tag.update(phase="index", target=0.0)
    h.find_index(ROT)
    time.sleep(0.3)
    h.wait(ROT)
    idx = h.position(ROT)
    print(f"index found at {idx:+.5f} deg (after homing it was +0.00121)")
    go(h, "index", 0.0, hold=0.2)
finally:
    running = False
    th.join()
    print("final", h.position(ROT), "errors:", h.query("ge"))
    h.close()

with open("spin_log.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t_s", "phase", "target", "rot_deg"])
    w.writerows(log)

t = np.array([r[0] for r in log])
ph = np.array([r[1] for r in log])
tg = np.array([r[2] for r in log])
p = np.array([r[3] for r in log])
print(f"\n{len(log)} samples, mean interval {np.diff(t).mean() * 1000:.2f} ms")

# spin: steady speed and ripple over the constant-velocity part
m = ph == "spin"
frac = p[m] / (360 * TURNS)
cv = (frac > 0.1) & (frac < 0.9)
ts, ps = t[m][cv], p[m][cv]
slope = np.polyfit(ts, ps, 1)
resid = ps - np.polyval(slope, ts)
win = max(int(0.1 / np.diff(ts).mean()), 2)
v100 = (ps[win:] - ps[:-win]) / (ts[win:] - ts[:-win])
print(f"spin: mean speed {slope[0]:.3f} deg/s, 100 ms speed min/max {v100.min():.2f}/{v100.max():.2f},"
      f" position deviation from constant speed {resid.std() * 1000:.1f} mdeg rms")

# steps: settle, final error, holding jitter
steps = []
m = ph == "steps"
idx = np.where(m)[0]
for seg in np.split(idx, np.where(np.diff(tg[idx]) != 0)[0] + 1):
    tt, pp, target = t[seg], p[seg], tg[seg[0]]
    err = pp - target
    arrive = np.argmax(np.abs(err) < 0.5)
    inband = np.abs(err) < 1e-3
    settle_i = next((i for i in range(arrive, len(err)) if inband[i:].all()), len(err) - 1)
    hold = tt > tt[-1] - 0.3
    steps.append(dict(target=target, overshoot=np.abs(err[arrive:]).max() * 1000,
                      settle_ms=(tt[settle_i] - tt[arrive]) * 1000,
                      final=err[hold].mean() * 1000, jitter=err[hold].std() * 1000))
print("\n target | max |err| after arrival | settle into +-1 mdeg | final error | hold jitter (std)")
for s in steps:
    print(f"  {s['target']:5.0f} | {s['overshoot']:8.2f} mdeg | {s['settle_ms']:6.0f} ms |"
          f" {s['final']:+7.3f} mdeg | {s['jitter']:.3f} mdeg")

# ---- plot ----
import matplotlib.pyplot as plt

BLUE, INK, INK2, GRID, SURF = "#2a78d6", "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "axes.titlecolor": INK,
                     "axes.spines.top": False, "axes.spines.right": False,
                     "axes.facecolor": SURF})
fig, axs = plt.subplots(4, 1, figsize=(13, 14), facecolor=SURF)


def style(ax, title, xl, yl):
    ax.set_title(title, loc="left", fontweight="bold")
    ax.set_xlabel(xl)
    ax.set_ylabel(yl)
    ax.grid(axis="y", color=GRID, lw=0.8)


m = (ph == "spin") | (ph == "spin_back")
tm, pm = t[m], p[m]
w = max(int(0.1 / np.diff(tm).mean()), 2)
vel = (pm[w:] - pm[:-w]) / (tm[w:] - tm[:-w])
axs[0].plot((tm[w:] + tm[:-w]) / 2 - tm[0], vel, color=BLUE, lw=2)
style(axs[0], f"{TURNS} turns out and back at 360°/s: speed (100 ms window)", "time [s]", "speed [deg/s]")

m = ph == "steps"
axs[1].plot(t[m] - t[m][0], p[m], color=BLUE, lw=2)
axs[1].set_yticks(range(0, 361, 45))
style(axs[1], "45° steps 0 → 360 → 0 at 360°/s, 0.5 s hold", "time [s]", "position [deg]")

lab = [f"{s['target']:.0f}" for s in steps]
x = np.arange(len(steps))
axs[2].axhline(0, color=INK2, lw=1)
axs[2].errorbar(x, [s["final"] for s in steps], yerr=[s["jitter"] for s in steps],
                fmt="o", color=BLUE, ms=8, capsize=4, lw=2)
axs[2].set_xticks(x, lab)
style(axs[2], "Final position error per stop (dot = mean of last 0.3 s, bar = ±1 std hold jitter)",
      "target [deg], in order of the sequence", "error [mdeg]")

axs[3].bar(x, [s["settle_ms"] for s in steps], color=BLUE, width=0.6)
axs[3].set_xticks(x, lab)
style(axs[3], "Settling time into ±1 mdeg after arrival", "target [deg], in order of the sequence",
      "settle [ms]")
fig.tight_layout()
fig.savefig("spin_steps.png", dpi=110, facecolor=SURF)
print("saved spin_steps.png")

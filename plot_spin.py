"""Plot spin_log.csv -> spin_steps.png: spin, steps, overshoot and final error per stop."""
import csv
import numpy as np
import matplotlib.pyplot as plt

BLUE, INK, INK2, GRID, SURF = "#2a78d6", "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
rows = list(csv.DictReader(open("spin_log.csv")))
t = np.array([float(r["t_s"]) for r in rows])
ph = np.array([r["phase"] for r in rows])
tg = np.array([float(r["target"]) for r in rows])
p = np.array([float(r["rot_deg"]) for r in rows])

steps, prev = [], 0.0
idx = np.where(ph == "steps")[0]
for seg in np.split(idx, np.where(np.diff(tg[idx]) != 0)[0] + 1):
    target, tt, err = tg[seg[0]], t[seg], p[seg] - tg[seg[0]]
    sign = np.sign(target - prev)
    prev = target
    hold = tt > tt[-1] - 0.3
    arrive = np.argmax(np.abs(err) < 0.5)
    settle = next(i for i in range(arrive, len(err)) if (np.abs(err[i:]) < 1e-3).all())
    steps.append(dict(target=target, over=max((err * sign).max(), 0) * 1000,
                      final=err[hold].mean() * 1000, jit=err[hold].std() * 1000,
                      settle=(tt[settle] - tt[arrive]) * 1000))

plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "axes.titlecolor": INK,
                     "axes.spines.top": False, "axes.spines.right": False, "axes.facecolor": SURF})
fig, axs = plt.subplots(4, 1, figsize=(13, 14), facecolor=SURF)


def style(ax, title, xl, yl):
    ax.set_title(title, loc="left", fontweight="bold")
    ax.set_xlabel(xl)
    ax.set_ylabel(yl)
    ax.grid(axis="y", color=GRID, lw=0.8)


m = (ph == "spin") | (ph == "spin_back")
axs[0].plot(t[m] - t[m][0], p[m] / 360, color=BLUE, lw=2)
style(axs[0], "10 turns out and back at 360°/s", "time [s]", "position [turns]")

m = ph == "steps"
axs[1].plot(t[m] - t[m][0], p[m], color=BLUE, lw=2)
axs[1].set_yticks(range(0, 361, 45))
style(axs[1], "45° steps 0 → 360 → 0 at 360°/s, 0.5 s hold", "time [s]", "position [deg]")

x = np.arange(len(steps))
lab = [f"{s['target']:.0f}" for s in steps]
axs[2].bar(x, [s["over"] for s in steps], color=BLUE, width=0.6)
for xi, s in zip(x, steps):
    axs[2].text(xi, s["over"] + 0.2, f"{s['over']:.1f}", ha="center", color=INK2, fontsize=9)
axs[2].set_xticks(x, lab)
style(axs[2], "Overshoot past target per stop (sampled every ~7 ms, so a lower bound)",
      "target [deg], in sequence order", "overshoot [mdeg]")

axs[3].axhline(0, color=INK2, lw=1)
axs[3].errorbar(x, [s["final"] for s in steps], yerr=[s["jit"] for s in steps],
                fmt="o", color=BLUE, ms=8, capsize=4, lw=2)
axs[3].set_xticks(x, lab)
style(axs[3], "Final error while holding (dot = mean of last 0.3 s, bar = ±1 std jitter)",
      "target [deg], in sequence order", "error [mdeg]")
fig.tight_layout()
fig.savefig("spin_steps.png", dpi=110, facecolor=SURF)
print("settle ms:", [round(s["settle"]) for s in steps])

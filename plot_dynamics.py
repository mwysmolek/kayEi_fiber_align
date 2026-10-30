"""Plot dynamics_log.csv -> dynamics.png (rotation sweep, goniometer sweep, settling zoom)."""
import csv
import numpy as np
import matplotlib.pyplot as plt

BLUE, INK, INK2, GRID, SURF = "#2a78d6", "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
rows = list(csv.DictReader(open("dynamics_log.csv")))
t = np.array([float(r["t_s"]) for r in rows])
ax_m = np.array([r["axis_moving"] for r in rows])
spd = np.array([float(r["speed_set"]) for r in rows])
tgt = np.array([float(r["target"]) for r in rows])
gon = np.array([float(r["gonio_deg"]) for r in rows])
rot = np.array([float(r["rot_deg"]) for r in rows])

plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "axes.titlecolor": INK,
                     "axes.spines.top": False, "axes.spines.right": False})
fig, axs = plt.subplots(3, 1, figsize=(13, 11), facecolor=SURF,
                        gridspec_kw={"height_ratios": [1, 1, 0.9]})


def sweep(ax, name, y, unit_fmt):
    m = ax_m == name
    t0 = t[m][0]
    ax.plot(t[m] - t0, y[m], color=BLUE, lw=2)
    for v in np.unique(spd[m]):
        mm = m & (spd == v)
        ts = t[mm] - t0
        ax.axvline(ts[0], color=GRID, lw=1, zorder=0)
        ax.text(ts.mean(), ax.get_ylim()[1] if False else y[m].max() * 1.08, unit_fmt(v),
                ha="center", va="bottom", color=INK2, fontsize=9)
    ax.set_xlabel("time [s]")
    ax.grid(axis="y", color=GRID, lw=0.8)
    ax.set_facecolor(SURF)
    ax.set_ylim(y[m].min() * 1.15, y[m].max() * 1.25)


sweep(axs[0], "rot", rot, lambda v: f"{v:g}°/s")
axs[0].set_title("Rotation stage (axis 2): ±moves, speed set per block", loc="left", fontweight="bold")
axs[0].set_ylabel("position [deg]")
sweep(axs[1], "gonio", gon, lambda v: f"{v:g}°/s")
axs[1].set_title("Goniometer (axis 1): up/down, speed set per block (reported = commanded, open loop)",
                 loc="left", fontweight="bold")
axs[1].set_ylabel("tilt [deg]")

# settling zoom: rotation 360 deg/s arriving at +360 and at 0
ax = axs[2]
m = (ax_m == "rot") & (spd == 360)
idx = np.where(m)[0]
segs = np.split(idx, np.where(np.diff(tgt[idx]) != 0)[0] + 1)
seg = max(segs, key=lambda s: abs(tgt[s[0]]))  # first move to +-360
tt, pp, target = t[seg], rot[seg], tgt[seg[0]]
i_arr = np.argmax(np.abs(pp - target) < 1.0)
ta = tt[i_arr]
w = (tt > ta - 0.05) & (tt < ta + 0.25)
ax.plot((tt[w] - ta) * 1000, (pp[w] - target) * 1000, color=BLUE, lw=2, marker="o", ms=4)
ax.axhline(0, color=INK2, lw=1)
ax.axhspan(-1, 1, color=GRID, zorder=0)
ax.text(240, 1.3, "±1 mdeg band", ha="right", color=INK2, fontsize=9)
ax.set_title(f"Rotation arriving at {target:+g}° at 360°/s: error vs time (sample ≈ 13 ms)",
             loc="left", fontweight="bold")
ax.set_xlabel("time from arrival [ms]")
ax.set_ylabel("error [mdeg]")
ax.set_ylim(-15, 15)
ax.grid(axis="y", color=GRID, lw=0.8)
ax.set_facecolor(SURF)
fig.tight_layout()
fig.savefig("dynamics.png", dpi=110, facecolor=SURF)
print("saved dynamics.png")

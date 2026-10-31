"""Evaluate logged LIVE runs (kayei_data/run_*.csv): per run the angle / offset error after the
first full turn, as a markdown table and a figure (docs/figures/kayei_comparison.png).
Run:  python kayei_eval.py [date-prefix, e.g. 20261031_17 | list.txt] [figure name]"""
import csv
import glob
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.ticker
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
UM_PER_PX = 1000 / 49.0


def load(path):
    rows = [r for r in csv.DictReader(open(path)) if r["angle_deg"] != ""]
    t = np.array([float(r["t"]) for r in rows])
    th = np.array([float(r["theta_deg"]) for r in rows])
    a = np.array([float(r["angle_deg"]) for r in rows])
    x = np.array([float(r["x_px"]) for r in rows]) * UM_PER_PX
    # turns: count wraps of theta (mod 360)
    turns = np.cumsum(np.r_[0, np.diff(th) < -180])
    return t, th, a, x, turns


def evaluate(path):
    t, th, a, x, turns = load(path)
    m = re.search(r"run_(\d{8}_\d{6})_(\w+?)_(\d+)dps(_scan)?\.csv", os.path.basename(path))
    stamp, mode, speed, scan = m.group(1), m.group(2), int(m.group(3)), bool(m.group(4))
    # evaluate after the first full turn AND after the first 2 s (start transients: axes
    # travelling to the reference, residual table not yet learning)
    ev = (turns >= 1) & (t - t[0] > 2.0)
    if ev.sum() < 20:
        ev = np.ones_like(turns, bool)   # run too short: use everything
    return dict(stamp=stamp, mode=mode, speed=speed, scan=scan, n=int(ev.sum()), turns=int(turns[-1]),
                duration=float(t[-1] - t[0]),
                a_rms=float(np.sqrt(np.mean(a[ev] ** 2))), a_max=float(np.abs(a[ev]).max()),
                x_rms=float(np.sqrt(np.mean(x[ev] ** 2))), x_max=float(np.abs(x[ev]).max()))


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else ""
    if prefix.endswith(".txt"):   # explicit list of run files (one basename per line)
        files = [os.path.join(HERE, "kayei_data", l.strip()) for l in open(prefix) if l.strip()]
    else:
        files = sorted(glob.glob(os.path.join(HERE, "kayei_data", f"run_{prefix}*.csv")))  # prefix may contain glob chars
    fig_name = sys.argv[2] if len(sys.argv) > 2 else "kayei_comparison.png"
    res = [evaluate(f) for f in files]
    names = {"as": "as is", "live": "live align", "kayei": "kayEi"}
    print("| Mode | Rotation | Scan | Turns evaluated | Angle rms / max [deg] | Offset rms / max [um] |")
    print("|---|---|---|---|---|---|")
    for r in res:
        print(f"| {names.get(r['mode'], r['mode'])} | {r['speed']} deg/s | {'yes' if r['scan'] else 'no'} | {r['turns']} "
              f"| {r['a_rms']:.4f} / {r['a_max']:.3f} | {r['x_rms']:.1f} / {r['x_max']:.0f} |")

    fig, axs = plt.subplots(1, 2, figsize=(12, 4.4))
    styles = {"as": ("C3", "as is"), "live": ("C0", "live align (feedback)"), "kayei": ("C2", "kayEi (predictive)")}
    for mode, (col, label) in styles.items():
        for scan, ls in ((False, "-"), (True, "--")):
            pts = sorted((r["speed"], r["a_rms"], r["x_rms"]) for r in res if r["mode"] == mode and r["scan"] == scan)
            if not pts:
                continue
            axs[0].plot([p[0] for p in pts], [p[1] for p in pts], ls, marker="o", color=col, label=f"{label}{' + scan' if scan else ''}")
            axs[1].plot([p[0] for p in pts], [p[2] for p in pts], ls, marker="o", color=col, label=f"{label}{' + scan' if scan else ''}")
    for ax, yl, tol in ((axs[0], "fiber angle error rms [deg]", 0.02), (axs[1], "centre offset error rms [um]", 20)):
        ax.axhline(tol, color="gray", ls=":", lw=1, label="step-by-step tolerance")
        ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlabel("rotation speed [deg/s]"); ax.set_ylabel(yl)
        ax.set_xticks([20, 50, 100, 200, 360]); ax.set_xticklabels(["20", "50", "100", "200", "360"])
        ax.xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        ax.grid(alpha=0.3, which="both"); ax.legend(fontsize=8)
    axs[0].set_title("Error after the first turn, all three LIVE modes")
    fig.tight_layout()
    out = os.path.join(HERE, "docs", "figures", fig_name)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=120)
    print(f"\nfigure: {out}")


if __name__ == "__main__":
    main()

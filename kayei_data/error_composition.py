"""Figure: what is left after kayEi on the tilted fiber at 100 deg/s - the per-turn error
profiles (repeatable part = their mean, non-repeatable = scatter around it), with learning off
and with the Fourier learner on. Run: python kayei_data/error_composition.py <run_off.csv> <run_on.csv>"""
import csv, os, sys
import numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import sys as _s; _s.path.insert(0, HERE); import fiber_align as _fa
UM = 1000 / _fa.PX_PER_MM

def profiles(path):
    r = list(csv.DictReader(open(path)))
    t = np.array([float(x["t"]) for x in r]); t -= t[0]; th = np.array([float(x["theta_deg"]) for x in r])
    a = np.array([float(x["angle_deg"]) for x in r]); x = np.array([float(x["x_px"]) for x in r]) * UM
    turn = np.cumsum(np.r_[0, np.diff(th) < -180])
    out = {}
    for nm, y in (("angle", a), ("offset", x)):
        prof = []
        for k in range(1, turn.max()):
            s = turn == k
            if s.sum() < 50: continue
            b = (th[s] // 5).astype(int); prof.append([y[s][b == i].mean() if (b == i).any() else np.nan for i in range(72)])
        prof = np.array(prof); m = np.nanmean(prof, 0)
        out[nm] = dict(prof=prof, mean=m, rep=np.nanstd(m), nonrep=np.nanmean(np.nanstd(prof - m, 0)), total=y[t > 3.6].std())
    return out

off, on = profiles(sys.argv[1]), profiles(sys.argv[2])
fig, axs = plt.subplots(2, 2, figsize=(13, 7.5), sharex=True)
thb = np.arange(72) * 5 + 2.5
for col, (lab, d) in enumerate((("residual learning off", off), ("Fourier learner on (harmonics 1-4)", on))):
    for row, (nm, unit) in enumerate((("angle", "deg"), ("offset", "um"))):
        ax = axs[row, col]; q = d[nm]
        for p in q["prof"]: ax.plot(thb, p, color="gray", lw=0.6, alpha=0.6)
        ax.plot(thb, q["mean"], color="C3", lw=2, label="turn-mean (repeatable)")
        ax.set_title(f"{lab}: {nm} rms {q['total']:.3f} {unit} = repeatable {q['rep']:.3f} + non-repeatable {q['nonrep']:.3f}", fontsize=9)
        ax.set_ylabel(f"{nm} error [{unit}]"); ax.grid(alpha=0.3)
        if row == 1: ax.set_xlabel("rotation angle [deg]")
        if row == 0 and col == 0: ax.legend(fontsize=8)
fig.suptitle("kayEi on the tilted fiber (1.26 deg / 1.2 mm wobble) at 100 deg/s: per-turn error profiles after the first turn", fontsize=10)
fig.tight_layout(); fig.savefig(os.path.join(HERE, "docs", "figures", "kayei_error_composition.png"), dpi=110)
for lab, d in (("off", off), ("on", on)):
    print(lab, {k: (round(v["total"], 4), round(v["rep"], 4), round(v["nonrep"], 4)) for k, v in d.items()})

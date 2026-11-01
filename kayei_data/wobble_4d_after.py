"""4D (3D + time) view of the fiber as the laser would see it: uncontrolled vs with kayEi.
Left: the uncontrolled fiber axis (5-position map, sine model, lab frame, rotating).
Right: what remains with kayEi running - the measured in-plane residual r(theta, s) from a long
kayEi run with the scan sweeping 5..45 mm (binned 5 deg x 10 mm, turn-averaged), and the
out-of-plane (towards the camera = laser focus) component unchanged, because no axis corrects it.
Run: python kayei_data/wobble_4d_after.py <wobble_map.csv> <run_kayei_scan.csv> <tag>"""
import csv, os, sys
import numpy as np, cv2, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE); import fiber_align as fa
UM = 1000 / fa.PX_PER_MM
SCANS = np.array([5.0, 15.0, 25.0, 35.0, 45.0])
before_csv, after_csv, tag = sys.argv[1], sys.argv[2], (sys.argv[3] if len(sys.argv) > 3 else "after")

def fit_sine(th, y):
    A = np.c_[np.ones_like(th), np.cos(np.radians(th)), np.sin(np.radians(th))]
    return np.linalg.lstsq(A, y, rcond=None)[0]

# ---- before: sine model per scan position -> axis components (mm)
B = np.array([[float(v) for v in r] for r in csv.reader(open(before_csv)) if r[0] != "scan_mm"])
ex = np.zeros(len(SCANS)); ey = np.zeros(len(SCANS)); x0 = np.zeros(len(SCANS))
for i, s in enumerate(SCANS):
    sel = B[:, 0] == s; c = fit_sine(B[sel, 1], B[sel, 3])
    x0[i], ex[i], ey[i] = c[0] * UM / 1000, c[1] * UM / 1000, -c[2] * UM / 1000
def before_xy(th):                      # lab frame, um, per scan position
    c, sn = np.cos(np.radians(th)), np.sin(np.radians(th))
    return (ex * c - ey * sn) * 1000, (ex * sn + ey * c) * 1000

# ---- after: measured residual table r(theta, s) in um, after the first turn and 2 s
R = list(csv.DictReader(open(after_csv)))
t = np.array([float(r["t"]) for r in R]); th = np.array([float(r["theta_deg"]) for r in R]) % 360
s = np.array([float(r["scan_mm"]) for r in R]); x = np.array([float(r["x_px"]) if r["x_px"] else np.nan for r in R]) * UM
a = np.array([float(r["angle_deg"]) if r["angle_deg"] else np.nan for r in R])
ok = (t - t[0] > 2.0 + 360 / 100.0) & np.isfinite(x)
TH = np.arange(0, 360, 5.0); tab = np.full((len(TH), len(SCANS)), np.nan); n_tab = np.zeros_like(tab)
for i, th0 in enumerate(TH):
    for j, s0 in enumerate(SCANS):
        m = ok & (th >= th0) & (th < th0 + 5) & (np.abs(s - s0) <= 5.0)
        if m.sum() >= 3: tab[i, j] = x[m].mean(); n_tab[i, j] = m.sum()
for j in range(len(SCANS)):                 # fill the few empty cells by circular interpolation
    col = tab[:, j]; bad = np.isnan(col)
    if bad.any() and (~bad).sum() > 3: col[bad] = np.interp(TH[bad], TH[~bad], col[~bad], period=360)
rms_after = np.sqrt(np.nanmean(x[ok] ** 2)); rms_before = np.sqrt(np.mean(((ex[:, None] * np.cos(np.radians(TH)) - ey[:, None] * np.sin(np.radians(TH))) * 1000) ** 2))
print(f"before: in-plane offset rms over the turn and the 5 positions {rms_before:.0f} um; after: residual rms {rms_after:.1f} um (angle {np.sqrt(np.nanmean(a[ok]**2)):.4f} deg), {ok.sum()} frames, cells filled {int((n_tab > 0).sum())}/{tab.size}")

# ---- movie
W, H, FPS, N = 1400, 800, 25, 150
out = os.path.join(HERE, "docs", "figures", f"wobble_4d_{tag}.mp4")
vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
lim_b = max(np.abs(np.r_[ex, ey]).max() * 1000 * 1.3, 50); lim_a = max(np.nanmax(np.abs(tab)) * 1.3, 20)
for k in range(N):
    th_now = 360 * k / N
    Xb, Yb = before_xy(th_now)
    Xa = np.array([np.interp(th_now, np.r_[TH, 360], np.r_[tab[:, j], tab[0, j]]) for j in range(len(SCANS))])
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
    ax = fig.add_subplot(2, 2, 1, projection="3d")
    ax.plot([0, 0], [0, 0], [0, 50], "k--", lw=1); ax.plot(Xb, Yb, SCANS, "o-", color="C3", lw=2)
    ax.set_xlim(-lim_b, lim_b); ax.set_ylim(-lim_b, lim_b); ax.set_zlim(0, 50)
    ax.set_xlabel("in-plane [um]"); ax.set_ylabel("towards camera [um]"); ax.set_zlabel("along fiber [mm]")
    ax.set_title(f"uncontrolled, rotation {th_now:5.1f} deg", fontsize=10)
    ax2 = fig.add_subplot(2, 2, 2, projection="3d")
    ax2.plot([0, 0], [0, 0], [0, 50], "k--", lw=1); ax2.plot(Xa, Yb, SCANS, "o-", color="C2", lw=2)
    ax2.set_xlim(-lim_a, lim_a); ax2.set_ylim(-lim_b, lim_b); ax2.set_zlim(0, 50)
    ax2.set_xlabel(f"in-plane residual [um], x{lim_b / lim_a:.0f} zoom"); ax2.set_ylabel("towards camera [um]"); ax2.set_zlabel("along fiber [mm]")
    ax2.set_title("with kayEi (100 deg/s + scan); focus axis not corrected", fontsize=10)
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(SCANS, Xb, "o-", color="C3", label="in-plane (camera / laser sees it)"); ax3.plot(SCANS, Yb, "s--", color="C0", label="towards camera (focus)")
    ax3.set_ylim(-lim_b, lim_b); ax3.set_xlabel("along fiber [mm]"); ax3.set_ylabel("offset from target [um]"); ax3.grid(alpha=0.3); ax3.legend(fontsize=8, loc="upper right")
    ax3.set_title(f"uncontrolled: in-plane rms {rms_before:.0f} um", fontsize=10)
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(SCANS, Xa, "o-", color="C2", label="in-plane residual (measured)"); ax4.plot(SCANS, Yb * lim_a / lim_b, "s--", color="C0", alpha=0.4, label=f"towards camera, scaled 1/{lim_b / lim_a:.0f}")
    ax4.set_ylim(-lim_a, lim_a); ax4.set_xlabel("along fiber [mm]"); ax4.set_ylabel("residual [um]"); ax4.grid(alpha=0.3); ax4.legend(fontsize=8, loc="upper right")
    ax4.set_title(f"with kayEi: in-plane rms {rms_after:.0f} um ({rms_before / rms_after:.0f}x smaller)", fontsize=10)
    fig.suptitle("The fiber as the laser would see it over one turn: uncontrolled vs kayEi", fontsize=11)
    fig.tight_layout(); fig.canvas.draw()
    img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]; frame = cv2.cvtColor(cv2.resize(img, (W, H)), cv2.COLOR_RGB2BGR)
    vw.write(frame)
    if k == N // 4: cv2.imwrite(os.path.join(HERE, "docs", "figures", f"wobble_4d_{tag}_still.png"), frame)
    plt.close(fig)
vw.release(); print("wrote", out)

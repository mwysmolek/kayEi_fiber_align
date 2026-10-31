"""Z stage dynamics seen by the camera: step response and sine tracking (like the goniometer tests)."""
import sys, time; sys.path.insert(0, r"C:\dev\hydra")
import numpy as np, fiber_align as fa
from kayei_learn import Rig, step_response, analyse_step
from pipython import pitools
rig = Rig(); z = rig.z
print("Z VEL", z.qVEL()["1"], "ACC", z.qACC()["1"] if hasattr(z, "qACC") else "?")
z0 = rig.zpos()
for sign in (+1, -1, +1, -1):
    rows = step_response(rig, lambda: z.MOV("1", z0 + (sign > 0) * 0.2), "x")
    pitools.waitontarget(z)
    r = analyse_step(rows, 15)
    print(f"Z step {sign*0.2:+.1f} mm: image responds after {r['t_first']*1000:4.0f} ms, settled after {r['t_settle']*1000:4.0f} ms, image step {r['step']:+.1f} px")
rig.z_to(z0)
def fitlag(t, y, amp, period):
    return min(((np.sqrt(np.mean((y - amp * np.sin(2 * np.pi * (t - lag) / period)) ** 2)), lag) for lag in np.arange(-0.05, 0.5, 0.002)))
_, m0 = rig.fresh(); x0 = m0["x"]
amp = 0.45  # mm, ~ the offset wobble
for vel in (10.0, 20.0):
    z.VEL("1", vel)
    for period in (3.6, 1.8, 1.0):
        cam, t0, last = [], time.time(), -1
        while time.time() - t0 < 2 * period + 0.5:
            t = time.time() - t0
            if t - last >= 0.02:
                z.MOV("1", z0 + amp * np.sin(2 * np.pi * t / period)); last = t
            tf, m = rig.fresh()
            if m: cam.append((time.time() - t0, (m["x"] - x0) / -49.5))   # image offset expressed in mm of Z
        t = np.array([c[0] for c in cam]); y = np.array([c[1] for c in cam]); sel = t > period / 2
        rms, lag = fitlag(t[sel], y[sel], amp, period)
        print(f"VEL {vel:4.0f} period {period:3.1f} s (peak {amp*2*np.pi/period:.2f} mm/s): fiber amplitude {(y[sel].max()-y[sel].min())/2:.3f} of {amp} mm, lag {lag*1000:4.0f} ms, rms {rms*1000:.0f} um, {len(cam)/(2*period+0.5):.0f} fps")
        z.MOV("1", z0); pitools.waitontarget(z); time.sleep(0.3)
z.VEL("1", 10.0)
rig.close()

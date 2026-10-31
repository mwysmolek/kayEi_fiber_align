"""kayEi: predictive fiber alignment.

The fiber is a straight line, offset and tilted with respect to the rotation axis, so what
the camera sees is a sine in the rotation angle th whose offset coefficients change linearly
along the fiber (scan position s):
    angle(th)   = a0 + A1 cos th + A2 sin th
    x(th, s)    = b0 + b1 s + (B1 + C1 s) cos th + (B2 + C2 s) sin th
Everything is measured once (learning turns), then the goniometer and Z are driven to where
the fiber WILL be (rotation angle known in advance from the encoder + commanded speed).
A residual table indexed by th learns what the sine does not explain (bearing / chuck
signature), and a slow integral trim absorbs drift. Camera feedback never has to chase.
"""
import threading
import time

import numpy as np

import fiber_align as fa

FRAME_DELAY = 0.030    # s between the frame and the encoder reading taken after it: measured 30 ms at 20 ms
                       # exposure and 31 ms at 4 ms exposure (the exposure length does not enter)
GONIO_LEAD = 0.100     # s: command the goniometer for the angle this far ahead (58 ms first response + travel)
Z_LEAD = 0.035         # s: Z measured like the goniometer (58 ms response, 60-70 ms sine lag) -> same lead law
VSHAPE = True          # velocity-shaped streaming: each tick also sets the stage speed to the speed the
                       # trajectory needs (x margin); without it a stepper executes the 50 Hz target stream
                       # as stop-and-go bursts at full speed (measured 2026-10-31 evening, 1.26 deg wobble)
VSHAPE_MARGIN = 1.15
TRACK_RATE = 50.0      # Hz: target stream rate of the tracker thread
VSHAPE_VSTEP = 0.10    # resend the stage speed only when it changed by this fraction (command traffic)
SYNC_GAIN = 0.3        # rotation clock: fraction of an encoder phase correction applied per sync
SYNC_RTT_MAX = 0.012   # s: encoder queries slower than this (Hydra busy) are not used for the clock
SYNC_FULL_ABOVE = 1.5  # deg: larger corrections (start-up) are applied fully
GONIO_VMIN, GONIO_VMAX = 0.02, 4.0     # deg/s (goniometer stepper: 4 deg/s is the configured maximum)
Z_VMIN, Z_VMAX = 0.05, 10.0            # mm/s (VT-80 on the C-663)
ILC_HARMONICS = 4      # residual learner: Fourier harmonics of the rotation angle (3-jaw chuck -> 3rd)
ILC_CAP_G, ILC_CAP_Z = 0.5, 0.5   # deg, mm: cap per coefficient
ILC_GAIN = 0.3         # fraction of the seen residual written into the table per visit
TRIM_GAIN = 0.03       # integral trim per frame
FB_GAIN_Z = 0.4        # proportional camera feedback on the offset into the Z command (the goniometer's
                       # ragged motion leaks into the offset through its lever arm; Z can take it out)


class RotClock:
    """Rotation angle as a function of time from the commanded speed, re-synced to the encoder."""

    def __init__(self, speed):
        self.w = speed
        self.th_q, self.t_q = 0.0, time.time()
        self.synced, self.rejected, self.log = False, 0, []   # log: (t, correction deg, rtt, gain)

    def sync(self, theta, t=None, rtt=None, gain=SYNC_GAIN):
        """Encoder reading theta taken at time t (round trip rtt). While rotating, the reading only
        nudges the phase (gain) and slow queries are ignored: a query delayed by other Hydra traffic
        carries a timestamp error of tens of ms = degrees of rotation (seen as command spikes)."""
        t = time.time() if t is None else t
        if self.synced and self.w != 0.0:
            corr = theta - self.at(t)
            if rtt is not None and rtt > SYNC_RTT_MAX:
                self.rejected += 1
                self.log.append((t, corr, rtt, 0.0))
                return False
            if abs(corr) > SYNC_FULL_ABOVE:   # start-up (rotation still accelerating): take it fully
                gain = 1.0
            self.log.append((t, corr, rtt if rtt is not None else -1.0, gain))
            theta = self.at(t) + gain * corr
        self.th_q, self.t_q, self.synced = theta, t, True
        return True

    def at(self, t):
        return self.th_q + self.w * (t - self.t_q)


class WobbleModel:
    def __init__(self):
        self.ca = None   # [a0, a1, A1, A2, D1, D2]: a0 + a1 s + (A1 + D1 s) cos + (A2 + D2 s) sin
        self.cx = None   # [b0, b1, B1, B2, C1, C2]  (s-terms are 0 when learned at one scan position)
        self.rms = (np.nan, np.nan)

    @staticmethod
    def _basis(th, s, with_s):
        th = np.radians(np.asarray(th, float))
        s = np.asarray(s, float) * np.ones_like(th)
        cols = [np.ones_like(th), np.cos(th), np.sin(th)]
        if with_s:
            cols += [s, s * np.cos(th), s * np.sin(th)]
        return np.column_stack(cols)

    def fit(self, th, s, angle, x):
        th, s, angle, x = (np.asarray(v, float) for v in (th, s, angle, x))
        with_s = np.ptp(s) > 1.0
        A = self._basis(th, s, with_s)
        def solve(y):
            c = np.linalg.lstsq(A, y, rcond=None)[0]
            full = np.array([c[0], c[3] if with_s else 0, c[1], c[2], c[4] if with_s else 0, c[5] if with_s else 0])
            return full, float((y - A @ c).std())
        self.ca, ra = solve(angle)
        self.cx, rx = solve(x)
        self.rms = (ra, rx)
        return self

    def predict(self, th, s):
        """(angle deg, x px) the camera would see with goniometer / Z at their learning positions"""
        c, k = np.cos(np.radians(th)), np.sin(np.radians(th))
        a0, a1, A1, A2, D1, D2 = self.ca
        b0, b1, B1, B2, C1, C2 = self.cx
        return (a0 + a1 * s + (A1 + D1 * s) * c + (A2 + D2 * s) * k,
                b0 + b1 * s + (B1 + C1 * s) * c + (B2 + C2 * s) * k)

    def describe(self):
        amp_a = np.hypot(self.ca[2], self.ca[3])
        amp_x = np.hypot(self.cx[2], self.cx[3])
        return (f"angle {amp_a:.3f} deg sine (mean {self.ca[0]:+.3f}, {self.ca[1]:+.4f} deg/mm, amplitude change "
                f"{np.hypot(self.ca[4], self.ca[5]):.4f} deg/mm), offset {amp_x:.1f} px sine (mean {self.cx[0]:+.1f}, "
                f"{self.cx[1]:+.3f} px/mm, amplitude change {np.hypot(self.cx[4], self.cx[5]):.2f} px/mm); "
                f"fit residual {self.rms[0]:.4f} deg / {self.rms[1]:.2f} px")


class Residual:
    """Learned periodic correction (iterative learning) as a low-order Fourier series over the
    rotation angle: what the camera still saw at that angle, in goniometer deg and Z mm, applied on
    the next visits. Harmonics 1..ILC_HARMONICS only (no DC: the mean belongs to the trim), so the
    learner cannot wind up on measurement noise or on a stage at its speed limit (a 72-bin table did,
    2026-10-31 evening, once the stages were fast enough to follow it); amplitude capped."""

    def __init__(self, harmonics=ILC_HARMONICS, gain=ILC_GAIN):
        self.k = np.arange(1, harmonics + 1)
        self.cg = np.zeros(2 * harmonics)     # cos/sin coefficients, goniometer deg
        self.cz = np.zeros(2 * harmonics)     # Z mm
        self.gain, self.n_per_turn, self.n = gain, 240.0, 0   # gain = fraction of the error removed per turn

    def _phi(self, th):
        a = np.radians(th) * self.k
        return np.r_[np.cos(a), np.sin(a)]

    def update(self, th, dg, dz):
        """dg, dz: the correction (deg, mm) that WOULD have removed what was seen at th"""
        mu = self.gain * 2.0 / self.n_per_turn       # per-sample step so that one turn applies 'gain'
        phi = self._phi(th)
        self.cg += mu * dg * phi
        self.cz += mu * dz * phi
        np.clip(self.cg, -ILC_CAP_G, ILC_CAP_G, out=self.cg)
        np.clip(self.cz, -ILC_CAP_Z, ILC_CAP_Z, out=self.cz)
        self.n += 1

    def get(self, th):
        phi = self._phi(th)
        return float(self.cg @ phi), float(self.cz @ phi)

    def amplitudes(self):
        h = len(self.k)
        return np.hypot(self.cg[:h], self.cg[h:]), np.hypot(self.cz[:h], self.cz[h:])

    def describe(self):
        ag, az = self.amplitudes()
        return (f"residual (Fourier 1..{len(self.k)}, {self.n} updates): gonio " + "/".join(f"{v:.4f}" for v in ag)
                + " deg, Z " + "/".join(f"{v * 1000:.1f}" for v in az) + " um")



class Predictor:
    """Goniometer and Z commands for a rotation angle and scan position.
    ref: (g_al, z_al) where the fiber was aligned (angle 0, centred) at scan position s_ref;
    cal(s): image response to goniometer (a_per_g, x_per_g(s)) and Z (x_per_z)."""

    def __init__(self, model, g_al, z_al, a_per_g, x_per_z, x_per_g_at, residual=None):
        self.model, self.g_al, self.z_al = model, g_al, z_al
        self.a_g, self.x_z = a_per_g, x_per_z
        # x_per_g depends on the scan position (lever arm from the goniometer pivot): linear fit
        pts = sorted(x_per_g_at.items())
        if len(pts) >= 2:
            (s0, v0), (s1, v1) = pts[0], pts[-1]
            self.xg_slope, self.xg_0 = (v1 - v0) / (s1 - s0), v0 - (v1 - v0) / (s1 - s0) * s0
        else:
            self.xg_slope, self.xg_0 = 0.0, pts[0][1]
        self.res = residual or Residual()
        self.trim_g, self.trim_z = 0.0, 0.0
        self.fb_z = 0.0
        self._s = 0.0

    def x_per_g(self, s):
        return self.xg_0 + self.xg_slope * s

    def _dg(self, th):
        """goniometer offset from the aligned reference that makes the fiber straight at th"""
        ang, _ = self.model.predict(th, self._s)
        return -ang / self.a_g + self.res.get(th)[0] + self.trim_g

    def command(self, th_g, th_z, s):
        """goniometer target for rotation angle th_g, Z target for th_z (different leads).
        Z must also undo the image shift the goniometer tilt causes (lever arm x_per_g(s)),
        using the tilt that will be in effect when the Z target applies, i.e. _dg(th_z)."""
        self._s = s
        dg = self._dg(th_g)
        _, x = self.model.predict(th_z, s)
        dz = -(x + self.x_per_g(s) * self._dg(th_z)) / self.x_z + self.res.get(th_z)[1] + self.trim_z + self.fb_z
        return self.g_al + dg, self.z_al + dz

    def seen(self, th, angle, x, s):
        """camera still saw (angle, x) at rotation angle th: the corrections that would have
        removed it go into the residual table (shape) and the trim (mean)"""
        dg = -angle / self.a_g
        dz = -(x + self.x_per_g(s) * dg) / self.x_z   # the extra tilt dg shifts the image too
        self.res.update(th, dg, dz)
        self.trim_g += TRIM_GAIN * dg
        self.trim_z += TRIM_GAIN * dz
        self.fb_z = FB_GAIN_Z * dz


class Tracker(threading.Thread):
    """Streams goniometer (and Z) targets at a fixed rate, computed from the rotation clock and
    the predictor - independent of the camera loop, so the command timing is regular.
    Own Hydra socket (the Hydra accepts several); Z device handed over for the duration."""

    def __init__(self, predictor, clock, scan_at, hydra_ip, z_dev=None, rate=50.0,
                 gonio_lead=GONIO_LEAD, z_lead=Z_LEAD, gonio_axis=1):
        super().__init__(daemon=True)
        self.p, self.clk, self.scan_at = predictor, clock, scan_at
        self.hydra_ip, self.z, self.rate = hydra_ip, z_dev, rate
        self.gonio_lead, self.z_lead, self.gonio_axis = gonio_lead, z_lead, gonio_axis
        self.stop_evt = threading.Event()
        self.error = None
        self.n_cmd, self.n_tick, self.g_last, self.z_last = 0, 0, None, None
        self.period_max = 0.0
        self.scan_fixed = 1.0
        self.saturated = False     # a stage is at its speed limit (learning must pause: wind-up otherwise)
        self.ticks = []            # (t, g, z, vg, vz) per tick, for diagnosis

    def tick_extra(self, now):
        pass

    def run(self):
        from hydra import Hydra
        h = Hydra(self.hydra_ip)
        dt = 1.0 / self.rate
        t_next = time.perf_counter()
        t_prev = time.time()
        vg_last = vz_last = None
        errcheck = self.z.errcheck if self.z is not None else None
        if self.z is not None:
            self.z.errcheck = False     # a GCS command costs 1 ms without the error query, 16 ms with it
        try:
            while not self.stop_evt.is_set():
                now = time.time()
                self.period_max = max(self.period_max, now - t_prev) if self.n_tick > 5 else 0.0
                t_prev = now
                s = self.scan_at(now)
                g, z = self.p.command(self.clk.at(now + self.gonio_lead), self.clk.at(now + self.z_lead), s)
                if abs(g) > 5.1:
                    raise RuntimeError(f"goniometer would need {g:+.3f} deg")
                if self.z is not None and not 0.2 < z < 24.8:
                    raise RuntimeError(f"Z would need {z:.3f} mm")
                if VSHAPE:
                    # speed the trajectory needs over the next tick, with margin so the stage never falls behind
                    g2, z2 = self.p.command(self.clk.at(now + self.gonio_lead + dt), self.clk.at(now + self.z_lead + dt), s)
                    if self.clk.w == 0.0:   # pre-positioning (rotation not started): full speed
                        vg, vz = GONIO_VMAX, Z_VMAX
                    else:
                        # trajectory speed, or more when the target jumped (trim / residual update), so that
                        # the stage catches up within a tick instead of crawling behind (wind-up otherwise)
                        dg_ = max(abs(g2 - g), abs(g - self.g_last) if self.g_last is not None else 0.0)
                        dz_ = max(abs(z2 - z), abs(z - self.z_last) if self.z_last is not None else 0.0)
                        vg = min(max(dg_ / dt * VSHAPE_MARGIN, GONIO_VMIN), GONIO_VMAX)
                        vz = min(max(dz_ / dt * VSHAPE_MARGIN, Z_VMIN), Z_VMAX)
                        self.saturated = vg >= GONIO_VMAX or vz >= Z_VMAX
                        self.v_peak = max(getattr(self, "v_peak", (0.0, 0.0))[0], vg), max(getattr(self, "v_peak", (0.0, 0.0))[1], vz)
                    if vg_last is None or abs(vg - vg_last) > VSHAPE_VSTEP * vg_last:
                        h.set_velocity(self.gonio_axis, vg); vg_last = vg
                    if self.z is not None and (vz_last is None or abs(vz - vz_last) > VSHAPE_VSTEP * vz_last):
                        self.z.VEL("1", vz); vz_last = vz
                self.ticks.append((now, g, z, vg_last or 0.0, vz_last or 0.0))
                if self.g_last is None or abs(g - self.g_last) > 0.0003:
                    h.move_abs(self.gonio_axis, g); self.g_last = g; self.n_cmd += 1
                if self.z is not None and (self.z_last is None or abs(z - self.z_last) > 0.0003):
                    self.z.MOV("1", z); self.z_last = z
                self.tick_extra(now)
                self.n_tick += 1
                t_next += dt
                lag = t_next - time.perf_counter()
                if lag > 0:
                    time.sleep(lag)
                else:
                    t_next = time.perf_counter()  # fell behind: don't burst
        except Exception as e:
            self.error = e
        finally:
            try:
                h.set_velocity(self.gonio_axis, GONIO_VMAX)
                if self.z is not None:
                    self.z.VEL("1", Z_VMAX); self.z.errcheck = errcheck
                    err = self.z.qERR()
                    if err and self.error is None:
                        self.error = RuntimeError(f"Z controller error {err} during the run")
            finally:
                h.close()

    def stop(self):
        self.stop_evt.set()
        self.join(timeout=2)


def gonio_lead(rot_speed):
    """velocity-shaped streaming, sine tracking test 2026-10-31 evening (1.26 deg amplitude, camera
    as sensor): best lead 47 ms at 100 deg/s, 72 ms at 200 deg/s. (Plain streaming, small wobble:
    0.025 + 0.0001 * speed.)"""
    return 0.022 + 0.00025 * rot_speed


def z_lead(rot_speed):
    """same test for Z (1.2 mm amplitude): 49 ms at 100 deg/s, 58 ms at 200 deg/s"""
    return 0.040 + 0.00009 * rot_speed


class AxisTracker(Tracker):
    """Tracker that also owns the scan stage: fixed position, or a sweep lo <-> hi at v mm/s,
    so that all PI traffic happens in this one thread."""

    def __init__(self, *a, scan_dev=None, sweep=None, **k):
        super().__init__(*a, **k)
        self.scan_dev, self.sweep = scan_dev, sweep     # sweep = (lo, hi, v) or None
        self.scan_from = self.scan_to = None
        self.t_scan = None
        self.scan_at = self.scan_pos

    def scan_pos(self, t):
        """estimated scan position from the commanded sweep (no USB query)"""
        if not self.sweep or self.t_scan is None:
            return self.scan_fixed
        lo, hi, v = self.sweep
        d = min((t - self.t_scan) * v, abs(self.scan_to - self.scan_from))
        return self.scan_from + d * np.sign(self.scan_to - self.scan_from)

    def run(self):
        if self.scan_dev is not None:
            self.scan_fixed = self.scan_dev.qPOS()["1"]
        if self.sweep:
            lo, hi, v = self.sweep
            self.scan_dev.VEL("1", v)
            self.scan_from, self.scan_to = self.scan_fixed, hi
            self.scan_dev.MOV("1", hi); self.t_scan = time.time()
        try:
            super().run()
        finally:
            if self.sweep:
                self.scan_dev.HLT("1", noraise=True)

    def tick_extra(self, now):
        # turnaround on the commanded timing (travel / v + settle), no PI query: a qONT costs
        # ~32 ms and would stall the goniometer stream
        if self.sweep and now - self.t_scan > abs(self.scan_to - self.scan_from) / self.sweep[2] + 0.4:
            lo, hi, v = self.sweep
            self.scan_from, self.scan_to = self.scan_to, (lo if self.scan_to == hi else hi)
            self.scan_dev.MOV("1", self.scan_to); self.t_scan = time.time()

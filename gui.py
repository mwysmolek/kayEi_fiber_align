"""Fiber station GUI: live camera with fiber overlay + live charts, all 4 axes (jog / go to),
exposure, calibrate / align / rotation sequence / scan + rotate, LIVE tracking (rotation and
scan running, goniometer + Z correcting every frame) and MP4 recording of the screen view.
Run:  C:\\dev\\hydra\\.venv\\Scripts\\python C:\\dev\\hydra\\gui.py
All hardware is driven from ONE worker thread (Hydra socket and PI DLL are not shared between
threads); buttons queue commands for it. Stop sets an abort flag (checked between alignment
steps / every live frame) and sends nabort to both Hydra axes over a separate connection."""
import collections
import csv
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
from PIL import Image, ImageTk
from pipython import GCSDevice, pitools

import fiber_align as fa
import kayei
from hydra import Hydra

SCAN_SN = "0165500183"
VIEW_W = 880                 # displayed / recorded image width
CHART_H = 64                 # height of each live chart strip
CHART_S = 30.0               # seconds shown in the charts
REC_FPS = 20                 # recorded movie runs in real time at this frame rate
LOG_FILE = __file__.rsplit("\\", 1)[0] + "\\gui_log.txt"
DATA_DIR = __file__.rsplit("\\", 1)[0] + "\\kayei_data"
LEARN_SPEED = 90.0          # deg/s for the kayEi learning turns
START_EXPOSURE_MS = fa.EXPOSURE_MS   # one place for the exposure: fiber_align.EXPOSURE_MS (kayei.FRAME_DELAY depends on it)
TRACK_MAX_FIT_RMS = 1.5      # px; frames with a worse straight-line fit are not used for steering
TRACK_GAIN = 0.6             # fraction of the measured error corrected per frame (live mode)
TRACK_TOL_ANGLE = 0.005      # deg, dead band in live mode
TRACK_TOL_PX = 0.3
# name: (unit, default jog step, min, max)
AXES = {"rotation": ("deg", 10.0, -36000, 35700), "goniometer": ("deg", 0.1, -5.1, 5.1),
        "z": ("mm", 0.1, 0.0, 25.0), "scan": ("mm", 1.0, 0.0, 50.0)}


class Aborted(Exception):
    pass


class Station(fa.Rig):
    """fa.Rig (camera, Hydra, Z) + the scan stage; every frame grabbed is published to the GUI."""

    def __init__(self, publish):
        super().__init__()
        self.publish = publish
        self.scan = GCSDevice()
        self.scan.ConnectUSB(SCAN_SN)
        if not self.scan.qFRF()["1"]:
            pitools.startup(self.scan, refmodes="FNL")

    def frame(self):
        f = super().frame()
        self.publish(f)
        return f

    def grab1(self, fresh=False):
        if fresh:
            self.cam.ClearImages()  # drop buffered frames -> the next one is exposed after now
        img = self.cam.GetImage(2000)
        return None if img.IsEmpty() else img.GetNPArray().squeeze().copy()

    def scan_to(self, pos):
        self.scan.MOV("1", pos); pitools.waitontarget(self.scan); time.sleep(0.2)

    def move(self, axis, pos):
        {"rotation": lambda p: self.hydra_to(fa.ROT, p), "goniometer": lambda p: self.hydra_to(fa.GONIO, p),
         "z": self.z_to, "scan": self.scan_to}[axis](pos)

    def positions(self):
        return {"rotation": self.h.position(fa.ROT), "goniometer": self.h.position(fa.GONIO),
                "z": self.zpos(), "scan": self.scan.qPOS()["1"]}

    def close(self):
        self.scan.CloseConnection()
        super().close()


class Worker(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.cmds = queue.Queue()
        self.logq = queue.Queue()
        self.results = queue.Queue()
        self.abort = threading.Event()
        self.quit = threading.Event()
        self.lock = threading.Lock()
        self.state = {"pos": {}, "busy": "connecting...", "mode": ""}
        self.hist = collections.deque(maxlen=4000)  # (t, angle, x_px, gonio, z)
        self.rig, self.cal, self.auto, self.exp_ms = None, None, False, 0.0
        self.live_pos = None  # set by do_live: positions from commands instead of slow queries
        self.kayei = None     # kayei.Predictor once learned
        self.kayei_ilc = True
        self.run_rows = []    # per-frame log of the current live run
        self.fps, self.t_last, self.t_pos = 0.0, time.time(), 0.0

    # --- called from the GUI thread ---
    def submit(self, name, fn):
        self.cmds.put((name, fn))

    def stop(self):
        self.abort.set()
        try:
            h = Hydra(fa.HYDRA_IP)
            for ax in (fa.GONIO, fa.ROT):
                h.stop(ax)
            h.close()
        except OSError as e:
            self.log(f"stop: Hydra not reachable ({e})")

    # --- worker thread ---
    def log(self, msg):
        line = time.strftime("%H:%M:%S  ") + msg
        self.logq.put(line)
        with open(LOG_FILE, "a") as fh:
            fh.write(time.strftime("%Y-%m-%d ") + line + "\n")

    def check_drives(self):
        """Hydra switches a drive off on a fault (status bit 0x100) - stop instead of carrying on"""
        for name, ax in (("goniometer", fa.GONIO), ("rotation", fa.ROT)):
            st = self.rig.h.status(ax)
            if st & 0x100:
                raise RuntimeError(f"{name} drive switched OFF (status {st:#x}, Hydra error {self.rig.h.query('ge')}); "
                                   f"press Calibrate/Align only after re-init + re-home")

    def check(self, msg):
        """log callback for fa.align; also the place where Stop takes effect"""
        self.log(msg.strip())
        if self.abort.is_set():
            raise Aborted

    def publish(self, f, m=None):
        if m is None:
            m = fa.measure(f)
        if m:
            xc = int(m["slope"] * f.shape[0] / 2 + m["icpt"])
            roi = f[:, max(xc - 100, 0):xc + 100]
        else:
            roi = f
        sharp = cv2.Laplacian(roi, cv2.CV_64F).var()
        now = time.time()
        self.fps = 0.9 * self.fps + 0.1 / max(now - self.t_last, 1e-3)
        self.t_last = now
        if self.auto:
            self.exp_ms = self.rig.cam.f.ExposureTime.Get() / 1000
        upd = {"frame": f, "m": m, "sharp": sharp, "fps": self.fps, "exp": self.exp_ms, "auto": self.auto}
        if self.live_pos is not None:
            upd["pos"] = dict(self.live_pos)
        elif now - self.t_pos > 0.2:
            upd["pos"] = self.rig.positions()
            upd["drives_off"] = [n for n, ax in (("goniometer", fa.GONIO), ("rotation", fa.ROT))
                                 if self.rig.h.status(ax) & 0x100]
            self.t_pos = now
        with self.lock:
            self.state.update(upd)
            p = self.state["pos"]
            if m and p:
                self.hist.append((now, m["angle"], m["x"], p["goniometer"], p["z"]))

    def set_mode(self, busy=None, mode=None):
        with self.lock:
            if busy is not None:
                self.state["busy"] = busy
            if mode is not None:
                self.state["mode"] = mode

    def run(self):
        try:
            self.rig = Station(self.publish)
            self.do_exposure(ms=START_EXPOSURE_MS, auto=False)
            self.auto_gain()
        except Exception as e:
            self.log(f"connect FAILED: {e!r}")
            self.set_mode(busy="not connected")
            return
        self.log("connected: camera, Hydra (gonio + rotation), PI Z + scan")
        self.set_mode(busy="")
        while not self.quit.is_set():
            try:
                name, fn = self.cmds.get_nowait()
            except queue.Empty:
                try:
                    f = self.rig.grab1()
                    if f is not None:
                        self.publish(f)
                except Exception as e:  # never let the live view die
                    self.log(f"live view: {e!r}")
                    time.sleep(0.5)
                continue
            self.abort.clear()
            self.set_mode(busy=name)
            try:
                fn()
            except Aborted:
                self.log(f"{name}: stopped")
            except Exception as e:
                self.log(f"{name}: ERROR {e}")
            self.set_mode(busy="", mode="")
            self.t_pos = 0
        self.rig.close()

    # --- commands (run in the worker thread) ---
    def do_move(self, axis, pos, relative=False):
        if relative:
            pos += self.rig.positions()[axis]
        lo, hi = AXES[axis][2:]
        if not lo <= pos <= hi:
            raise ValueError(f"{axis} {pos:.4f} outside {lo}..{hi}")
        self.rig.move(axis, pos)

    def do_exposure(self, ms=None, auto=None):
        if auto is not None:
            self.auto = auto
            self.rig.cam.GetFeature("ExposureAuto").SetString("Continuous" if auto else "Off")
        if ms is not None:
            e = self.rig.cam.f.ExposureTime
            e.Set(min(max(ms * 1000, e.GetMin()), e.GetMax()))
        self.exp_ms = self.rig.cam.f.ExposureTime.Get() / 1000

    def do_reset_drives(self):
        h = self.rig.h
        for name, ax in (("goniometer", fa.GONIO), ("rotation", fa.ROT)):
            if not h.powered(ax):
                h.init(ax)
                time.sleep(2)
                self.log(f"{name}: drive powered again (status {h.status(ax):#x})")
            else:
                self.log(f"{name}: drive on (status {h.status(ax):#x})")
        self.log(f"Hydra last error: {h.query('ge')}")

    def do_home_rotation(self):
        h = self.rig.h
        if not h.powered(fa.ROT):
            h.init(fa.ROT)
            time.sleep(2)
        self.rig.hydra_to(fa.ROT, h.position(fa.ROT) - 30)  # search the index in + direction, as always
        h.find_index(fa.ROT); time.sleep(0.3); h.wait(fa.ROT)
        idx = h.position(fa.ROT)
        h.set_position(fa.ROT, 0)
        self.log(f"rotation homed: index found at {idx:.4f} deg (old count), now 0")

    def do_home_gonio(self):
        h = self.rig.h
        if not h.powered(fa.GONIO):
            h.init(fa.GONIO)
            time.sleep(2)
        h.calibrate(fa.GONIO); time.sleep(0.2); h.wait(fa.GONIO)
        h.range_measure(fa.GONIO); time.sleep(0.2); h.wait(fa.GONIO)
        lo, hi = map(float, h.limits(fa.GONIO).split())
        self.rig.hydra_to(fa.GONIO, (lo + hi) / 2)
        h.set_position(fa.GONIO, 0)
        self.log(f"goniometer homed on its switches, range {hi - lo:.4f} deg, 0 = centre")

    def do_home_all(self):
        self.do_home_gonio()
        self.do_home_rotation()

    def do_ref_pi(self):
        for name, pi in (("z", self.rig.z), ("scan", self.rig.scan)):
            back = pi.qPOS()["1"]
            pi.FNL("1")
            pitools.waitonreferencing(pi)
            pi.MOV("1", back); pitools.waitontarget(pi)
            self.log(f"{name}: referenced on negative limit, back at {pi.qPOS()['1']:.4f} mm")

    def auto_gain(self):
        """keep the 20 ms frame; raise the camera gain until the fiber is detected (illumination varies)"""
        gain = self.rig.cam.f.Gain
        for g in (1.0, 2.0, 4.0, 6.0, 9.0):
            gain.Set(g)
            f = None
            for _ in range(3):
                f = self.rig.grab1()
            if f is not None and fa.measure(f) is not None:
                break
        self.log(f"camera gain {gain.Get():g}" + ("" if f is not None and fa.measure(f) else " - fiber NOT found"))

    def do_calibrate(self):
        m0, self.cal = fa.calibrate(self.rig)
        self.log("calibration: " + ", ".join(f"{k} {v:+.4f}" for k, v in self.cal.items()))

    def do_align(self):
        if self.cal is None:
            self.do_calibrate()
        t = time.time()
        m, _, its = fa.align(self.rig, self.cal, log=self.check)
        self.log(f"aligned in {its} it, {time.time() - t:.1f} s: angle {m['angle']:+.4f} deg, x {m['x']:+.2f} px")
        return m, its, time.time() - t

    def do_sequence(self, n, scan_positions=(None,)):
        """for each scan position (None = stay): rotate through n angles, align at each"""
        if self.cal is None:
            self.do_calibrate()
        for s in scan_positions:
            if s is not None:
                self.log(f"scan -> {s:.3f} mm")
                self.rig.scan_to(s)
            for k in range(n):
                if self.abort.is_set():
                    raise Aborted
                a = 360.0 * k / n
                self.log(f"rotation {a:.1f} deg")
                self.rig.hydra_to(fa.ROT, a)
                m, its, dt = self.do_align()
                p = self.rig.positions()
                self.results.put((p["scan"], a, p["goniometer"], p["z"], m["angle"], m["x"], its, dt))
        self.rig.hydra_to(fa.ROT, 0.0)
        self.log("sequence done, rotation back at 0")

    def do_live(self, rot_speed, scan=None, compensate=True, mode_name=None):
        if mode_name == "kayei":
            return self.do_kayei(rot_speed, scan)
        """Rotation turns continuously, scan sweeps lo<->hi; every fresh frame the goniometer
        removes the fiber angle and Z the offset (proportional, dead band, no waiting).
        compensate=False: align once, then only rotate + scan and watch the fiber wander."""
        rig, h = self.rig, self.rig.h
        if self.cal is None:
            self.do_calibrate()
        self.do_align()
        mode = "LIVE align" if compensate else "AS IS (no compensation)"
        t_start = time.time()
        a_g, x_z = self.cal["a_per_g"], self.cal["x_per_z"]
        v_rot, v_scan = h.velocity(fa.ROT), rig.scan.qVEL()["1"]
        lost, n, t0, t_gonio, t_rot = 0, 0, time.time(), 0.0, 0.0
        # PI queries over USB take ~32 ms and a MOV ~16 ms, so the loop works with COMMANDED
        # Z / goniometer positions (open-loop stepper / fast servo) and only polls what it must
        p = rig.positions()
        g_cmd, z_cmd = p["goniometer"], p["z"]
        self.set_mode(mode=mode)
        self.run_rows = []
        try:
            h.rezero_turn(fa.ROT); time.sleep(0.1)
            h.set_velocity(fa.ROT, rot_speed)
            h.move_abs(fa.ROT, AXES["rotation"][3])  # far away: keeps turning until stopped
            if scan:
                lo, hi, v = scan
                rig.scan.VEL("1", v)
                scan_target, scan_from = hi, p["scan"]
                rig.scan.MOV("1", scan_target)
                t_scan = time.time()
            self.log(f"{mode}: rotation {rot_speed} deg/s" + (f", scan {lo}..{hi} mm at {v} mm/s" if scan else ""))
            while not self.abort.is_set():
                f = rig.grab1(fresh=True)
                if f is None:
                    continue
                now = time.time()
                if now - t_rot > 0.2:
                    p["rotation"] = h.position(fa.ROT)
                    self.check_drives()
                    t_rot = now
                if scan:  # estimated from the commanded sweep (no USB query)
                    s = min((now - t_scan) * v, abs(scan_target - scan_from))
                    p["scan"] = scan_from + s * np.sign(scan_target - scan_from)
                p["goniometer"], p["z"] = g_cmd, z_cmd
                self.live_pos = p
                m = fa.measure(f)
                self.publish(f, m)
                n += 1
                self.run_rows.append((now, p["rotation"] % 360, p["scan"], m["angle"] if m else "", m["x"] if m else "", g_cmd, z_cmd))
                if not compensate:
                    continue  # just watch (fiber may even leave the image)
                if m is None or m["rms"] > TRACK_MAX_FIT_RMS:
                    lost += 1
                    if lost > 20:
                        raise RuntimeError("fiber lost from the image")
                    continue
                lost = 0
                # hold-off: right after a command the Hydra may not report "moving" yet
                if (abs(m["angle"]) > TRACK_TOL_ANGLE and now - t_gonio > 0.1
                        and not h.is_moving(fa.GONIO)):
                    g_cmd -= TRACK_GAIN * m["angle"] / a_g
                    if abs(g_cmd) > 5.1:
                        raise RuntimeError(f"goniometer would need {g_cmd:+.3f} deg")
                    h.move_abs(fa.GONIO, g_cmd)
                    t_gonio = time.time()
                if abs(m["x"]) > TRACK_TOL_PX:
                    z_cmd -= TRACK_GAIN * m["x"] / x_z
                    if not 0.2 < z_cmd < 24.8:
                        raise RuntimeError(f"Z would need {z_cmd:.3f} mm")
                    rig.z.MOV("1", z_cmd)  # PI retargets a running move
                if scan and now - t_scan > abs(scan_target - scan_from) / v + 0.3 and rig.scan.qONT()["1"]:
                    scan_from, scan_target = scan_target, (lo if scan_target == hi else hi)
                    rig.scan.MOV("1", scan_target)
                    t_scan = time.time()
            raise Aborted
        finally:
            self.live_pos = None
            h.stop(fa.ROT)
            rig.scan.HLT("1", noraise=True)
            time.sleep(0.5)
            rig.scan.qERR()  # clear "stopped by HLT"
            h.set_velocity(fa.ROT, v_rot)
            rig.scan.VEL("1", v_scan)
            dt = time.time() - t0
            self.log(f"{mode} ended after {dt:.1f} s, {n} frames ({n / max(dt, 1e-3):.1f}/s), "
                     f"rotation at {h.position(fa.ROT):.1f} deg")

            self.summarize_run(mode, t_start, x_z, rot_speed, scan)

    def summarize_run(self, mode, t_start, x_z, rot_speed, scan):
        with self.lock:
            run = [r for r in self.hist if r[0] >= t_start]
        if run:
            a = np.array([r[1] for r in run])
            x = np.array([r[2] for r in run]) * 1000 / abs(x_z)
            self.log(f"{mode} errors: angle rms {np.sqrt(np.mean(a ** 2)):.4f} max {np.abs(a).max():.4f} deg | "
                     f"offset rms {np.sqrt(np.mean(x ** 2)):.1f} max {np.abs(x).max():.1f} um")
        if self.run_rows:
            import csv, os
            os.makedirs(DATA_DIR, exist_ok=True)
            tag = mode.split()[0].lower().replace("(", "")
            path = os.path.join(DATA_DIR, time.strftime(f"run_%Y%m%d_%H%M%S_{tag}_{rot_speed:.0f}dps{'_scan' if scan else ''}.csv"))
            with open(path, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t", "theta_deg", "scan_mm", "angle_deg", "x_px", "g_cmd_deg", "z_cmd_mm"])
                w.writerows(self.run_rows)
            self.log(f"run logged: {os.path.basename(path)} ({len(self.run_rows)} frames)")
            self.run_rows = []

    # ---------------------------------------------------------------- kayEi
    def learn_turns(self, clk_speed, n_turns, s):
        """n full turns with goniometer and Z fixed; returns (theta, scan, angle, x) samples
        (theta absolute mod 360 - rezero_turn keeps the count consistent with the encoder index)"""
        rig, h = self.rig, self.rig.h
        h.rezero_turn(fa.ROT); time.sleep(0.1)
        clk = kayei.RotClock(clk_speed)
        th0 = h.position(fa.ROT); clk.sync(th0)
        v0 = h.velocity(fa.ROT); h.set_velocity(fa.ROT, clk_speed)
        h.move_abs(fa.ROT, th0 + 360 * n_turns + 30)
        out, t_sync, t_start = [], time.time(), time.time()
        try:
            while True:
                f = rig.grab1(fresh=True)
                now = time.time()
                if f is None:
                    continue
                if now - t_sync > 0.25:
                    tq = time.time(); thq = h.position(fa.ROT); t1 = time.time(); clk.sync(thq, (tq + t1) / 2, rtt=t1 - tq); t_sync = t1
                    if thq - th0 < 10 and now - t_start > 2.0:
                        raise RuntimeError(f"rotation not moving (Hydra error {h.query('ge')})")
                    self.check_drives()
                if self.abort.is_set():
                    raise Aborted
                th_f = clk.at(now - kayei.FRAME_DELAY)
                m = fa.measure(f)
                self.live_pos = {"rotation": th_f, "goniometer": self._g_fixed, "z": self._z_fixed, "scan": s}
                self.publish(f, m)
                if m and m["rms"] <= TRACK_MAX_FIT_RMS:
                    out.append((th_f % 360, s, m["angle"], m["x"]))
                if th_f - th0 >= 360 * n_turns:
                    break
        finally:
            h.stop(fa.ROT); time.sleep(0.3); h.set_velocity(fa.ROT, v0)
            self.live_pos = None
        return out

    def do_kayei_learn(self, scan=None):
        """Calibrate + align at the current scan position, then 2 learning turns there (and at the
        other end of the scan range if scanning), fit the wobble model, build the predictor."""
        rig = self.rig
        self.do_calibrate()
        cal0 = dict(self.cal)
        self.do_align()
        p = rig.positions()
        g_al, z_al, s0 = p["goniometer"], p["z"], p["scan"]
        self._g_fixed, self._z_fixed = g_al, z_al
        self.log(f"kayEi learn: reference gonio {g_al:+.4f} z {z_al:.4f} at scan {s0:.2f}; 2 turns at {LEARN_SPEED:.0f} deg/s")
        samples = self.learn_turns(LEARN_SPEED, 2, s0)
        x_per_g_at = {s0: cal0["x_per_g"]}
        a_g, x_z = [cal0["a_per_g"]], [cal0["x_per_z"]]
        if scan:
            lo, hi, v = scan
            s1 = hi if abs(hi - s0) > abs(lo - s0) else lo
            self.log(f"kayEi learn: scan -> {s1:.2f} mm, calibrate + 2 turns there")
            rig.scan_to(s1)
            self.do_calibrate()
            x_per_g_at[s1] = self.cal["x_per_g"]; a_g.append(self.cal["a_per_g"]); x_z.append(self.cal["x_per_z"])
            rig.hydra_to(fa.GONIO, g_al); rig.z_to(z_al)
            samples += self.learn_turns(LEARN_SPEED, 2, s1)
            rig.scan_to(s0)
        th, ss, ang, x = (np.array([r[i] for r in samples]) for i in range(4))
        import csv, os
        os.makedirs(DATA_DIR, exist_ok=True)
        with open(os.path.join(DATA_DIR, time.strftime("learn_%Y%m%d_%H%M%S.csv")), "w", newline="") as fh:
            w = csv.writer(fh); w.writerow(["theta_deg", "scan_mm", "angle_deg", "x_px"]); w.writerows(samples)
        model = kayei.WobbleModel().fit(th, ss, ang, x)
        self.kayei = kayei.Predictor(model, g_al, z_al, float(np.mean(a_g)), float(np.mean(x_z)), x_per_g_at)
        self.log("kayEi model: " + model.describe())
        self.log(f"kayEi gonio lever arm: x_per_g {self.kayei.xg_0:+.1f} {self.kayei.xg_slope:+.2f}*s px/deg")

    def do_kayei(self, rot_speed, scan=None):
        """Predictive mode: tracker thread streams goniometer / Z (and sweeps the scan) from the
        rotation clock and the learned model; the camera loop measures, learns the residual,
        trims drift and re-syncs the clock."""
        rig, h = self.rig, self.rig.h
        if self.kayei is None:
            self.do_kayei_learn(scan)
        pred = self.kayei
        mode = "kayEi"
        self.set_mode(mode=mode)
        t_start = time.time()
        self.run_rows = []
        h.rezero_turn(fa.ROT); time.sleep(0.1)
        pred.res.n_per_turn = max(30.0, 65.0 * 360.0 / rot_speed)   # frames per turn (camera ~65 fps)
        clk = kayei.RotClock(rot_speed)
        th0 = h.position(fa.ROT); clk.sync(th0)
        v_rot, v_scan = h.velocity(fa.ROT), rig.scan.qVEL()["1"]
        tr = kayei.AxisTracker(pred, clk, None, fa.HYDRA_IP, rig.z, rate=kayei.TRACK_RATE,
                               gonio_lead=kayei.gonio_lead(rot_speed), z_lead=kayei.z_lead(rot_speed), gonio_axis=fa.GONIO,
                               scan_dev=rig.scan, sweep=scan)
        n, lost, t_sync = 0, 0, time.time()
        try:
            # pre-position: tracker first (clock stands still -> targets for the current angle),
            # let goniometer and Z settle, THEN start the rotation. No start transient for the
            # residual table to learn; learning is enabled only after the first full turn anyway.
            clk.w = 0.0
            tr.start()
            time.sleep(0.6)
            t_wait = time.time()
            while not (rig.z.qONT()["1"] and not h.is_moving(fa.GONIO)) and time.time() - t_wait < 5:
                time.sleep(0.1)       # Z may have a long way to go after a previous run
            clk.w = rot_speed
            clk.sync(h.position(fa.ROT))
            h.set_velocity(fa.ROT, rot_speed)
            h.move_abs(fa.ROT, AXES["rotation"][3])
            t_rot = time.time()
            th_learn_from = th0 + 360.0
            self.log(f"{mode}: rotation {rot_speed} deg/s, leads gonio {kayei.gonio_lead(rot_speed) * 1000:.0f} / Z {kayei.z_lead(rot_speed) * 1000:.0f} ms{' (velocity-shaped)' if kayei.VSHAPE else ''}"
                     + (f", scan {scan[0]}..{scan[1]} mm at {scan[2]} mm/s" if scan else "")
                     + f", residual learning {'on' if self.kayei_ilc else 'off'}")
            while not self.abort.is_set():
                f = rig.grab1(fresh=True)
                now = time.time()
                if f is None:
                    continue
                if tr.error:
                    raise tr.error
                if now - t_sync > 0.25:
                    tq = time.time(); thq = h.position(fa.ROT); t1 = time.time(); clk.sync(thq, (tq + t1) / 2, rtt=t1 - tq); t_sync = t1
                    if thq - th0 < 10 and now - t_rot > 3.0:
                        raise RuntimeError(f"rotation not moving (Hydra error {h.query('ge')})")
                    self.check_drives()
                th_f = clk.at(now - kayei.FRAME_DELAY)
                s = tr.scan_pos(now) if scan else tr.scan_fixed
                self.live_pos = {"rotation": th_f, "goniometer": tr.g_last if tr.g_last is not None else pred.g_al,
                                 "z": tr.z_last if tr.z_last is not None else pred.z_al, "scan": s}
                m = fa.measure(f)
                self.publish(f, m)
                n += 1
                ok = m is not None and m["rms"] <= TRACK_MAX_FIT_RMS
                self.run_rows.append((now, th_f % 360, s, m["angle"] if m else "", m["x"] if m else "",
                                      self.live_pos["goniometer"], self.live_pos["z"]))
                if not ok:
                    lost += 1
                    if lost > 20:
                        raise RuntimeError("fiber lost from the image")
                    continue
                lost = 0
                if self.kayei_ilc and th_f >= th_learn_from and not tr.saturated:
                    pred.seen(th_f, m["angle"], m["x"], s)
            raise Aborted
        finally:
            tr.stop()
            self.live_pos = None
            h.stop(fa.ROT)
            time.sleep(0.5)
            rig.scan.qERR()
            h.set_velocity(fa.ROT, v_rot)
            rig.scan.VEL("1", v_scan)
            dt = time.time() - t_start
            self.log(f"{mode} ended after {dt:.1f} s, {n} frames ({n / max(dt, 1e-3):.1f}/s); tracker {tr.n_tick} ticks, "
                     f"{tr.n_cmd} gonio cmds, worst gap {tr.period_max * 1000:.0f} ms, {clk.rejected} slow syncs ignored, peak speeds {getattr(tr, 'v_peak', (0, 0))[0]:.2f} deg/s / {getattr(tr, 'v_peak', (0, 0))[1]:.2f} mm/s; trim gonio {pred.trim_g:+.4f} deg "
                     f"z {pred.trim_z * 1000:+.1f} um; {pred.res.describe()}")
            if clk.log:
                cl = np.array(clk.log)
                self.log(f"{mode} clock: {len(cl)} syncs, correction rms {cl[:, 1].std():.2f} deg, max {np.abs(cl[:, 1]).max():.2f} deg, "
                         f"{(np.abs(cl[:, 1]) > 1.0).sum()} above 1 deg, rtt median {np.median(cl[:, 2]) * 1000:.1f} ms")
            stamp = time.strftime("%Y%m%d_%H%M%S")
            with open(os.path.join(DATA_DIR, f"ticks_{stamp}_{rot_speed:.0f}dps.csv"), "w", newline="") as fh:
                csv.writer(fh).writerows([("t", "g_cmd", "z_cmd", "v_g", "v_z")] + list(tr.ticks))
            with open(os.path.join(DATA_DIR, f"syncs_{stamp}_{rot_speed:.0f}dps.csv"), "w", newline="") as fh:
                csv.writer(fh).writerows([("t", "corr_deg", "rtt_s", "gain")] + list(clk.log))
            self.summarize_run(mode, t_start, pred.x_z, rot_speed, scan)


def draw_chart(hist, now, w, cal):
    """three strips: fiber angle error, centre offset, goniometer position (last CHART_S s)"""
    um = 1000 / abs(cal["x_per_z"]) if cal else 1.0
    strips = [("fiber angle", "deg", 1, 1.0, 0.1, fa.TOL_ANGLE),
              ("centre offset", "um" if cal else "px", 2, um, 50.0 if cal else 2.5, fa.TOL_PX * um),
              ("goniometer", "deg", 3, 1.0, None, None)]
    img = np.full((CHART_H * len(strips), w, 3), 22, np.uint8)
    pts = [r for r in hist if now - r[0] <= CHART_S]
    x0 = 150
    for i, (name, unit, col, k, span, tol) in enumerate(strips):
        top = i * CHART_H
        cv2.line(img, (0, top), (w, top), (60, 60, 60), 1)
        vals = np.array([r[col] * k for r in pts]) if pts else np.zeros(0)
        if span is None:  # autoscale around the data
            c = (vals.max() + vals.min()) / 2 if len(vals) else 0.0
            half = max((vals.max() - vals.min()) / 2 * 1.2, 0.05) if len(vals) else 0.05
        else:  # fixed tolerance view, widened when the data go beyond it (e.g. AS IS runs)
            c = 0.0
            half = max(span, 1.15 * np.abs(vals).max()) if len(vals) else span
        ymap = lambda v: int(top + CHART_H / 2 - (v - c) / half * (CHART_H / 2 - 4))
        if tol:
            cv2.rectangle(img, (x0, ymap(tol)), (w, ymap(-tol)), (40, 70, 40), -1)
        cv2.line(img, (x0, ymap(c)), (w, ymap(c)), (80, 80, 80), 1)
        if len(vals) > 1:
            xs = x0 + (np.array([r[0] for r in pts]) - (now - CHART_S)) / CHART_S * (w - x0)
            ys = np.clip([ymap(v) for v in vals], top + 1, top + CHART_H - 2)
            color = ((80, 200, 255), (255, 200, 60), (120, 255, 120))[i]
            cv2.polylines(img, [np.stack([xs, ys], 1).astype(np.int32)], False, color, 2, cv2.LINE_AA)
        last = f"{vals[-1]:+.3f}" if len(vals) else "-"
        cv2.putText(img, name, (8, top + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(img, f"{last} {unit}", (8, top + 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        rng = f"+/-{half:.3g}" if span else f"{c - half:+.3f}..{c + half:+.3f}"
        cv2.putText(img, rng, (w - 110, top + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1, cv2.LINE_AA)
    return img


def text(img, s, org, scale=0.6, color=(255, 255, 255)):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def compose(f, st, hist, cal, rec=False):
    """The GUI's view: image with fiber overlay, heads-up display, live charts. Used by the
    GUI and by the headless recorder (kayei_record.py), so movies look the same either way."""
    h, w = f.shape
    view = cv2.cvtColor(f, cv2.COLOR_GRAY2BGR)
    m = st["m"]
    um_px = 1000 / abs(cal["x_per_z"]) if cal else None
    cv2.line(view, (w // 2, 0), (w // 2, h), (60, 60, 255), 2)
    if m:
        for off, col in ((0, (0, 255, 0)), (-m["width"] / 2, (0, 200, 255)), (m["width"] / 2, (0, 200, 255))):
            x0, x1 = m["icpt"] + off, m["slope"] * h + m["icpt"] + off
            cv2.line(view, (int(round(x0)), 0), (int(round(x1)), h), col, 2)
    view = cv2.resize(view, (VIEW_W, int(h * VIEW_W / w)), interpolation=cv2.INTER_AREA)
    p = st.get("pos", {})
    if p:
        text(view, f"ROT {p['rotation'] % 360:7.2f} deg   GONIO {p['goniometer']:+7.4f} deg   "
                   f"Z {p['z']:7.4f} mm   SCAN {p['scan']:6.3f} mm", (12, 26))
    if m:
        ok = abs(m["angle"]) < fa.TOL_ANGLE and abs(m["x"]) < fa.TOL_PX
        off = f"{m['x'] * um_px:+6.1f} um" if um_px else f"{m['x']:+6.2f} px"
        text(view, f"fiber angle {m['angle']:+.4f} deg   offset {off}", (12, 52),
             color=(120, 255, 120) if ok else (0, 210, 255))
    if st.get("mode"):
        label = (f"LIVE ALIGN  {st['fps']:4.1f} corrections/s" if st["mode"] == "LIVE align"
                 else f"kayEi PREDICTIVE  {st['fps']:4.1f} fps" if st["mode"] == "kayEi"
                 else f"{st['mode']}  {st['fps']:4.1f} fps")
        text(view, label, (12, view.shape[0] - 16), 0.7, (80, 200, 255))
    if rec and int(time.time() * 2) % 2:
        cv2.circle(view, (VIEW_W - 24, 22), 9, (0, 0, 255), -1)
    return np.vstack([view, draw_chart(hist, time.time(), VIEW_W, cal)])


class App:
    def __init__(self, root):
        self.root, self.w = root, Worker()
        root.title("Fiber station")
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.photo, self.last_frame, self.writer = None, None, None

        self.img = ttk.Label(root)
        self.img.grid(row=0, column=0, rowspan=2, sticky="nw", padx=6, pady=6)
        side = ttk.Frame(root)
        side.grid(row=0, column=1, sticky="nw", padx=6, pady=6)

        # fiber readouts
        fb = ttk.LabelFrame(side, text="Fiber (image)")
        fb.pack(fill="x", pady=2)
        self.v = {k: tk.StringVar(value="-") for k in ("angle", "x", "width", "sharp", "cam", "status", "busy")}
        for r, (k, label) in enumerate([("angle", "angle"), ("x", "offset"), ("width", "width"),
                                        ("sharp", "sharpness"), ("cam", "camera")]):
            ttk.Label(fb, text=label).grid(row=r, column=0, sticky="w", padx=4)
            ttk.Label(fb, textvariable=self.v[k], font=("Consolas", 10)).grid(row=r, column=1, sticky="w")
        self.status = tk.Label(fb, textvariable=self.v["status"], font=("Segoe UI", 11, "bold"), width=24)
        self.status.grid(row=5, column=0, columnspan=2, pady=3)

        # axes
        ax = ttk.LabelFrame(side, text="Axes")
        ax.pack(fill="x", pady=2)
        self.pos, self.step, self.goto = {}, {}, {}
        for r, (name, (unit, step, lo, hi)) in enumerate(AXES.items()):
            ttk.Label(ax, text=name).grid(row=r, column=0, sticky="w", padx=4)
            self.pos[name] = tk.StringVar(value="-")
            ttk.Label(ax, textvariable=self.pos[name], width=12, font=("Consolas", 10), anchor="e").grid(row=r, column=1)
            ttk.Label(ax, text=unit).grid(row=r, column=2, sticky="w")
            self.step[name] = tk.StringVar(value=str(step))
            ttk.Button(ax, text="-", width=3, command=lambda n=name: self.jog(n, -1)).grid(row=r, column=3)
            ttk.Entry(ax, textvariable=self.step[name], width=6).grid(row=r, column=4)
            ttk.Button(ax, text="+", width=3, command=lambda n=name: self.jog(n, +1)).grid(row=r, column=5)
            self.goto[name] = tk.StringVar()
            ttk.Entry(ax, textvariable=self.goto[name], width=8).grid(row=r, column=6, padx=(8, 0))
            ttk.Button(ax, text="Go", width=4, command=lambda n=name: self.go(n)).grid(row=r, column=7)

        # camera
        cm = ttk.LabelFrame(side, text="Camera")
        cm.pack(fill="x", pady=2)
        self.exp = tk.StringVar(value=f"{START_EXPOSURE_MS:g}")
        self.auto = tk.BooleanVar(value=False)
        ttk.Label(cm, text="exposure ms").grid(row=0, column=0, padx=4)
        ttk.Entry(cm, textvariable=self.exp, width=7).grid(row=0, column=1)
        ttk.Button(cm, text="Set", command=self.set_exp).grid(row=0, column=2)
        ttk.Checkbutton(cm, text="auto", variable=self.auto, command=self.set_auto).grid(row=0, column=3, padx=6)
        ttk.Button(cm, text="Auto gain", command=lambda: self.w.submit("auto gain", self.w.auto_gain)).grid(row=0, column=4)
        ttk.Button(cm, text="Snapshot", command=self.snapshot).grid(row=0, column=6)
        self.rec_btn = tk.Button(cm, text="● Record", command=self.toggle_rec)
        self.rec_btn.grid(row=0, column=7, padx=4)

        # homing / reset
        hm = ttk.LabelFrame(side, text="Home / reset")
        hm.pack(fill="x", pady=2)
        for i, (label, name, fn) in enumerate([
                ("Reset drives", "reset drives", self.w.do_reset_drives),
                ("Home rotation", "home rotation", self.w.do_home_rotation),
                ("Home gonio", "home goniometer", self.w.do_home_gonio),
                ("Home all Hydra", "home all", self.w.do_home_all),
                ("Ref. PI stages", "reference PI", self.w.do_ref_pi)]):
            ttk.Button(hm, text=label, command=lambda n=name, f=fn: self.w.submit(n, f)).grid(
                row=0, column=i, padx=1, pady=2)

        # step-by-step alignment
        al = ttk.LabelFrame(side, text="Align (step by step)")
        al.pack(fill="x", pady=2)
        ttk.Button(al, text="Calibrate", command=lambda: self.w.submit("calibrate", self.w.do_calibrate)).grid(row=0, column=0, padx=2, pady=2)
        ttk.Button(al, text="Align here", command=lambda: self.w.submit("align", self.w.do_align)).grid(row=0, column=1, padx=2)
        ttk.Label(al, text="angles").grid(row=0, column=2, padx=(10, 2))
        self.n = tk.StringVar(value="6")
        ttk.Spinbox(al, from_=1, to=72, textvariable=self.n, width=4).grid(row=0, column=3)
        ttk.Button(al, text="Rotate + align", command=self.sequence).grid(row=0, column=4, padx=2)
        ttk.Label(al, text="scan from / to / step mm").grid(row=1, column=0, columnspan=2, sticky="w", padx=4)
        self.s_from, self.s_to, self.s_step = tk.StringVar(value="5"), tk.StringVar(value="45"), tk.StringVar(value="10")
        sf = ttk.Frame(al)
        sf.grid(row=1, column=2, columnspan=2, sticky="w")
        for var in (self.s_from, self.s_to, self.s_step):
            ttk.Entry(sf, textvariable=var, width=5).pack(side="left", padx=1)
        ttk.Button(al, text="Scan + rotate + align", command=self.scan_sequence).grid(row=1, column=4, padx=2, pady=2)

        # live
        lv = ttk.LabelFrame(side, text="LIVE (rotate + scan continuously)")
        lv.pack(fill="x", pady=2)
        self.l_mode = tk.StringVar(value="align")
        md = ttk.Frame(lv)
        md.grid(row=1, column=0, columnspan=4, sticky="w", padx=4)
        ttk.Radiobutton(md, text="As is (no compensation)", value="asis", variable=self.l_mode).pack(side="left")
        ttk.Radiobutton(md, text="Live align", value="align", variable=self.l_mode).pack(side="left", padx=8)
        ttk.Radiobutton(md, text="kayEi (predictive)", value="kayei", variable=self.l_mode).pack(side="left")
        self.k_relearn = tk.BooleanVar(value=False)
        self.k_ilc = tk.BooleanVar(value=True)
        kf = ttk.Frame(lv)
        kf.grid(row=3, column=0, columnspan=4, sticky="w", padx=4)
        ttk.Checkbutton(kf, text="relearn model at start", variable=self.k_relearn).pack(side="left")
        ttk.Checkbutton(kf, text="residual learning", variable=self.k_ilc).pack(side="left", padx=8)
        ttk.Label(lv, text="rotation deg/s").grid(row=0, column=0, sticky="w", padx=4)
        self.l_rot = tk.StringVar(value="20")
        ttk.Entry(lv, textvariable=self.l_rot, width=6).grid(row=0, column=1, sticky="w")
        self.l_scan = tk.BooleanVar(value=True)
        ttk.Checkbutton(lv, text="scan", variable=self.l_scan).grid(row=0, column=2, padx=(8, 0))
        self.l_from, self.l_to, self.l_v = tk.StringVar(value="5"), tk.StringVar(value="45"), tk.StringVar(value="1")
        lf = ttk.Frame(lv)
        lf.grid(row=0, column=3, sticky="w")
        for var, lab in ((self.l_from, "from"), (self.l_to, "to"), (self.l_v, "mm/s")):
            ttk.Entry(lf, textvariable=var, width=4).pack(side="left", padx=1)
            ttk.Label(lf, text=lab).pack(side="left")
        ttk.Button(lv, text="Start LIVE", command=self.live).grid(row=4, column=0, columnspan=2, sticky="w", padx=4, pady=2)
        tk.Button(lv, text="STOP", bg="#c0392b", fg="white", font=("Segoe UI", 10, "bold"), width=10,
                  command=self.w.stop).grid(row=4, column=3, sticky="e", padx=4)
        ttk.Label(side, textvariable=self.v["busy"], foreground="#2a78d6").pack(anchor="w")

        cols = ("scan", "rot", "gonio", "z", "angle", "x px", "it", "s")
        self.table = ttk.Treeview(side, columns=cols, show="headings", height=6)
        for c, wd in zip(cols, (55, 50, 70, 70, 70, 55, 30, 40)):
            self.table.heading(c, text=c)
            self.table.column(c, width=wd, anchor="e")
        self.table.pack(fill="x", pady=2)

        self.logbox = tk.Text(root, height=6, width=60, font=("Consolas", 9))
        self.logbox.grid(row=1, column=1, sticky="nsew", padx=6, pady=6)

        self.w.start()
        self.root.after(50, self.refresh)

    # --- actions ---
    def _num(self, var):
        try:
            return float(var.get())
        except ValueError:
            self.add_log(f"not a number: {var.get()!r}")
            return None

    def jog(self, name, sign):
        d = self._num(self.step[name])
        if d is not None:
            self.w.submit(f"{name} {sign * d:+g}", lambda: self.w.do_move(name, sign * d, relative=True))

    def go(self, name):
        p = self._num(self.goto[name])
        if p is not None:
            self.w.submit(f"{name} -> {p:g}", lambda: self.w.do_move(name, p))

    def set_exp(self):
        ms = self._num(self.exp)
        if ms is not None:
            self.auto.set(False)
            self.w.submit("exposure", lambda: self.w.do_exposure(ms=ms, auto=False))

    def set_auto(self):
        a = self.auto.get()
        self.w.submit("exposure", lambda: self.w.do_exposure(auto=a))

    def sequence(self):
        try:
            n = int(self.n.get())
        except ValueError:
            return
        self.table.delete(*self.table.get_children())
        self.w.submit(f"rotate x{n}", lambda: self.w.do_sequence(n))

    def scan_sequence(self):
        vals = [self._num(v) for v in (self.s_from, self.s_to, self.s_step)]
        try:
            n = int(self.n.get())
        except ValueError:
            return
        if None in vals or vals[2] <= 0:
            return
        pts = list(np.arange(vals[0], vals[1] + 1e-9, vals[2]))
        if not all(0 <= p <= 50 for p in pts):
            self.add_log("scan positions must be within 0..50 mm")
            return
        self.table.delete(*self.table.get_children())
        self.w.submit(f"scan {len(pts)} x rotate {n}", lambda: self.w.do_sequence(n, pts))

    def live(self):
        rot = self._num(self.l_rot)
        if rot is None or not 0 < rot <= 360:
            self.add_log("rotation speed must be 0..360 deg/s")
            return
        scan = None
        if self.l_scan.get():
            scan = tuple(self._num(v) for v in (self.l_from, self.l_to, self.l_v))
            if None in scan or not (0 <= scan[0] < scan[1] <= 50 and 0 < scan[2] <= 20):
                self.add_log("live scan: need 0 <= from < to <= 50 mm, 0 < speed <= 20 mm/s")
                return
        mode = self.l_mode.get()
        if mode == "kayei":
            if self.k_relearn.get():
                self.w.kayei = None
            self.w.kayei_ilc = self.k_ilc.get()
            self.w.submit("kayEi", lambda: self.w.do_live(rot, scan, mode_name="kayei"))
            return
        comp = mode == "align"
        self.w.submit("LIVE align" if comp else "AS IS", lambda: self.w.do_live(rot, scan, compensate=comp))

    def snapshot(self):
        if self.last_frame is not None:
            name = time.strftime("snapshot_%Y%m%d_%H%M%S.png")
            cv2.imwrite(name, self.last_frame)
            self.add_log(f"saved {name}")

    def toggle_rec(self):
        if self.writer is None:
            self.rec_name = time.strftime("live_%Y%m%d_%H%M%S.mp4")
            self.writer, self.rec_t0, self.rec_n = "pending", time.time(), 0
            self.rec_btn.configure(text="■ Stop rec", bg="#c0392b", fg="white")
            self.add_log(f"recording -> {self.rec_name}")
        else:
            if self.writer != "pending":
                self.writer.release()
            self.writer = None
            self.rec_btn.configure(text="● Record", bg="SystemButtonFace", fg="black")
            self.add_log(f"saved {self.rec_name} ({self.rec_n / REC_FPS:.1f} s)")

    def record(self, view_bgr):
        if self.writer is None:
            return
        if self.writer == "pending":
            h, w = view_bgr.shape[:2]
            self.writer = cv2.VideoWriter(self.rec_name, cv2.VideoWriter_fourcc(*"mp4v"), REC_FPS, (w, h))
        # duplicate frames so the movie plays in real time
        due = int((time.time() - self.rec_t0) * REC_FPS) + 1
        while self.rec_n < due:
            self.writer.write(view_bgr)
            self.rec_n += 1

    def add_log(self, line):
        self.logbox.insert("end", line + "\n")
        self.logbox.see("end")

    # --- periodic refresh ---
    def refresh(self):
        with self.w.lock:
            st = dict(self.w.state)
            hist = list(self.w.hist)
        while not self.w.logq.empty():
            self.add_log(self.w.logq.get())
        while not self.w.results.empty():
            r = self.w.results.get()
            self.table.insert("", "end", values=(f"{r[0]:.2f}", f"{r[1]:.1f}", f"{r[2]:+.4f}", f"{r[3]:.4f}",
                                                 f"{r[4]:+.4f}", f"{r[5]:+.2f}", r[6], f"{r[7]:.1f}"))
            self.table.see(self.table.get_children()[-1])
        off = st.get("drives_off", [])
        for name, p in st.get("pos", {}).items():
            self.pos[name].set("DRIVE OFF" if name in off else f"{p:10.4f}")
        self.v["busy"].set(f"busy: {st['busy']}" if st.get("busy") else "idle")

        f = st.get("frame")
        if f is not None and f is not self.last_frame:
            self.last_frame = f
            self.draw(f, st, hist)
        self.root.after(30, self.refresh)

    def draw(self, f, st, hist):
        view = compose(f, st, hist, self.w.cal, rec=self.writer is not None)
        self.record(view)
        m, cal = st["m"], self.w.cal
        um_px = 1000 / abs(cal["x_per_z"]) if cal else None

        # side panel readouts
        if m:
            ok = abs(m["angle"]) < fa.TOL_ANGLE and abs(m["x"]) < fa.TOL_PX
            self.v["angle"].set(f"{m['angle']:+8.4f} deg")
            self.v["x"].set(f"{m['x']:+8.2f} px" + (f"  ({m['x'] * um_px:+.1f} um)" if um_px else ""))
            self.v["width"].set(f"{m['width']:8.1f} px" + (f"  ({m['width'] * um_px:.0f} um)" if um_px else ""))
            self.v["status"].set("ALIGNED" if ok else "not aligned")
            self.status.configure(bg="#2e9e5b" if ok else "#d9a400", fg="white")
        else:
            for k in ("angle", "x", "width"):
                self.v[k].set("-")
            self.v["status"].set("NO FIBER FOUND")
            self.status.configure(bg="#c0392b", fg="white")
        sat = 100 * np.count_nonzero(f == 255) / f.size
        self.v["sharp"].set(f"{st['sharp']:8.1f}")
        self.v["cam"].set(f"{st['fps']:4.1f} fps  {st['exp']:6.1f} ms{' (auto)' if st['auto'] else ''}  sat {sat:.1f} %")
        self.photo = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(view, cv2.COLOR_BGR2RGB)))
        self.img.configure(image=self.photo)

    def on_close(self):
        if self.writer not in (None, "pending"):
            self.writer.release()
        self.w.stop()
        self.w.quit.set()
        self.w.join(timeout=5)
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()

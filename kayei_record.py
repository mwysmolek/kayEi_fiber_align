"""Demo run with movies: AS IS -> LIVE align -> kayEi (with its learning phase) at one speed,
with scanning, recorded headlessly with the GUI's own view composition (gui.compose).
Writes kayei_data/demo_<stamp>_<mode>.mp4 (one per mode) and demo_<stamp>_all.mp4, plus the
per-frame run CSVs the worker always writes.  Run: python kayei_record.py [speed] [seconds]"""
import os
import sys
import threading
import time

import cv2

import gui
import kayei

HERE = os.path.dirname(os.path.abspath(__file__))
SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 200.0
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
SCAN = (5.0, 45.0, 1.0)
FPS = 20


class Recorder(threading.Thread):
    """composes the GUI view from the worker state at FPS and writes it to one or more MP4s"""

    def __init__(self, w):
        super().__init__(daemon=True)
        self.w, self.writers, self.stop_evt = w, {}, threading.Event()
        self.size = None
        self.lock = threading.Lock()

    def open(self, name):
        path = os.path.join(HERE, "kayei_data", name)
        with self.lock:
            self.writers[name] = [path, None]   # writer created on the first frame (size known then)
        return path

    def close(self, name):
        with self.lock:
            entry = self.writers.pop(name, None)
        if entry and entry[1] is not None:
            entry[1].release()

    def run(self):
        t_next = time.perf_counter()
        while not self.stop_evt.is_set():
            with self.w.lock:
                st = dict(self.w.state)
                hist = list(self.w.hist)
            f = st.get("frame")
            if f is not None:
                view = gui.compose(f, st, hist, self.w.cal, rec=True)
                with self.lock:
                    for name, entry in self.writers.items():
                        if entry[1] is None:
                            h, wd = view.shape[:2]
                            entry[1] = cv2.VideoWriter(entry[0], cv2.VideoWriter_fourcc(*"mp4v"), FPS, (wd, h))
                        entry[1].write(view)
            t_next += 1.0 / FPS
            lag = t_next - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
            else:
                t_next = time.perf_counter()

    def stop(self):
        self.stop_evt.set()
        self.join(2)
        for name in list(self.writers):
            self.close(name)


def main():
    w = gui.Worker()
    w.start()
    while w.state["busy"] == "connecting...":
        time.sleep(0.2)

    def drain():
        while not w.logq.empty():
            print(w.logq.get(), flush=True)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    rec = Recorder(w)
    rec.start()
    rec.open(f"demo_{stamp}_all.mp4")
    tag = f"{SPEED:.0f} deg/s + scan"
    runs = (("asis", "AS IS", lambda: w.do_live(SPEED, SCAN, compensate=False)),
            ("align", "LIVE align", lambda: w.do_live(SPEED, SCAN, compensate=True)),
            ("kayei", "kayEi", lambda: w.do_live(SPEED, SCAN, mode_name="kayei")))
    w.kayei = None   # kayEi learns in front of the camera, as part of the movie
    for key, name, fn in runs:
        print(f"\n### {name} {tag}", flush=True)
        rec.open(f"demo_{stamp}_{key}.mp4")
        w.submit(name, fn)
        t0 = time.time()
        extra = 0
        while True:
            time.sleep(0.5); drain()
            if not w.state["busy"]:
                break
            if key == "kayei" and w.state.get("mode") == "kayEi" and extra == 0:
                extra = time.time()          # learning done: the clock for SECONDS starts now
            if (key != "kayei" and time.time() - t0 > SECONDS) or (extra and time.time() - extra > SECONDS):
                w.stop()
                while w.state["busy"]:
                    time.sleep(0.2)
                break
        time.sleep(1.0); drain()
        rec.close(f"demo_{stamp}_{key}.mp4")
    rec.stop()
    w.quit.set(); w.join(5)
    print(f"\nmovies: kayei_data/demo_{stamp}_*.mp4", flush=True)


if __name__ == "__main__":
    main()

"""Full comparison for the write-up: AS IS / LIVE align / kayEi at several rotation speeds,
without and with scanning. Headless, through the GUI worker (same code path as the buttons).
kayEi learns once (both scan ends); residual table and trim are reset per run.
Each run is logged to kayei_data/run_*.csv by the worker; evaluate with kayei_eval.py.
Run:  python kayei_sweep.py   (~20 min)"""
import sys
import time

import gui
import kayei

SPEEDS = (20.0, 50.0, 100.0, 200.0, 360.0)
SCAN = (5.0, 45.0, 1.0)
SECONDS = {20.0: 45, 50.0: 30, 100.0: 25, 200.0: 25, 360.0: 25}   # >= 2 turns + settle

w = gui.Worker()
w.start()
while w.state["busy"] == "connecting...":
    time.sleep(0.2)


def drain():
    while not w.logq.empty():
        print(w.logq.get(), flush=True)


def run(name, fn, secs):
    print(f"\n### {name}", flush=True)
    w.submit(name, fn)
    t0 = time.time()
    while time.time() - t0 < secs:
        time.sleep(0.5)
        drain()
        if not w.state["busy"]:   # ended by itself (error) - don't wait for nothing
            break
    w.stop()
    while w.state["busy"]:
        time.sleep(0.2)
    time.sleep(0.5)
    drain()


# learn the kayEi model once, with both scan ends
w.submit("kayEi learn", lambda: w.do_kayei_learn(SCAN))
while w.state["busy"] or w.kayei is None:
    time.sleep(0.5); drain()
    if not w.state["busy"] and w.kayei is None:
        sys.exit("learning failed")
drain()

for speed in SPEEDS:
    for scan in (None, SCAN):
        tag = f"{speed:.0f} deg/s" + (" + scan" if scan else "")
        run(f"AS IS {tag}", lambda s=speed, sc=scan: w.do_live(s, sc, compensate=False), SECONDS[speed])
        run(f"LIVE align {tag}", lambda s=speed, sc=scan: w.do_live(s, sc, compensate=True), SECONDS[speed])
        w.kayei.res = kayei.Residual(); w.kayei.trim_g = w.kayei.trim_z = 0.0
        run(f"kayEi {tag}", lambda s=speed, sc=scan: w.do_live(s, sc, mode_name="kayei"), SECONDS[speed])

w.quit.set()
w.join(5)
print("sweep done", flush=True)

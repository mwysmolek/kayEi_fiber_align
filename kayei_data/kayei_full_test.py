"""headless kayEi through the GUI worker: learn, run at 100 deg/s without and with scan"""
import sys, time
sys.path.insert(0, r"C:\dev\hydra")
import gui
w = gui.Worker(); w.start()
while w.state["busy"] == "connecting...": time.sleep(0.2)
def drain():
    while not w.logq.empty(): print(w.logq.get())
def run(name, fn, secs):
    w.submit(name, fn); t0 = time.time()
    while time.time() - t0 < secs:
        time.sleep(0.5); drain()
    w.stop()
    while w.state["busy"]: time.sleep(0.2)
    time.sleep(0.5); drain()
runs = [
        ("kayEi 100 scan", lambda: w.do_live(100.0, (5.0, 45.0, 1.0), mode_name="kayei"), 60)]
for name, fn, secs in runs:
    if "scan" in name and "no" not in name:
        w.kayei = None   # relearn with both scan ends
    run(name, fn, secs)
w.quit.set(); w.join(5)

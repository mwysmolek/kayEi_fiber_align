import sys, time
sys.path.insert(0, r"C:\dev\hydra")
import gui, kayei
w = gui.Worker(); w.start()
while w.state["busy"] == "connecting...": time.sleep(0.2)
def drain():
    while not w.logq.empty():
        l = w.logq.get()
        if " it" not in l and "calibration" not in l: print(l, flush=True)
w.submit("learn", lambda: w.do_kayei_learn(None))
while w.state["busy"] or w.kayei is None:
    time.sleep(0.5); drain()
    if not w.state["busy"] and w.kayei is None: sys.exit("learning failed")
import os
w.kayei_ilc = os.environ.get("ILC", "1") == "1"
print("residual learning", w.kayei_ilc)
kayei.TRACK_RATE = float(os.environ.get("RATE", "50"))
kayei.FB_GAIN_Z = float(os.environ.get("FB", "0.4")); print("FB_GAIN_Z", kayei.FB_GAIN_Z)
acc = os.environ.get("ACC")
if acc: w.rig.h.send(f"{acc} 1 setnaccel")
print("rate", kayei.TRACK_RATE, "gonio accel", w.rig.h.query("1 getnaccel").strip())
for speed in tuple(float(v) for v in os.environ.get("SPEEDS", "100,200").split(",")):
    w.kayei.res = kayei.Residual(); w.kayei.trim_g = w.kayei.trim_z = w.kayei.fb_z = 0.0
    print(f"### kayEi {speed:.0f} deg/s", flush=True)
    w.submit("k", lambda s=speed: w.do_live(s, None, mode_name="kayei")); t0 = time.time()
    while time.time() - t0 < 25:
        time.sleep(0.5); drain()
        if not w.state["busy"]: break
    w.stop()
    while w.state["busy"]: time.sleep(0.2)
    time.sleep(0.5); drain()
w.rig.h.send('50 1 setnaccel'); print('gonio accel restored', w.rig.h.query('1 getnaccel').strip())
w.quit.set(); w.join(5)

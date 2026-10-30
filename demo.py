"""Simultaneous moves of goniometer (axis 1) and rotation stage (axis 2).
Assumes both axes are homed (python hydra.py 172.20.5.201 --home)."""
import threading
import time

from hydra import Hydra

IP = "172.20.5.201"
GONIO, ROT = 1, 2
# (goniometer deg, rotation deg)
SEQUENCE = [(3.0, 90.0), (-3.0, 180.0), (5.0, -90.0), (-5.0, 45.0), (0.0, 0.0)]

h = Hydra(IP)
log = []
moving = True


def logger():
    # separate connection so polling does not interleave with the mover's replies
    mon = Hydra(IP)
    t0 = time.time()
    while moving:
        log.append((time.time() - t0, mon.position(GONIO), mon.position(ROT)))
        time.sleep(0.1)
    mon.close()


th = threading.Thread(target=logger)
th.start()
try:
    for g, r in SEQUENCE:
        t = time.time()
        h.move_together({GONIO: g, ROT: r})
        print(f"-> gonio {h.position(GONIO):+8.4f}  rot {h.position(ROT):+9.4f}"
              f"   (target {g:+.1f}, {r:+.1f}; {time.time() - t:.2f}s)")
finally:
    moving = False
    th.join()
    h.close()

print("\n  t[s]    gonio      rot")
for t, g, r in log[::5]:
    print(f"{t:6.2f} {g:+8.3f} {r:+9.3f}")

"""Speed sweep of both Hydra axes with fast position logging.
Rotation: +/- moves at increasing speed. Goniometer: up/down at increasing speed.
Writes dynamics_log.csv (t, axis, speed, target, gonio, rot) and prints per-move stats."""
import csv
import threading
import time

from hydra import Hydra

IP = "172.20.5.201"
GONIO, ROT = 1, 2
ROT_SPEEDS = [0.5, 2, 10, 45, 90, 180, 360]
GONIO_SPEEDS = [0.25, 0.5, 1, 2, 4]

log = []
tag = {"axis": "", "speed": 0.0, "target": 0.0}
running = True


def logger():
    mon = Hydra(IP)
    t0 = time.perf_counter()
    while running:
        g, r = mon.position(GONIO), mon.position(ROT)
        log.append((time.perf_counter() - t0, tag["axis"], tag["speed"], tag["target"], g, r))
    mon.close()


def move(h, axis, name, speed, target):
    tag.update(axis=name, speed=speed, target=target)
    t = time.perf_counter()
    h.move_abs(axis, target)
    time.sleep(0.05)
    h.wait(axis, timeout=120)
    time.sleep(0.3)  # capture settling
    return time.perf_counter() - t


def stats(name, col):
    # per move: (speed, target) segments in log
    rows = [r for r in log if r[1] == name]
    seg = []
    for r in rows:
        if not seg or (r[2], r[3]) != (seg[-1][0][2], seg[-1][0][3]):
            seg.append([])
        seg[-1].append(r)
    print(f"\n{name}: speed set | move | peak measured | time to 90% speed | overshoot | settle (<1e-3)")
    for s in seg:
        t = [x[0] for x in s]
        p = [x[col] for x in s]
        speed, target = s[0][2], s[0][3]
        start = p[0]
        if abs(target - start) < 1e-6:
            continue
        v = [(p[i + 2] - p[i - 2]) / (t[i + 2] - t[i - 2]) for i in range(2, len(p) - 2)]
        if not v:
            continue
        sign = 1 if target > start else -1
        vpk = max(x * sign for x in v)
        i_move = next((i for i, x in enumerate(v) if x * sign > 0.02 * speed), 0)
        i_90 = next((i for i, x in enumerate(v) if x * sign > 0.9 * speed), None)
        t_acc = (t[i_90 + 2] - t[i_move + 2]) * 1000 if i_90 is not None else float("nan")
        over = max((x - target) * sign for x in p)
        t_set = next((t[i] for i in range(len(p)) if all(abs(q - target) < 1e-3 for q in p[i:])), None)
        t_end = next((t[i] for i in range(len(p)) if (p[i] - target) * sign >= -1e-3), t[-1])
        print(f"  {speed:7.2f} deg/s | {start:+8.3f} -> {target:+8.3f} | {vpk:8.2f} deg/s"
              f" | {t_acc:6.0f} ms | {max(over, 0):.5f} deg"
              f" | {((t_set or t[-1]) - t_end) * 1000:5.0f} ms after reaching target")


h = Hydra(IP)
v_rot0, v_gon0 = h.velocity(ROT), h.velocity(GONIO)
th = threading.Thread(target=logger)
th.start()
try:
    for v in ROT_SPEEDS:
        d = min(max(v * 2, 3), 360)
        h.set_velocity(ROT, v)
        for tgt in (d, 0, -d, 0):
            dt = move(h, ROT, "rot", v, tgt)
            print(f"rot  {v:6.1f} deg/s -> {tgt:+7.1f}  {dt:5.2f}s  at {h.position(ROT):+.4f}")
    h.set_velocity(ROT, v_rot0)
    for v in GONIO_SPEEDS:
        a = min(4, max(1, v * 2))
        h.set_velocity(GONIO, v)
        for tgt in (a, -a, 0):
            dt = move(h, GONIO, "gonio", v, tgt)
            print(f"gon  {v:6.2f} deg/s -> {tgt:+7.1f}  {dt:5.2f}s  at {h.position(GONIO):+.4f}")
finally:
    h.set_velocity(ROT, v_rot0)
    h.set_velocity(GONIO, v_gon0)
    running = False
    th.join()
    print("restored velocities:", h.velocity(GONIO), h.velocity(ROT), "errors:", h.query("ge"))
    h.close()

with open("dynamics_log.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t_s", "axis_moving", "speed_set", "target", "gonio_deg", "rot_deg"])
    w.writerows(log)
dt = [b[0] - a[0] for a, b in zip(log, log[1:])]
print(f"\nlogged {len(log)} samples, mean sample interval {1000 * sum(dt) / len(dt):.2f} ms")
stats("rot", 5)
stats("gonio", 4)

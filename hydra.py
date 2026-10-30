"""Minimal PI/Micos Hydra (Venus-3) TCP client for two axes:
axis 1 = goniometer (stepper, +/-5 deg), axis 2 = rotation stage (brushless servo, 360 deg/rev).

Venus-3 is postfix: parameters first, then axis number, then command, e.g. "10.0 1 nm".
Usage:
    python hydra.py 172.20.5.201          # connect, identify, read status/positions
    python hydra.py 172.20.5.201 --home   # home both axes (switches / encoder index)
    python hydra.py 172.20.5.201 --move   # small +/- relative test move on each axis
"""
import socket
import sys
import time

PORT = 400
AXES = {"goniometer": 1, "rotation": 2}
TEST_STEP = 1.0  # in the axis' configured unit (usually degrees for these stages)


class Hydra:
    def __init__(self, ip, port=PORT, timeout=3.0):
        self.sock = socket.create_connection((ip, port), timeout=timeout)

    def send(self, cmd):
        self.sock.sendall((cmd + " \r\n").encode())

    def query(self, cmd):
        self.send(cmd)
        buf = b""
        while not buf.endswith(b"\r\n"):
            chunk = self.sock.recv(1024)
            if not chunk:
                raise ConnectionError("Hydra closed the connection")
            buf += chunk
        return buf.decode().strip()

    def identify(self):
        return self.query("identify")

    def version(self):
        return self.query("version")

    def status(self, axis):
        return int(self.query(f"{axis} nst"))

    def init(self, axis):
        self.send(f"{axis} init")  # power the drive, clear faults (servo may commutate)

    def powered(self, axis):
        return not self.status(axis) & 0x100

    def is_moving(self, axis):
        return bool(self.status(axis) & 1)  # bit0 moving; 0x80 e-stop, 0x100 power off

    def position(self, axis):
        return float(self.query(f"{axis} np"))

    def limits(self, axis):
        return self.query(f"{axis} getnlimit")

    def calibrate(self, axis):
        self.send(f"{axis} ncal")  # home to low (cal) switch, sets 0 there

    def range_measure(self, axis):
        self.send(f"{axis} nrm")  # drive to high switch, sets upper limit to full travel

    def set_position(self, axis, pos):
        self.send(f"{pos} {axis} setnpos")  # redefine current position (software zero)

    def move_abs(self, axis, pos):
        self.send(f"{pos} {axis} nm")

    def move_rel(self, axis, delta):
        self.send(f"{delta} {axis} nr")

    def stop(self, axis):
        self.send(f"{axis} nabort")

    def wait(self, axis, timeout=60.0):
        t0 = time.time()
        while self.is_moving(axis):
            if time.time() - t0 > timeout:
                self.stop(axis)
                raise TimeoutError(f"axis {axis} did not finish moving")
            time.sleep(0.05)

    def velocity(self, axis):
        return float(self.query(f"{axis} gnv"))

    def set_velocity(self, axis, vel):
        self.send(f"{vel} {axis} snv")

    def move_together(self, targets, sync=True):
        """Start absolute moves {axis: pos} on all axes at once and wait for all.
        sync=True slows the faster axes so every axis arrives together."""
        vel0 = {ax: self.velocity(ax) for ax in targets}
        if sync:
            dist = {ax: abs(p - self.position(ax)) for ax, p in targets.items()}
            t = max(dist[ax] / vel0[ax] for ax in targets)
            if t > 0:
                for ax in targets:
                    self.set_velocity(ax, max(dist[ax] / t, 1e-3))
        try:
            for ax, p in targets.items():
                self.move_abs(ax, p)
            time.sleep(0.1)
            for ax in targets:
                self.wait(ax)
        finally:
            for ax, v in vel0.items():
                self.set_velocity(ax, v)

    def find_index(self, axis, distance=370):
        self.send(f"{distance} {axis} nrefmove")  # search encoder index within +distance

    def home_all(self, gonio=1, rot=2):
        """Power both drives, home goniometer on its switches and zero it at the centre
        of its travel (~ +/-5.16 deg), home rotation stage on its encoder index (0 = mark)."""
        for ax in (gonio, rot):
            if not self.powered(ax):
                self.init(ax)
                time.sleep(2)
        self.calibrate(gonio); time.sleep(0.2); self.wait(gonio)
        self.range_measure(gonio); time.sleep(0.2); self.wait(gonio)
        lo, hi = map(float, self.limits(gonio).split())
        self.move_abs(gonio, (lo + hi) / 2)
        self.find_index(rot); time.sleep(0.3); self.wait(rot)
        self.set_position(rot, 0)
        self.wait(gonio)
        self.set_position(gonio, 0)  # 0 = centre, limits shift to +/- half range

    def close(self):
        self.sock.close()


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    ip, do_move = sys.argv[1], "--move" in sys.argv
    h = Hydra(ip)
    try:
        print("identify:", h.identify())
        print("version :", h.version())
        for name, ax in AXES.items():
            print(f"[{name} / axis {ax}] status={h.status(ax)} "
                  f"pos={h.position(ax)} limits={h.limits(ax)}")
        if "--home" in sys.argv:
            h.home_all(AXES["goniometer"], AXES["rotation"])
            print("homed:", {n: h.position(a) for n, a in AXES.items()})
        if do_move:
            for name, ax in AXES.items():
                if not h.powered(ax):
                    h.init(ax)
                    time.sleep(2)
                    print(f"  {name}: init -> status={h.status(ax)}")
                start = h.position(ax)
                for step in (TEST_STEP, -TEST_STEP):
                    h.move_rel(ax, step)
                    h.wait(ax)
                    print(f"  {name}: moved {step:+} -> {h.position(ax):.4f}")
                print(f"  {name}: back at start? error={h.position(ax) - start:+.4f}")
    finally:
        h.close()


if __name__ == "__main__":
    main()

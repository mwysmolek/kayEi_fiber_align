"""Easter egg: the two PI stepper stages sing. A stepper's pitch is its full-step rate, 200 Hz per
mm/s on the VT-80 (3200 microsteps/mm = 200 full steps/mm). Notes are timed moves back and forth
around mid travel; melody on the scan stage (50 mm), bass on Z (25 mm).
Run: python kayei_data/music.py [tetris|scale|tune A4]   (HZ_PER_MMS can be corrected by ear)"""
import sys, time, threading
sys.path.insert(0, r"C:\dev\hydra")
import fiber_align as fa
from kayei_learn import pi_stage, SCAN_SN

HZ_PER_MMS = 200.0
NOTE = {n: 440.0 * 2 ** ((i - 9) / 12) for i, n in enumerate(["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"])}
def freq(name):               # "E5", "A#4", "R" = rest
    if name == "R": return 0.0
    return NOTE[name[:-1]] * 2 ** (int(name[-1]) - 4)

class Voice:
    def __init__(self, dev, lo, hi):
        self.d, self.lo, self.hi = dev, lo, hi; self.d.errcheck = False
        self.d.ACC("1", 400); self.d.DEC("1", 400); self.pos = self.d.qPOS("1")["1"]; self.dir = 1
    def play(self, notes, bpm):
        beat = 60.0 / bpm
        for name, beats in notes:
            dur = beats * beat; f = freq(name)
            if f == 0 or f / HZ_PER_MMS > 10.0:
                time.sleep(dur); continue
            v = f / HZ_PER_MMS; dist = v * dur
            if not self.lo <= self.pos + self.dir * dist <= self.hi: self.dir = -self.dir
            self.pos += self.dir * dist
            self.d.VEL("1", v); self.d.MOV("1", self.pos); time.sleep(dur * 0.97); self.d.HLT("1", noraise=True); time.sleep(dur * 0.03)
            self.pos = self.d.qPOS("1")["1"]
    def close(self):
        self.d.VEL("1", 10.0 if self.hi > 24 else 10.0); self.d.ACC("1", 100); self.d.DEC("1", 100); self.d.errcheck = True; self.d.CloseConnection()

TETRIS = [("E5", 1), ("B4", .5), ("C5", .5), ("D5", 1), ("C5", .5), ("B4", .5), ("A4", 1), ("A4", .5), ("C5", .5), ("E5", 1), ("D5", .5), ("C5", .5),
          ("B4", 1.5), ("C5", .5), ("D5", 1), ("E5", 1), ("C5", 1), ("A4", 1), ("A4", 1), ("R", 1),
          ("R", .5), ("D5", 1), ("F5", .5), ("A5", 1), ("G5", .5), ("F5", .5), ("E5", 1.5), ("C5", .5), ("E5", 1), ("D5", .5), ("C5", .5),
          ("B4", 1), ("B4", .5), ("C5", .5), ("D5", 1), ("E5", 1), ("C5", 1), ("A4", 1), ("A4", 1), ("R", 1)]
BASS = [("E3", 1), ("E3", 1), ("E3", 1), ("E3", 1), ("A2", 1), ("A2", 1), ("A2", 1), ("A2", 1), ("G#2", 1), ("G#2", 1), ("E3", 1), ("E3", 1), ("A2", 1), ("A2", 1), ("A2", 1), ("R", 1),
        ("D3", 1.5), ("D3", 1.5), ("F3", 1), ("A2", 1), ("A2", 1), ("C3", 1), ("C3", 1), ("E3", 1), ("E3", 1), ("G#2", 1), ("G#2", 1), ("A2", 1), ("A2", 1), ("A2", 1), ("R", 1)]
SCALE = [(n, .5) for n in ("C4", "D4", "E4", "F4", "G4", "A4", "B4", "C5", "D5", "E5", "F5", "G5", "A5", "B5", "C6")]

what = sys.argv[1] if len(sys.argv) > 1 else "tetris"
mel = Voice(pi_stage(SCAN_SN, "scan"), 8.0, 42.0); bass = Voice(pi_stage(fa.Z_SN, "Z"), 4.0, 21.0)
try:
    if what == "tune":
        n = sys.argv[2] if len(sys.argv) > 2 else "A4"; print(f"{n} = {freq(n):.1f} Hz -> {freq(n)/HZ_PER_MMS:.3f} mm/s for 2 s"); mel.play([(n, 4)], 120)
    elif what == "scale":
        mel.play(SCALE, 120)
    else:
        bpm = 150
        t = threading.Thread(target=bass.play, args=(BASS, bpm)); t.start(); mel.play(TETRIS, bpm); t.join()
finally:
    mel.close(); bass.close(); print("done")

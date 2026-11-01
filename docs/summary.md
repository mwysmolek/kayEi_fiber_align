# kayEi fiber alignment station: what was done, how it went, what it is worth

Status 2026-11-01, end of day. Details, numbers and raw data: [kayei_method.md](kayei_method.md).

## 1. What exists now

A camera-guided station that keeps a rotating, scanning fiber straight and in position for
side processing with a laser:

- **Hardware under one Python program**: Hydra controller (goniometer + rotation stage), two PI
  C-663 linear stages (camera height Z, scan along the fiber), Baumer GigE camera, two optics
  (6 mm lens, 4x objective). GUI with live image, overlay, charts, axis jog/go/home, calibration,
  alignment sequences, three LIVE modes, MP4 recording.
- **Measurement**: sub-pixel fit of the fiber's centre line from both edges, for glowing fibers,
  dark-on-bright fibers, backlit silhouettes and transparent fibers with a bright core, robust
  against glints, reflections and dust; noise 0.0002 deg / 0.1 um per frame with the 6 mm lens,
  0.016 um with the 4x objective on the long tube, 0.2 um on the short tube now in use.
- **Three ways to align**: step-by-step (align, rotate, align), feedback while rotating ("live
  align"), and **kayEi**: predictive control that learns the fiber's wobble in two turns and
  drives goniometer and Z to where the fiber will be, with velocity-shaped streaming, a
  filtered rotation clock, a Fourier residual learner, and camera feedback on top.
- **Characterisation tools**: latency and wobble identification, image scale and fiber diameter
  from Z steps, the fiber's 3D axis relative to the rotation axis from one camera, 2D/3D/4D
  (3D + time) plots, before/after movies, a 30-run comparison protocol with evaluation and figure.
- **Documentation**: a 1050-line write-up with 35 figures and movies, every number tied to a script and a
  data file, an experiment log, a reproduction recipe; a project file with the per-sample recipe;
  36 commits on GitHub (`mwysmolek/kayEi_fiber_align`), tag `EO-6mm` for the 6 mm lens state.

## 2. How it went, in four sessions

| Session | What happened | Turning points |
|---|---|---|
| **2026-10-20** (~1.3 h) | Hydra found on the network, goniometer + rotation driven, homed, dynamics measured, camera found, live view | - |
| **2026-10-30** (~5 h) | Two PI stages brought up (wrong stage parameter sets found and fixed), 4 axes together, new camera + lens, fiber found and measured, calibration, alignment at 6 angles, GUI, live tracking, as-is vs feedback (18x better), home/reset, recording | drive faults from manual extreme tilts; feedback degrades above 50 deg/s |
| **2026-10-31** (~7.5 h incl. evening) | kayEi: idea to 30-run proof in 1.5 h (0.007-0.016 deg, 8-22 um, 20-360 deg/s); fiber diameter and image scale; 3D/4D wobble map; PhD write-up; then the fiber re-clamped 12x more tilted: steppers found to execute target streams as stop-and-go, learner wind-up, clock corruption, feedback instability - all found by measurement and fixed the same hour; second 30-run comparison | two Hydra firmware traps; a hanging wire; the goniometer's 4 deg/s limit reached by construction |
| **2026-11-01** (~5.3 h incl. evening) | New sample (opaque rod) and backlight: silhouette measurement (three fixes), dead goniometer limit switch found by stepping with the camera as reference, drive-trip guard, rod characterised, third 30-run comparison (kayEi 0.008-0.043 deg / 6-45 um over the whole speed range, no mechanical limit), demo movies, before/after 4D movie; then the 4x objective: 2.35 um/px, 6 um at 100 deg/s in focus, and the depth-of-field limit that makes scanning impossible without a focus axis; then the short tube (6.6 um/px, scanning works again, 12 um at 100 deg/s) and a third sample, a transparent 0.55 mm fiber: fit extended for its bright core, wobble 1.8 deg (the largest yet), kayEi 0.038 deg / 49 um at 100 deg/s with the goniometer at its cap | the goniometer homing crashed twice into its end stop before the dead switch was proven; the 4x sweep had to be declared invalid; a headless check hung for 1.5 h with its error filtered away |

Pattern of the whole thing: nearly every step forward was preceded by a measurement that
contradicted an assumption (the stages follow a stream; the wire is harmless; the switch works;
only one edge fits the frame; the lens has depth of field). The write-up keeps those, because
they are the method.

## 3. What was gained

Alignment error while the fiber rotates, after the first turn (rms; angle / lateral offset):

| | as is (no control) | feedback ("live align") | **kayEi** |
|---|---|---|---|
| 1.1 mm glass fiber, small wobble, 100 deg/s (10-31 afternoon) | 0.21 deg / 123 um | 0.08 deg / 15 um | **0.009 deg / 12 um** |
| same fiber at 360 deg/s | 0.30 deg / 154 um | 0.24 deg / 66 um | **0.014 deg / 21 um** |
| same fiber re-clamped, 1.26 deg / 1.2 mm wobble, 100 deg/s | 1.5 deg / 1.4 mm | 0.49 deg / 77 um | **0.03 deg / 28-45 um** |
| opaque rod, 0.46 deg / 300 um wobble, 20 deg/s | 0.55 deg / 318 um | 0.037 deg / 17 um | **0.008 deg / 6 um** |
| rod, 360 deg/s with scanning | 0.26 deg / 173 um | diverges | **0.043 deg / 45 um** |
| rod through the 4x objective, 100 deg/s, in focus | 0.28 deg / 269 um | - | **0.011 deg / 6 um** |
| rod, 4x on the short tube, 100 deg/s with scanning | 0.26 deg / 257 um | - | **0.015 deg / 14 um** |
| transparent 0.55 mm fiber, 1.84 deg / 1.4 mm wobble, 100 deg/s | 1.84 deg / 1413 um | - | **0.038 deg / 49 um** (goniometer at 4 deg/s) |

Other things measured along the way that the station did not have before:

- all latencies of the chain (58 ms command to image, 30 ms frame to encoder, stage leads);
- the fiber's full 3D centre line relative to the rotation axis from a single camera, to a few um;
- fiber diameter to +/-0.004 mm and the image scale to 0.1 % from the Z stage alone;
- how a stepper really executes a 50 Hz stream of targets, how to stream so that it does not
  stop-and-go, what a PI command costs over USB with and without error checking;
- the physical limits, in numbers: 4 deg/s and 0.034 deg ripple on the goniometer, backlash
  0.13 deg, depth of field of the 4x, the out-of-plane (focus) wobble that no axis corrects yet.

## 4. Time

| | agent session span | commits | person's messages |
|---|---|---|---|
| 2026-10-20 | ~1.3 h | (pre-git) | a handful |
| 2026-10-30 | ~5 h | 2 (repo created at the end) | ~19 |
| 2026-10-31 | ~7.5 h (15:20-22:45, with breaks) | 21 | 7 |
| 2026-11-01 | ~5.3 h (16:10-21:30) | 13 | ~30, mostly one line ("looks good", "stop the goniometer", "1") |
| total | **~19 h of agent time** | 36 | |

Produced in that time: 39 Python files, 4,300 lines; 200 logged runs and 475 MB of data; 35
figures and movies; a 1050-line document. The person's own hands-on time per day is not in
any log (the document's section 7.5 has the blanks); what the logs do show is the shape of
the collaboration: the person supplies hardware, light, samples, decisions and the word
"stop"; the agent measures, writes code, reads the results, decides what to measure next and
writes it all down. Of the 2026-11-01 session, for example, the person's part was installing
the rod, building the backlight in three iterations, turning the goniometer to centre by hand
and saying when things looked right - and the result is a characterised sample, a third
30-run comparison and a tested second optic, with the day's two hardware faults diagnosed.

kayEi itself, from the first design message to a 30-run proof across the whole speed range,
took about 1.5 hours of agent time on 2026-10-31, an hour of which ran with half the axes
unplugged. The redesign for the large wobble the same evening took 35 minutes from the first
failed run to the configuration that held.

## 5. What is next

1. A **focus axis**: the one addition that turns the 4x objective's 6 um into a scanning
   instrument; kayEi already predicts the out-of-plane motion it would have to follow.
2. **Goniometer**: dead lower switch to repair; a smoother motion mode or a servo would remove
   the 0.034 deg ripple that is now the floor; its 4 deg/s cap decides the usable speed for a
   given tilt.
3. **Samples**: the transparent 0.55 mm fiber now in the chuck at 30-60 deg/s (its 1.8 deg
   wobble saturates the goniometer at 100 deg/s), then thinner fibers and a glass capillary
   (recipe in PROJECT.md; a 125 um fiber is 19 px on the current optic), the detection limit
   of the fit, and whether the chuck's grip of a glass fiber (sine residual 0.015-0.018 deg)
   can be made as clean as the rod's (0.003 deg).
4. **Lever arm**: moving the camera's view closer to the goniometer pivot would stop the
   goniometer's ripple from leaking 40 um into the offset.

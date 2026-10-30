# Fiber alignment station

## Goal

Simulate **laser processing of the side of an optical fiber**: the fiber's side
surface must sit in the focal plane of a structuring laser while it is rotated
and translated. The camera finds the **fiber edge**, the stages bring it to the
target, and the cycle repeats along and around the fiber. Built step by step;
this file tracks what exists, what is verified, and what is still to come.

### Process (nothing fancy)

```
align  : find fiber edge -> make it horizontal (goniometer) and at target height
repeat :
    rotate (rotation stage)        -> re-align (horizontal + height)
    ... until the full circumference is covered
    translate along the fiber      -> re-align, then rotate/re-align again
```

- The laser shoots at the **upper edge** of the fiber; that is the edge we track.
- **Horizontal** = fiber axis lies in the laser's focal plane; corrected with the
  goniometer, which tilts the fiber **up/down** (in the camera's view plane).
- **Height / focus**: the laser focus position is **simulated by the vertical
  stage position** (camera height). Aligned = upper edge at the reference height.
- Re-alignment after each rotation compensates fiber runout / chuck eccentricity.

### Stages of ambition

1. Alignment only, no rotation.
2. Rotate 0 / 180 deg, re-align at each.
3. 0 / 120 / 240 deg, then a few other angle sets.
4. **Stretch goal:** live, simultaneous rotation + alignment (edge tracking while
   the fiber turns), as fast as the slowest link allows. Likely limits: camera
   frame rate (exposure-bound, ~10 fps default), image processing time,
   goniometer speed (4 deg/s, open-loop stepper), C-663 stage speed, and
   per-command latency of the controllers. **Reached 2026-10-30** (see Live tracking).

## Hardware

| Part | Role | Connection | Status |
|---|---|---|---|
| PI/Micos **Hydra TT** controller (fw 5.23) | drives goniometer + rotation stage | Ethernet 2, `172.20.5.201:400`, Venus-3 | working |
| **Goniometer** (Hydra axis 1, 2-phase stepper, open loop) | tilts the rotation stage (and so the fiber axis) | Hydra | working, homed |
| **Rotation stage** (Hydra axis 2, brushless servo, encoder, 360 deg/rev) | carries a **3-jaw chuck holding the fiber**, rotates the fiber about its axis | Hydra | working, homed |
| **Baumer VCXG-15M.I.K34** GigE camera (1440x1080 Mono8), **s/n 700009549658, MAC 00:06:BE:11:70:BA** (since 2026-10-30; the 10-20 camera was s/n 700010291417, MAC 00:06:BE:11:B5:8B) | looks at the fiber from the side | Ethernet 2, `172.20.5.20` (temporary IP) | working, new lens fitted and focused |
| **Z stage** (vertical): VT-80 25 mm + **PI C-663.11 SN 0165500187** | moves the camera up/down | USB (pipython + PI GCS2 DLL) | working, referenced |
| **Scan stage** (horizontal): VT-80 50 mm + **PI C-663.11 SN 0165500183** (carries the Z stage) | moves the camera along the fiber | USB (pipython + PI GCS2 DLL) | working, referenced |

Linear stages (2026-10-30): both are **VT-80**, 2-phase stepper, 3200 counts/mm, max 20 mm/s.

Camera image (new lens): **rotated 90 deg** - the fiber appears **vertical** in the image; Z moves
it sideways (-49.5 px/mm Z, i.e. **~20 um/px**); goniometer turns it ~1:1 (-1.01 deg image angle
per deg) and shifts it -70 px/deg. Fiber (whatever is in the chuck) ~54 px = ~1.1 mm wide.

| Controller SN | Stage | CST (stage parameter set) |
|---|---|---|
| 0165500183 | VT-80, 50 mm | `62309220` (was wrongly `62309260` = 200 mm; fixed, saved with `WPA`) |
| 0165500187 | VT-80, 25 mm | `62309210-0000` (was wrongly `62309220` = 50 mm; fixed, saved with `WPA`) |

Mapping verified by travel: 187 hit its + limit switch at 26.8 mm (= 25 mm stage),
183 drove to 35 mm (= 50 mm stage).

No reference switch on these stages (`FRF` fails, error 31): reference on the negative
limit with `pitools.startup(pi, refmodes='FNL')`. Requires PI Software Suite (USB driver,
`PI_GCS2_DLL`) and `pipython` in `.venv`; PIMikroMove itself is not needed.
After a limit-switch hit, error 216 is latched and raised on the next query; re-reference with FNL.

Mechanical stack:

```
goniometer -> rotation stage -> 3-jaw chuck -> fiber
horizontal stage (along fiber) -> vertical stage -> camera (looking at fiber)
```

## Network (PC side)

- Ethernet 2 (Marvell AQtion 10G) has static `172.20.5.10/24` (set once, persistent).
- Hydra IP `172.20.5.201` was found by capturing its boot-time ARP (`find_hydra.ps1`).
- Camera IP `172.20.5.20` is **temporary** (GVCP FORCEIP); lost on camera power-cycle.
  Re-apply with `python gige_forceip.py 00:06:BE:11:70:BA 172.20.5.20` (current camera;
  prints "no ack" but works - check with ping) or make it persistent.

## Axis conventions (verified)

- **Goniometer (axis 1):** homed on limit switches (`ncal`, `nrm`), then zeroed at
  the centre. Range **+/-5.16 deg**, 0 = centre. 4 deg/s. Re-home after every power-up
  (open loop, no encoder).
- **Rotation (axis 2):** homed on the encoder index mark (`nrefmove`, approach in
  + direction), 0 = index mark, repeatability ~0.00003 deg. 360 deg/s, 10000 deg/s^2.
  Switches masked; never run `ncal` on it (would spin until the soft limit).
- Drives are off after Hydra power-up; `init` powers them. The Hydra also switches a
  drive off on a fault (status bit `0x100`, error 1004 seen) - `init` + re-home.
- Soft limits move with every `setnpos` (re-home), e.g. rotation now ~ -38864..33135.

## Software (`C:\dev\hydra`)

| File | Purpose |
|---|---|
| `hydra.py` | Hydra client: move, wait, status, `home_all()`, `move_together()`; CLI `--home`, `--move` |
| `demo.py` | simultaneous goniometer + rotation moves |
| `gige_discover.py` / `gige_forceip.py` | find GigE cameras / set temporary camera IP |
| `cam_auto.py` | one frame with auto exposure -> `first_image.png` |
| `live.py` | live view + **focus meter** (a = auto exp, +/- exposure, z = zoom, r = reset peak, s = snapshot, q = quit) |
| `find_hydra.ps1` | admin script that found the Hydra's IP |
| `dynamics.py` / `plot_dynamics.py` | speed sweep of both axes -> `dynamics_log.csv`, `dynamics.png` |
| `all_axes.py` | all 4 axes simultaneously, out-and-back at increasing speeds -> `all_axes.png` |
| `find_fiber.py` | Z scan with contact sheet to find the fiber (`find_fiber_sheet.png`) |
| `fiber_align.py` | edge fit (angle + centre), gonio/Z calibration (`--calib`), align at N rotations -> `fiber_align.csv` |
| `gui.py` | **GUI (main tool)**: live image + fiber overlay + live charts, 4 axes jog/go-to, exposure, snapshot, **MP4 record**, home/reset buttons, calibrate / align / rotate+align / scan+rotate+align, **LIVE** (as is / live align / "kayEi - coming soon"), results table, log -> `gui_log.txt` |
| `.venv` | Python 3.13 + Baumer neoAPI 1.6.0, numpy, Pillow, OpenCV, matplotlib, pipython 2.13 (neoAPI has no 3.14 wheel) |

Run everything with `C:\dev\hydra\.venv\Scripts\python`.

## Measured dynamics (2026-10-20)

- Rotation: set speeds 0.5-360 deg/s reached within 0.01 %; 10 turns at 360 deg/s
  out and back -> +0.00003 deg. 45 deg steps at 360 deg/s: settle into +/-1 mdeg in
  50-74 ms, overshoot mostly <0.1 mdeg (up to 10.5 mdeg, varies), hold error
  +/-0.02 mdeg (sign follows approach direction), jitter 0.015 mdeg.
- Goniometer: 0.25-4 deg/s exact (commanded; no encoder, physical motion unverified).
- Hydra query latency ~6.5 ms. Slowest links for live alignment: goniometer (4 deg/s)
  and camera frame time (~200 ms in current dim scene).
- **Open:** encoder index found at +0.0012 deg right after homing, but at -0.004 to
  -0.006 deg after heavy spinning (drifting between checks). Suspected warm-up.
  Cold re-check 2026-10-30 (after Hydra power-cycle, 3 index searches right after
  homing): +0.00064 / -0.00069 / -0.00102 deg, i.e. within ~+/-1 mdeg cold, which
  supports warm-up. Still to confirm: cold home -> spin hard for minutes -> re-check.

## Live tracking (2026-10-30, gui.py LIVE)

- Rotation turns continuously, scan sweeps, every fresh frame: goniometer removes the fiber
  angle, Z the offset (gain 0.6, dead band 0.005 deg / 0.3 px).
- 20 deg/s rotation + 2 mm/s scan: **44 corrections/s, angle error rms 0.019 deg (max 0.07),
  offset rms 9.6 um (max 54 um)**. Goniometer traces a sine (+/-0.4 deg) = fiber wobble vs
  rotation axis.
- Fiber edge detection: fiber is a transparent cylinder (bright rims, dim core, glints).
  Use OUTERMOST edges, each at half of its own rim height; needs exposure >= 10 ms
  (20 ms default, ~50 fps). At 5 ms the rims are too dim.
- PI C-663 over USB: ~32 ms per query, ~16 ms per MOV -> live loop uses commanded
  positions, polls only when needed. Hydra ~7 ms per query, camera frame 20 ms.
- AS IS vs LIVE align (same run, 20 deg/s + 2 mm/s scan, ~30 s each): angle rms 0.41 -> 0.022 deg
  (max 0.68 -> 0.059), offset rms 182 -> 10 um (max 352 -> 49 um), i.e. ~18x better.
- Rotation drive faults (Hydra switches drive off, status 0x100, error 1004) happened after
  extreme manual tilt tests; encoder count stayed exact. Recover: Reset drives + Home rotation.
- User session results (1 mm/s scan), live align: 20 deg/s -> angle rms 0.021 deg / offset
  9.7 um; 50 deg/s -> 0.039 deg / 9.1 um; **100 deg/s -> 0.111 deg / 27.6 um** (goniometer
  can't follow the faster wobble with pure feedback). AS IS at 20 deg/s: 0.27 deg / 151 um.
- Fiber tilt vs rotation axis (wobble): ~0.3-0.4 deg, clean sine (fit residual 0.01 deg).
- Possible next gains (= "kayEi", the future ultimate algorithm): feed-forward of the
  sine (learn gonio/Z vs rotation angle, move ahead of time, feedback only for the rest),
  faster PI link (RS-232?), separate threads for camera and commands.

## Session 2026-10-30 (summary)

- PI Software Suite installed (USB driver + GCS DLL), pipython in `.venv`; both C-663 fixed
  (wrong stage parameter sets, swapped mapping found by travel) and saved with WPA.
- 4 axes simultaneously (`all_axes.py`). Cold index re-check done (see above).
- New camera + lens: found the fiber by Z scan, centred it; camera image is rotated 90 deg.
- `fiber_align.py`: edge-fit measurement, gonio/Z calibration, alignment at 6 rotations
  (all within 0.02 deg / 1 px in 1-2 iterations, ~1.5 s each).
- `gui.py` with LIVE tracking, AS IS comparison (~18x better aligned), charts, recording,
  home/reset buttons, drive-fault detection, log file.
- Drive faults were caused by extreme manual tilt tests, not hardware.

## Next session

0. Push the local git repo (`main`, first commit `f435af3`) to a new GitHub repository.
   Check the git identity first (repo-local: name `wysmo`, email wysmolek@gmail.com).
1. Start: Hydra power-cycled? -> GUI **Home all Hydra** (or `python hydra.py 172.20.5.201 --home`).
   Camera power-cycled? -> `python gige_forceip.py 00:06:BE:11:70:BA 172.20.5.20`.
   Then `C:\dev\hydra\.venv\Scripts\python C:\dev\hydra\gui.py`, **Calibrate**.
2. Record the demo movie (Record -> AS IS -> STOP -> Live align -> STOP -> Stop rec).
3. Lower exposure / check what is in the chuck (1.1 mm wide: coated fiber? capillary?).
4. Track the **upper edge** instead of the centre line (laser simulation needs the edge),
   decide which image side is "upper" with the camera rotated 90 deg (or rotate the camera).
5. "kayEi": feed-forward of the wobble sine to keep accuracy at 100+ deg/s.
6. Focus is not tracked (no axis moves camera-fiber distance) - check focus over the turn.
7. Optional: make camera IP persistent; hard-spin index re-check.

## Roadmap

1. [x] Connect to Hydra, identify axes, test moves
2. [x] Home both Hydra axes (switches / encoder index), goniometer zeroed at centre
3. [x] Simultaneous moves of both axes
4. [x] Find camera, grab image, auto exposure, live view
5. [ ] Make camera IP persistent
6. [x] Fit the proper lens, focus on the fiber; pixel size ~20 um/px (from Z calibration)
7. [~] Detect the fiber in the image: both edges, centre line, angle (done); **upper edge
       as the tracked reference: to do**
8. [x] Connect both C-663.11 controllers over USB, reference (FNL) and test
9. [x] Camera-to-stage calibration (gonio/Z -> image angle/offset, `fiber_align.py --calib`)
10. [x] Align once (no rotation): goniometer -> fiber straight, Z -> centred (centre line)
11. [x] Rotate 6 angles, re-align at each (GUI: any N)
12. [x] Translate along the fiber and repeat (GUI: scan + rotate + align)
13. [x] Stretch: live simultaneous rotation + scan + alignment (44 corrections/s);
        slowest links: PI USB queries (32 ms), goniometer feedback at high rotation speed
14. [ ] "kayEi": feed-forward / learned alignment for high rotation speeds

## Open questions

- What exactly is in the chuck (~1.1 mm wide)? Which image side is the "upper" edge?
- Translation step along the fiber.
- Hydra error 1004 meaning (no Hydra manual in the project folder).

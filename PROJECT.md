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

## kayEi - predictive alignment (2026-10-31)

Design, measurements and results: **`docs/kayei_method.md`** (PhD write-up, kept current).
In short: the wobble is a sine in the rotation angle (residual 0.007 deg / 0.1 px,
repeatable), the rotation angle is known in advance, so goniometer and Z are driven to
where the fiber will be (fixed-rate tracker thread, lead ~30-45 ms), a residual table
learns the rest, a slow trim removes drift. At 100 deg/s with scan: 0.008 deg / 7 um
(feedback at 100 deg/s: 0.11 deg / 28 um). Full comparison 20-360 deg/s: `kayei_sweep.py`
-> `kayei_eval.py` -> docs section 7.3.

Evening 2026-10-31, fiber re-clamped tilted (1.24 deg, 1.75 mm eccentric = 12x the wobble):
the steppers turned out to execute a 50 Hz target stream as stop-and-go bursts, so the
tracker now streams **velocity-shaped** (speed per tick = trajectory speed, `kayei.VSHAPE`),
the residual learner is a capped **Fourier series** (the bin table wound up), the rotation
clock filters its encoder re-syncs, and a proportional Z offset feedback was added. Result
0.031 deg / 31 um at 100 deg/s (97.5 % removed); from 200 deg/s the goniometer's 4 deg/s is
the limit by construction. Exposure is now 4 ms (`fiber_align.EXPOSURE_MS`, the fiber glows
under the new lamp). Details: docs 5c, 6.7, 7.3d-f.

Files: `kayei.py` (model, clock, predictor, tracker), `kayei_learn.py` (latency / wobble
measurements), `kayei_sweep.py`, `kayei_eval.py`, data + scratch tests in `kayei_data/`.
GUI: LIVE mode "kayEi (predictive)" with "relearn model" and "residual learning" switches.

Hydra traps (fixed in `hydra.py`): `setnpos v` makes the position **-v** and drags the soft
limits to [pos-72000, pos]; a move beyond a soft limit is refused with **error 1004**.
`Hydra.rezero_turn()` (called at every LIVE start) keeps endless rotation inside the limits.
Camera: `gige_forceip.py` prints "no ack" but works; illumination varies (kayei_learn.Rig
raises the camera gain automatically, the GUI does not yet).

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

## Session 2026-10-31 (summary)

Full detail with numbers, figures and raw data: `docs/kayei_method.md` (sections 3.1, 5b, 5c,
6, 7.1-7.5, 8). In one paragraph: measured all latencies (58 ms command -> image, 30 ms
frame -> encoder), showed the wobble is a repeatable sine, built **kayEi** (feed-forward of
the wobble from a learned model, fixed-rate tracker thread, residual learner, trim) and
validated it against as-is and feedback at 20-360 deg/s with and without scanning. Measured
the image scale (20.46 um/px) and fiber diameter (1.114 mm), reconstructed the 3D fiber axis
from one camera. Evening: fiber re-clamped 1.24 deg tilted / 1.75 mm eccentric - found that
the steppers execute a dense target stream as stop-and-go bursts (velocity-shaped streaming
now), that the bin residual table winds up (Fourier learner now), that the rotation clock
needs filtered re-syncs, and that the feedback gain must scale with frame rate. kayEi on the
tilted fiber: 0.03 deg / 20-50 um up to 100 deg/s; goniometer-limited (4 deg/s) above.
Pushed to github.com/mwysmolek/kayEi_fiber_align.

## Session 2026-11-01 (summary)

Sample 2: an opaque black rod, 1.106 mm, backlit (silhouette). Morning: Hydra power-cycled,
the goniometer's LOWER LIMIT SWITCH turned out dead (homing ground into the end stop twice;
truth found by stepping with the camera as reference; `home_all` now uses the upper switch
only); both drives tripped once at their first move after power-up (`move_abs` re-powers).
Silhouette measurement added to `fiber_align.measure` (POLARITY, bounded dark run, local
background, MAD clipping; a lamp reflection on the rod flank needed the last one). Rod: tilt
0.44 deg, ecc. 0.49 mm, sine residual 0.003 deg / 2 um (clean chuck grip). 30-run comparison:
kayEi 0.008-0.043 deg / 6-45 um over 20-360 deg/s with and without scan, no mechanical
limit reached; feedback 0.035-0.29 deg. Docs 7.7.

Evening: Olympus 4x objective, first on a long tube (2.35 um/px: 6 um at 100 deg/s in focus,
but the depth of field is smaller than the rod's out-of-plane motion along the scan - sweep
invalid, focus axis is the next hardware step), then on a short tube (6.57 um/px, scanning
works: 0.011 deg / 12 um, with scan 0.015 / 14). Sample 3, a transparent 0.55 mm fiber: the
fit bridges its bright core (`GAP_MM`), 549 um, exposure 10 ms; wobble 1.77 deg tilt / 2.2 mm
eccentricity - the largest so far, 3.2 deg/s of goniometer needed at 100 deg/s; kayEi
0.038 deg / 49 um at 100 deg/s, goniometer at its 4 deg/s cap; the scan run stopped after
10 s for an unlogged reason. Docs 7.8, 7.9, docs/summary.md.

## Optics log

- **EO 6 mm** (Edmund Optics 6 mm lens on the Baumer VCXG-15M): everything up to and including
  2026-11-01 17:40 - all of docs/kayei_method.md sections 3-7.7, image scale 20.45 um/px
  (48.9 px/mm), 1440 x 1080 px = 29.4 x 22.1 mm field, exposures 20 / 4 / 2 ms depending on
  the light. Git tag `EO-6mm` marks this state.
- **Olympus Plan N 4x on an extension tube** (from 2026-11-01 18:00): measured 425.4 px/mm =
  2.351 um/px (effective 1.47x), field 3.39 x 2.54 mm, exposure 20 ms with the backlight, rod
  1.1 mm = 477 px fits with +/-1.1 mm margin (two-edge fit stays; `EDGE_ONLY` exists for
  thicker samples). Per-frame noise 0.016 um (was 0.14). Lever arm 524 px/deg -> the rod MUST
  be centred in the frame and calibration steps are 0.5 deg / 0.5 mm (automatic from
  `PX_PER_MM`). All scale constants now come from `fiber_align.PX_PER_MM`. Soft focus (35 um
  edge) and dust on the sensor window - cosmetic so far. kayEi 100 deg/s: 0.011 deg / 6 um.
  `fiber_diameter.py` Z steps (+/-2 mm) exceed this field - use +/-0.25 mm if needed.
  **Depth of field is the limit** (+/-40 um): the rod's out-of-plane tilt moves it 0.3 mm towards/away
  from the objective over the 40 mm scan -> silhouette blurred beyond use at scan 5-15 mm when
  focused at 31; the 30-run sweep on this optic is invalid for that reason (docs 7.8). Usable in
  focus at one scan position (6 um at 100 deg/s); scanning needs a focus axis or re-focusing.

- **Olympus 4x on the SHORT tube** (2026-11-01 19:30, current): 152.1 px/mm = 6.57 um/px (0.52x),
  field 9.5 x 7.1 mm, 20 ms (background clipped white on purpose: dust specks drop out of the
  thresholds, angle noise 0.0002 deg). Depth of field covers the 40 mm scan (blur 11-15 px
  everywhere). kayEi 100 deg/s: 0.011 deg / 12 um, with scan 0.015 / 14 um. Lever 265 px/deg.
  This is the working configuration; a sensor-window cleaning would allow unclipped exposures.
  Exposure per sample on this optic: rod 20 ms (clipped), transparent 0.55 mm fiber 10 ms
  (`fiber_align.EXPOSURE_MS`).

## Next session: more samples (thin fibers, glass capillary)

Samples so far: (1) 1.1 mm glass fiber, (2) 1.1 mm opaque rod, (3) transparent 0.55 mm fiber
(in the chuck at the end of 2026-11-01; docs 7.9). First thing to do with (3): kayEi at
30-60 deg/s without and with scan, where its 1.84 deg wobble leaves the goniometer margin
(the 100 deg/s check was goniometer-limited, 0.038 deg / 49 um, and the scan run stopped after
10 s unexplained - watch `kayei_data/*.log` for the worker's message this time). Then the
30-run sweep if wanted, re-clamp straighter if the tilt is the limit.

Headless check scripts (scratchpad `check_tube2.py` pattern): write the RAW output to a log
file (the grep filter in `transp_seq.sh` swallowed the error), and put a timeout on the
`while w.state["busy"]` wait - two of them hung for 1.5 h with the hardware open. At the end
of a day: kill leftover `python.exe`, then read and restore the axis defaults (rot 360 deg/s,
gonio 4 deg/s accel 50, Z VEL 10; scratchpad `wrapup_restore.py` did that on 2026-11-01).

The plan is to put other samples in the chuck (thinner fibers, a glass capillary) and see
what the measurement and kayEi achieve. Per sample, in this order (each step ~1-5 min):

1. **Start-up**: Hydra power-cycled? -> GUI **Home all Hydra** (since 2026-11-01 the goniometer homes on
   its UPPER switch only - the lower switch is dead, the stage hits its end ~0.3 deg before it;
   never run `ncal` on axis 1). Camera power-cycled? ->
   `python gige_forceip.py 00:06:BE:11:70:BA 172.20.5.20`. PI hub plugged in.
   `C:\dev\hydra\.venv\Scripts\python gui.py`, **Ref. PI stages**, **Calibrate**.
2. **Exposure / detection check** (illumination or sample changed):
   `python kayei_data/expo_sweep.py` prints width, noise and saturation per exposure; set
   `fiber_align.EXPOSURE_MS` accordingly (4x short tube + backlight: 20 ms rod, 10 ms transparent
   0.55 mm fiber; EO lens: 4 ms glowing fiber, 2 ms backlit rod). The band fit needs a contrast of
   at least 40 counts and a width of at least 3 px: a 125 um fiber is ~19 px at 6.57 um/px, a 250 um
   one ~38 px (it was 6 / 12 px with the 6 mm lens). If a
   thin fiber is not found, raise the gain (GUI "Auto gain") or the exposure before touching
   the code; if it is found but noisy, the band fit's `BAND` (30 rows) can be made larger.
   A transparent sample in the backlight shows two dark rims and a bright core (cylinder lens);
   `fiber_align.GAP_MM` (0.6 mm) bridges the core so the outer rims are fitted - raise it for a
   wider capillary; check `width` is stable frame to frame (0.55 mm fiber: 84.6-85.4 px).
3. **Find and centre**: GUI Z scan / `find_fiber.py` if the sample is out of the frame.
4. **Scale and diameter**: `python fiber_diameter.py` (Z steps -> px/mm, width -> mm).
5. **Uncontrolled motion, 3D axis**: `python fiber_wobble_map.py` (5 scan positions, 2 turns
   each at 60 deg/s; figures wobble_2d/3d.png + wobble_4d.mp4 - rename them per sample, the
   script overwrites). Tells the tilt/eccentricity of the clamping and the sine residual,
   i.e. how much there is for kayEi to do and whether the goniometer can do it (needs
   amplitude x 2 pi x turns/s < 4 deg/s).
6. **kayEi**: GUI LIVE -> kayEi (predictive) with "relearn model" on, or headless
   `python kayei_sweep.py` (30 runs, 20 min) -> `python kayei_eval.py <date-prefix>` (table
   + figure). Movies: `python kayei_record.py 100 40`.
7. Put the numbers in `docs/kayei_method.md` (new section per sample) and commit.

Things to keep in mind from the evening (details docs 6.7, 7.3e-f):
- Goniometer: 4 deg/s, accel 50 (400 loses steps), 0.034 deg motion ripple when streamed;
  its ripple leaks into the offset through the lever arm (-62 px/deg at scan 25 mm).
- Run-to-run scatter of kayEi is +/-10 %; the 100 deg/s offset drifted 25 -> 45 um over the
  evening at fixed settings (clamping?) - check the chuck before the first run.
- Live align (feedback) diverges from 100 deg/s with scan on a 1.26 deg wobble; the tilt
  of the new sample decides whether it is a usable baseline.
- Open from before: person's hands-on minutes for docs 7.5; upper-edge tracking; focus
  (out-of-plane) is predicted but not corrected; a Hydra velocity/trajectory mode would
  remove the goniometer's ripple (Venus-3 manual).

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
14. [x] "kayEi": predictive alignment (feed-forward + learned residual), 0.008 deg / 7 um at 100 deg/s

## Open questions

- What exactly is in the chuck (~1.1 mm wide)? Which image side is the "upper" edge?
- Translation step along the fiber.
- Hydra error 1004 meaning (no Hydra manual in the project folder).

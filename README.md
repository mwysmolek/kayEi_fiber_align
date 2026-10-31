# kayEi_fiber_align

Fiber alignment station: an optical fiber in a 3-jaw chuck on a rotation stage (on a
goniometer, both on a PI/Micos Hydra), a Baumer GigE camera on two PI C-663 linear
stages (Z = up/down, scan = along the fiber). The camera finds the fiber, the
goniometer makes it straight, the Z stage centres it - at discrete rotation angles
or live while the fiber rotates and the camera scans.

Everything about the setup, what is verified and what is next is in
[PROJECT.md](PROJECT.md). The method, all measurements, figures and results (feedback vs
predictive alignment "kayEi", 20-360 deg/s) are in **[docs/kayei_method.md](docs/kayei_method.md)**.

## Run

```
C:\dev\hydra\.venv\Scripts\python gui.py      # main tool: live view, axes, align, LIVE, record
C:\dev\hydra\.venv\Scripts\python live.py     # plain live view with focus meter
```

| File | Purpose |
|---|---|
| `gui.py` | GUI: live image + fiber overlay + charts, axes, home/reset, align sequences, LIVE modes, MP4 recording |
| `fiber_align.py` | fiber edge fit, goniometer/Z calibration, alignment loop |
| `hydra.py` | Hydra (Venus-3) client for goniometer + rotation stage |
| `kayei.py` | kayEi: wobble model, rotation clock, predictor, fixed-rate tracker thread, residual learning |
| `kayei_learn.py`, `fiber_diameter.py`, `fiber_wobble_map.py` | identification: latency, wobble sine, image scale / diameter, 3D wobble map |
| `kayei_sweep.py`, `kayei_eval.py`, `kayei_record.py` | comparison sweep, evaluation + figure, recorded demo run |
| `find_fiber.py`, `all_axes.py`, `dynamics.py`, `spin_steps.py` | earlier measurements / tests |

Needs: Python 3.13 venv with Baumer neoAPI, pipython, OpenCV, numpy, Pillow,
matplotlib; PI Software Suite (USB driver + GCS DLL) for the C-663 controllers.

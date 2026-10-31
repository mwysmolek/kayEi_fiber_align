import sys, time
sys.path.insert(0, r"C:\dev\hydra")
import numpy as np, cv2
from kayei_learn import Rig
rig = Rig()
try:
    for e in (20, 10, 6, 4, 2.5, 1.5, 1.0):
        rig.cam.f.ExposureTime.Set(e * 1000); time.sleep(0.3)
        ms, ts = [], []
        for i in range(20):
            t, m = rig.fresh(); ts.append(t)
            if m: ms.append((m["angle"], m["x"], m["width"], m["rms"], m["n"]))
        rig.cam.ClearImages(); img = rig.cam.GetImage(2000).GetNPArray().squeeze()
        ms = np.array(ms)
        line = f"exp {e:4.1f} ms: period {np.mean(np.diff(ts))*1000:5.1f} ms, bg median {np.median(img):5.1f}, p99.9 {np.percentile(img, 99.9):5.0f}, sat {(img>=254).mean()*100:5.2f}% | det {len(ms)}/20"
        if len(ms):
            line += f" angle sd {ms[:,0].std():.4f} x sd {ms[:,1].std():.3f} width {ms[:,2].mean():6.2f}+-{ms[:,2].std():.2f} rms {ms[:,3].mean():.2f} n {ms[:,4].mean():.0f}"
        print(line, flush=True)
        cv2.imwrite(rf"C:\dev\hydra\kayei_data\illum_{e:g}ms.png", img)
    rig.cam.f.ExposureTime.Set(20000)
finally:
    rig.close()

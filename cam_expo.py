import neoapi, numpy as np
from PIL import Image
cam = neoapi.Cam(); cam.Connect("172.20.5.20")
best = None
for us in (20000, 100000, 400000):
    cam.f.ExposureTime.Set(us)
    cam.GetImage()  # flush frame taken with old exposure
    a = cam.GetImage().GetNPArray().squeeze()
    sat = (a == 255).mean() * 100
    print(f"exposure {us/1000:6.0f} ms: mean {a.mean():6.1f}  max {a.max():3d}  saturated {sat:.2f}%")
    if best is None or (a.mean() < 150 and sat < 1): best = (us, a)
us, a = best
Image.fromarray(a).save("test_image.png"); print("saved test_image.png at", us/1000, "ms")

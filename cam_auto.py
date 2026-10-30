"""Grab one frame with the camera's own auto exposure and save it as first_image.png."""
import time
import neoapi
from PIL import Image

cam = neoapi.Cam()
cam.Connect("172.20.5.20")
cam.GetFeature("ExposureAuto").SetString("Continuous")


def grab():
    while True:
        img = cam.GetImage(2000)  # ms timeout, long exposures need more than the default
        if not img.IsEmpty():
            return img.GetNPArray().squeeze()


history = []
t0 = time.time()
while time.time() - t0 < 60:
    a = grab()
    exp = cam.GetFeature("ExposureTime").GetDouble()
    history.append(exp)
    print(f"{time.time() - t0:5.1f}s exposure {exp / 1000:7.1f} ms  mean {a.mean():6.1f}")
    recent = history[-6:]
    if len(recent) == 6 and max(recent) - min(recent) < 0.02 * max(recent):
        break  # exposure unchanged (within 2%) over the last 6 frames
cam.GetFeature("ExposureAuto").SetString("Off")  # freeze the settled exposure
Image.fromarray(a).save("first_image.png")
print(f"saved first_image.png (exposure {exp / 1000:.1f} ms, mean {a.mean():.1f}, "
      f"max {a.max()}, saturated {(a == 255).mean() * 100:.2f}%)")

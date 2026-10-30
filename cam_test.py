import neoapi, numpy as np
from PIL import Image
cam = neoapi.Cam()
cam.Connect("172.20.5.20")
f = cam.f
for name in ("DeviceModelName", "DeviceSerialNumber", "DeviceFirmwareVersion", "Width", "Height",
             "PixelFormat", "ExposureTime", "Gain", "AcquisitionFrameRate", "DeviceTemperature",
             "GevSCPSPacketSize", "GevCurrentIPConfigurationPersistentIP"):
    try:
        print(f"{name:40} {cam.GetFeature(name).GetString()}")
    except Exception as e:
        print(f"{name:40} <{type(e).__name__}>")
img = cam.GetImage()
a = img.GetNPArray().squeeze()
print("image", a.shape, a.dtype, "min", a.min(), "max", a.max(), "mean", round(float(a.mean()), 1))
Image.fromarray(a).save("test_image.png")
print("saved test_image.png")

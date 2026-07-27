import os, sys, time
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
import cv2
from dwarflab_controller import DwarfLab

ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.137.13"
out = r"C:\Users\TUTU\Desktop\workspace\frame_tele_4k.png"

d = DwarfLab(host=ip)
print("connecting ws ...", flush=True)
ok = d.connect(timeout=10)
print("ws connected:", ok, flush=True)
if not ok:
    print("ABORT: no ws connection", flush=True)
    sys.exit(1)

d.set_master_lock(True)
d.enter_camera(1)
time.sleep(1.0)

print("switching tele resolution to 4K (resolution_type=0) BEFORE open_camera", flush=True)
d.switch_resolution(0)   # 0 = XGZ_RESOLUTION_TYPE_4K, 1 = 1080P
time.sleep(1.0)

d.open_camera(rtsp_encode_type=1)
time.sleep(3.0)

url = f"rtsp://{ip}/ch0/stream0"
print("opening", url, flush=True)
cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
print("rtsp opened:", cap.isOpened(), flush=True)
frame = None
for i in range(100):
    okf, fr = cap.read()
    if okf and fr is not None:
        frame = fr
        break
    time.sleep(0.1)
if frame is not None:
    print("FRAME shape:", frame.shape, flush=True)
    cv2.imwrite(out, frame)
    print("saved", out, flush=True)
else:
    print("NO FRAME RECEIVED", flush=True)
cap.release()
try:
    d.disconnect()
except Exception:
    pass

import os, sys, time
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
import cv2
from dwarflab_controller import DwarfLab, _field

ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.137.13"
CMD_CAMERA_TELE_SWITCH_RESOLUTION = 10047

d = DwarfLab(host=ip)
print("connecting ws ...", flush=True)
ok = d.connect(timeout=10)
print("ws connected:", ok, flush=True)
if not ok:
    sys.exit(1)

d.set_master_lock(True)
d.enter_camera(1)
time.sleep(1.0)
d.open_camera(rtsp_encode_type=1)
time.sleep(2.5)

url = f"rtsp://{ip}/ch0/stream0"

def grab_shape():
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    shape = None
    for i in range(60):
        okf, fr = cap.read()
        if okf and fr is not None:
            shape = fr.shape
            break
        time.sleep(0.1)
    cap.release()
    return shape

print("baseline (no switch sent yet):", grab_shape(), flush=True)

for val in [0, 1, 2, 3]:
    payload = _field(1, 0, val)  # force explicit encoding even for val=0
    print(f">>> sending switch_resolution explicit field value={val} payload={payload!r}", flush=True)
    d.send(CMD_CAMERA_TELE_SWITCH_RESOLUTION, payload)
    time.sleep(3.0)
    shape = grab_shape()
    print(f"    resulting frame shape for value={val}: {shape}", flush=True)

try:
    d.disconnect()
except Exception:
    pass

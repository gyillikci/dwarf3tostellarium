import os, sys, time
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
import cv2
from dwarflab_controller import DwarfLab, parse_ws_packet, describe_resp_code

ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.137.13"

d = DwarfLab(host=ip)

orig_on_message = d._on_message
def spy(ws, msg):
    if isinstance(msg, bytes):
        pkt = parse_ws_packet(msg)
        print(f"[RAW] cmd={pkt['cmd']} type={pkt['type']}"
              f"{' ('+describe_resp_code(pkt['type'])+')' if pkt['type']<0 else ''}"
              f" data={pkt['data'][:60]!r} len={len(pkt['data'])}", flush=True)
    return orig_on_message(ws, msg)
d._on_message = spy

print("connecting ws ...", flush=True)
ok = d.connect(timeout=10)
print("ws connected:", ok, flush=True)
if not ok:
    sys.exit(1)

d.set_master_lock(True)
d.enter_camera(1)
time.sleep(1.0)
d.open_camera(rtsp_encode_type=1)
time.sleep(1.5)

print(">>> requesting current params (CMD 10036)", flush=True)
d.get_all_params()
time.sleep(1.5)

print(">>> switch_resolution(0) = 4K", flush=True)
d.switch_resolution(0)
time.sleep(2.0)
print("last_error after switch:", d.state.get("last_error"), flush=True)

print(">>> requesting params again", flush=True)
d.get_all_params()
time.sleep(1.5)

time.sleep(3)
try:
    d.disconnect()
except Exception:
    pass

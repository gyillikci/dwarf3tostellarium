import os, sys, time
from dwarflab_controller import DwarfLab, _field

ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.137.13"
CMD_CAMERA_TELE_SWITCH_RESOLUTION = 10047
record_seconds = 5

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
time.sleep(2.0)

print("explicitly setting resolution_type=0 (4K) before recording", flush=True)
d.send(CMD_CAMERA_TELE_SWITCH_RESOLUTION, _field(1, 0, 0))
time.sleep(1.5)

print("start_record()", flush=True)
d.start_record()
time.sleep(record_seconds)
print("stop_record()", flush=True)
d.stop_record()
time.sleep(3.0)

try:
    d.disconnect()
except Exception:
    pass
print("done", flush=True)

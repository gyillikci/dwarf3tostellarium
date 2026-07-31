"""
Live test of two commands newly discovered in DWARF3 firmware v1.5.0.1
(present in v1.5.0.1's magni, absent from v1.3.34):

  - ReqGetDeviceStateInfo  (cmd 16405, MODULE_TASK_CENTER=14) — no payload,
    returns a consolidated device/camera/motor state snapshot including the
    real on-device tele/wide FOV (h_fov/v_fov), resolution, temperatures, etc.
  - ReqSetPreviewQuality   (cmd 10050=tele / 12036=wide) — {level, quality}.
    NOTE: these two cmd IDs were previously mislabeled in this repo as
    "v3_open_tele"/"v3_open_wide" (CMD_V3_CAMERA_*_OPEN_CAMERA); ground truth
    from the firmware's own zlog strings says they are SET_PREVIEW_QUALITY,
    not camera-open. See memory dwarf3-firmware-version-diff.md.

Read-only-first: queries GetDeviceStateInfo before touching anything else.
Then grabs a wide RTSP still, nudges preview quality, grabs another still,
and reports simple size/sharpness deltas so the effect can be judged even
without a human looking at the live view. Always reverts to level=1 at the
end (the value this repo's existing code has always sent, proven harmless).
"""
import sys
import time
import struct

sys.path.insert(0, r"C:\Users\TUTU\Desktop\workspace\dwarf3\dwarf3tostellarium")
import dwarflab_controller as ctl_mod

CMD_GET_DEVICE_STATE_INFO = 16405
MODULE_TASK_CENTER = 14
CMD_SET_PREVIEW_QUALITY_TELE = 10050
CMD_SET_PREVIEW_QUALITY_WIDE = 12036


def build_descriptor_pool():
    """Load the recovered FileDescriptorProto blobs (extracted straight from
    magni's own protobuf reflection data) into a real descriptor pool so we
    can parse ResGetDeviceStateInfo properly instead of guessing offsets."""
    from google.protobuf import descriptor_pb2, descriptor_pool, message_factory

    base = r"C:\Users\TUTU\Desktop\workspace\dwarf3\firmware"
    pool = descriptor_pool.DescriptorPool()
    # dependency order matters: base -> notify/astro/panorama -> task_center
    order = ["base.proto", "astro.proto", "panorama.proto", "notify.proto", "task_center.proto"]
    for name in order:
        path = rf"{base}\descriptors_{name.replace('.', '_')}.bin"
        with open(path, "rb") as f:
            fdp = descriptor_pb2.FileDescriptorProto()
            fdp.ParseFromString(f.read())
        pool.Add(fdp)
    ResGetDeviceStateInfo = message_factory.GetMessageClass(
        pool.FindMessageTypeByName("ResGetDeviceStateInfo"))
    return ResGetDeviceStateInfo


def find_device_ip():
    import subprocess
    out = subprocess.run(["arp", "-a"], capture_output=True, text=True).stdout
    target_mac_variants = {"9c-b8-b4-91-6d-e2", "9c:b8:b4:91:6d:e2"}
    for line in out.splitlines():
        line_l = line.lower()
        if any(m in line_l for m in target_mac_variants):
            return line.split()[0]
    return None


def grab_rtsp_frame(url, out_path):
    import cv2
    cap = cv2.VideoCapture(url)
    ok, frame = False, None
    for _ in range(30):
        ok, frame = cap.read()
        if ok:
            break
    cap.release()
    if not ok:
        return None
    cv2.imwrite(out_path, frame)
    import cv2 as _cv2
    gray = _cv2.cvtColor(frame, _cv2.COLOR_BGR2GRAY)
    sharpness = _cv2.Laplacian(gray, _cv2.CV_64F).var()
    return {"path": out_path, "shape": frame.shape, "sharpness": sharpness}


def main():
    ip = find_device_ip()
    if not ip:
        print("DEVICE NOT FOUND on hotspot ARP table (9c:b8:b4:91:6d:e2). "
              "Power on / connect the DWARF3 and retry.")
        return
    print(f"Device found at {ip}")

    ResGetDeviceStateInfo = build_descriptor_pool()

    responses = {}

    def on_notify(pkt):
        if pkt["cmd"] == CMD_GET_DEVICE_STATE_INFO:
            responses["device_state_raw"] = pkt["data"]

    c = ctl_mod.DwarfLab(host=ip, on_notify=on_notify)
    ok = c.connect(timeout=10)
    print("connected:", ok)
    if not ok:
        print("Could not connect to control WebSocket. Is another client "
              "(the phone app) holding the single-client lock?")
        return

    try:
        # --- 1. Read-only first: GetDeviceStateInfo ---
        print("\n=== Sending ReqGetDeviceStateInfo (16405) ===")
        c.send(CMD_GET_DEVICE_STATE_INFO, b"")
        time.sleep(2)
        raw = responses.get("device_state_raw")
        if raw is None:
            print("No response received for 16405 within 2s.")
        else:
            print(f"Raw response: {len(raw)} bytes")
            try:
                msg = ResGetDeviceStateInfo()
                msg.ParseFromString(raw)
                print(msg)
                tele = msg.tele_camera_state_info
                wide = msg.wide_camera_state_info
                print(f"TELE h_fov={tele.h_fov} v_fov={tele.v_fov} "
                      f"res={tele.resolution_width}x{tele.resolution_height}")
                print(f"WIDE h_fov={wide.h_fov} v_fov={wide.v_fov} "
                      f"res={wide.resolution_width}x{wide.resolution_height}")
            except Exception as e:
                print(f"Decode failed ({e}); raw hex: {raw.hex()}")

        # --- 2. Grab a wide-stream still before touching preview quality ---
        wide_url = f"rtsp://{ip}:554/ch1/stream0"
        print(f"\n=== Grabbing baseline wide frame from {wide_url} ===")
        before = grab_rtsp_frame(wide_url, "scratch_wide_before.jpg")
        print("before:", before)

        # --- 3. Bump wide preview quality: level=2 (existing code has only
        #        ever sent level=1; this is the first test of a higher value) ---
        print("\n=== Sending ReqSetPreviewQuality(wide) level=2 quality=90 (cmd 12036) ===")
        payload = ctl_mod._field(1, 0, 2) + ctl_mod._field(2, 0, 90)
        c.send(CMD_SET_PREVIEW_QUALITY_WIDE, payload)
        time.sleep(2)
        print("last_error:", c.state.get("last_error"))

        after = grab_rtsp_frame(wide_url, "scratch_wide_after_level2.jpg")
        print("after (level=2):", after)

        if before and after:
            print(f"\nSharpness delta: {after['sharpness'] - before['sharpness']:+.2f} "
                  f"(before={before['sharpness']:.2f}, after={after['sharpness']:.2f})")

    finally:
        # Always revert to the value this repo's existing code has always
        # sent, known harmless.
        print("\n=== Reverting wide preview quality to level=1 (known-safe default) ===")
        payload = ctl_mod._field(1, 0, 1)
        c.send(CMD_SET_PREVIEW_QUALITY_WIDE, payload)
        time.sleep(1)
        c.disconnect()


if __name__ == "__main__":
    main()

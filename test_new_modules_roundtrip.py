"""Round-trip self-test for the 2026-07-31 PANORAMA/SHOOTING_SCHEDULE/PARAM/
DEVICE/VOICE_ASSISTANT additions: build a payload with the new DwarfLab
methods (never connecting to a real device — `send` is monkeypatched to just
capture bytes) and decode it back with dwarf_protobuf's schema decoder,
asserting the round-tripped fields match what was sent in.
"""
import dwarflab_controller as D
import dwarf_protobuf as P

captured = []


class FakeDwarfLab(D.DwarfLab):
    def __init__(self):
        pass  # skip the real __init__ (no socket/state needed for this test)

    def send(self, cmd, data=b""):
        captured.append((cmd, data))


def last():
    cmd, data = captured[-1]
    return cmd, P.decode_with_schema(data, P.SCHEMAS[cmd])


d = FakeDwarfLab()
fails = []


def check(label, cond):
    print(("PASS " if cond else "FAIL ") + label)
    if not cond:
        fails.append(label)


# ── Panorama ─────────────────────────────────────────────────────────────
d.panorama_start_stitch_upload(resource_id=12345, user_id="u1", ak="AK", sk="SK",
                                token="TOK", bucket="b1", bucket_prefix="pfx",
                                panorama_name="pano1")
cmd, dec = last()
check("panorama stitch upload cmd id", cmd == 15503)
check("panorama stitch upload fields", dec == {
    "resource_id": 12345, "user_id": "u1", "app_platform": 0,
    "panorama_name": "pano1", "ak": "AK", "sk": "SK", "token": "TOK",
    "bucket": "b1", "bucket_prefix": "pfx", "from": "", "env_type": "",
})

d.panorama_update_framing_rect(0.1, 0.2, 0.8, 0.9)
cmd, dec = last()
check("panorama framing rect", dec == {
    "norm_x_tl": 0.1, "norm_y_tl": 0.2, "norm_x_br": 0.8, "norm_y_br": 0.9})

# ── Shooting Schedule ────────────────────────────────────────────────────
d.shooting_schedule_sync(schedule_id="sch1", schedule_name="Deep Sky",
                          device_id=2, start_time=1000, end_time=2000)
cmd, dec = last()
check("shooting_schedule sync cmd id", cmd == 16100)
inner = dec["shooting_schedule"]
check("shooting_schedule sync nested fields",
      inner["schedule_id"] == "sch1" and inner["schedule_name"] == "Deep Sky" and
      inner["device_id"] == 2 and inner["start_time"] == 1000 and
      inner["end_time"] == 2000)

d.shooting_schedule_delete("sch1", password="pw")
cmd, dec = last()
check("shooting_schedule delete", cmd == 16108 and
      dec == {"id": "sch1", "password": "pw"})

d.shooting_schedule_get_task_by_id("task1")
cmd, dec = last()
check("shooting_schedule get_task_by_id (zlog-confirmed 16104)",
      cmd == 16104 and dec == {"id": "task1"})

# ── Param ────────────────────────────────────────────────────────────────
d.param_set_exposure(72339069014638593, 135, mode=1)
cmd, dec = last()
check("param set_exposure", cmd == 16700 and
      dec == {"param_id": 72339069014638593, "mode": 1, "value": 135})

d.param_set_float(999, 3.5)
cmd, dec = last()
check("param set_float wire-type-5 round-trip", cmd == 16704 and
      dec["param_id"] == 999 and abs(dec["value"] - 3.5) < 1e-5)

d.param_set_bool(1000, True)
cmd, dec = last()
check("param set_bool", cmd == 16705 and dec == {"param_id": 1000, "value": True})

d.param_set_auto(camera_type=0, shooting_tech=1, is_auto=True)
cmd, dec = last()
check("param set_auto", cmd == 16706 and
      dec == {"camera_type": 0, "shooting_tech": 1, "is_auto": True})

# ── Device ───────────────────────────────────────────────────────────────
d.set_lens_defog(True)
cmd, dec = last()
check("device lens_defog", cmd == 17000 and dec == {"state": 1})

# ── Voice Assistant ──────────────────────────────────────────────────────
d.voice_take_photo()
cmd, dec = last()
check("voice take_photo", cmd == 16800 and
      dec["command_type"] == "TAKE_PHOTO")

d.voice_move(azimuth_deg=45.0, altitude_deg=10.0, speed=2)
cmd, dec = last()
check("voice move nested params", cmd == 16800 and
      dec["command_type"] == "MOVE" and
      abs(dec["move_params"]["azimuth_angle"] - 45.0) < 1e-6 and
      abs(dec["move_params"]["altitude_angle"] - 10.0) < 1e-6 and
      dec["move_params"]["speed"] == 2)

d.voice_start_panorama(rows=2, columns=3)
cmd, dec = last()
check("voice start_panorama nested params", cmd == 16800 and
      dec["command_type"] == "START_PANORAMA" and
      dec["panorama_params"] == {"rows": 2, "columns": 3})

print()
if fails:
    print(f"{len(fails)} FAILURE(S):", fails)
    raise SystemExit(1)
print(f"ALL {len(captured)} ROUND-TRIP CHECKS PASSED")

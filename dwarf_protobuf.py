#!/usr/bin/env python3
"""
dwarf_protobuf.py — schema-aware protobuf deciphering for the DWARF 3 WsCmd
protocol.

EXTENSION to dwarf3tostellarium. `dwarf_capture_decode.py` reconstructs the
transport (pcapng -> TCP -> WebSocket -> WsCmd) and dumps *unknown* payloads as
anonymous `fN` fields. That is enough to spot a gap, but it does not actually
*decipher* a packet: a joystick vector shows up as a 64-bit blob, an RA/Dec goto
as two giant uint64s, and the envelope `type` as a bare `6:2`.

This module adds the missing decode layer:

  * a wire-type-aware recursive decoder that interprets every field the way the
    firmware actually encodes it — varint / signed / zigzag / bool, i64 -> double,
    i32 -> float, length-delimited -> nested-message | string | bytes (auto), and
    packed repeated scalars;
  * a SCHEMA REGISTRY mapping known command ids to named, typed fields and enum
    tables, so payloads decipher to human-readable dicts
    (e.g. `{action: OPEN}`, `{ra: 5.5757, dec: 22.01, name: "M45"}`), and
  * `decode_wscmd()`, which decodes the WsCmd envelope AND its `data` payload
    against the schema selected by the packet's own `cmd`, in one call.

It has no hard dependency on the controller (imports its CMD_ names when present,
falls back to an inline table otherwise) and no third-party dependencies, so it
works standalone on a capture host. Run it directly for a self-test:

    python3 dwarf_protobuf.py            # decodes hand-built sample packets
"""
from __future__ import annotations

import struct
from typing import Any

# ── command-id -> short name (shared with the controller when importable) ───────
try:
    import dwarflab_controller as _D  # type: ignore
    CMD_NAMES: dict[int, str] = {
        v: k[4:] for k, v in vars(_D).items()
        if k.startswith("CMD_") and isinstance(v, int)
    }
except Exception:  # pragma: no cover - standalone fallback
    CMD_NAMES = {
        10050: "V3_CAMERA_TELE_OPEN_CAMERA", 12036: "V3_CAMERA_WIDE_OPEN_CAMERA",
        11002: "ASTRO_START_GOTO_DSO", 11003: "ASTRO_START_GOTO_SOLAR_SYSTEM",
        11013: "ASTRO_START_ONE_CLICK_GOTO_DSO",
        11014: "ASTRO_START_ONE_CLICK_GOTO_SOLAR_SYSTEM",
        13010: "SYSTEM_SET_LOCATION", 14006: "STEP_MOTOR_JOYSTICK",
        14800: "TRACK_START_TRACK", 14801: "TRACK_STOP_TRACK",
        15201: "NOTIFY_ELE", 15203: "NOTIFY_TEMPERATURES",
        15211: "NOTIFY_STATE_ASTRO_GOTO", 15212: "NOTIFY_STATE_ASTRO_TRACKING",
        15225: "NOTIFY_TRACK_RESULT", 15233: "NOTIFY_ASTRO_TARGET_STATUS",
        15252: "NOTIFY_WIDE_TRACK_RESULT",
        15284: "NOTIFY_WIDE_TRACK_STATE",
    }

# WsCmd.type — CAPTURE-VERIFIED: 2=notify, 3=command-ack (the controller's
# 0=request / 1=response model is incomplete; see TRACKING_FINDINGS.md §6).
MSG_TYPE_NAMES = {0: "REQUEST", 1: "RESPONSE", 2: "NOTIFY", 3: "ACK"}


# ── low-level protobuf primitives ───────────────────────────────────────────────
def read_varint(buf: bytes, i: int) -> tuple[int, int]:
    r = s = 0
    while True:
        b = buf[i]; i += 1
        r |= (b & 0x7F) << s
        if not (b & 0x80):
            return r, i
        s += 7


def to_signed(v: int) -> int:
    """Two's-complement (int32/int64 negative encoding, e.g. the -100 sentinel)."""
    v &= 0xFFFFFFFFFFFFFFFF
    return v - 0x10000000000000000 if v >= 0x8000000000000000 else v


def zigzag(v: int) -> int:
    """ZigZag decode for proto sint32 / sint64."""
    return (v >> 1) ^ -(v & 1)


def looks_like_utf8_text(seg: bytes) -> bool:
    if not seg:
        return False
    try:
        seg.decode("utf-8")
    except UnicodeDecodeError:
        return False
    # printable-ish: allow tab/newline/CR + normal glyphs, reject control bytes
    return all(9 <= c <= 13 or 32 <= c for c in seg)


def looks_like_message(seg: bytes) -> bool:
    """True if `seg` cleanly parses as a protobuf message end-to-end. Used to tell
    a nested submessage from an opaque bytes blob when no schema says which."""
    i, n = 0, len(seg)
    if n == 0:
        return False
    try:
        while i < n:
            tag, i = read_varint(seg, i)
            fn, wt = tag >> 3, tag & 7
            if fn == 0 or wt in (3, 4, 6, 7):
                return False
            if wt == 0:
                _, i = read_varint(seg, i)
            elif wt == 1:
                i += 8
            elif wt == 2:
                ln, i = read_varint(seg, i); i += ln
            elif wt == 5:
                i += 4
        return i == n
    except (IndexError, struct.error):
        return False


# ── schema registry ─────────────────────────────────────────────────────────────
# type tokens a field spec may use:
#   "int"     plain varint                    "sint"   two's-complement signed
#   "zigzag"  proto sint32/64                 "bool"   varint -> True/False
#   "double"  wire-type 1 (8 bytes)           "float"  wire-type 5 (4 bytes)
#   "u64"/"u32" raw fixed ints                "str"    utf-8 string
#   "bytes"   opaque                          "px10"   varint / 10.0 (deci-units)
#   ("enum", {v: name})                       ("msg",  "<schema-name>")
Field = tuple[int, str, Any]  # (field_no, name, type-token)

_ACTION = ("enum", {0: "OPEN_OR_CLOSE_0", 1: "ACTION_1"})  # per-cmd meaning differs
_GOTO_STATE = ("enum", {0: "IDLE", 1: "SLEWING", 2: "SOLVING", 3: "TRACKING",
                        4: "FAILED", 5: "DONE"})

# named nested schemas referenced by ("msg", name)
NESTED_SCHEMAS: dict[str, list[Field]] = {
    "box_xywh":      [(1, "x", "sint"), (2, "y", "sint"),
                      (3, "w", "int"),  (4, "h", "int")],
    "mode_switch":   [(1, "mode", "int")],
    "mot_object":    [(1, "id", "int"), (2, "x", "sint"), (3, "y", "sint"),
                      (4, "w", "int"),  (5, "h", "int"), (6, "cls", "int")],
    # target echoed back inside the 11003 goto-solar-system ACK; same 4 fields as the
    # front of an 11014 request, minus its trailing `mode` — see SCHEMAS[11003/11014].
    "solar_target":  [(1, "solar_id", "int"), (2, "coord1", "double"),
                      (3, "coord2", "double"), (4, "name", "str")],
    # nested inside NOTIFY_ASTRO_TARGET_STATUS (15233) — see SCHEMAS[15233].
    "target_status": [(1, "state", "int"), (2, "name", "str")],

    # ── added 2026-07-31: PANORAMA/SHOOTING_SCHEDULE/TASK_CENTER/VOICE_ASSISTANT
    # nested messages, from proto_reconstructed/*.proto (recovered from magni's own
    # embedded FileDescriptorProto blobs — see dwarf3-tracking-protocol memory,
    # "Full formal protobuf schema recovered directly from bin/magni"). Field layout
    # is schema-exact (parsed from the real descriptor), not guessed; only the CMD-ID
    # <-> message assignment for a few gaps is inferred (flagged per-field in SCHEMAS
    # below, not here).
    "shooting_task_msg": [
        (1, "schedule_id", "str"), (2, "params", "str"), (3, "state", "int"),
        (4, "code", "int"), (5, "created_time", "int"), (6, "updated_time", "int"),
        (7, "schedule_task_id", "str"), (8, "param_mode", "int"),
        (9, "param_version", "int"), (10, "create_from", "int"),
    ],
    "shooting_schedule_msg": [
        (1, "schedule_id", "str"), (2, "schedule_name", "str"), (3, "device_id", "int"),
        (4, "mac_address", "str"), (5, "start_time", "int"), (6, "end_time", "int"),
        (7, "result", "int"), (8, "created_time", "int"), (9, "updated_time", "int"),
        (10, "state", "int"), (11, "lock", "int"), (12, "password", "str"),
        (13, "shooting_tasks", ("repeated", ("msg", "shooting_task_msg"))),
        (14, "param_mode", "int"), (15, "param_version", "int"), (16, "params", "str"),
        (17, "schedule_time", "int"), (18, "sync_state", "int"),
    ],
    # ReqEnterCamera{client_param=3}/ClientParams{encode_type=1} — the real proto
    # shape behind what dwarflab_controller.py's v3_mode_switch() sends as {3:{1:m}}.
    "client_params": [(1, "encode_type", "int")],
    "voice_move_params": [(1, "azimuth_angle", "double"), (2, "altitude_angle", "double"),
                          (3, "speed", "int")],
    "voice_focus_params": [(1, "is_infinity", "bool")],
    "voice_sentry_params": [(1, "type", "int")],
    "voice_calibration_params": [(1, "lon", "double"), (2, "lat", "double")],
    "voice_panorama_params": [(1, "rows", "int"), (2, "columns", "int")],
    "deep_sky_target": [(1, "ra", "double"), (2, "dec", "double"),
                        (3, "target_name", "str"), (4, "lon", "double"),
                        (5, "lat", "double"), (6, "ir_index", "int")],
    "solar_system_target": [(1, "index", "int"), (2, "lon", "double"),
                            (3, "lat", "double"), (4, "target_name", "str")],
    "voice_goto_params": [(1, "deep_sky", ("msg", "deep_sky_target")),
                          (2, "solar_system", ("msg", "solar_system_target")),
                          (7, "shooting_mode", "int")],
    "voice_track_params": [(1, "deep_sky", ("msg", "deep_sky_target")),
                           (2, "solar_system", ("msg", "solar_system_target"))],
}

_VOICE_CMD_TYPE = ("enum", {
    0: "UNKNOWN", 1: "GET_STATUS", 2: "TAKE_PHOTO", 3: "START_RECORD", 4: "STOP_RECORD",
    5: "START_TIMELAPSE", 6: "STOP_TIMELAPSE", 7: "START_BURST", 8: "STOP_BURST",
    9: "START_ASTRO", 10: "STOP_ASTRO", 11: "START_SENTRY", 12: "STOP_SENTRY",
    13: "MOVE", 14: "GOTO_TARGET", 15: "STOP_GOTO", 16: "CALIBRATION",
    17: "STOP_CALIBRATION", 18: "AUTO_FOCUS", 19: "STOP_FOCUS", 20: "STOP_ALL",
    21: "START_TRACK", 22: "STOP_TRACK", 23: "ADD_SCHEDULE", 24: "CANCEL_SCHEDULE",
    25: "END_CONVERSATION", 26: "START_PANORAMA", 27: "STOP_PANORAMA",
})

# cmd id -> payload schema.  Requests (phone -> device) and notifies (device ->
# phone) alike, keyed by the WsCmd `cmd` field. Everything here is grounded in the
# controller's payload builders and the capture-verified findings.
SCHEMAS: dict[int, list[Field]] = {
    # ── requests (phone -> device) ──────────────────────────────────────────────
    10050: [(1, "action", _ACTION)],                       # V3 open tele (1=open)
    12036: [(1, "action", _ACTION)],                       # V3 open wide (empty=open)
    11002: [(1, "ra", "double"), (2, "dec", "double"),
            (3, "name", "str"), (4, "goto_only", "bool")],  # ReqGotoDSO — NOT re-verified
            # against a live capture (unlike 11013 below); may have the same field-4 drift.
    # CAPTURE-VERIFIED 2026-07-24: a live one-click-goto-DSO request (target "HD 89822")
    # carried TWO more fields than this old 4-field model assumed, and the old field 4
    # ("goto_only" bool) is actually a double on the wire — decode_with_schema silently
    # ignored the schema's "bool" token and fell through to the wire-type-1 (double)
    # branch, which is why the generic decode below showed goto_only=29.036102 instead
    # of a bool. That value, and field 5 (41.087562), match — to 4 decimal places — the
    # "coord1"/"coord2" fields seen in an unrelated 11014 (Moon) goto from a different
    # session: not a coincidence. Both are the app's currently-configured OBSERVER
    # lat/lon (from SYSTEM_SET_LOCATION / GPS), tacked onto every goto-family request
    # for the device's local plate-solve/tracking math — not per-target data.
    11013: [(1, "ra", "double"), (2, "dec", "double"), (3, "name", "str"),
            (4, "lat", "double"), (5, "lon", "double"), (6, "mode", "int")],
    # CAPTURE-VERIFIED 2026-07-23/24: ASTRO_START_ONE_CLICK_GOTO_SOLAR_SYSTEM request, seen
    # sent phone->device with {solar_id:8, lat:29.036198, lon:41.08766, name:"Moon",
    # mode:9}; `confirm` (f6) only showed up on a second send once the app reported
    # steady tracking. lat/lon here are the observer's own coordinates (see the 11013
    # comment above for the cross-session match that pinned this down), not the target's.
    11014: [(1, "solar_id", "int"), (2, "lat", "double"), (3, "lon", "double"),
            (4, "name", "str"), (5, "mode", "int"), (6, "confirm", "bool")],
    13004: [(1, "lock", "bool")],                           # ReqSetMasterLock
    13010: [(1, "lat", "double"), (2, "lon", "double"),
            (3, "alt", "double")],                          # ReqSetLocation
    14006: [(1, "angle_deg", "double"), (2, "length", "double")],  # joystick
    14800: [(1, "x", "sint"), (2, "y", "sint"), (3, "w", "int"),
            (4, "h", "int"), (5, "field5", "int")],         # ReqStartTrack (+f5=1)
    14804: [(1, "mode", "int")],                            # MOT start
    14808: [(1, "id", "int")],                              # lock wide detection
    14805: [(1, "id", "int")],                              # lock tele detection
    16404: [(3, "config", ("msg", "mode_switch"))],         # V3 mode switch {3:{1:m}}
    # CAPTURE-VERIFIED 2026-07-23: FOCUS_MANUAL_SINGLE_STEP / START|STOP_MANUAL_CONTINUOUS
    # / START_ASTRO_AUTO_FOCUS all sent with a genuinely EMPTY data payload across many
    # samples (72 single-steps, several start/stop pairs, several auto-focus triggers) —
    # bare triggers, not a repo gap. This CONTRADICTS dwarflab_controller's model, whose
    # focus_step(s)/focus_in()/focus_out() build a signed p_int payload for direction —
    # that field was never observed on the wire, so either the app no longer parameterizes
    # direction this way, or it's carried by a different mechanism entirely (a prior
    # command, session state, or a separate cmd id per direction we haven't captured).
    15001: [],                                              # FOCUS_MANUAL_SINGLE_STEP
    15002: [],                                              # FOCUS_START_MANUAL_CONTINUOUS
    15003: [],                                              # FOCUS_STOP_MANUAL_CONTINUOUS
    15004: [],                                              # FOCUS_START_ASTRO_AUTO_FOCUS

    # ── PANORAMA (module 10, 15500-15599) — added 2026-07-31. IDs zlog/ack-call
    # CONFIRMED per firmware/ack_calls_parsed.txt except where marked UNCONFIRMED
    # (those cmd ids are gaps in the 295-site bare-ack scan — likely because they're
    # data-returning GET-style handlers that use the OTHER universal sender
    # (FUN_00793398), not the plain-ack one the scan searched; the field layout
    # itself is still schema-exact from the recovered panorama.proto, only the
    # cmd-id<->message ASSIGNMENT for these three is inferred by proto declaration
    # order, matching the pattern already proven correct for the confirmed ones).
    15500: [],                                              # ReqStartPanoramaByGrid (CONFIRMED)
    15501: [],                                              # ReqStopPanorama (CONFIRMED)
    15502: [(1, "yaw_range", "float"), (2, "pitch_range", "float")],  # UNCONFIRMED id
    15503: [(1, "resource_id", "u64"), (2, "user_id", "str"), (3, "app_platform", "int"),
            (4, "panorama_name", "str"), (5, "ak", "str"), (6, "sk", "str"),
            (7, "token", "str"), (8, "bucket", "str"), (9, "bucket_prefix", "str"),
            (10, "from", "str"), (11, "env_type", "str")],  # ReqStartPanoramaStitchUpload (CONFIRMED)
    15504: [(1, "user_id", "str")],                         # ReqStopPanoramaStitchUpload (CONFIRMED)
    15505: [],                                              # ReqGetPanoramaCurrentUploadState, UNCONFIRMED id
    15506: [(1, "panorama_name", "str")],                   # ReqGetUploadPredict, UNCONFIRMED id
    15507: [(1, "panorama_name", "str")],                   # ReqCompressPanorama (CONFIRMED)
    15508: [],                                              # ReqStopCompressPanorama (CONFIRMED)
    15509: [],                                              # ReqStartPanoramaFraming (CONFIRMED)
    15510: [],                                              # ReqStopPanoramaFraming (CONFIRMED)
    15511: [],                                              # ReqResetPanoramaFraming (CONFIRMED)
    15512: [(1, "norm_x_tl", "double"), (2, "norm_y_tl", "double"),
            (3, "norm_x_br", "double"), (4, "norm_y_br", "double")],  # ReqUpdatePanoramaFramingRect (CONFIRMED)
    15513: [],                                              # ReqStopPanoramaFramingAndStartGrid (CONFIRMED)
    15514: [],                                              # ReqNotifyPanoramaFraming (CONFIRMED, device->phone)

    # ── SHOOTING_SCHEDULE (module 13, 16100-16108) — added 2026-07-31. Field layout
    # schema-exact; three cmd ids (16102/16103/16106/16107) UNCONFIRMED by zlog (same
    # "GET/data-returning handler not in the bare-ack scan" reasoning as panorama
    # above), assigned by proto declaration order (9 Req* messages, sequential
    # 16100-16108 — the same pattern independently confirmed for 16100/16101/16104/
    # 16105/16108, so high confidence, just not individually zlog-verified).
    16100: [(1, "shooting_schedule", ("msg", "shooting_schedule_msg"))],  # ReqSyncShootingSchedule (CONFIRMED)
    16101: [(1, "id", "str"), (2, "password", "str")],      # ReqCancelShootingSchedule (CONFIRMED)
    16102: [],                                              # ReqGetAllShootingSchedule, UNCONFIRMED id
    16103: [(1, "id", "str")],                              # ReqGetShootingScheduleById, UNCONFIRMED id
    16104: [(1, "id", "str")],                              # ReqGetShootingTaskById (CONFIRMED, zlog "getShootingTaskId")
    16105: [(1, "shooting_schedule", ("msg", "shooting_schedule_msg"))],  # ReqReplaceShootingSchedule (CONFIRMED)
    16106: [(1, "id", "str"), (2, "password", "str")],      # ReqUnlockShootingSchedule, UNCONFIRMED id
    16107: [(1, "id", "str"), (2, "password", "str")],      # ReqLockShootingSchedule, UNCONFIRMED id
    16108: [(1, "id", "str"), (2, "password", "str")],      # ReqDeleteShootingSchedule (CONFIRMED)

    # ── PARAM (module 15, 16700-16706) — added 2026-07-31. ALL 7 cmd ids CONFIRMED
    # by zlog debug strings (firmware/ack_calls_parsed.txt: CMD_PARAM_SET_EXPOSURE
    # .. CMD_PARAM_SET_AUTO_PARAMS); param_id is the 64-bit id whose bit 44 encodes
    # camera (0=tele/1=wide) — see dwarf3-tracking-protocol memory "parameter
    # commands DO bake camera identity into a 64-bit paramId".
    16700: [(1, "param_id", "u64"), (2, "mode", "int"), (3, "value", "int")],   # ReqSetExposure
    16701: [(1, "param_id", "u64"), (2, "mode", "int"), (3, "value", "int")],   # ReqSetGain
    16702: [(1, "param_id", "u64"), (2, "mode", "int"), (3, "value", "int")],   # ReqSetWb
    16703: [(1, "param_id", "u64"), (2, "value", "int")],                      # ReqSetGeneralIntParam
    16704: [(1, "param_id", "u64"), (2, "value", "float")],                    # ReqSetGeneralFloatParam
    16705: [(1, "param_id", "u64"), (2, "value", "bool")],                     # ReqSetGeneralBoolParams
    16706: [(1, "camera_type", "int"), (2, "shooting_tech", "int"),
            (3, "is_auto", "bool")],                                          # ReqSetAutoParam

    # ── DEVICE (module 17, 17000-17002) — added 2026-07-31. ALL 3 CONFIRMED by zlog
    # (CMD_DEVICE_LENS_DEFOG/AUTO_COOLING/AUTO_SHUTDOWN).
    17000: [(1, "state", "int")],                           # ReqLensDefog
    17001: [(1, "state", "int")],                            # ReqAutoCooling
    17002: [(1, "state", "int")],                            # ReqAutoShutdown

    # ── TASK_CENTER (module 14, 16400-16405) — added 2026-07-31. 16403-16405
    # CONFIRMED (16403 by existing capture-verified CMD_V3_DEVICE_CONFIG_SHOOTING_MODE
    # naming, 16404 by both capture verification AND proto field match — see
    # dwarflab_controller.py v3_mode_switch()/enter_camera(), 16405 by zlog). The
    # remaining trio (16400/16401/16402 = ReqStartTask/ReqStopTask/
    # ReqSwitchShootingTech in some order) is a GENUINE OPEN GAP: this module's own
    # proto declaration order does NOT match its confirmed IDs (ReqEnterCamera is
    # declared 3rd but confirmed at 16404, the 5th slot), so simple sequential
    # assignment — which worked for panorama/shooting_schedule above — is proven
    # UNRELIABLE for this specific module. Deliberately NOT schema'd/guessed here;
    # they'll fall through to the generic fN decoder until zlog-confirmed.
    16403: [(1, "mode", "int")],                             # ReqSwitchShootingMode
    16404: [(3, "client_param", ("msg", "client_params"))],  # ReqEnterCamera
    16405: [],                                               # ReqGetDeviceStateInfo

    # ── VOICE_ASSISTANT (module 16, single dispatcher 16800) — added 2026-07-31.
    # Confirmed structurally (not per-field zlog) from dwarf3-tracking-protocol memory:
    # "a single generic CMD_VOICE_ASSISTANT_TASK=16800 handler with an internal
    # switch(task_type) on a protobuf field — all 26+ voice-assistant actions live
    # behind this one command id". Only the simpler param sub-messages are modeled
    # here (move/goto/track/focus/sentry/calibration/panorama); the larger ones
    # (astro/record/timelapse/burst/schedule params) are deliberately left off the
    # schema so they fall through to the generic fN decoder rather than silently
    # dropping data behind an incomplete nested-schema entry.
    16800: [(1, "command_type", _VOICE_CMD_TYPE), (2, "shooting_mode", "int"),
            (16, "move_params", ("msg", "voice_move_params")),
            (17, "goto_params", ("msg", "voice_goto_params")),
            (18, "calibration_params", ("msg", "voice_calibration_params")),
            (19, "focus_params", ("msg", "voice_focus_params")),
            (20, "track_params", ("msg", "voice_track_params")),
            (23, "panorama_params", ("msg", "voice_panorama_params"))],

    # ── notifies / acks (device -> phone) ───────────────────────────────────────
    # CAPTURE-VERIFIED 2026-07-23: only ever observed as an ACK (device->phone), echoing
    # the target set by the 11014 one-click-solar-system request: {status:-11531,
    # target:{solar_id:8, coord1:29.036198, coord2:41.08766, name:"Moon"}}. `status`'s
    # meaning is unconfirmed (not a small sentinel like the -100 "no target" code
    # elsewhere). dwarflab_controller.goto_solar() models a plain-int REQUEST for this
    # cmd id (bare field 1 = solar-system index) that this capture never sent — if that
    # path gets captured, its payload will NOT match this schema and needs a fork by
    # direction/type, not a blind merge.
    11003: [(1, "status", "sint"), (2, "target", ("msg", "solar_target"))],
    15201: [(1, "battery", "int")],
    15202: [(1, "battery", "int")],
    15203: [(1, "temp", "px10"), (2, "cmos_temp", "px10")],
    # CAPTURE-VERIFIED 2026-07-24, ONE sample only: {f1:true, f2:true} during a live
    # one-click-goto (plate-solve) run. Naming f1/f2 would be a guess off a single data
    # point, so they're left generic (f1/f2) until a capture with a FAILED solve or a
    # false value on either field turns up to pin down what each one actually flags.
    15210: [],
    15211: [(1, "state", _GOTO_STATE)],
    15212: [(1, "tracking", "bool")],
    15225: [(1, "x", "sint"), (2, "y", "sint"), (3, "w", "int"), (4, "h", "int")],
    15232: [(1, "x", "sint"), (2, "y", "sint"), (3, "w", "int"), (4, "h", "int")],
    # CAPTURE-VERIFIED 2026-07-23: nested at field 3 only, {1:state, 2:name}. `state` was
    # 3 immediately after an ASTRO_START_ONE_CLICK_GOTO_SOLAR_SYSTEM request, then 1 once
    # the app reported the mount was steadily tracking — 2 real samples, not enough to
    # build a confident enum (and it does NOT reuse _GOTO_STATE's numbering, which would
    # put 3 = already-tracking on the first sample).
    15233: [(3, "target", ("msg", "target_status"))],
    15252: [(1, "x", "sint"), (2, "y", "sint"), (3, "w", "int"), (4, "h", "int")],
    15238: [(1, "objects", ("repeated", ("msg", "mot_object")))],
    15251: [(1, "objects", ("repeated", ("msg", "mot_object")))],
    15257: [(1, "position", "int")],
    15300: [(1, "position", "int")],
    15267: [(1, "changing", "bool"), (2, "mode", "int"), (3, "sub_mode", "int")],
    15284: [(1, "state", "int"), (2, "sub_state", "int")],
    15292: [(1, "cmos_temp", "px10")],
    15302: [(1, "enabled", "bool")],
    15303: [(1, "enabled", "bool")],

    # ── UDP :9900 heartbeat (not a WsCmd, decoded via schema name below) ─────────
}

# the WsCmd envelope itself, so the top-level frame deciphers to named fields
WSCMD_ENVELOPE: list[Field] = [
    (1, "major", "int"), (2, "minor", "int"), (3, "device_id", "int"),
    (4, "module_id", "int"), (5, "cmd", "int"), (6, "type", ("enum", MSG_TYPE_NAMES)),
    (7, "data", "bytes"), (8, "client_id", "str"),
]

HEARTBEAT_SCHEMA: list[Field] = [
    (1, "flag", "int"), (2, "unix_ms", "int"), (3, "tag", "str"),
]


# ── generic (schema-free) recursive decoder ─────────────────────────────────────
def decode_generic(data: bytes, max_depth: int = 6) -> dict:
    """Best-effort typed decode with NO schema. Every field becomes `fN` and each
    value is interpreted by its wire type, with heuristics:
      * varint          -> int, plus signed/zigzag/bool hints when they look valid
      * i64 (wt 1)      -> {"double": .., "u64": ..}  (protocol uses doubles a lot)
      * i32 (wt 5)      -> {"float": .., "u32": ..}
      * length-delim.   -> recurse if it parses as a message, else str, else hex
    Repeated fields collapse into a list. Returns {} on malformed input.
    """
    out: dict[str, Any] = {}

    def put(key: str, val: Any) -> None:
        if key in out:
            if not isinstance(out[key], list):
                out[key] = [out[key]]
            out[key].append(val)
        else:
            out[key] = val

    i, n = 0, len(data)
    try:
        while i < n:
            tag, i = read_varint(data, i)
            fn, wt = tag >> 3, tag & 7
            key = f"f{fn}"
            if wt == 0:
                v, i = read_varint(data, i)
                sv = to_signed(v)
                if v <= 1:
                    put(key, bool(v) if v in (0, 1) else v)
                elif sv < 0:            # clearly a negative int (e.g. -100 sentinel)
                    put(key, sv)
                else:
                    put(key, v)
            elif wt == 1:
                raw = struct.unpack_from("<Q", data, i)[0]; i += 8
                dbl = struct.unpack_from("<d", data, i - 8)[0]
                put(key, {"double": round(dbl, 6), "u64": raw})
            elif wt == 5:
                raw = struct.unpack_from("<I", data, i)[0]; i += 4
                flt = struct.unpack_from("<f", data, i - 4)[0]
                put(key, {"float": round(flt, 6), "u32": raw})
            elif wt == 2:
                ln, i = read_varint(data, i); seg = data[i:i + ln]; i += ln
                if max_depth > 0 and looks_like_message(seg):
                    put(key, decode_generic(seg, max_depth - 1))
                elif looks_like_utf8_text(seg):
                    put(key, seg.decode("utf-8"))
                else:
                    put(key, seg.hex(" ") if seg else "")
            else:
                break
    except (IndexError, struct.error):
        pass
    return out


# ── schema-guided decoder ───────────────────────────────────────────────────────
def _decode_scalar(token: Any, wt: int, data: bytes, i: int):
    """Decode one field value given its schema type token. Returns (value, new_i).
    Falls back to a generic interpretation when the wire type disagrees with the
    schema (firmware/version drift), so a bad spec never crashes the decode."""
    if wt == 0:
        v, i = read_varint(data, i)
        if isinstance(token, tuple) and token[0] == "enum":
            sv = to_signed(v)
            return token[1].get(v, token[1].get(sv, f"UNKNOWN({sv})")), i
        if token == "bool":
            return bool(v), i
        if token in ("sint",):
            return to_signed(v), i
        if token == "zigzag":
            return zigzag(v), i
        if token == "px10":
            return round(to_signed(v) / 10.0, 1), i
        return v, i
    if wt == 1:
        if token == "u64":
            return struct.unpack_from("<Q", data, i)[0], i + 8
        return round(struct.unpack_from("<d", data, i)[0], 6), i + 8
    if wt == 5:
        if token == "u32":
            return struct.unpack_from("<I", data, i)[0], i + 4
        return round(struct.unpack_from("<f", data, i)[0], 6), i + 4
    if wt == 2:
        ln, i = read_varint(data, i); seg = data[i:i + ln]; i += ln
        if isinstance(token, tuple) and token[0] == "msg":
            return decode_with_schema(seg, NESTED_SCHEMAS.get(token[1], [])), i
        if token == "str":
            return seg.decode("utf-8", "replace"), i
        if token == "bytes":
            return seg.hex(" "), i
        # unknown token on a length-delimited field: best effort
        if looks_like_message(seg):
            return decode_generic(seg), i
        return seg.decode("utf-8") if looks_like_utf8_text(seg) else seg.hex(" "), i
    raise ValueError(f"bad wire type {wt}")


def decode_with_schema(data: bytes, schema: list[Field]) -> dict:
    """Decode `data` using a field schema. Fields present in the schema get their
    declared name/type; unknown fields fall back to the generic `fN` decode so
    nothing is silently dropped (that's how new firmware fields get noticed)."""
    by_num: dict[int, tuple[str, Any]] = {fn: (name, tok) for fn, name, tok in schema}
    repeated: dict[str, Any] = {}
    for fn, name, tok in schema:
        if isinstance(tok, tuple) and tok[0] == "repeated":
            repeated[name] = tok[1]
    out: dict[str, Any] = {}
    i, n = 0, len(data)
    try:
        while i < n:
            tag, i = read_varint(data, i)
            fn, wt = tag >> 3, tag & 7
            if fn in by_num:
                name, tok = by_num[fn]
                if isinstance(tok, tuple) and tok[0] == "repeated":
                    inner = tok[1]
                    if wt == 2 and isinstance(inner, tuple) and inner[0] == "msg":
                        ln, i = read_varint(data, i); seg = data[i:i + ln]; i += ln
                        out.setdefault(name, []).append(
                            decode_with_schema(seg, NESTED_SCHEMAS.get(inner[1], [])))
                        continue
                    val, i = _decode_scalar(inner, wt, data, i)
                    out.setdefault(name, []).append(val)
                    continue
                out[name], i = _decode_scalar(tok, wt, data, i)
            else:
                # unknown field: keep it, generically typed
                key = f"f{fn}"
                if wt == 0:
                    v, i = read_varint(data, i); out[key] = to_signed(v)
                elif wt == 1:
                    out[key] = round(struct.unpack_from("<d", data, i)[0], 6); i += 8
                elif wt == 5:
                    out[key] = round(struct.unpack_from("<f", data, i)[0], 6); i += 4
                elif wt == 2:
                    ln, i = read_varint(data, i); seg = data[i:i + ln]; i += ln
                    out[key] = (decode_generic(seg) if looks_like_message(seg)
                                else seg.decode("utf-8") if looks_like_utf8_text(seg)
                                else seg.hex(" "))
                else:
                    break
    except (IndexError, struct.error):
        pass
    return out


# ── top-level WsCmd deciphering ─────────────────────────────────────────────────
def decode_wscmd(raw: bytes) -> dict:
    """Decipher a full WsCmd frame: envelope fields (named + enum-resolved) AND the
    inner `data` payload decoded against the schema for its `cmd`. Returns:
        {major, minor, device_id, module_id, cmd, cmd_name, type, type_name,
         client_id, payload: {..named..}}
    When no schema is registered for the cmd, `payload` holds the generic decode
    and `payload_schema` is "generic" so callers can flag repo gaps.
    """
    env: dict[str, Any] = {}
    data = b""
    i, n = 0, len(raw)
    try:
        while i < n:
            tag, i = read_varint(raw, i)
            fn, wt = tag >> 3, tag & 7
            if wt == 0:
                v, i = read_varint(raw, i)
                env[fn] = v
            elif wt == 2:
                ln, i = read_varint(raw, i); seg = raw[i:i + ln]; i += ln
                if fn == 7:
                    data = seg
                elif fn == 8:
                    env[8] = seg.decode("utf-8", "replace")
                else:
                    # Envelope field we don't model, or a mis-parsed frame (e.g. a
                    # truncated capture — pkt-size 512 — splitting a WS frame across
                    # packets misaligns the reassembly). CAPTURE-VERIFIED: this has
                    # produced garbage where `fn` collided with major(1)/client_id(8),
                    # leaving a raw `bytes` in `result` that crashes json.dumps downstream.
                    env[fn] = (seg.decode("utf-8") if looks_like_utf8_text(seg)
                               else seg.hex(" "))
            elif wt == 1:
                i += 8
            elif wt == 5:
                i += 4
            else:
                break
    except (IndexError, struct.error):
        pass

    cmd = env.get(5, 0)
    mtype = env.get(6, 0)
    schema = SCHEMAS.get(cmd)
    result = {
        "major": env.get(1, 0), "minor": env.get(2, 0),
        "device_id": env.get(3, 0), "module_id": env.get(4, 0),
        "cmd": cmd, "cmd_name": CMD_NAMES.get(cmd, f"UNKNOWN_{cmd}"),
        "type": mtype, "type_name": MSG_TYPE_NAMES.get(mtype, str(mtype)),
        "client_id": env.get(8, ""),
    }
    if schema is not None:
        result["payload"] = decode_with_schema(data, schema)
        result["payload_schema"] = "known"
    else:
        result["payload"] = decode_generic(data)
        result["payload_schema"] = "generic"
    return result


def decode_heartbeat(raw: bytes) -> dict:
    """Decipher the UDP :9900 app heartbeat protobuf {1:flag, 2:unix_ms, 3:'txtl'}."""
    return decode_with_schema(raw, HEARTBEAT_SCHEMA)


# ── self-test ───────────────────────────────────────────────────────────────────
def _build_field(fn: int, wt: int, val) -> bytes:
    tag = bytes()
    t = (fn << 3) | wt

    def _uv(v: int) -> bytes:
        v &= 0xFFFFFFFFFFFFFFFF
        b = bytearray()
        while True:
            b.append(v & 0x7F); v >>= 7
            if not v:
                break
        for k in range(len(b) - 1):
            b[k] |= 0x80
        return bytes(b)

    tag = _uv(t)
    if wt == 0:
        return tag + _uv(val)
    if wt == 1:
        return tag + struct.pack("<d", val)
    if wt == 5:
        return tag + struct.pack("<f", val)
    if wt == 2:
        if isinstance(val, str):
            val = val.encode()
        return tag + _uv(len(val)) + val
    raise ValueError(wt)


def _selftest() -> int:
    import json
    print("dwarf_protobuf self-test\n" + "=" * 40)

    # 1) a one-click-goto-DSO request payload, wrapped in a WsCmd envelope. Field
    # layout capture-verified 2026-07-24: {ra, dec, name, lat, lon, mode} — the
    # trailing lat/lon are the OBSERVER's position (tacked onto every goto-family
    # request), not the target's.
    goto = (_build_field(1, 1, 5.5757) + _build_field(2, 1, 22.0139) +
            _build_field(3, 2, "M45") + _build_field(4, 1, 48.8566) +
            _build_field(5, 1, 2.3522) + _build_field(6, 0, 1))
    env = (_build_field(1, 0, 1) + _build_field(2, 0, 20) + _build_field(3, 0, 2) +
           _build_field(4, 0, 3) + _build_field(5, 0, 11013) + _build_field(6, 0, 0) +
           _build_field(7, 2, goto) + _build_field(8, 2, "uuid.171.iOS"))
    d = decode_wscmd(env)
    print("GOTO request:", json.dumps(d, indent=2))
    assert d["cmd_name"].endswith("ONE_CLICK_GOTO_DSO")
    assert d["type_name"] == "REQUEST"
    assert abs(d["payload"]["ra"] - 5.5757) < 1e-4
    assert d["payload"]["name"] == "M45" and d["payload"]["mode"] == 1
    assert abs(d["payload"]["lat"] - 48.8566) < 1e-4

    # 2) a wide track-result notify carrying a box, plus the -100 no-target case
    box = (_build_field(1, 0, 975) + _build_field(2, 0, 280) +
           _build_field(3, 0, 382) + _build_field(4, 0, 210))
    env2 = (_build_field(5, 0, 15252) + _build_field(6, 0, 2) + _build_field(7, 2, box))
    d2 = decode_wscmd(env2)
    print("\nWIDE_TRACK_RESULT notify:", json.dumps(d2, indent=2))
    assert d2["type_name"] == "NOTIFY"
    assert d2["payload"] == {"x": 975, "y": 280, "w": 382, "h": 210}

    no_target = _build_field(1, 0, to_signed(-100) & 0xFFFFFFFFFFFFFFFF) + \
        _build_field(2, 0, to_signed(-100) & 0xFFFFFFFFFFFFFFFF)
    dn = decode_with_schema(no_target, SCHEMAS[15252])
    print("no-target box:", dn)
    assert dn["x"] == -100 and dn["y"] == -100

    # 3) an UNKNOWN command falls back to the generic typed decode (double shown)
    unk = _build_field(1, 1, 3.14159) + _build_field(2, 0, 7) + _build_field(3, 2, "hi")
    env3 = _build_field(5, 0, 99999) + _build_field(7, 2, unk)
    d3 = decode_wscmd(env3)
    print("\nUNKNOWN cmd (generic):", json.dumps(d3, indent=2))
    assert d3["payload_schema"] == "generic"
    assert abs(d3["payload"]["f1"]["double"] - 3.14159) < 1e-4

    # 4) heartbeat
    hb = _build_field(1, 0, 1) + _build_field(2, 0, 1721600000000) + _build_field(3, 2, "txtl")
    dh = decode_heartbeat(hb)
    print("\nheartbeat:", dh)
    assert dh["tag"] == "txtl" and dh["flag"] == 1

    print("\n" + "=" * 40 + "\nALL SELF-TESTS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(_selftest())

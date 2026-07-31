"""
dwarflab_controller.py
WebSocket + HTTP controller for the DWARF 3 smart telescope.

Reverse-engineered from com.convergence.dwarflab APK (June 2026 release, 170 MB).
All command numbers verified against WsCmd.java in decompiled source.

Changelog vs previous version:
  - Added CMD_CAMERA_TELE_START_RECORD = 10005  (was missing; now distinct from stop)
  - Removed CMD_CAMERA_TELE_STOP_BURST = 10004  (removed in new APK)
  - Added new module ranges: MODULE_VOICE_ASSISTANT (16800-16899),
    MODULE_DEVICE (17000-17099)
  - Added new commands: CMD_VOICE_ASSISTANT_TASK=16800,
    CMD_DEVICE_AUTO_COOLING=17001, CMD_DEVICE_AUTO_SHUTDOWN=17002
  - Added new notify handlers: CMD_NOTIFY_WIDE_FOCUS_POSITION=15300,
    CMD_NOTIFY_LENS_DEFOG_STATE=15301, CMD_NOTIFY_AUTO_COOLING_STATE=15302,
    CMD_NOTIFY_AUTO_SHUTDOWN_STATE=15303
  - goto_dso() now accepts optional goto_only flag (new ReqGotoDSO field)
  - open_camera() / open_camera_wide() now use CameraType-aware payload
    (new APK sends cameraType field; old binning-only payload still accepted)
  - Fixed: CMD_ASTRO_WIDE_GO_LIVE = 11020 (was 11019 — that's STOP_EQ_SOLVING)
"""
import struct, uuid, threading, time, logging, socket, requests, websocket
log = logging.getLogger("dwarflab")

# ── Module IDs ────────────────────────────────────────────────────────────────
MODULE_NONE            = 0
MODULE_CAMERA_TELE     = 1   # 10000-10499
MODULE_ASTRO           = 3   # 11000-11499
MODULE_CAMERA_WIDE     = 2   # 12000-12499
MODULE_SYSTEM          = 4   # 13000-13299
MODULE_RGB_POWER       = 5   # 13500-13799
MODULE_MOTOR           = 6   # 14000-14499
MODULE_TRACK           = 7   # 14800-14899
MODULE_FOCUS           = 8   # 15000-15199
MODULE_NOTIFY          = 9   # 15200-15499
MODULE_PANORAMA        = 10  # 15500-15599
MODULE_ITIPS           = 11  # 15700-15799
MODULE_TEST            = 12
MODULE_SHOOTING_SCHEDULE = 13  # 16100-16399
MODULE_TASK_CENTER     = 14  # 16400-16599
MODULE_PARAM           = 15  # 16700-16799
MODULE_VOICE_ASSISTANT = 16  # 16800-16899  NEW in June 2026 APK
MODULE_DEVICE          = 17  # 17000-17099  NEW in June 2026 APK

def _module_for_cmd(c):
    if 10000 <= c < 10500: return MODULE_CAMERA_TELE
    if 11000 <= c < 11500: return MODULE_ASTRO
    if 12000 <= c < 12500: return MODULE_CAMERA_WIDE
    if 13000 <= c < 13300: return MODULE_SYSTEM
    if 13500 <= c < 13800: return MODULE_RGB_POWER
    if 14000 <= c < 14500: return MODULE_MOTOR
    if 14800 <= c < 14900: return MODULE_TRACK
    if 15000 <= c < 15200: return MODULE_FOCUS
    if 15200 <= c < 15500: return MODULE_NOTIFY
    if 15500 <= c < 15600: return MODULE_PANORAMA
    if 15700 <= c < 15800: return MODULE_ITIPS
    if 16100 <= c < 16400: return MODULE_SHOOTING_SCHEDULE
    if 16400 <= c < 16600: return MODULE_TASK_CENTER
    if 16700 <= c < 16800: return MODULE_PARAM
    if 16800 <= c < 16900: return MODULE_VOICE_ASSISTANT  # NEW
    if 17000 <= c < 17100: return MODULE_DEVICE            # NEW
    return MODULE_NONE

MSG_TYPE_REQUEST  = 0
MSG_TYPE_RESPONSE = 1

# ── Camera Tele commands (10000-10499) ────────────────────────────────────────
CMD_CAMERA_TELE_OPEN_CAMERA              = 10000
CMD_CAMERA_TELE_CLOSE_CAMERA             = 10001
CMD_CAMERA_TELE_PHOTOGRAPH               = 10002
CMD_CAMERA_TELE_BURST                    = 10003
# CMD_CAMERA_TELE_STOP_BURST             = 10004  REMOVED in June 2026 APK
CMD_CAMERA_TELE_START_RECORD             = 10005  # NEW (was missing before)
CMD_CAMERA_TELE_STOP_RECORD              = 10006
CMD_CAMERA_TELE_SET_EXP                  = 10009
CMD_CAMERA_TELE_SET_GAIN                 = 10013
CMD_CAMERA_TELE_SET_BRIGHTNESS           = 10015
CMD_CAMERA_TELE_SET_CONTRAST             = 10017
CMD_CAMERA_TELE_SET_SATURATION           = 10019
CMD_CAMERA_TELE_SET_SHARPNESS            = 10023
CMD_CAMERA_TELE_SET_WB_MODE              = 10025
CMD_CAMERA_TELE_SET_WB_CT               = 10029
CMD_CAMERA_TELE_SET_IRCUT                = 10031
CMD_CAMERA_TELE_START_TIMELAPSE_PHOTO    = 10033
CMD_CAMERA_TELE_STOP_TIMELAPSE_PHOTO     = 10034
CMD_CAMERA_TELE_GET_ALL_PARAMS           = 10036
CMD_CAMERA_TELE_SET_JPG_QUALITY          = 10040
CMD_CAMERA_TELE_PHOTO_RAW                = 10041
CMD_CAMERA_TELE_SWITCH_RESOLUTION        = 10047
CMD_CAMERA_TELE_SWITCH_FRAMERATE         = 10048

# ── Astro commands (11000-11499) ──────────────────────────────────────────────
CMD_ASTRO_START_CALIBRATION              = 11000
CMD_ASTRO_STOP_CALIBRATION               = 11001
CMD_ASTRO_START_GOTO_DSO                 = 11002
CMD_ASTRO_START_GOTO_SOLAR_SYSTEM        = 11003
CMD_ASTRO_STOP_GOTO                      = 11004
CMD_ASTRO_START_CAPTURE_RAW_LIVE_STACKING = 11005
CMD_ASTRO_STOP_CAPTURE_RAW_LIVE_STACKING  = 11006
CMD_ASTRO_GO_LIVE                        = 11010
# CAPTURE-VERIFIED 2026-07-23: mirrors 11013 the way 11003 mirrors 11002 (DSO cmd,
# solar-system cmd = +1). Request payload {1:solar_id, 2:coord1(double), 3:coord2(double),
# 4:name, 5:mode, 6:confirm(bool)}; observed with solar_id=8, name="Moon", mode=9,
# confirm=true on a follow-up sent once tracking had settled.
CMD_ASTRO_START_ONE_CLICK_GOTO_DSO       = 11013
CMD_ASTRO_START_ONE_CLICK_GOTO_SOLAR_SYSTEM = 11014
CMD_ASTRO_STOP_ONE_CLICK_GOTO            = 11015
CMD_ASTRO_START_WIDE_CAPTURE_LIVE_STACKING = 11016
CMD_ASTRO_STOP_WIDE_CAPTURE_LIVE_STACKING  = 11017
CMD_ASTRO_START_EQ_SOLVING               = 11018
CMD_ASTRO_STOP_EQ_SOLVING                = 11019
CMD_ASTRO_WIDE_GO_LIVE                   = 11020  # FIXED: was 11019 (conflict); APK: ordinal 82
CMD_ASTRO_START_AI_ENHANCE               = 11029
CMD_ASTRO_STOP_AI_ENHANCE                = 11030
CMD_ASTRO_START_ONE_CLICK_SHOOTING       = 11042
CMD_ASTRO_START_SKY_TARGET_FINDER        = 11047
CMD_ASTRO_STOP_SKY_TARGET_FINDER         = 11048

# ── Camera Wide commands (12000-12499) ────────────────────────────────────────
CMD_CAMERA_WIDE_OPEN_CAMERA              = 12000
CMD_CAMERA_WIDE_START_RECORD             = 12005  # confirmed present in new APK
CMD_CAMERA_WIDE_STOP_RECORD              = 12006

# ── System commands (13000-13299) ─────────────────────────────────────────────
CMD_SYSTEM_SET_TIME                      = 13000
CMD_SYSTEM_SET_MASTERLOCK                = 13004  # ReqsetMasterLock{ bool lock = 1 }
CMD_SYSTEM_SET_LOCATION                  = 13010

# ── RGB/Power commands (13500-13799) ──────────────────────────────────────────
CMD_RGB_POWER_OPEN_RGB                   = 13500
CMD_RGB_POWER_CLOSE_RGB                  = 13501
CMD_RGB_POWER_POWER_DOWN                 = 13502
CMD_RGB_POWER_POWERIND_ON                = 13503
CMD_RGB_POWER_POWERIND_OFF               = 13504
CMD_RGB_POWER_REBOOT                     = 13505

# ── Motor commands (14000-14499) ──────────────────────────────────────────────
# GHIDRA-derived (magni FUN_00448fb0 registration table, 2026-07-26 pass) full
# cmd->handler map for 14000-14013. Every per-axis command below selects one
# of 4 motor-axis objects via a protobuf field (0-3) before acting - DWARF3
# has (at least) 4 independently addressable motor axes at the firmware
# level, not just RA/DEC. Names/semantics below not confirmed live except
# where an explicit zlog name was found in the decompile (marked CONFIRMED);
# the rest are inferred purely from callee behavior (marked INFERRED) - treat
# with appropriate skepticism until tested against a live axis-index sweep.
CMD_STEP_MOTOR_MOVE_TO_ANGLE             = 14000  # INFERRED: axis-select + absolute move (FUN_007457c0)
CMD_STEP_MOTOR_RUN_TO                    = 14001  # CONFIRMED zlog name "runTo"
CMD_STEP_MOTOR_STOP_AXIS                 = 14002  # INFERRED: axis-select, parameterless halt vtable call
CMD_STEP_MOTOR_RESET                     = 14003  # CONFIRMED zlog name "reset"
CMD_STEP_MOTOR_CANCEL_MOVE               = 14004  # INFERRED: axis-select + FUN_00743960 (abort)
CMD_STEP_MOTOR_STOP_MOVE                 = 14005  # INFERRED: axis-select + FUN_00742798 (softer stop)
CMD_STEP_MOTOR_JOYSTICK                  = 14006  # CMD_STEP_MOTOR_SERVICE_JOYSTICK
CMD_STEP_MOTOR_JOYSTICK_FIXED_ANGLE      = 14007  # CMD_STEP_MOTOR_SERVICE_JOYSTICK_FIXED_ANGLE
CMD_STEP_MOTOR_JOYSTICK_STOP             = 14008  # CMD_STEP_MOTOR_SERVICE_JOYSTICK_STOP
CMD_STEP_MOTOR_DUAL_CAMERA_LINKAGE       = 14009  # CONFIRMED zlog name "startDualCameraLinkage" (syncs tele+wide motor movement)
CMD_STEP_MOTOR_RUN_TO_RAMPED             = 14010  # INFERRED: axis-select + speed-ramped move (2 variants by flag)
# GHIDRA-derived (magni FUN_00448fb0 handler table, not in the app's WsCmd
# list at all): empty-payload requests that start/stop a continuous
# CMD_NOTIFY_DEVICE_ATTITUDE (15295) stream from the mount's built-in gyro.
CMD_MOT_START_DEVICE_ATTITUDE_NOTIFY     = 14012
CMD_MOT_STOP_DEVICE_ATTITUDE_NOTIFY      = 14013
CMD_STEP_MOTOR_GET_POSITION              = 14011  # CONFIRMED zlog name "getPosition"

# ── Track commands (14800-14899) ──────────────────────────────────────────────
CMD_TRACK_START_TRACK                    = 14800
CMD_TRACK_STOP_TRACK                     = 14801
CMD_SENTRY_MODE_START                    = 14802  # ReqStartSentryMode{ int32 mode=1 }
CMD_SENTRY_MODE_STOP                     = 14803
CMD_MOT_START                            = 14804  # Multi-Object Tracking start
CMD_MOT_TRACK_ONE                        = 14805  # tele: lock one detected id
CMD_UFOTRACK_MODE_START                  = 14806  # UFO mode (reuses sentry, v1.5+)
CMD_UFOTRACK_MODE_STOP                   = 14807
CMD_MOT_WIDE_TRACK_ONE                   = 14808  # wide: lock one detected id
CMD_WIDE_TELE_TRACK_SWITCH               = 14809  # 30-class detect, wide/tele switch
CMD_UFO_HAND_AOTO_MODE                   = 14810  # UFO manual(0)/auto(1) select

# ── V3 camera bring-up + MOT (authoritative dwarfAlp proto / DWARF API2) ───────
# CORRECTION: an earlier version of this file WRONGLY labelled cmd 11043 as a
# "wide AI track start". The authoritative DWARF proto shows 11043 =
# V3_ASTRO_GET_PRESETS and 11040 = V3_ASTRO_GET_PARAMS — NEITHER is a track
# command. They appeared next to the box stream only because tracking was already
# running. DWARF 3 runs V3 firmware; the real subject-tracking pipeline is below.
CMD_V3_CAMERA_TELE_OPEN_CAMERA           = 10050  # V3ReqOpenTeleCamera {action:1=open}
CMD_V3_CAMERA_WIDE_OPEN_CAMERA           = 12036  # V3ReqOpenWideCamera {action:0=open,1=close}
CMD_V3_ASTRO_GET_PARAMS                  = 11040
CMD_V3_ASTRO_GET_PRESETS                 = 11043  # NOT a track command (was mislabelled)
CMD_V3_DEVICE_CONFIG_SHOOTING_MODE       = 16403  # {mode_id:1=photo,3=burst,4=video,5=timelapse}
CMD_V3_DEVICE_CONFIG_MODE_SWITCH         = 16404
# Real subject-tracking commands live in MODULE_TRACK (already defined above):
#   14800 CMD_TRACK_START_TRACK   {x,y,w,h}  -> basic correlation tracker
#                                              (locks only a distinct/MOVING target)
#   14809 CMD_WIDE_TELE_TRACK_SWITCH         -> enable the 30-class object detector
#   14804 CMD_MOT_START                       -> start multi-object tracking
#   14808 CMD_MOT_WIDE_TRACK_ONE {id}         -> lock a detected object by id (wide)
#   14805 CMD_MOT_TRACK_ONE      {id}         -> lock a detected object by id (tele)
# Detected boxes+ids arrive via 15238 / 15251 (multi) and 15252 (single)._

# ── Focus commands (15000-15199) ──────────────────────────────────────────────
CMD_FOCUS_AUTO_FOCUS                     = 15000
CMD_FOCUS_MANUAL_SINGLE_STEP             = 15001
CMD_FOCUS_START_MANUAL_CONTINUOUS        = 15002
CMD_FOCUS_STOP_MANUAL_CONTINUOUS         = 15003
CMD_FOCUS_START_ASTRO_AUTO_FOCUS         = 15004
CMD_FOCUS_STOP_ASTRO_AUTO_FOCUS          = 15005

# ── Notify commands (15200-15499) ─────────────────────────────────────────────
CMD_NOTIFY_ELE                           = 15201
CMD_NOTIFY_ELE_STATUS                    = 15202
CMD_NOTIFY_TEMPERATURES                  = 15203  # SDCARD_INFO in APK; carries temps on Dwarf3
CMD_NOTIFY_STATE_CAPTURE_RAW_LIVE_STACKING = 15208
CMD_NOTIFY_PROGRASS_CAPTURE_RAW_LIVE_STACKING = 15209
CMD_NOTIFY_STATE_ASTRO_CALIBRATION       = 15210
CMD_NOTIFY_STATE_ASTRO_GOTO              = 15211
CMD_NOTIFY_STATE_ASTRO_TRACKING          = 15212
CMD_NOTIFY_TRACK_RESULT                  = 15225  # tele single-target box {x,y,w,h}
CMD_NOTIFY_SENTRY_MODE_STATE             = 15231  # sentinel/UFO state machine code
CMD_NOTIFY_SENTRY_MODE_TRACK_RESULT      = 15232  # sentinel-mode box {x,y,w,h}
# CAPTURE-VERIFIED 2026-07-23: nested at field 3, {1:state, 2:name}; state seen as 3 right
# after an ASTRO_START_ONE_CLICK_GOTO_SOLAR_SYSTEM request, then 1 once the app reported
# steady-state tracking — exact enum meaning still unconfirmed, but the transition matches
# goto -> tracking.
CMD_NOTIFY_ASTRO_TARGET_STATUS           = 15233
CMD_NOTIFY_MULTI_TRACK_RESULT            = 15238  # tele multi-box (nested repeated)
CMD_NOTIFY_UFO_MODE_STATE                = 15240  # sentinel-UFO mode state
CMD_NOTIFY_WIDE_MULTI_TRACK_RESULT       = 15251  # wide multi-box (nested repeated)
CMD_NOTIFY_WIDE_TRACK_RESULT             = 15252  # wide single-target box {x,y,w,h}
CMD_NOTIFY_WIDE_TRACK_STATE              = 15284  # CAPTURE-VERIFIED (June 2026): the
        # firmware emits this during a wide track; payload {1: active, 2: state}.
        # NOT present in the decompiled WsCmd table — found by decoding a live
        # iOS-app capture (see dwarf_capture_decode.py / TRACKING_FINDINGS.md).
CMD_NOTIFY_TEMPERATURE                   = 15243
# GHIDRA-derived: DeviceAttitude{pitch=1, yaw=2, roll=3} (all double), sent by
# a dedicated background thread in magni while CMD_MOT_START_DEVICE_ATTITUDE_
# NOTIFY (14012) is active. The firmware's own sender only ever populates 2 of
# the 3 fields in the decompiled path we traced (a heading-like angle minus a
# fixed 157.75 deg offset, plus a second angle) — treat the third as unverified
# until confirmed live.
CMD_NOTIFY_DEVICE_ATTITUDE               = 15295

# ── Tracking coordinate reference (CAPTURE-VERIFIED) ───────────────────────────
# Track-result boxes are TOP-LEFT (x, y) + (w, h) in a FIXED reference frame,
# NOT in normalised [0,1] and NOT in the live RTSP frame's pixel size. Across a
# real session the box edges approached x+w≈1280 and y+h≈720, i.e. a 1280x720
# reference. Scale boxes by these constants (or the value the firmware actually
# renders against) rather than the decoded frame dimensions when the wide RTSP
# stream is delivered at a different resolution.
TRACK_REF_W = 1280
TRACK_REF_H = 720
TRACK_NO_TARGET = -100   # firmware sends x=y=-100 (negative varint) when no lock

# Tracker error / status codes (arrive as the packet cmd id, 4.14.2)
CODE_TRACK_TRACKER_INITING               = 14900  # tracker is initializing
CODE_TRACK_TRACKER_FAILED                = 14901  # tracker failed to lock
CMD_NOTIFY_FOCUS_POSITION                = 15257
CMD_NOTIFY_CMOS_TEMPERATURE              = 15292
CMD_NOTIFY_WIDE_FOCUS_POSITION           = 15300  # NEW: wide lens focus position
CMD_NOTIFY_LENS_DEFOG_STATE              = 15301  # NEW: lens defog state
CMD_NOTIFY_AUTO_COOLING_STATE            = 15302  # NEW: auto cooling state
CMD_NOTIFY_AUTO_SHUTDOWN_STATE           = 15303  # NEW: auto shutdown state

# ── Task Center commands (16400-16599) ────────────────────────────────────────
CMD_GLOBAL_TASK_MANAGER_ENTER_CAMERA     = 16404
CMD_GLOBAL_TASK_GET_DEVICE_STATE_INFO    = 16405
CMD_GLOBAL_VOICE_ASSISTANT_TASK          = 16406  # NEW in June 2026 APK

# ── Voice Assistant commands (16800-16899) ────────────────────────────────────
CMD_VOICE_ASSISTANT_TASK                 = 16800  # NEW in June 2026 APK

# ── Device commands (17000-17099) ─────────────────────────────────────────────
CMD_DEVICE_LENS_DEFOG                    = 17000  # zlog-confirmed (2026-07-31), was missing
CMD_DEVICE_AUTO_COOLING                  = 17001  # NEW in June 2026 APK
CMD_DEVICE_AUTO_SHUTDOWN                 = 17002  # NEW in June 2026 APK

# ── Panorama commands (15500-15599) — added 2026-07-31 from firmware Ghidra
# ack-call scan (ack_calls_parsed.txt) + proto_reconstructed/panorama.proto.
# IDs marked UNCONFIRMED are gaps in the 295-site bare-ack scan (likely
# data-returning GET handlers using the other universal sender, FUN_00793398,
# which the scan didn't cover) — assigned by proto declaration-order inference,
# same pattern independently confirmed for every ID around them. See
# dwarf_protobuf.py SCHEMAS for the full per-field payload layout.
CMD_PANORAMA_START_GRID                  = 15500
CMD_PANORAMA_STOP                        = 15501
CMD_PANORAMA_START_BY_EULER_RANGE        = 15502  # UNCONFIRMED id
CMD_PANORAMA_START_STITCH_UPLOAD         = 15503
CMD_PANORAMA_STOP_STITCH_UPLOAD          = 15504
CMD_PANORAMA_GET_CURRENT_UPLOAD_STATE    = 15505  # UNCONFIRMED id
CMD_PANORAMA_GET_UPLOAD_PREDICT          = 15506  # UNCONFIRMED id
CMD_PANORAMA_START_COMPRESS              = 15507
CMD_PANORAMA_STOP_COMPRESS               = 15508
CMD_PANORAMA_START_FRAMING               = 15509
CMD_PANORAMA_STOP_FRAMING                = 15510
CMD_PANORAMA_RESET_FRAMING               = 15511
CMD_PANORAMA_UPDATE_FRAMING_RECT         = 15512
CMD_PANORAMA_STOP_FRAMING_AND_START_GRID = 15513
CMD_NOTIFY_PANORAMA_FRAMING              = 15514  # device -> phone

# ── Shooting Schedule commands (16100-16108) — added 2026-07-31, same sourcing
# as Panorama above. UNCONFIRMED ids (16102/16103/16106/16107) are proto-order
# inferred between zlog-confirmed neighbors (16100/16101/16104/16105/16108).
CMD_SHOOTING_SCHEDULE_SYNC               = 16100
CMD_SHOOTING_SCHEDULE_CANCEL             = 16101
CMD_SHOOTING_SCHEDULE_GET_ALL            = 16102  # UNCONFIRMED id
CMD_SHOOTING_SCHEDULE_GET_BY_ID          = 16103  # UNCONFIRMED id
CMD_SHOOTING_SCHEDULE_GET_TASK_ID        = 16104
CMD_SHOOTING_SCHEDULE_REPLACE            = 16105
CMD_SHOOTING_SCHEDULE_UNLOCK             = 16106  # UNCONFIRMED id
CMD_SHOOTING_SCHEDULE_LOCK               = 16107  # UNCONFIRMED id
CMD_SHOOTING_SCHEDULE_DELETE             = 16108

# ── Param commands (16700-16706) — added 2026-07-31. ALL 7 zlog-CONFIRMED
# (CMD_PARAM_SET_EXPOSURE .. CMD_PARAM_SET_AUTO_PARAMS in ack_calls_parsed.txt).
# param_id is the 64-bit paramId whose bit 44 selects camera (0=tele/1=wide) —
# see the "parameter commands DO bake camera identity" finding in
# dwarf3-tracking-protocol memory; fetch the real per-camera/per-field param_id
# table from GET http://<device>:8082/getDefaultParamsConfig at runtime rather
# than hardcoding it here (it may change per firmware build).
CMD_PARAM_SET_EXPOSURE                   = 16700
CMD_PARAM_SET_GAIN                       = 16701
CMD_PARAM_SET_WB                         = 16702
CMD_PARAM_SET_GENERAL_INT_PARAM          = 16703
CMD_PARAM_SET_GENERAL_FLOAT_PARAM        = 16704
CMD_PARAM_SET_GENERAL_BOOL_PARAM         = 16705
CMD_PARAM_SET_AUTO_PARAMS                = 16706

# ── Proto3 primitives ─────────────────────────────────────────────────────────
def _varint(v):
    v = v & 0xFFFFFFFFFFFFFFFF  # two's-complement uint64
    b = []
    while True:
        b.append(v & 0x7F); v >>= 7
        if v == 0: break
    for i in range(len(b) - 1): b[i] |= 0x80
    return bytes(b)

def _field(fn, wt, val):
    tag = _varint((fn << 3) | wt)
    if wt == 0: return tag + _varint(val)
    if wt == 1: return tag + struct.pack("<d", val)
    if wt == 2:
        if isinstance(val, str): val = val.encode()
        return tag + _varint(len(val)) + val
    if wt == 5: return tag + struct.pack("<f", val)  # float32 (proto3 `float`)
    raise ValueError(f"bad wt {wt}")

def _dvarint(buf, pos):
    r = 0; s = 0
    while True:
        b = buf[pos]; pos += 1; r |= (b & 0x7F) << s
        if not (b & 0x80): break
        s += 7
    return r, pos

def _to_signed(v):
    """Interpret a decoded protobuf varint as a (sign-extended) signed int."""
    v &= 0xFFFFFFFFFFFFFFFF
    if v >= 0x8000000000000000:
        v -= 0x10000000000000000
    return v

def _parse_varint_fields(data):
    """Walk a protobuf message and return {field_number: signed_value} for all
    varint (wire type 0) fields. Length-delimited / fixed fields are skipped.
    Used to decode ResNotifyTrackResult {x=1,y=2,w=3,h=4}."""
    out = {}
    i, n = 0, len(data)
    try:
        while i < n:
            tag, i = _dvarint(data, i)
            fn, wt = tag >> 3, tag & 7
            if wt == 0:
                v, i = _dvarint(data, i); out[fn] = _to_signed(v)
            elif wt == 2:
                ln, i = _dvarint(data, i); i += ln
            elif wt == 1:
                i += 8
            elif wt == 5:
                i += 4
            else:
                break
    except IndexError:
        pass
    return out

def _parse_double_fields(data):
    """Walk a protobuf message and return {field_number: float} for all
    fixed64 (wire type 1, e.g. `double`) fields. Used to decode
    DeviceAttitude {pitch=1, yaw=2, roll=3}, all `double`."""
    out = {}
    i, n = 0, len(data)
    try:
        while i < n:
            tag, i = _dvarint(data, i)
            fn, wt = tag >> 3, tag & 7
            if wt == 1:
                out[fn] = struct.unpack("<d", data[i:i + 8])[0]
                i += 8
            elif wt == 0:
                _, i = _dvarint(data, i)
            elif wt == 2:
                ln, i = _dvarint(data, i); i += ln
            elif wt == 5:
                i += 4
            else:
                break
    except IndexError:
        pass
    return out

def _parse_multi_track(data):
    """Best-effort decode of CMD_NOTIFY_MULTI_TRACK_RESULT (15238) /
    CMD_NOTIFY_WIDE_MULTI_TRACK_RESULT (15251). The exact layout is undocumented
    (no populated sample captured yet); this treats each top-level length-delimited
    field as one detected-object sub-message and returns its varint fields, e.g.
    {1:id, 2:x, 3:y, 4:w, 5:h} OR {1:x,2:y,3:w,4:h} depending on firmware. Refine
    the field mapping once a real multi-box sample is captured with
    dwarf_capture_decode.py."""
    out = []
    i, n = 0, len(data)
    try:
        while i < n:
            tag, i = _dvarint(data, i); fn = tag >> 3; wt = tag & 7
            if wt == 2:
                ln, i = _dvarint(data, i); sub = data[i:i + ln]; i += ln
                f = _parse_varint_fields(sub)
                if f:
                    out.append(f)
            elif wt == 0:
                _, i = _dvarint(data, i)
            elif wt == 1:
                i += 8
            elif wt == 5:
                i += 4
            else:
                break
    except IndexError:
        pass
    return out

# ── WsRespCode table (from com/convergence/dwarflab/data/bean/ws/WsRespCode.java,
# the APK's complete enum, 129 entries) ────────────────────────────────────────
# Arrives in the packet's `type` field (f6) on a response/ack. 0 = WS_OK, any
# negative value is a specific failure. Each module's error range mirrors its
# cmd-id range with a fixed offset, e.g. CAMERA_TELE cmds 10000-10047 <->
# errors -10500..-10518, ASTRO cmds 11000s <-> -11500s, etc.
WSRESPCODE = {
    0: "WS_OK",
    -1: "WS_PARSE_PROTOBUF_ERROR",
    -2: "WS_SDCARD_NOT_EXIST",
    -3: "WS_INVAID_PARAM",
    -4: "WS_SDCARD_WRITE_ERROR",
    -5: "WS_DEVICE_NOT_ACTIVATED",
    -6: "WS_SDCARD_FULL_ERROR",
    -10500: "CODE_CAMERA_TELE_OPENED",
    -10501: "CODE_CAMERA_TELE_CLOSED",
    -10502: "CODE_CAMERA_TELE_ISP_SET_FAILED",
    -10503: "CODE_CAMERA_TELE_OPEN_FAILED",
    -10504: "CODE_CAMERA_TELE_START_RECORD_FAILED",
    -10505: "CODE_CAMERA_TELE_STOP_RECORD_FAILED",
    -10506: "CODE_CAMERA_TELE_CAPTURE_RAW_FAILED",
    -10507: "CODE_CAMERA_TELE_WORKING_BUSY",
    -10508: "CODE_CAMERA_TELE_GET_IMAGE_FAILED",
    -10509: "CODE_CAMERA_TELE_RUNNING_PHOTO",
    -10510: "CODE_CAMERA_TELE_RUNNING_RECORD",
    -10511: "CODE_CAMERA_TELE_RUNNING_PANORAMA",
    -10512: "CODE_CAMERA_TELE_RUNNING_TIMELAPSE",
    -10513: "CODE_CAMERA_TELE_RUNNING_CAPTURE_DARK",
    -10514: "CODE_CAMERA_TELE_RUNNING_CAPTURE_LIVE_STACKING",
    -10515: "CODE_CAMERA_TELE_EXP_TOO_LONG",
    -10516: "CODE_CAMERA_TELE_SWITCH_WORK_MODE_FAILED",
    -10517: "CODE_CAMERA_TELE_RUNNING_TRACK",
    -10518: "CODE_CAMERA_TELE_RECORD_FILE_ERROR",
    -11500: "CODE_ASTRO_PLATE_SOLVING_FAILED",
    -11501: "CODE_ASTRO_FUNCTION_BUSY",
    -11502: "CODE_ASTRO_DARK_GAIN_OUT_OF_RANGE",
    -11503: "CODE_ASTRO_DARK_NOT_FOUND",
    -11504: "CODE_ASTRO_CALIBRATION_FAILED",
    -11505: "CODE_ASTRO_GOTO_FAILED",
    -11506: "CODE_ASTRO_DARK_RUNNING",
    -11507: "CODE_ASTRO_CALIBRATION_RUNNING",
    -11508: "CODE_ASTRO_GOTO_RUNNING",
    -11509: "CODE_ASTRO_LIVE_STACKING_RUNNING",
    -11510: "CODE_ASTRO_RESET_PITCH_MOTOR_FAILED",
    -11511: "CODE_ASTRO_NEED_CALIBRATION",
    -11512: "CODE_ASTRO_GOTO_READ_MOTOR_POSITION_AND_PLATE_SOLVING_FAILED",
    -11513: "CODE_ASTRO_NEED_GOTO",
    -11514: "CODE_ASTRO_NEED_ADJUST_SHOOT_PARAM",
    -11515: "CODE_ASTRO_CALIBRATION_PLATE_SOLVING_FAILED_TOO_MUCH",
    -11516: "CODE_ASTRO_EQ_SOLVING_FAILED",
    -11517: "CODE_ASTRO_SKY_SEARCH_FAILED",
    -11518: "CODE_ASTRO_NEED_GOTO_DSO",
    -11519: "CODE_ASTRO_RESTACK_CAMERA_MISMATCH",
    -11520: "CODE_ASTRO_RESTACK_BINNING_MISMATCH",
    -11521: "CODE_ASTRO_RESTACK_FILTER_MISMATCH",
    -11522: "CODE_ASTRO_RESTACK_TARGET_MISMATCH",
    -11523: "CODE_ASTRO_RESTACK_DARKFRAME_MISMATCH",
    -11524: "CODE_ASTRO_RESTACK_FAILED",
    -11525: "CODE_ASTRO_RESTACK_INVALID_DATA",
    -11526: "CODE_ASTRO_OVEREXPOSURE_WARNING",
    -11527: "CODE_ASTRO_EXP_TOO_LONG",
    -11528: "CODE_ASTRO_NEED_EQ",
    -11529: "CODE_ASTRO_STAR_TOO_FEW",
    -11530: "CODE_ASTRO_DARK_TEMP_MISMATCH",
    -11531: "CODE_ASTRO_SUN_MOON_NOT_FOUND",
    -12500: "CODE_CAMERA_WIDE_OPENED",
    -12501: "CODE_CAMERA_WIDE_CLOSED",
    -12502: "CODE_CAMERA_WIDE_CANNOT_FOUND",
    -12503: "CODE_CAMERA_WIDE_OPEN_FAILED",
    -12504: "CODE_CAMERA_WIDE_CLOSE_FAILED",
    -12505: "CODE_CAMERA_WIDE_SET_ISP_FAILED",
    -12506: "CODE_CAMERA_WIDE_PHOTOGRAPHING",
    -12507: "CODE_CAMERA_WIDE_TIMELAPSE_RECORDING",
    -12508: "CODE_CAMERA_WIDE_EXP_TOO_LONG",
    -12509: "CODE_CAMERA_WIDE_RECORD_FILE_ERROR",
    -13300: "CODE_SYSTEM_SET_TIME_FAILED",
    -13301: "CODE_SYSTEM_SET_TIMEZONE_FAILED",
    -13800: "CODE_RGB_POWER_UART_INIT_FAILED",
    -13801: "CODE_RGB_POWER_UART_SEND_FAILED",
    -14500: "CODE_STEP_MOTOR_IS_RUNNING",
    -14501: "CODE_STEP_MOTOR_IS_STOPPED",
    -14502: "CODE_STEP_MOTOR_PARALLEL_IN",
    -14503: "CODE_STEP_MOTOR_PARALLEL_END",
    -14504: "CODE_STEP_MOTOR_INVALID_PARAMETER_ID",
    -14505: "CODE_STEP_MOTOR_INVALID_PARAMETER_ANGLE",
    -14506: "CODE_STEP_MOTOR_INVALID_PARAMETER_SPEED",
    -14507: "CODE_STEP_MOTOR_INVALID_PARAMETER_SPEED_RAMPING",
    -14508: "CODE_STEP_MOTOR_INVALID_PARAMETER_RESOLUTION",
    -14509: "CODE_STEP_MOTOR_INVALID_PARAMETER_POSITION",
    -14510: "CODE_STEP_MOTOR_OVERTIME_GET_LIMIT_RETURN",
    -14511: "CODE_STEP_MOTOR_OVERTIME_GET_RESET_RETURN",
    -14512: "CODE_STEP_MOTOR_OVERTIME_GET_ABSOLUTE_POSITION_RETURN",
    -14513: "CODE_STEP_MOTOR_OVERTIME_GET_RELATIVE_POSITION_RETURN",
    -14514: "CODE_STEP_MOTOR_OVERTIME_WAIT_TO_STOP",
    -14515: "CODE_STEP_MOTOR_OVERTIME_WAIT_TO_RUN",
    -14516: "CODE_STEP_MOTOR_LIMIT_SPEED_TO_MAX",
    -14517: "CODE_STEP_MOTOR_LIMIT_SPEED_TO_MIN",
    -14518: "CODE_STEP_MOTOR_LIMIT_POSITION_WARNING",
    -14519: "CODE_STEP_MOTOR_LIMIT_POSITION_HIT",
    -14520: "CODE_STEP_MOTOR_NEED_RESET",
    -14521: "CODE_STEP_MOTOR_OVERTIME_GET_PE_SWITCH_RETURN",
    -14522: "CODE_STEP_MOTOR_OVERTIME_TO_RESET",
    -14900: "CODE_TRACK_TRACKER_INITING",
    -14901: "CODE_TRACK_TRACKER_FAILED",
    -14902: "CODE_TRACK_SENTRY_MODE_INITING",
    -14903: "CODE_TRACK_SENTRY_MODE_FAILED",
    -14904: "CODE_UFOTRACK_MODE_INITING",
    -14905: "CODE_UFOTRACK_MODE_FAILED",
    -14906: "CODE_UFO_DAY_AUTO_MODE",
    -15100: "CODE_FOCUS_ASTRO_AUTO_FOCUS_SLOW_ERROR",
    -15101: "CODE_FOCUS_ASTRO_AUTO_FOCUS_FAST_ERROR",
    -15106: "CODE_FOCUS_EXP_TOO_LONG",
    -15107: "CODE_FOCUS_INFINITY_POS_ERROR",
    -15108: "CODE_FOCUS_GET_NOW_POS_FAILED",
    -15600: "CODE_PANORAMA_PHOTO_FAILED",
    -15601: "CODE_PANORAMA_MOTOR_RESET_FAILED",
    -15602: "CODE_PANORAMA_UPLOAD_USER_STOP",
    -15603: "CODE_PANORAMA_UPLOAD_FILE_CHECK_FAILED",
    -15604: "CODE_PANORAMA_UPLOAD_COMPRESS_FAILED",
    -15605: "CODE_PANORAMA_UPLOAD_UPLOAD_FAILED",
    -15606: "CODE_PANORAMA_UPLOAD_NOT_EXIST",
    -15607: "CODE_PANORAMA_UPLOAD_IS_RUNNING",
    -15608: "CODE_PANORAMA_UPLOAD_CAMERA_BUSY",
    -15609: "CODE_PANORAMA_UPLOAD_NOT_IN_STA",
    -15612: "CODE_PANORAMA_COMPRESSION_IS_RUNNING",
    -15614: "CODE_PANORAMA_COMPOSE_IS_IDEL",
    -15615: "CODE_PANORAMA_COMPOSE_IS_RUNNING",
    -16300: "CODE_SHOOTING_SCHEDULE_DEVICE_ID_NOT_MATCH",
    -16301: "CODE_SHOOTING_SCHEDULE_INVALID_SHOOTING_DURATION",
    -16302: "CODE_SHOOTING_SCHEDULE_TIME_CONFLICT",
    -16303: "CODE_SHOOTING_SCHEDULE_INVALID_TASK_DURATION",
    -16305: "CODE_SHOOTING_SCHEDULE_DATABASE_OPERATION_FAILED",
    -16306: "CODE_SHOOTING_SCHEDULE_PASSWORD_ERROR",
    -16307: "CODE_SHOOTING_SCHEDULE_SHOOTING",
    -16308: "CODE_SHOOTING_SCHEDULE_START_TIME_TOO_FAR",
    -16600: "CODE_GLOBAL_TASK_MANAGER_BUSY",
}

def describe_resp_code(code):
    """Human-readable name for a WsRespCode `type` value, e.g. -11528 ->
    'CODE_ASTRO_NEED_EQ'. Falls back to 'UNKNOWN(code)' for anything not in
    the APK's enum (e.g. a future firmware version added a new code)."""
    return WSRESPCODE.get(code, f"UNKNOWN({code})")

# ── Packet builder / parser ───────────────────────────────────────────────────
def build_ws_packet(cmd, data=b"", device_id=1, client_id=""):
    mid = _module_for_cmd(cmd)
    pkt = (_field(1,0,1) + _field(2,0,20) + _field(3,0,device_id) +
           _field(4,0,mid) + _field(5,0,cmd) + _field(6,0,0))
    if data:      pkt += _field(7, 2, data)
    if client_id: pkt += _field(8, 2, client_id)
    return pkt

def parse_ws_packet(raw):
    r = {"major_version":0,"minor_version":0,"device_id":0,
         "module_id":0,"cmd":0,"type":0,"data":b"","client_id":""}
    i = 0
    while i < len(raw):
        try:
            tag, i = _dvarint(raw, i)
            fn = tag >> 3; wt = tag & 7
            if wt == 0:
                v, i = _dvarint(raw, i)
                k = {1:"major_version",2:"minor_version",3:"device_id",
                     4:"module_id",5:"cmd",6:"type"}.get(fn)
                if k: r[k] = v
            elif wt == 2:
                ln, i = _dvarint(raw, i)
                pay = raw[i:i+ln]; i += ln
                if fn == 7: r["data"] = pay
                elif fn == 8: r["client_id"] = pay.decode("utf-8", "replace")
            elif wt == 1: i += 8
            elif wt == 5: i += 4
            else: break
        except: break
    return r

# ── Payload builders ──────────────────────────────────────────────────────────
def p_int(v): return _field(1, 0, v) if v != 0 else b""

def p_goto_dso(ra, dec, name="", goto_only=False):
    """
    ReqGotoDSO payload.
    ra, dec: float (hours / degrees).
    goto_only: new field in June 2026 APK — if True, only slews without stacking.
    """
    d = _field(1, 1, float(ra)) + _field(2, 1, float(dec))
    if name: d += _field(3, 2, name)
    if goto_only: d += _field(4, 0, 1)
    return d

def p_location(lat, lon, alt=0.0):
    return _field(1, 1, float(lat)) + _field(2, 1, float(lon)) + _field(3, 1, float(alt))

def p_joystick(vector_angle_deg, vector_length):
    """
    ReqMotorServiceJoystick: field1=vector_angle(double), field2=vector_length(double).
    Both are 64-bit doubles (proto wire type 1).
    vector_angle_deg: 0=East, 90=North, 180=West, 270=South.
    vector_length: 0.0-1.0 (proportion of max motor speed).
    """
    return _field(1, 1, float(vector_angle_deg)) + _field(2, 1, float(vector_length))

def xy_to_polar(x, y):
    """
    Convert cartesian joystick x/y (-100..100) to (vector_angle_deg, vector_length).
    x positive=East, y positive=North.
    Matches PolarDpadJoystickView atan2(-cy, cx) convention.
    """
    import math
    if x == 0 and y == 0: return 0.0, 0.0
    angle = math.degrees(math.atan2(y, x))
    if angle < 0: angle += 360.0
    length = min(1.0, math.hypot(x, y) / 100.0)
    return angle, length

# ── Main controller class ─────────────────────────────────────────────────────
class DwarfLab:
    DEFAULT_IP = "192.168.88.1"
    WS_PORT    = 9900
    HTTP_PORT  = 8082

    def __init__(self, host=DEFAULT_IP, device_id=1, on_notify=None):
        self.host      = host
        self.device_id = device_id
        self.client_id = str(uuid.uuid4())
        self._ws        = None
        self._ws_thread = None
        self._connected = threading.Event()
        self.on_notify  = on_notify
        self.state = {
            "connected": False,
            "battery": None,
            "temperature": None,
            "cmos_temp": None,
            "wide_focus_position": None,  # NEW
            "auto_cooling": None,          # NEW
            "auto_shutdown": None,         # NEW
            "goto_state": None,
            "tracking": False,
            "stacking": False,
            "stacking_progress": 0,
            "calibrating": False,
            "focus_position": None,
            "last_cmd": None,
            "track_box": None,      # (x, y, w, h) live tracked box in TRACK_REF px
            "track_box_ts": 0.0,    # time.time() of last track-box update
            "track_box_src": None,  # "tele" | "wide"
            "track_state": None,    # (active, state) from CMD_NOTIFY_WIDE_TRACK_STATE
            "multi_boxes": [],      # detected objects from MOT (15238/15251)
            "multi_boxes_ts": 0.0,
            "multi_boxes_src": None,
        }

    def _ws_url(self):
        return f"ws://{self.host}:{self.WS_PORT}/?client_id={self.client_id}"

    def _start_ws(self):
        self._ws = websocket.WebSocketApp(
            self._ws_url(),
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
        )
        t = threading.Thread(
            target=self._ws.run_forever,
            kwargs={"ping_interval": 20, "ping_timeout": 10},
            daemon=True,
        )
        t.start()
        return t

    def connect(self, timeout=10.0):
        self._ws_thread = self._start_ws()
        ok = self._connected.wait(timeout)
        if ok: self.state["connected"] = True
        threading.Thread(target=self._reconnect_loop, daemon=True).start()
        return ok

    def _reconnect_loop(self):
        while True:
            time.sleep(5)
            if not self._connected.is_set():
                log.warning("WS disconnected — reconnecting...")
                self.state["connected"] = False
                try:
                    if self._ws: self._ws.close()
                except Exception: pass
                self._connected.clear()
                self._ws_thread = self._start_ws()
                ok = self._connected.wait(10)
                if ok:
                    log.info("WS reconnected")
                    self.state["connected"] = True
                else:
                    log.warning("WS reconnect failed, will retry...")

    def disconnect(self):
        if self._ws: self._ws.close()
        self._connected.clear()
        self.state["connected"] = False

    def _on_open(self, ws):
        self._connected.set()

    def _on_message(self, ws, msg):
        if not isinstance(msg, bytes): return
        pkt = parse_ws_packet(msg)
        cmd  = pkt["cmd"]
        data = pkt["data"]

        # WsRespCode: a negative `type` on any packet is a specific named
        # failure (see WSRESPCODE / describe_resp_code()), not just "-3".
        if pkt["type"] < 0:
            name = describe_resp_code(pkt["type"])
            self.state["last_error"] = {"cmd": cmd, "code": pkt["type"],
                                         "name": name, "ts": time.time()}
            log.warning(f"WS error resp: cmd={cmd} code={pkt['type']} ({name})")

        # Battery
        if cmd == CMD_NOTIFY_ELE:
            try: v, _ = _dvarint(data, 1); self.state["battery"] = v
            except: pass
        elif cmd == CMD_NOTIFY_ELE_STATUS:
            try: v, _ = _dvarint(data, 1); self.state["battery"] = v
            except: pass
        # Temperatures
        elif cmd == CMD_NOTIFY_TEMPERATURES:
            try:
                i = 0
                tag, i = _dvarint(data, i); v1, i = _dvarint(data, i)
                tag, i = _dvarint(data, i); v2, i = _dvarint(data, i)
                self.state["temperature"] = round(v1 / 10.0, 1)
                self.state["cmos_temp"]   = round(v2 / 10.0, 1)
            except: pass
        elif cmd == CMD_NOTIFY_TEMPERATURE:
            try: v, _ = _dvarint(data, 1); self.state["temperature"] = v
            except: pass
        elif cmd == CMD_NOTIFY_CMOS_TEMPERATURE:
            try: v, _ = _dvarint(data, 1); self.state["cmos_temp"] = v
            except: pass
        # Focus positions
        elif cmd == CMD_NOTIFY_FOCUS_POSITION:
            try: v, _ = _dvarint(data, 1); self.state["focus_position"] = v
            except: pass
        elif cmd == CMD_NOTIFY_WIDE_FOCUS_POSITION:    # NEW
            try: v, _ = _dvarint(data, 1); self.state["wide_focus_position"] = v
            except: pass
        # Device state notifications (NEW)
        elif cmd == CMD_NOTIFY_AUTO_COOLING_STATE:
            try: v, _ = _dvarint(data, 1); self.state["auto_cooling"] = bool(v)
            except: pass
        elif cmd == CMD_NOTIFY_AUTO_SHUTDOWN_STATE:
            try: v, _ = _dvarint(data, 1); self.state["auto_shutdown"] = bool(v)
            except: pass
        # GoTo / tracking / stacking
        elif cmd == CMD_NOTIFY_STATE_ASTRO_GOTO:
            try: v, _ = _dvarint(data, 1); self.state["goto_state"] = v
            except: pass
        elif cmd == CMD_NOTIFY_STATE_ASTRO_TRACKING:
            try: v, _ = _dvarint(data, 1); self.state["tracking"] = bool(v)
            except: pass
        elif cmd == CMD_NOTIFY_STATE_CAPTURE_RAW_LIVE_STACKING:
            try: v, _ = _dvarint(data, 1); self.state["stacking"] = bool(v)
            except: pass
        elif cmd == CMD_NOTIFY_PROGRASS_CAPTURE_RAW_LIVE_STACKING:
            try: v, _ = _dvarint(data, 1); self.state["stacking_progress"] = v
            except: pass

        # Live tracking box (the morphing ROI the firmware reports)
        elif cmd in (CMD_NOTIFY_TRACK_RESULT, CMD_NOTIFY_WIDE_TRACK_RESULT,
                     CMD_NOTIFY_SENTRY_MODE_TRACK_RESULT):
            try:
                f = _parse_varint_fields(data)
                box = (f.get(1, -100), f.get(2, -100), f.get(3, 0), f.get(4, 0))
                self.state["track_box"] = box
                self.state["track_box_ts"] = time.time()
                self.state["track_box_src"] = (
                    "wide" if cmd == CMD_NOTIFY_WIDE_TRACK_RESULT else "tele")
                print(f"[TRACKBOX] cmd={cmd} src={self.state['track_box_src']} "
                      f"box={box} raw_fields={f}", flush=True)
            except: pass

        # Built-in mount gyro attitude (GHIDRA-derived; see start_attitude_notify())
        elif cmd == CMD_NOTIFY_DEVICE_ATTITUDE:
            try:
                f = _parse_double_fields(data)
                self.state["device_attitude"] = (f.get(1), f.get(2), f.get(3))
                self.state["device_attitude_ts"] = time.time()
            except: pass

        # Wide track state (CAPTURE-VERIFIED; absent from the APK table)
        elif cmd == CMD_NOTIFY_WIDE_TRACK_STATE:
            try:
                f = _parse_varint_fields(data)
                self.state["track_state"] = (f.get(1, 0), f.get(2, 0))
            except: pass

        # Multi-object detection results (MOT) — detected boxes + ids
        elif cmd in (CMD_NOTIFY_MULTI_TRACK_RESULT, CMD_NOTIFY_WIDE_MULTI_TRACK_RESULT):
            try:
                self.state["multi_boxes"] = _parse_multi_track(data)
                self.state["multi_boxes_ts"] = time.time()
                self.state["multi_boxes_src"] = (
                    "wide" if cmd == CMD_NOTIFY_WIDE_MULTI_TRACK_RESULT else "tele")
            except: pass

        if self.on_notify: self.on_notify(pkt)

    def _on_error(self, ws, e):
        log.error(f"WS error: {e}")
        self.state["connected"] = False

    def _on_close(self, ws, c, r):
        self._connected.clear()
        self.state["connected"] = False

    def send(self, cmd, data=b""):
        if not self._connected.is_set(): return False
        self._ws.send(
            build_ws_packet(cmd, data, self.device_id, self.client_id),
            opcode=websocket.ABNF.OPCODE_BINARY,
        )
        self.state["last_cmd"] = cmd
        return True

    # ── Camera initialisation sequence ────────────────────────────────────────
    def enter_camera(self, encode_type=1):
        """
        CMD_GLOBAL_TASK_MANAGER_ENTER_CAMERA (16404).
        MUST be sent before open_camera() on Dwarf3 firmware v1.8+.
        encode_type 1 = H.265/HEVC (activates RTSP encoder).
        Payload: ReqEnterCamera { client_param { encode_type: 1 } }
        """
        client_params = _field(1, 0, encode_type)
        data = _field(1, 2, client_params)
        self.send(CMD_GLOBAL_TASK_MANAGER_ENTER_CAMERA, data)

    def open_camera(self, binning=False, rtsp_encode_type=1):
        """
        CMD_CAMERA_TELE_OPEN_CAMERA (10000). Must be called after enter_camera().
        New APK uses CameraType enum internally but the wire payload is unchanged:
        ReqOpenCamera { binning: bool, rtspEncodeType: 1 }
        rtsp_encode_type=1 (H.265) is required to activate the RTSP stream.
        """
        data = _field(1, 0, 1 if binning else 0) + _field(2, 0, rtsp_encode_type)
        self.send(CMD_CAMERA_TELE_OPEN_CAMERA, data)

    def open_camera_wide(self, binning=False, rtsp_encode_type=1):
        """CMD_CAMERA_WIDE_OPEN_CAMERA (12000) — same payload as open_camera()."""
        data = _field(1, 0, 1 if binning else 0) + _field(2, 0, rtsp_encode_type)
        self.send(CMD_CAMERA_WIDE_OPEN_CAMERA, data)

    # ── Camera controls ───────────────────────────────────────────────────────
    def close_camera(self):          self.send(CMD_CAMERA_TELE_CLOSE_CAMERA)
    def take_photo(self):            self.send(CMD_CAMERA_TELE_PHOTOGRAPH)
    def take_photo_raw(self):        self.send(CMD_CAMERA_TELE_PHOTO_RAW)
    def start_burst(self, n=3):      self.send(CMD_CAMERA_TELE_BURST, p_int(n))
    def start_record(self):          self.send(CMD_CAMERA_TELE_START_RECORD)
    def stop_record(self):           self.send(CMD_CAMERA_TELE_STOP_RECORD)
    def start_record_wide(self):     self.send(CMD_CAMERA_WIDE_START_RECORD)
    def stop_record_wide(self):      self.send(CMD_CAMERA_WIDE_STOP_RECORD)
    def start_timelapse(self):       self.send(CMD_CAMERA_TELE_START_TIMELAPSE_PHOTO)
    def stop_timelapse(self):        self.send(CMD_CAMERA_TELE_STOP_TIMELAPSE_PHOTO)
    def set_exposure(self, i):       self.send(CMD_CAMERA_TELE_SET_EXP, p_int(i))
    def set_gain(self, i):           self.send(CMD_CAMERA_TELE_SET_GAIN, p_int(i))
    def set_brightness(self, v):     self.send(CMD_CAMERA_TELE_SET_BRIGHTNESS, p_int(v))
    def set_contrast(self, v):       self.send(CMD_CAMERA_TELE_SET_CONTRAST, p_int(v))
    def set_saturation(self, v):     self.send(CMD_CAMERA_TELE_SET_SATURATION, p_int(v))
    def set_sharpness(self, v):      self.send(CMD_CAMERA_TELE_SET_SHARPNESS, p_int(v))
    def set_wb_mode(self, m):        self.send(CMD_CAMERA_TELE_SET_WB_MODE, p_int(m))
    def set_wb_ct(self, i):          self.send(CMD_CAMERA_TELE_SET_WB_CT, p_int(i))
    def set_ircut(self, v):          self.send(CMD_CAMERA_TELE_SET_IRCUT, p_int(v))
    def set_jpg_quality(self, q):    self.send(CMD_CAMERA_TELE_SET_JPG_QUALITY, p_int(q))
    def switch_resolution(self, i):  self.send(CMD_CAMERA_TELE_SWITCH_RESOLUTION, p_int(i))
    def switch_framerate(self, i):   self.send(CMD_CAMERA_TELE_SWITCH_FRAMERATE, p_int(i))
    def get_all_params(self):        self.send(CMD_CAMERA_TELE_GET_ALL_PARAMS)

    # ── Astro controls ────────────────────────────────────────────────────────
    def start_calibration(self, lat=0.0, lon=0.0):
        d = _field(1, 1, float(lon)) + _field(2, 1, float(lat))
        self.send(CMD_ASTRO_START_CALIBRATION, d)
    def stop_calibration(self):      self.send(CMD_ASTRO_STOP_CALIBRATION)

    def goto_dso(self, ra, dec, name="", goto_only=False):
        """
        GoTo deep sky object by RA (hours) / Dec (degrees).
        goto_only=True: slew only, no stacking (new field in June 2026 APK).
        """
        self.send(CMD_ASTRO_START_GOTO_DSO, p_goto_dso(ra, dec, name, goto_only))

    def goto_solar(self, i):         self.send(CMD_ASTRO_START_GOTO_SOLAR_SYSTEM, p_int(i))
    def stop_goto(self):             self.send(CMD_ASTRO_STOP_GOTO)
    def one_click_goto_dso(self, ra, dec, name=""):
        self.send(CMD_ASTRO_START_ONE_CLICK_GOTO_DSO, p_goto_dso(ra, dec, name))
    def stop_one_click_goto(self):   self.send(CMD_ASTRO_STOP_ONE_CLICK_GOTO)
    def go_live(self):               self.send(CMD_ASTRO_GO_LIVE)
    def go_live_wide(self):          self.send(CMD_ASTRO_WIDE_GO_LIVE)

    def start_stacking(self, exp_ms=10000, gain=0, count=0):
        d = b""
        if exp_ms: d += _field(1, 0, exp_ms)
        if gain:   d += _field(2, 0, gain)
        if count:  d += _field(3, 0, count)
        self.send(CMD_ASTRO_START_CAPTURE_RAW_LIVE_STACKING, d)
    def stop_stacking(self):         self.send(CMD_ASTRO_STOP_CAPTURE_RAW_LIVE_STACKING)
    def start_plate_solve(self):     self.send(CMD_ASTRO_START_EQ_SOLVING)
    def stop_plate_solve(self):      self.send(CMD_ASTRO_STOP_EQ_SOLVING)
    def start_sky_finder(self):      self.send(CMD_ASTRO_START_SKY_TARGET_FINDER)
    def stop_sky_finder(self):       self.send(CMD_ASTRO_STOP_SKY_TARGET_FINDER)
    def start_ai_enhance(self):      self.send(CMD_ASTRO_START_AI_ENHANCE)
    def stop_ai_enhance(self):       self.send(CMD_ASTRO_STOP_AI_ENHANCE)
    def one_click_shoot(self):       self.send(CMD_ASTRO_START_ONE_CLICK_SHOOTING)

    # ── Focus controls ────────────────────────────────────────────────────────
    def auto_focus(self, center_x=None, center_y=None):
        """
        CMD_FOCUS_AUTO_FOCUS (15000) — normal-mode autofocus.
        ReqNormalAutoFocus { uint32 mode=1; uint32 center_x=2; uint32 center_y=3 }
        mode 0 = global focus, mode 1 = area focus around (center_x, center_y) in
        the active camera's preview pixel coordinates. Pass a centre to focus on a
        specific region of the currently selected feed; omit it for global focus.
        NOTE: the DWARF 3 has a single motorised focuser on the TELEPHOTO optics;
        the wide-angle lens is fixed-focus, so focus only physically moves the tele.
        """
        if center_x is None or center_y is None:
            return self.send(CMD_FOCUS_AUTO_FOCUS, _field(1, 0, 0))
        data = (_field(1, 0, 1) +
                _field(2, 0, int(center_x)) +
                _field(3, 0, int(center_y)))
        return self.send(CMD_FOCUS_AUTO_FOCUS, data)
    def focus_step(self, s=1):       self.send(CMD_FOCUS_MANUAL_SINGLE_STEP, p_int(s))
    def focus_in(self):              self.send(CMD_FOCUS_START_MANUAL_CONTINUOUS, p_int(-1))
    def focus_out(self):             self.send(CMD_FOCUS_START_MANUAL_CONTINUOUS, p_int(1))
    def focus_stop(self):            self.send(CMD_FOCUS_STOP_MANUAL_CONTINUOUS)
    def astro_focus(self):           self.send(CMD_FOCUS_START_ASTRO_AUTO_FOCUS)
    def stop_astro_focus(self):      self.send(CMD_FOCUS_STOP_ASTRO_AUTO_FOCUS)

    # ── Motor / joystick controls ─────────────────────────────────────────────
    def joystick(self, x, y):
        """
        Move motors. x/y in range -100..100.
        Positive x = East, positive y = North.
        Sends ReqMotorServiceJoystick with polar vector_angle + vector_length doubles.
        """
        angle, length = xy_to_polar(x, y)
        self.send(CMD_STEP_MOTOR_JOYSTICK, p_joystick(angle, length))

    def joystick_stop(self):         self.send(CMD_STEP_MOTOR_JOYSTICK_STOP)

    # ── Tracking ──────────────────────────────────────────────────────────────
    def start_tracking(self):        self.send(CMD_TRACK_START_TRACK)
    def stop_tracking(self):         self.send(CMD_TRACK_STOP_TRACK)

    # ── Built-in mount gyro (GHIDRA-derived; not in the app's own WsCmd list) ──
    def start_attitude_notify(self):
        """CMD_MOT_START_DEVICE_ATTITUDE_NOTIFY (14012), empty payload. Spawns a
        background thread in the firmware (traced via Ghidra: magni@0x7367c0)
        that streams CMD_NOTIFY_DEVICE_ATTITUDE (15295) {pitch,yaw,roll} from
        the mount's built-in gyro until stop_attitude_notify() is called."""
        return self.send(CMD_MOT_START_DEVICE_ATTITUDE_NOTIFY)

    def stop_attitude_notify(self):
        """CMD_MOT_STOP_DEVICE_ATTITUDE_NOTIFY (14013), empty payload."""
        return self.send(CMD_MOT_STOP_DEVICE_ATTITUDE_NOTIFY)

    # ── V3 camera + MOT (Multi-Object Tracking) pipeline ───────────────────────
    # The DWARF 3 (V3 firmware) AI subject tracking runs through the 30-class
    # object detector + MOT, NOT the basic 14800 correlation tracker. Typical
    # sequence (each step waits for the device to settle):
    #   v3_open_wide() -> wide_tele_track_switch(1) -> start_mot()
    #   ... device streams detected boxes+ids on 15251/15238 ...
    #   mot_wide_track_one(id)  -> lock the chosen object
    # WARNING: in live probing, sending V3 open-camera commands out of the app's
    # exact order destabilised the device (it emitted POWER_OFF 15229 and dropped
    # the link). Drive these deliberately, one at a time, and watch the hardware.
    def v3_open_tele(self, action=1):   # 1 = open, 0 = close
        return self.send(CMD_V3_CAMERA_TELE_OPEN_CAMERA, _field(1, 0, int(action)))

    def v3_open_wide(self, close=False):
        # CAPTURE-VERIFIED: app OPENS wide with an EMPTY payload (no action field);
        # sending {1:1} = CLOSE and destabilised the device. Only send a field to
        # close.
        data = _field(1, 0, 1) if close else b""
        return self.send(CMD_V3_CAMERA_WIDE_OPEN_CAMERA, data)

    def wide_tele_track_switch(self, camera=1):  # 0 = tele, 1 = wide; enables detector
        return self.send(CMD_WIDE_TELE_TRACK_SWITCH, _field(1, 0, int(camera)))

    def start_mot(self):
        """CMD_MOT_START (14804) — start multi-object detection/tracking."""
        return self.send(CMD_MOT_START)

    def mot_wide_track_one(self, obj_id):
        """CMD_MOT_WIDE_TRACK_ONE (14808) — lock a detected wide object by id."""
        return self.send(CMD_MOT_WIDE_TRACK_ONE, _field(1, 0, int(obj_id)))

    def mot_tele_track_one(self, obj_id):
        """CMD_MOT_TRACK_ONE (14805) — lock a detected tele object by id."""
        return self.send(CMD_MOT_TRACK_ONE, _field(1, 0, int(obj_id)))

    def start_track_roi(self, x, y, w, h, field5):
        """
        CMD_TRACK_START_TRACK (14800) with a manual ROI.
        ReqStartTrack { int32 x=1; int32 y=2; int32 w=3; int32 h=4; int32 camId=5 }
        Coordinates are in *wide-stream pixel* space (≈1920x1080, NOT 1280x720).

        field5 IS TrackProto.ReqStartTrack.camId (0=Tele, 1=Wide, 15=General —
        confirmed from the DwarfLab APK's decompiled protobuf source, see
        CameraType.java / TrackProto.java). It is proto3, so camId=0 (Tele) is
        never written to the wire — only Wide's camId=1 ever appears as a real
        byte. There is no default: every caller must pass the camId matching
        the camera actually being tracked, or the firmware calibrates the lock
        against the wrong camera's FOV (this was the root cause of the
        long-standing tele slew overshoot — every track command was silently
        hardcoded to camId=1/Wide regardless of which camera was selected).
        """
        x, y, w, h = int(x), int(y), int(w), int(h)
        data = (_field(1, 0, x) + _field(2, 0, y) +
                _field(3, 0, w) + _field(4, 0, h))
        if field5 is not None:
            data += _field(5, 0, int(field5))
        return self.send(CMD_TRACK_START_TRACK, data)

    def v3_mode_switch(self, value=1):
        """CMD_V3_DEVICE_CONFIG_MODE_SWITCH (16404). CAPTURE-VERIFIED payload:
        V3ReqModeSwitch { inner(3) { value(1) = 1 } } — i.e. {3:{1:1}}. Puts the
        V3 firmware into the camera/tracking mode before opening cameras."""
        return self.send(CMD_V3_DEVICE_CONFIG_MODE_SWITCH,
                         _field(3, 2, _field(1, 0, int(value))))

    def track_box_normalized(self):
        """Return the current tracked box as (nx, ny, nw, nh) in [0,1], using the
        CAPTURE-VERIFIED fixed reference frame (TRACK_REF_W x TRACK_REF_H) rather
        than the live decoded video size. Returns None when there is no fresh
        lock. Multiply by your display rect to overlay correctly regardless of the
        RTSP stream resolution."""
        box = self.state.get("track_box")
        if (not box or box[0] <= TRACK_NO_TARGET or box[1] <= TRACK_NO_TARGET):
            return None
        x, y, w, h = box
        return (x / TRACK_REF_W, y / TRACK_REF_H, w / TRACK_REF_W, h / TRACK_REF_H)

    # ── App-level keepalives (CAPTURE-VERIFIED) ────────────────────────────────
    # The iOS app keeps the session alive with TWO heartbeats the controller did
    # not previously send: a WebSocket TEXT "ping" frame, and a UDP :9900 protobuf
    # {1:1, 2:<unix_ms>, 3:"txtl"}. websocket-client's protocol-level PING (used
    # in _start_ws) is usually sufficient, but if the firmware drops the tracker
    # or host-lock when only protocol pings are seen, enable these to mimic the
    # app exactly. Opt-in so existing behaviour is unchanged.
    def start_app_keepalive(self, ws_ping=True, udp_txtl=True, interval=1.0):
        def _loop():
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if udp_txtl else None
            while self._connected.is_set():
                if ws_ping and self._ws:
                    try: self._ws.send("ping")           # text frame (opcode 1)
                    except Exception: pass
                if udp and udp_txtl:
                    pkt = (_field(1, 0, 1) +
                           _field(2, 0, int(time.time() * 1000)) +
                           _field(3, 2, "txtl"))
                    try: udp.sendto(pkt, (self.host, self.WS_PORT))
                    except Exception: pass
                time.sleep(interval)
            if udp: udp.close()
        threading.Thread(target=_loop, daemon=True, name="dwarf-keepalive").start()

    def set_master_lock(self, lock=True):
        """
        CMD_SYSTEM_SET_MASTERLOCK (13004) — acquire/release the host (master)
        lock. The DWARF 3 firmware silently ignores control commands from a
        client that does not hold this lock, so this must be acquired before
        motion/tracking commands take effect. Device replies on 15223
        (ResNotifyHostSlaveMode); mode=0 + lock=true means HOST acquired.
        """
        return self.send(CMD_SYSTEM_SET_MASTERLOCK, _field(1, 0, 1 if lock else 0))

    # ── Sentinel / UFO auto-track modes ───────────────────────────────────────
    # In these modes the DWARF firmware AUTO-detects moving objects and drives
    # its OWN motors to follow them (no software loop needed). UFO mode (v1.5+)
    # reuses the Sentinel message ReqStartSentryMode{ int32 mode = 1 }. The
    # `mode` int selects the detection profile (bird / airplane / generic UFO).
    def start_sentry_mode(self, mode=0):
        """CMD_SENTRY_MODE_START (14802) — Sentinel auto-detect & track."""
        return self.send(CMD_SENTRY_MODE_START, _field(1, 0, int(mode)))

    def stop_sentry_mode(self):
        """CMD_SENTRY_MODE_STOP (14803)."""
        return self.send(CMD_SENTRY_MODE_STOP)

    def start_ufo_mode(self, mode=1):
        """CMD_UFOTRACK_MODE_START (14806) — UFO/aircraft auto-track."""
        return self.send(CMD_UFOTRACK_MODE_START, _field(1, 0, int(mode)))

    def stop_ufo_mode(self):
        """CMD_UFOTRACK_MODE_STOP (14807)."""
        return self.send(CMD_UFOTRACK_MODE_STOP)

    def set_ufo_hand_auto(self, auto=True):
        """CMD_UFO_HAND_AOTO_MODE (14810) — UFO manual(0) / automatic(1)."""
        return self.send(CMD_UFO_HAND_AOTO_MODE, _field(1, 0, 1 if auto else 0))

    # ── New device commands (June 2026 APK) ───────────────────────────────────
    def set_auto_cooling(self, enabled=True):
        """CMD_DEVICE_AUTO_COOLING (17001) — toggle auto cooling fan."""
        self.send(CMD_DEVICE_AUTO_COOLING, p_int(1 if enabled else 0))

    def set_auto_shutdown(self, enabled=True):
        """CMD_DEVICE_AUTO_SHUTDOWN (17002) — toggle auto shutdown."""
        self.send(CMD_DEVICE_AUTO_SHUTDOWN, p_int(1 if enabled else 0))

    def set_lens_defog(self, enabled=True):
        """CMD_DEVICE_LENS_DEFOG (17000) — toggle the lens heater/defog element."""
        self.send(CMD_DEVICE_LENS_DEFOG, p_int(1 if enabled else 0))

    # ── Panorama (15500-15599, added 2026-07-31) ──────────────────────────────
    def panorama_start_grid(self):
        """CMD_PANORAMA_START_GRID (15500) — start a full-grid panorama sweep."""
        self.send(CMD_PANORAMA_START_GRID)

    def panorama_stop(self):
        """CMD_PANORAMA_STOP (15501)."""
        self.send(CMD_PANORAMA_STOP)

    def panorama_start_stitch_upload(self, resource_id, user_id, ak, sk, token,
                                      bucket, bucket_prefix, panorama_name="",
                                      app_platform=0, from_="", env_type=""):
        """CMD_PANORAMA_START_STITCH_UPLOAD (15503) — hand the device short-lived
        cloud (S3/COS) credentials so it can upload the stitched panorama itself.
        Field layout schema-exact from panorama.proto; ak/sk/token are the same
        kind of short-lived STS credential noted in dwarf3-emmc-root-access
        memory ("Cloud/AWS/COS credentials ... short-lived STS tokens fetched at
        runtime") — do not hardcode long-lived keys here."""
        data = (_field(1, 0, resource_id) + _field(2, 2, user_id) +
                _field(3, 0, app_platform) + _field(4, 2, panorama_name) +
                _field(5, 2, ak) + _field(6, 2, sk) + _field(7, 2, token) +
                _field(8, 2, bucket) + _field(9, 2, bucket_prefix) +
                _field(10, 2, from_) + _field(11, 2, env_type))
        self.send(CMD_PANORAMA_START_STITCH_UPLOAD, data)

    def panorama_stop_stitch_upload(self, user_id=""):
        """CMD_PANORAMA_STOP_STITCH_UPLOAD (15504)."""
        self.send(CMD_PANORAMA_STOP_STITCH_UPLOAD, _field(1, 2, user_id))

    def panorama_compress(self, panorama_name):
        """CMD_PANORAMA_START_COMPRESS (15507)."""
        self.send(CMD_PANORAMA_START_COMPRESS, _field(1, 2, panorama_name))

    def panorama_stop_compress(self):
        """CMD_PANORAMA_STOP_COMPRESS (15508)."""
        self.send(CMD_PANORAMA_STOP_COMPRESS)

    def panorama_start_framing(self):
        """CMD_PANORAMA_START_FRAMING (15509) — interactive framing-rect mode."""
        self.send(CMD_PANORAMA_START_FRAMING)

    def panorama_stop_framing(self):
        """CMD_PANORAMA_STOP_FRAMING (15510)."""
        self.send(CMD_PANORAMA_STOP_FRAMING)

    def panorama_reset_framing(self):
        """CMD_PANORAMA_RESET_FRAMING (15511)."""
        self.send(CMD_PANORAMA_RESET_FRAMING)

    def panorama_update_framing_rect(self, x_tl, y_tl, x_br, y_br):
        """CMD_PANORAMA_UPDATE_FRAMING_RECT (15512) — normalized [0,1] rect corners."""
        data = (_field(1, 1, x_tl) + _field(2, 1, y_tl) +
                _field(3, 1, x_br) + _field(4, 1, y_br))
        self.send(CMD_PANORAMA_UPDATE_FRAMING_RECT, data)

    def panorama_stop_framing_and_start_grid(self):
        """CMD_PANORAMA_STOP_FRAMING_AND_START_GRID (15513)."""
        self.send(CMD_PANORAMA_STOP_FRAMING_AND_START_GRID)

    # ── Shooting Schedule (16100-16108, added 2026-07-31) ─────────────────────
    # NOTE: sync/replace take a full ShootingScheduleMsg (18 fields, see
    # dwarf_protobuf.py NESTED_SCHEMAS["shooting_schedule_msg"]) — building one by
    # hand from these low-level primitives is exposed here (_shooting_schedule_msg)
    # rather than wrapped in a single do-everything method, since the real app
    # likely fills most fields from local schedule-editor UI state this repo has
    # no equivalent of yet.
    def _shooting_schedule_msg(self, schedule_id="", schedule_name="", device_id=0,
                                mac_address="", start_time=0, end_time=0,
                                password="", schedule_time=0):
        """Build a (partial) ShootingScheduleMsg — only the fields a caller is
        likely to set from a scripted client; state/result/sync_state/tasks are
        left at their proto3 zero-value defaults (server-managed fields)."""
        return (_field(1, 2, schedule_id) + _field(2, 2, schedule_name) +
                _field(3, 0, device_id) + _field(4, 2, mac_address) +
                _field(5, 0, int(start_time)) + _field(6, 0, int(end_time)) +
                _field(12, 2, password) + _field(17, 0, int(schedule_time)))

    def shooting_schedule_sync(self, **kw):
        """CMD_SHOOTING_SCHEDULE_SYNC (16100). kwargs -> _shooting_schedule_msg()."""
        self.send(CMD_SHOOTING_SCHEDULE_SYNC,
                  _field(1, 2, self._shooting_schedule_msg(**kw)))

    def shooting_schedule_cancel(self, schedule_id, password=""):
        """CMD_SHOOTING_SCHEDULE_CANCEL (16101)."""
        self.send(CMD_SHOOTING_SCHEDULE_CANCEL,
                  _field(1, 2, schedule_id) + _field(2, 2, password))

    def shooting_schedule_get_all(self):
        """CMD_SHOOTING_SCHEDULE_GET_ALL (16102, UNCONFIRMED id — see CMD constant)."""
        self.send(CMD_SHOOTING_SCHEDULE_GET_ALL)

    def shooting_schedule_get_by_id(self, schedule_id):
        """CMD_SHOOTING_SCHEDULE_GET_BY_ID (16103, UNCONFIRMED id)."""
        self.send(CMD_SHOOTING_SCHEDULE_GET_BY_ID, _field(1, 2, schedule_id))

    def shooting_schedule_get_task_by_id(self, task_id):
        """CMD_SHOOTING_SCHEDULE_GET_TASK_ID (16104) — zlog-confirmed."""
        self.send(CMD_SHOOTING_SCHEDULE_GET_TASK_ID, _field(1, 2, task_id))

    def shooting_schedule_replace(self, **kw):
        """CMD_SHOOTING_SCHEDULE_REPLACE (16105). kwargs -> _shooting_schedule_msg()."""
        self.send(CMD_SHOOTING_SCHEDULE_REPLACE,
                  _field(1, 2, self._shooting_schedule_msg(**kw)))

    def shooting_schedule_unlock(self, schedule_id, password):
        """CMD_SHOOTING_SCHEDULE_UNLOCK (16106, UNCONFIRMED id)."""
        self.send(CMD_SHOOTING_SCHEDULE_UNLOCK,
                  _field(1, 2, schedule_id) + _field(2, 2, password))

    def shooting_schedule_lock(self, schedule_id, password):
        """CMD_SHOOTING_SCHEDULE_LOCK (16107, UNCONFIRMED id)."""
        self.send(CMD_SHOOTING_SCHEDULE_LOCK,
                  _field(1, 2, schedule_id) + _field(2, 2, password))

    def shooting_schedule_delete(self, schedule_id, password=""):
        """CMD_SHOOTING_SCHEDULE_DELETE (16108)."""
        self.send(CMD_SHOOTING_SCHEDULE_DELETE,
                  _field(1, 2, schedule_id) + _field(2, 2, password))

    # ── Param (16700-16706, added 2026-07-31) ─────────────────────────────────
    # param_id is the 64-bit id from GET http://<host>:8082/getDefaultParamsConfig
    # (bit 44 set = wide camera, clear = tele — see dwarf3-tracking-protocol
    # memory). Callers should fetch that table at runtime, not hardcode ids.
    def param_set_exposure(self, param_id, value, mode=0):
        """CMD_PARAM_SET_EXPOSURE (16700). `mode`: manual(0)/auto — unconfirmed
        exact enum, mirrors ReqSetExposure{param_id,mode,value}."""
        data = _field(1, 0, param_id) + _field(2, 0, mode) + _field(3, 0, value)
        self.send(CMD_PARAM_SET_EXPOSURE, data)

    def param_set_gain(self, param_id, value, mode=0):
        """CMD_PARAM_SET_GAIN (16701)."""
        data = _field(1, 0, param_id) + _field(2, 0, mode) + _field(3, 0, value)
        self.send(CMD_PARAM_SET_GAIN, data)

    def param_set_wb(self, param_id, value, mode=0):
        """CMD_PARAM_SET_WB (16702) — white balance."""
        data = _field(1, 0, param_id) + _field(2, 0, mode) + _field(3, 0, value)
        self.send(CMD_PARAM_SET_WB, data)

    def param_set_int(self, param_id, value):
        """CMD_PARAM_SET_GENERAL_INT_PARAM (16703) — generic int-valued param."""
        self.send(CMD_PARAM_SET_GENERAL_INT_PARAM,
                  _field(1, 0, param_id) + _field(2, 0, value))

    def param_set_float(self, param_id, value):
        """CMD_PARAM_SET_GENERAL_FLOAT_PARAM (16704) — generic float-valued param."""
        self.send(CMD_PARAM_SET_GENERAL_FLOAT_PARAM,
                  _field(1, 0, param_id) + _field(2, 5, value))

    def param_set_bool(self, param_id, value):
        """CMD_PARAM_SET_GENERAL_BOOL_PARAM (16705) — generic bool-valued param."""
        self.send(CMD_PARAM_SET_GENERAL_BOOL_PARAM,
                  _field(1, 0, param_id) + _field(2, 0, 1 if value else 0))

    def param_set_auto(self, camera_type, shooting_tech, is_auto=True):
        """CMD_PARAM_SET_AUTO_PARAMS (16706) — {camera_type, shooting_tech, is_auto}."""
        data = (_field(1, 0, camera_type) + _field(2, 0, shooting_tech) +
                _field(3, 0, 1 if is_auto else 0))
        self.send(CMD_PARAM_SET_AUTO_PARAMS, data)

    # ── Voice Assistant (single dispatcher 16800, added 2026-07-31) ───────────
    # Firmware runs every voice action through this ONE command id with an
    # internal switch on `command_type` (see dwarf3-tracking-protocol memory,
    # "Voice Assistant module clarified"). Exposed as one low-level builder plus
    # thin convenience wrappers for the parameter-free actions; the
    # parameterized ones (move/goto/focus/track/calibration/panorama) take a
    # pre-built nested-message blob — build with `_field`/`_field`-of-`_field`,
    # matching dwarf_protobuf.py's NESTED_SCHEMAS shapes for each param message.
    VOICE_CMD_GET_STATUS = 1; VOICE_CMD_TAKE_PHOTO = 2
    VOICE_CMD_START_RECORD = 3; VOICE_CMD_STOP_RECORD = 4
    VOICE_CMD_START_TIMELAPSE = 5; VOICE_CMD_STOP_TIMELAPSE = 6
    VOICE_CMD_START_BURST = 7; VOICE_CMD_STOP_BURST = 8
    VOICE_CMD_START_ASTRO = 9; VOICE_CMD_STOP_ASTRO = 10
    VOICE_CMD_START_SENTRY = 11; VOICE_CMD_STOP_SENTRY = 12
    VOICE_CMD_MOVE = 13; VOICE_CMD_GOTO_TARGET = 14; VOICE_CMD_STOP_GOTO = 15
    VOICE_CMD_CALIBRATION = 16; VOICE_CMD_STOP_CALIBRATION = 17
    VOICE_CMD_AUTO_FOCUS = 18; VOICE_CMD_STOP_FOCUS = 19; VOICE_CMD_STOP_ALL = 20
    VOICE_CMD_START_TRACK = 21; VOICE_CMD_STOP_TRACK = 22
    VOICE_CMD_ADD_SCHEDULE = 23; VOICE_CMD_CANCEL_SCHEDULE = 24
    VOICE_CMD_END_CONVERSATION = 25
    VOICE_CMD_START_PANORAMA = 26; VOICE_CMD_STOP_PANORAMA = 27

    def voice_command(self, command_type, shooting_mode=None, param_field=None,
                       param_bytes=b""):
        """Low-level ReqVoiceCommand builder. `param_field`/`param_bytes`: the
        proto field number (10-23, see voice_assistant.proto) + pre-encoded
        nested-message bytes for command types that need one (MOVE=16,
        GOTO_TARGET=17, CALIBRATION=18, AUTO_FOCUS=19, START_TRACK=20,
        START_PANORAMA=23 — matching dwarf_protobuf.py's NESTED_SCHEMAS)."""
        data = _field(1, 0, command_type)
        if shooting_mode is not None:
            data += _field(2, 0, shooting_mode)
        if param_field is not None and param_bytes:
            data += _field(param_field, 2, param_bytes)
        self.send(CMD_VOICE_ASSISTANT_TASK, data)

    def voice_get_status(self):        self.voice_command(self.VOICE_CMD_GET_STATUS)
    def voice_take_photo(self):        self.voice_command(self.VOICE_CMD_TAKE_PHOTO)
    def voice_start_record(self):      self.voice_command(self.VOICE_CMD_START_RECORD)
    def voice_stop_record(self):       self.voice_command(self.VOICE_CMD_STOP_RECORD)
    def voice_start_astro(self):       self.voice_command(self.VOICE_CMD_START_ASTRO)
    def voice_stop_astro(self):        self.voice_command(self.VOICE_CMD_STOP_ASTRO)
    def voice_start_sentry(self):      self.voice_command(self.VOICE_CMD_START_SENTRY)
    def voice_stop_sentry(self):       self.voice_command(self.VOICE_CMD_STOP_SENTRY)
    def voice_stop_goto(self):         self.voice_command(self.VOICE_CMD_STOP_GOTO)
    def voice_stop_calibration(self):  self.voice_command(self.VOICE_CMD_STOP_CALIBRATION)
    def voice_auto_focus(self):        self.voice_command(self.VOICE_CMD_AUTO_FOCUS)
    def voice_stop_focus(self):        self.voice_command(self.VOICE_CMD_STOP_FOCUS)
    def voice_stop_all(self):          self.voice_command(self.VOICE_CMD_STOP_ALL)
    def voice_start_track(self):       self.voice_command(self.VOICE_CMD_START_TRACK)
    def voice_stop_track(self):        self.voice_command(self.VOICE_CMD_STOP_TRACK)
    def voice_end_conversation(self):  self.voice_command(self.VOICE_CMD_END_CONVERSATION)
    def voice_stop_panorama(self):     self.voice_command(self.VOICE_CMD_STOP_PANORAMA)

    def voice_move(self, azimuth_deg, altitude_deg, speed=1):
        """VOICE_CMD_MOVE (13) with a nested VoiceMoveParams at field 16."""
        p = _field(1, 1, azimuth_deg) + _field(2, 1, altitude_deg) + _field(3, 0, speed)
        self.voice_command(self.VOICE_CMD_MOVE, param_field=16, param_bytes=p)

    def voice_calibration(self, lon, lat):
        """VOICE_CMD_CALIBRATION (16) with a nested VoiceCalibrationParams at field 18."""
        p = _field(1, 1, lon) + _field(2, 1, lat)
        self.voice_command(self.VOICE_CMD_CALIBRATION, param_field=18, param_bytes=p)

    def voice_start_panorama(self, rows=1, columns=1):
        """VOICE_CMD_START_PANORAMA (26) with a nested VoicePanoramaParams at field 23."""
        p = _field(1, 0, rows) + _field(2, 0, columns)
        self.voice_command(self.VOICE_CMD_START_PANORAMA, param_field=23, param_bytes=p)

    # ── System controls ───────────────────────────────────────────────────────
    def set_location(self, lat, lon, alt=0):
        self.send(CMD_SYSTEM_SET_LOCATION, p_location(lat, lon, alt))
    def sync_time(self):
        self.send(CMD_SYSTEM_SET_TIME, p_int(int(time.time() * 1000)))
    def reboot(self):                self.send(CMD_RGB_POWER_REBOOT)
    def power_down(self):            self.send(CMD_RGB_POWER_POWER_DOWN)
    def led_on(self):                self.send(CMD_RGB_POWER_POWERIND_ON)
    def led_off(self):               self.send(CMD_RGB_POWER_POWERIND_OFF)
    def get_device_state(self):      self.send(CMD_GLOBAL_TASK_GET_DEVICE_STATE_INFO)

    # ── HTTP REST API ─────────────────────────────────────────────────────────
    def http_device_info(self):
        try:
            r = requests.post(
                f"http://{self.host}:{self.HTTP_PORT}/deviceInfo",
                json={}, timeout=5
            )
            return r.json()
        except Exception as e:
            return {"error": str(e)}

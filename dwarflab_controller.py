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
CMD_STEP_MOTOR_JOYSTICK                  = 14006  # CMD_STEP_MOTOR_SERVICE_JOYSTICK
CMD_STEP_MOTOR_JOYSTICK_FIXED_ANGLE      = 14007  # CMD_STEP_MOTOR_SERVICE_JOYSTICK_FIXED_ANGLE
CMD_STEP_MOTOR_JOYSTICK_STOP             = 14008  # CMD_STEP_MOTOR_SERVICE_JOYSTICK_STOP

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
CMD_DEVICE_AUTO_COOLING                  = 17001  # NEW in June 2026 APK
CMD_DEVICE_AUTO_SHUTDOWN                 = 17002  # NEW in June 2026 APK

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

    def start_track_roi(self, x, y, w, h, field5=1):
        """
        CMD_TRACK_START_TRACK (14800) with a manual ROI.
        ReqStartTrack { int32 x=1; int32 y=2; int32 w=3; int32 h=4; int32 f5=5 }
        Coordinates are in *wide-stream pixel* space (≈1920x1080, NOT 1280x720).

        CAPTURE-VERIFIED: the iOS app sends a FIFTH field (=1) that the public
        proto omits, AND only locks reliably after the V3 camera bring-up
        (v3_mode_switch + v3_open_tele/wide). Sending the 4-field form alone runs
        the basic correlation tracker, which usually fails to lock (returns -100).
        A real app lock observed here was 2592/2592 valid boxes with f5=1.
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

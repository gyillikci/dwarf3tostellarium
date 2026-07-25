"""
roi_gui.py — Live video + drag-to-select ROI tracking UI for the DWARF 3.

Shows the telescope's live RTSP feed in a window. Drag a rectangle with the
mouse over the video to define a Region Of Interest (ROI); the selected box is
sent to the device as a manual-track command (CMD_TRACK_START_TRACK / 14800) in
native frame-pixel coordinates.

Run:
    .\\.venv\\Scripts\\python.exe roi_gui.py --ip 192.168.1.102

Dependencies (install into this venv):
    pip install opencv-python numpy Pillow

(Uses Tkinter, which ships with Python, for the window — no Qt needed.)

Notes
-----
* The DWARF 3 firmware ignores control commands from a client that does not hold
  the host (master) lock, so the host lock is requested automatically on connect.
* Manual track is a *visual follower*: the mount only rotates once it locks onto a
  distinct MOVING object inside the ROI. A static/indoor scene will not move.
* RTSP URLs: tele = rtsp://<ip>/ch0/stream0 , wide = rtsp://<ip>/ch1/stream0
"""
from __future__ import annotations

import argparse
import asyncio
import ftplib
import json
import math
import os
import queue
import threading
import time
import tkinter as tk
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from tkinter import ttk

import numpy as np

# OpenCV/FFmpeg RTSP options, set *before* the first VideoCapture call.
# Beyond forcing TCP transport, these trim the FFmpeg-side buffering that adds
# most of the glass-to-glass latency on the Dwarf's H.265 stream:
#   fflags;nobuffer + flags;low_delay  -> don't hold frames, decode ASAP
#   max_delay;0 + reorder_queue_size;0 -> no reorder/jitter buffering
#   probesize;32 + analyzeduration;0   -> minimal stream probing at open
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    "rtsp_transport;tcp"
    "|fflags;nobuffer"
    "|flags;low_delay"
    "|max_delay;0"
    "|reorder_queue_size;0"
    "|probesize;32"
    "|analyzeduration;0",
)

import cv2  # noqa: E402
from PIL import Image, ImageTk  # noqa: E402

from dwarflab_controller import (  # noqa: E402
    DwarfLab,
    _parse_varint_fields,
    CMD_NOTIFY_TRACK_RESULT,
    CMD_NOTIFY_WIDE_TRACK_RESULT,
    CMD_NOTIFY_SENTRY_MODE_TRACK_RESULT,
    CMD_NOTIFY_SENTRY_MODE_STATE,
    CMD_NOTIFY_MULTI_TRACK_RESULT,
    CMD_NOTIFY_WIDE_MULTI_TRACK_RESULT,
    CMD_NOTIFY_UFO_MODE_STATE,
    CODE_TRACK_TRACKER_INITING,
    CODE_TRACK_TRACKER_FAILED,
    TRACK_REF_W,
    TRACK_REF_H,
    TRACK_NO_TARGET,
)

_SENTRY_STATE = {0: "IDLE", 1: "INIT", 2: "DETECT", 3: "TRACK",
                 4: "FINISH", 5: "STOPPING"}
_BOX_CMDS = {
    CMD_NOTIFY_TRACK_RESULT: "tele",
    CMD_NOTIFY_WIDE_TRACK_RESULT: "wide",
    CMD_NOTIFY_SENTRY_MODE_TRACK_RESULT: "sentry",
}


CAMERAS = {"Tele (ch0)": "ch0", "Wide (ch1)": "ch1"}

# ── Closed-loop tuning ────────────────────────────────────────────────────────
# The DWARF firmware reports a live (morphing/resizing) tracking box; we read it
# and continuously drive the joystick motors to keep that box centred. This is
# the "continuous interaction between motors and target" that makes tracking work.
LOOP_MS     = 120     # control period (ms)
BOX_STALE_S = 1.5     # ignore tracking boxes older than this (target lost)
DEADZONE    = 0.06    # |error| (fraction of half-frame) treated as centred
GAIN        = 70.0    # proportional gain: error(-1..1) -> joystick units
MAX_SPEED   = 45.0    # clamp joystick magnitude per axis
MIN_SPEED   = 9.0     # minimum command to overcome motor stiction
AZ_SIGN     = +1.0    # flip if azimuth (left/right) correction goes wrong way
ALT_SIGN    = +1.0    # flip if altitude (up/down) correction goes wrong way

# ── Keyboard jog (arrow keys) ───────────────────────────────────────────────────
# Manual joystick control while an arrow key is held, using the same AZ_SIGN/
# ALT_SIGN convention as the ROI-based corrections above so it stays consistent
# with whatever those get tuned to.
ARROW_KEY_SPEED = 30.0   # joystick magnitude while a key is held

# ── ROI nudge (WASD) ────────────────────────────────────────────────────────────
# Slide the *selection box* itself (not the mount) with W/A/S/D — separate keys
# from the arrow-key jog above so both can be used independently (e.g. nudge the
# box to correct for drift, then let it re-lock). Re-sending the track command on
# every single keystroke while held (OS auto-repeat) would flood the device, so
# the actual retrack is debounced to fire once, shortly after nudging stops.
ROI_NUDGE_STEP_PX = 20     # frame pixels moved per keypress
ROI_NUDGE_DEBOUNCE_MS = 150

# ── Live photo matching ──────────────────────────────────────────────────────
# Continuously matches the live tele feed against reference.jpg (same SIFT +
# RANSAC pipeline as the one-shot Match button) and draws a crosshair where
# the reference's center currently falls in the live view. SIFT on a full
# 1920x1080 frame takes ~100-300ms, so this runs in a background thread,
# throttled, with only one match in flight at a time — never every frame.
LIVE_MATCH_INTERVAL_MS = 600

# ── Direct slew tuning (Center-on-ROI) ────────────────────────────────────────
# Drives the motors open-loop to bring the selected ROI centre to the frame
# centre, independent of the firmware visual tracker. Drag a box -> the mount
# slews toward it. Repeat to refine. This always produces motor motion.
SLEW_SPEED     = 35.0   # joystick magnitude used during a manual slew
SLEW_TIME_FULL = 1.10   # seconds of slew for a full half-frame offset, TUNED ON WIDE
SLEW_MIN_OFF   = 0.04   # ignore tiny offsets (already centred)

# Tele's FOV is much narrower than wide's, so the same fractional-frame ROI
# offset is a much smaller *angle* to slew through on tele than on wide.
# SLEW_SPEED/SLEW_TIME_FULL above were tuned against the wide camera; reusing
# them verbatim for tele way overshoots (mount keeps slewing long after the
# target has left the tele frame). Best-effort published-spec values — retune
# here if the mount over/undershoots.
TELE_FOV_DEG = 3.4
WIDE_FOV_DEG = 60.0
TELE_SLEW_SCALE = TELE_FOV_DEG / WIDE_FOV_DEG   # total angle to move, relative to wide

# How TELE_SLEW_SCALE is split between speed and duration. speed_scale =
# TELE_SLEW_SCALE**TELE_SLEW_SPEED_EXP, dur_scale = TELE_SLEW_SCALE**(1-EXP),
# so their product always equals TELE_SLEW_SCALE regardless of the split.
#   EXP=0    -> full speed, all reduction on duration: field-tested TOO
#               AGGRESSIVE (overshoots) despite the short pulse.
#   EXP=0.25 -> field-tested STILL TOO FAST.
#   EXP=0.5  -> sqrt/sqrt split: field-tested TOO WEAK (speed drops below
#               what's needed to reliably break the mount's static friction).
# 0.375 (between the too-fast 0.25 and too-weak 0.5) is the current
# best-effort middle ground — retune here if it still over/undershoots.
TELE_SLEW_SPEED_EXP = 0.375

# Soft-start: ramp the joystick magnitude up over this many steps/ms instead of
# snapping to full speed in one command, mirroring the app's own behaviour and
# avoiding a stiction-breakaway lurch (worst on tele's narrow FOV).
SLEW_RAMP_STEPS = 6
SLEW_RAMP_STEP_MS = 35

# ── Sentinel/UFO auto-track profiles ──────────────────────────────────────────
# Firmware auto-detects + drives its own motors in these modes. The `mode` int
# in ReqStartSentryMode selects the detection profile. Exact codes are not in
# the public doc, so these are best-effort and easy to retune here.
#   kind: "sentry" -> 14802/14803,  "ufo" -> 14806/14807
AUTO_MODES = {
    "Bird":     {"kind": "ufo", "mode": 0},
    "Airplane": {"kind": "ufo", "mode": 1},
    "UFO":      {"kind": "ufo", "mode": 2},
}


# ── RTSP frame grabber thread ─────────────────────────────────────────────────
class FrameGrabber(threading.Thread):
    """Reads frames from an RTSP URL and pushes the latest one into a queue."""

    def __init__(self, url: str, out: "queue.Queue[np.ndarray]",
                 status_cb) -> None:
        super().__init__(daemon=True)
        self._url = url
        self._out = out
        self._status = status_cb
        self._running = True

    def run(self) -> None:
        self._status(f"opening {self._url} ...")
        cap = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            self._status("RTSP open FAILED — is the camera started?")
            return
        self._status("RTSP stream live")
        miss = 0
        while self._running:
            ok, frame = cap.read()
            if not ok or frame is None:
                miss += 1
                if miss > 60:
                    self._status("RTSP stream dropped")
                    break
                continue
            miss = 0
            # keep only the most recent frame
            try:
                while True:
                    self._out.get_nowait()
            except queue.Empty:
                pass
            self._out.put(frame)
        cap.release()

    def stop(self) -> None:
        self._running = False


# ── WitMotion IMU monitor (optional, BLE via wit_imu.WitIMU) ──────────────────
class WitMonitor:
    """Run a WitMotion BLE IMU in a background asyncio thread and keep the latest
    attitude sample available for the Tk main thread.

    The transport, frame decode and Sun/Moon calibration all live in wit_imu.py;
    this is just the glue that lets the Tkinter GUI show a live roll/pitch/yaw
    readout and snapshot the mount attitude at capture time. wit_imu (and its
    ``bleak`` dependency) are imported lazily so the GUI still runs without them.
    """

    CALIB_PATH = "wit_calibration.json"

    def __init__(self, status_cb, address=None, name=None) -> None:
        self._status = status_cb
        self._address = address
        self._name = name
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._imu = None                    # wit_imu.WitIMU
        self._calib = None                  # wit_imu.Calibration | None
        self._latest = None                 # (SensorData, ts) — atomic swap
        self._connected = False
        self._err: str | None = None

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def active(self) -> bool:
        """True while the background thread is alive (connecting or connected)."""
        return self._thread is not None and self._thread.is_alive()

    @property
    def error(self) -> str | None:
        return self._err

    def _on_data(self, sample) -> None:
        # Called from the asyncio thread for every decoded frame. A single tuple
        # assignment is atomic in CPython, so the Tk thread can read it freely.
        self._latest = (sample, time.time())

    def start(self) -> None:
        if self.active:
            return
        self._err = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            from wit_imu import WitIMU, Calibration   # lazy: needs bleak
        except Exception as exc:                       # ImportError if bleak absent
            self._err = f"WIT unavailable ({exc})"
            self._status(self._err)
            return
        try:
            p = Path(self.CALIB_PATH)
            if p.exists():
                self._calib = Calibration.load(p)      # Sun/Moon offset, if made
        except Exception:
            self._calib = None
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._imu = WitIMU(on_data=self._on_data)
        try:
            self._loop.run_until_complete(
                self._imu.connect(address=self._address, name=self._name))
            self._connected = True
            self._status("WIT IMU connected — live attitude streaming.")
            self._loop.run_forever()
        except Exception as exc:
            self._err = f"WIT connect failed ({exc})"
            self._status(self._err)
        finally:
            self._connected = False
            try:
                self._loop.run_until_complete(self._imu.disconnect())
            except Exception:
                pass
            self._loop.close()
            self._loop = None
            self._imu = None

    def stop(self) -> None:
        if self._loop is not None and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._connected = False
        self._latest = None
        self._thread = None

    def snapshot(self) -> dict | None:
        """Return the current attitude as a plain dict, or None if no sample yet."""
        item = self._latest
        if item is None:
            return None
        sample, ts = item
        rec = {
            "age_s": round(time.time() - ts, 3),
            "roll": round(sample.angle.x, 3),
            "pitch": round(sample.angle.y, 3),
            "yaw": round(sample.angle.z, 3),
            "acc_g": [round(sample.acceleration.x, 4),
                      round(sample.acceleration.y, 4),
                      round(sample.acceleration.z, 4)],
        }
        if self._calib is not None:
            alt, az = self._calib.apply(sample)         # true alt/az from offsets
            rec["altitude"] = round(alt, 3)
            rec["azimuth"] = round(az, 3)
            rec["calibrated"] = True
        else:
            # uncalibrated: raw pitch doubles as altitude (see wit_imu README)
            rec["altitude"] = round(sample.altitude, 3)
            rec["calibrated"] = False
        return rec


# ── Main application ──────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, ip: str,
                 wit_address: str | None = None,
                 wit_name: str | None = None) -> None:
        self.root = root
        self.ip = ip
        self.ctl: DwarfLab | None = None
        self.grabber: FrameGrabber | None = None
        self.frame_q: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=2)
        # Secondary preview: always the OTHER camera (whichever isn't
        # selected), read-only, shows the ROI projected across via FOV ratio.
        self.grabber2: FrameGrabber | None = None
        self.frame_q2: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=2)
        self._photo2: ImageTk.PhotoImage | None = None
        self._disp2 = (0, 0, 0, 0)
        self._frame_wh2 = (0, 0)

        # WitMotion IMU (optional, connected independently of the DWARF)
        self.wit: WitMonitor | None = None
        self.wit_address = wit_address
        self.wit_name = wit_name
        self.capture_log = "captures.jsonl"
        self.photo_dir = "captured_photos"   # local folder for FTP-downloaded photos
        self.last_photo_path: str | None = None   # most recently downloaded photo
        self.reference_photo = "reference.jpg"     # principal photo to match against
        self.match_log = "match_log.ndjson"
        self._latest_frame: np.ndarray | None = None    # primary canvas's frame
        self._latest_frame2: np.ndarray | None = None   # secondary canvas's frame
        self._live_match_busy = False       # one match job in flight at a time
        self._live_ref_kp = None            # cached reference SIFT (computed once)
        self._live_ref_des = None
        self._live_ref_shape: tuple[int, int] | None = None   # (h, w)
        self._live_marker_id: int | None = None
        self._live_ref_gray: np.ndarray | None = None   # cached reference pixels
        self.match_win: tk.Toplevel | None = None        # popup: lines + timing + angle

        self.roi: tuple[int, int, int, int] | None = None   # frame-pixel ROI
        self._drag_start: tuple[int, int] | None = None      # canvas coords
        self._rect_id: int | None = None
        self._photo: ImageTk.PhotoImage | None = None
        self._tracking = False   # closed loop requested (Start Track pressed)
        self._mot = False        # MOT detector running (AI Track pressed)
        self._multi_disp = []    # [(id, sx, sy, ex, ey)] clickable detected boxes
        self._moving = False     # joystick currently commanded non-zero
        self._slewing = False    # a manual Center-on-ROI slew is in progress
        self._keys_down: set[str] = set()   # arrow keys currently held (keyboard jog)
        self._roi_nudge_after_id: str | None = None   # pending debounced retrack
        # live diagnostics captured from the device notify stream (WS thread)
        self._diag = {
            "counts": defaultdict(int),
            "last_box": {"tele": None, "wide": None, "sentry": None},
            "multi": {"tele": 0, "wide": 0},
            "tracker": None,
            "sentry_state": None,
            "ufo_state": None,
        }
        # geometry of the displayed (letterboxed) frame within the canvas
        self._disp = (0, 0, 0, 0)   # x, y, w, h
        self._frame_wh = (0, 0)

        root.title(f"DWARF 3 — Live ROI Tracker ({ip})")
        root.geometry("1024x700")
        root.configure(bg="#1a1a1f")

        # -- toolbar --
        bar = tk.Frame(root, bg="#1a1a1f")
        bar.pack(side=tk.TOP, fill=tk.X, padx=8, pady=6)

        tk.Label(bar, text="Camera:", fg="#ddd", bg="#1a1a1f").pack(side=tk.LEFT)
        self.cam_var = tk.StringVar(value="Wide (ch1)")
        cam = ttk.Combobox(bar, textvariable=self.cam_var, values=list(CAMERAS),
                           state="readonly", width=12)
        cam.pack(side=tk.LEFT, padx=(4, 12))
        cam.bind("<<ComboboxSelected>>", lambda _e: self._switch_camera())
        # ttk.Combobox has its own Up/Down class bindings that cycle its
        # selection and consume the event before it reaches root — override
        # with instance bindings (these win) so arrow keys jog the mount
        # instead of changing the camera dropdown while it has focus.
        for _key in ("Up", "Down", "Left", "Right"):
            cam.bind(f"<KeyPress-{_key}>", lambda _e, k=_key: self._jog_press_break(k))
            cam.bind(f"<KeyRelease-{_key}>", lambda _e, k=_key: self._jog_release_break(k))
        # Same problem for W/A/S/D: readonly Combobox jumps to the option
        # starting with that letter (type-ahead) unless we intercept first.
        for _key in ("w", "a", "s", "d", "W", "A", "S", "D"):
            cam.bind(f"<KeyPress-{_key}>", lambda _e, k=_key.lower(): self._roi_nudge_break(k))

        self.btn_connect = tk.Button(bar, text="Connect", width=11,
                                     command=self._toggle_connect)
        self.btn_connect.pack(side=tk.LEFT)

        self.btn_track = tk.Button(bar, text="Start Track ROI", width=14,
                                   state=tk.DISABLED, command=self._start_track)
        self.btn_track.pack(side=tk.RIGHT)
        self.btn_ai = tk.Button(bar, text="AI Track (MOT)", width=13,
                                state=tk.DISABLED, command=self._start_ai_track)
        self.btn_ai.pack(side=tk.RIGHT, padx=4)
        self.btn_center = tk.Button(bar, text="Center on ROI", width=13,
                                    state=tk.DISABLED, command=self._center_on_roi)
        self.btn_center.pack(side=tk.RIGHT, padx=4)
        self.btn_stop = tk.Button(bar, text="Stop Track", width=11,
                                  state=tk.DISABLED, command=self._stop_track)
        self.btn_stop.pack(side=tk.RIGHT, padx=4)
        self.btn_clear = tk.Button(bar, text="Clear ROI", width=10,
                                   command=self._clear_roi)
        self.btn_clear.pack(side=tk.RIGHT)

        self.loop_var = tk.BooleanVar(value=True)
        self.chk_loop = tk.Checkbutton(
            bar, text="Auto-center motors", variable=self.loop_var,
            fg="#ddd", bg="#1a1a1f", selectcolor="#1a1a1f",
            activebackground="#1a1a1f", activeforeground="#ddd",
            command=self._on_loop_toggle)
        self.chk_loop.pack(side=tk.RIGHT, padx=10)

        # -- mode toolbar (firmware auto-detect & track profiles) --
        bar2 = tk.Frame(root, bg="#1a1a1f")
        bar2.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 6))
        tk.Label(bar2, text="Auto-track:", fg="#ddd", bg="#1a1a1f").pack(
            side=tk.LEFT)
        self.mode_var = tk.StringVar(value="")
        self.mode_btns: dict[str, tk.Button] = {}
        for name in AUTO_MODES:
            b = tk.Button(bar2, text=name, width=10, state=tk.DISABLED,
                          command=lambda n=name: self._start_auto_mode(n))
            b.pack(side=tk.LEFT, padx=3)
            self.mode_btns[name] = b
        self.btn_auto_stop = tk.Button(bar2, text="Stop Auto", width=10,
                                       state=tk.DISABLED,
                                       command=self._stop_auto_mode)
        self.btn_auto_stop.pack(side=tk.LEFT, padx=(12, 3))
        tk.Label(bar2,
                 text="(firmware auto-detects & drives its own motors)",
                 fg="#777", bg="#1a1a1f").pack(side=tk.LEFT, padx=8)

        # -- focus controls (operate on the selected camera) --
        self.focus_btns: list[tk.Button] = []
        self.btn_focus_far = tk.Button(bar2, text="Focus ►", width=8,
                                       state=tk.DISABLED)
        self.btn_focus_far.pack(side=tk.RIGHT, padx=(2, 0))
        self.btn_focus_far.bind("<ButtonPress-1>", self._focus_far_press)
        self.btn_focus_far.bind("<ButtonRelease-1>", self._focus_release)
        self.btn_focus_step_out = tk.Button(bar2, text="+", width=2,
                                            state=tk.DISABLED,
                                            command=lambda: self._focus_step(1))
        self.btn_focus_step_out.pack(side=tk.RIGHT, padx=2)
        self.btn_focus_step_in = tk.Button(bar2, text="−", width=2,
                                           state=tk.DISABLED,
                                           command=lambda: self._focus_step(-1))
        self.btn_focus_step_in.pack(side=tk.RIGHT, padx=2)
        self.btn_focus_near = tk.Button(bar2, text="◄ Focus", width=8,
                                        state=tk.DISABLED)
        self.btn_focus_near.pack(side=tk.RIGHT, padx=(0, 2))
        self.btn_focus_near.bind("<ButtonPress-1>", self._focus_near_press)
        self.btn_focus_near.bind("<ButtonRelease-1>", self._focus_release)
        self.btn_autofocus = tk.Button(bar2, text="Auto Focus", width=11,
                                       state=tk.DISABLED, command=self._auto_focus)
        self.btn_autofocus.pack(side=tk.RIGHT, padx=(0, 4))
        tk.Label(bar2, text="Focus:", fg="#ddd", bg="#1a1a1f").pack(
            side=tk.RIGHT, padx=(12, 2))
        self.focus_btns = [self.btn_autofocus, self.btn_focus_near,
                           self.btn_focus_step_in, self.btn_focus_step_out,
                           self.btn_focus_far]

        # -- capture + IMU toolbar --
        bar3 = tk.Frame(root, bg="#1a1a1f")
        bar3.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 6))
        tk.Label(bar3, text="Capture:", fg="#ddd", bg="#1a1a1f").pack(
            side=tk.LEFT)
        # Telephoto still (CMD_CAMERA_TELE_PHOTOGRAPH). Enabled once connected.
        self.btn_capture = tk.Button(bar3, text="📷 Photo (Tele)", width=14,
                                     state=tk.DISABLED,
                                     command=self._capture_photo)
        self.btn_capture.pack(side=tk.LEFT, padx=(4, 12))
        self.wit_attach = tk.BooleanVar(value=True)
        tk.Checkbutton(
            bar3, text="Record IMU attitude with photo",
            variable=self.wit_attach, fg="#ddd", bg="#1a1a1f",
            selectcolor="#1a1a1f", activebackground="#1a1a1f",
            activeforeground="#ddd").pack(side=tk.LEFT)
        # Sub-pixel photo registration (reference.jpg vs the latest capture).
        # Local-file-only, so it works whether or not the DWARF is connected.
        self.btn_match = tk.Button(bar3, text="🎯 Match", width=10,
                                   command=self._match_photos)
        self.btn_match.pack(side=tk.LEFT, padx=(12, 0))
        self.live_match_var = tk.BooleanVar(value=False)
        tk.Checkbutton(
            bar3, text="Live match (tele vs reference)",
            variable=self.live_match_var, fg="#ddd", bg="#1a1a1f",
            selectcolor="#1a1a1f", activebackground="#1a1a1f",
            activeforeground="#ddd",
            command=self._toggle_live_match).pack(side=tk.LEFT, padx=(8, 0))

        # IMU (WitMotion) connect — independent of the DWARF connection
        self.btn_wit = tk.Button(bar3, text="Connect WIT IMU", width=15,
                                 command=self._toggle_wit)
        self.btn_wit.pack(side=tk.RIGHT)
        tk.Label(bar3, text="IMU:", fg="#ddd", bg="#1a1a1f").pack(
            side=tk.RIGHT, padx=(12, 2))

        # -- video canvases: primary (interactive) + secondary (other camera,
        # read-only preview showing the ROI projected across via FOV ratio) --
        video_row = tk.Frame(root, bg="#1a1a1f")
        video_row.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=4)

        primary_col = tk.Frame(video_row, bg="#1a1a1f")
        primary_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(primary_col, bg="#0e0e12", highlightthickness=0,
                                cursor="crosshair")
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas.bind("<ButtonPress-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.create_text(
            10, 10, anchor=tk.NW, fill="#888", tags="hint",
            text="No video — click Connect, then drag a box over the feed.")

        secondary_col = tk.Frame(video_row, bg="#1a1a1f", width=360)
        secondary_col.pack(side=tk.LEFT, fill=tk.Y, padx=(6, 0))
        secondary_col.pack_propagate(False)
        self.secondary_label_var = tk.StringVar(value="other camera")
        tk.Label(secondary_col, textvariable=self.secondary_label_var,
                 fg="#888", bg="#1a1a1f").pack(side=tk.TOP, anchor=tk.W)
        self.canvas2 = tk.Canvas(secondary_col, bg="#0e0e12", highlightthickness=0)
        self.canvas2.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # -- status bar --
        self.status_var = tk.StringVar(
            value="Disconnected. Connect, then drag on the video to select a ROI.")
        tk.Label(root, textvariable=self.status_var, anchor=tk.W, fg="#bbb",
                 bg="#26262e").pack(side=tk.BOTTOM, fill=tk.X)

        # -- diagnostics row (raw device track traffic) --
        self.diag_var = tk.StringVar(value="diag: (not connected)")
        tk.Label(root, textvariable=self.diag_var, anchor=tk.W, fg="#8fdcff",
                 bg="#1c1c24", font=("Consolas", 9)).pack(
            side=tk.BOTTOM, fill=tk.X)

        # -- IMU attitude row (WitMotion roll/pitch/yaw + calibrated alt/az) --
        self.wit_var = tk.StringVar(value="IMU: (not connected)")
        tk.Label(root, textvariable=self.wit_var, anchor=tk.W, fg="#ffd479",
                 bg="#1c1c24", font=("Consolas", 9)).pack(
            side=tk.BOTTOM, fill=tk.X)

        # -- live-match row (own line so it isn't overwritten by _status) --
        self.live_match_status_var = tk.StringVar(value="Live match: off")
        tk.Label(root, textvariable=self.live_match_status_var, anchor=tk.W,
                 fg="#22d3ee", bg="#1c1c24", font=("Consolas", 9)).pack(
            side=tk.BOTTOM, fill=tk.X)

        # Arrow-key keyboard jog. Bound on root (not the canvas) so it fires
        # regardless of which widget has focus, as long as no Entry/text
        # widget steals it — this GUI has none.
        for _key in ("Up", "Down", "Left", "Right"):
            root.bind(f"<KeyPress-{_key}>",
                      lambda _e, k=_key: self._on_arrow_press(k))
            root.bind(f"<KeyRelease-{_key}>",
                      lambda _e, k=_key: self._on_arrow_release(k))
        # Safety net: if the window loses focus while a key is held, no
        # KeyRelease ever fires and the motor would keep running — force-stop.
        root.bind("<FocusOut>", lambda _e: self._on_arrow_focus_lost())

        # WASD: slide the ROI selection box itself (not the mount).
        for _key in ("w", "a", "s", "d"):
            root.bind(f"<KeyPress-{_key}>", lambda _e, k=_key: self._roi_nudge(k))

        root.focus_set()

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(30, self._poll_frame)
        self.root.after(LOOP_MS, self._control_tick)
        self.root.after(300, self._diag_tick)
        self.root.after(200, self._wit_tick)
        self.root.after(LIVE_MATCH_INTERVAL_MS, self._live_match_tick)

    # -- status (updates a Tk variable; safe to call from worker threads) --
    def _status(self, msg: str) -> None:
        # Python 3.12 enforces that Tcl calls come from the thread that
        # created the interpreter — a direct .set() from FrameGrabber/WIT's
        # background threads now raises "main thread is not in main loop"
        # instead of silently working. Marshal onto the Tk main thread.
        self.root.after(0, lambda: self.status_var.set(msg))

    def _rtsp_url(self) -> str:
        return f"rtsp://{self.ip}/{CAMERAS[self.cam_var.get()]}/stream0"

    def _is_wide_selected(self) -> bool:
        return CAMERAS.get(self.cam_var.get()) == "ch1"

    def _other_camera_ch(self) -> str:
        return "ch0" if self._is_wide_selected() else "ch1"

    def _secondary_rtsp_url(self) -> str:
        return f"rtsp://{self.ip}/{self._other_camera_ch()}/stream0"

    # ── connect / disconnect ─────────────────────────────────────────────────
    def _toggle_connect(self) -> None:
        if self.ctl is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        self._status(f"Connecting to {self.ip} ...")
        self.root.update_idletasks()
        self._diag = {
            "counts": defaultdict(int),
            "last_box": {"tele": None, "wide": None, "sentry": None},
            "multi": {"tele": 0, "wide": 0},
            "tracker": None,
            "sentry_state": None,
            "ufo_state": None,
        }
        ctl = DwarfLab(host=self.ip, on_notify=self._on_notify)
        if not ctl.connect(timeout=10.0):
            self._status("WS connect FAILED — check the IP / that the device is on.")
            return
        self.ctl = ctl
        ctl.set_master_lock(True)        # firmware ignores commands without the lock
        self._open_camera()
        self._start_stream()
        self.btn_connect.config(text="Disconnect")
        self.btn_track.config(state=tk.NORMAL)
        self.btn_ai.config(state=tk.NORMAL)
        self.btn_center.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.NORMAL)
        self.btn_capture.config(state=tk.NORMAL)
        for b in self.mode_btns.values():
            b.config(state=tk.NORMAL)
        self.btn_auto_stop.config(state=tk.NORMAL)
        for b in self.focus_btns:
            b.config(state=tk.NORMAL)
        self._status("Connected + host lock requested. Drag a box on the video.")

    def _open_camera(self) -> None:
        if self.ctl is None:
            return
        # CAPTURE-VERIFIED V3 bring-up (the app's order). This is what arms the
        # firmware so 14800 actually locks. NOTE: open wide with an EMPTY payload
        # (v3_open_wide) — {1:1} = CLOSE and knocks the device offline.
        #
        # 14800 TRACK_START_TRACK has no camera-select field at all, yet the app
        # clearly gets it to lock tele vs wide correctly (capture-verified: tele's
        # NOTIFY channel roamed with every drag while wide's sat frozen on a static
        # blob the whole session). The firmware must be inferring the target
        # camera from open/mode state — and the app's own capture (coldstart
        # test) showed it toggling close+reopen ending with whichever camera the
        # user had just switched to opened LAST. Unconditionally opening
        # tele-then-wide every time (old code) always left wide "freshest",
        # which would silently route every track to wide regardless of what's
        # selected — open whichever camera is currently selected last instead.
        is_wide = CAMERAS.get(self.cam_var.get()) == "ch1"
        self.ctl.v3_mode_switch(1)     # 16404 {3:{1:1}}
        if is_wide:
            self.ctl.v3_open_tele(1)       # 10050 {1:1}
            self.ctl.v3_open_wide()        # 12036 (empty) = open — selected, last
        else:
            self.ctl.v3_open_wide()        # 12036 (empty) = open
            self.ctl.v3_open_tele(1)       # 10050 {1:1} — selected, last

    def _disconnect(self) -> None:
        self._stop_stream()
        self._tracking = False
        self._slewing = False
        if self.ctl is not None:
            try:
                if self._moving:
                    self.ctl.joystick_stop()
                self.ctl.stop_tracking()
                self.ctl.set_master_lock(False)
                self.ctl.disconnect()
            except Exception:
                pass
            self.ctl = None
        self._moving = False
        self.btn_connect.config(text="Connect")
        self.btn_track.config(state=tk.DISABLED)
        self.btn_ai.config(state=tk.DISABLED)
        self.btn_center.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.DISABLED)
        self.btn_capture.config(state=tk.DISABLED)
        for b in self.mode_btns.values():
            b.config(state=tk.DISABLED)
        self.btn_auto_stop.config(state=tk.DISABLED)
        for b in self.focus_btns:
            b.config(state=tk.DISABLED)
        self._status("Disconnected.")

    # ── streaming ────────────────────────────────────────────────────────────
    def _start_stream(self) -> None:
        self._stop_stream()
        self.grabber = FrameGrabber(self._rtsp_url(), self.frame_q, self._status)
        self.grabber.start()
        self.grabber2 = FrameGrabber(self._secondary_rtsp_url(), self.frame_q2,
                                     self._status)
        self.grabber2.start()
        other = "Tele (ch0)" if self._is_wide_selected() else "Wide (ch1)"
        self.secondary_label_var.set(f"{other} — projected ROI")

    def _stop_stream(self) -> None:
        if self.grabber is not None:
            self.grabber.stop()
            self.grabber = None
        if self.grabber2 is not None:
            self.grabber2.stop()
            self.grabber2 = None

    def _switch_camera(self) -> None:
        if self.ctl is None:
            return
        self._open_camera()
        self._start_stream()

    # ── frame rendering (main thread, Tk timer) ──────────────────────────────
    def _poll_frame(self) -> None:
        try:
            frame = self.frame_q.get_nowait()
        except queue.Empty:
            frame = None
        if frame is not None:
            self._show_frame(frame)
        try:
            frame2 = self.frame_q2.get_nowait()
        except queue.Empty:
            frame2 = None
        if frame2 is not None:
            self._show_frame2(frame2)
        self.root.after(30, self._poll_frame)

    def _show_frame(self, frame: np.ndarray) -> None:
        self._latest_frame = frame   # cached for live-match background thread
        cw = max(1, self.canvas.winfo_width())
        ch = max(1, self.canvas.winfo_height())
        fh, fw = frame.shape[:2]
        self._frame_wh = (fw, fh)
        scale = min(cw / fw, ch / fh)
        dw, dh = max(1, int(fw * scale)), max(1, int(fh * scale))
        ox, oy = (cw - dw) // 2, (ch - dh) // 2
        self._disp = (ox, oy, dw, dh)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb).resize((dw, dh))
        self._photo = ImageTk.PhotoImage(img)
        self.canvas.delete("hint")
        self.canvas.delete("video")
        self.canvas.create_image(ox, oy, anchor=tk.NW, image=self._photo,
                                 tags="video")
        self.canvas.tag_lower("video")
        self._draw_track_box(ox, oy, dw, dh, fw, fh)
        self._draw_multi_boxes(ox, oy, dw, dh, fw, fh)
        if self._rect_id is not None:
            self.canvas.tag_raise(self._rect_id)

    def _project_roi(self, roi: tuple[int, int, int, int], from_wide: bool,
                     target_fw: int, target_fh: int) -> tuple[float, float, float, float]:
        """Project an ROI drawn in one camera's frame into the other's,
        assuming both share a boresight and scaling by the published FOV
        ratio (TELE_FOV_DEG/WIDE_FOV_DEG — approximate, not capture-verified,
        same caveat as the Center-on-ROI slew tuning). Tele has a much
        narrower FOV, so the same physical box is much LARGER in tele-frame
        pixels than in wide-frame pixels."""
        x, y, w, h = roi
        src_fw, src_fh = self._frame_wh
        scale = (WIDE_FOV_DEG / TELE_FOV_DEG) if from_wide else (TELE_FOV_DEG / WIDE_FOV_DEG)
        cx, cy = src_fw / 2.0, src_fh / 2.0
        bx, by = x + w / 2.0, y + h / 2.0
        pcx = target_fw / 2.0 + (bx - cx) * scale
        pcy = target_fh / 2.0 + (by - cy) * scale
        pw, ph = w * scale, h * scale
        return (pcx - pw / 2.0, pcy - ph / 2.0, pw, ph)

    def _show_frame2(self, frame: np.ndarray) -> None:
        self._latest_frame2 = frame   # cached for live-match (tele-only target)
        cw = max(1, self.canvas2.winfo_width())
        ch = max(1, self.canvas2.winfo_height())
        fh, fw = frame.shape[:2]
        self._frame_wh2 = (fw, fh)
        scale = min(cw / fw, ch / fh)
        dw, dh = max(1, int(fw * scale)), max(1, int(fh * scale))
        ox, oy = (cw - dw) // 2, (ch - dh) // 2
        self._disp2 = (ox, oy, dw, dh)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb).resize((dw, dh))
        self._photo2 = ImageTk.PhotoImage(img)
        self.canvas2.delete("video2")
        self.canvas2.create_image(ox, oy, anchor=tk.NW, image=self._photo2,
                                  tags="video2")
        self.canvas2.tag_lower("video2")

        self.canvas2.delete("projbox")
        if self.roi is not None and self._frame_wh[0] > 0 and fw > 0:
            px, py, pw, ph = self._project_roi(
                self.roi, self._is_wide_selected(), fw, fh)
            sx = ox + px / fw * dw
            sy = oy + py / fh * dh
            ex = ox + (px + pw) / fw * dw
            ey = oy + (py + ph) / fh * dh
            self.canvas2.create_rectangle(sx, sy, ex, ey, outline="#22d3ee",
                                          width=2, dash=(4, 2), tags="projbox")

    def _draw_track_box(self, ox, oy, dw, dh, fw, fh) -> None:
        """Overlay the live (morphing) box the firmware is tracking, in green.

        CAPTURE-VERIFIED: box coords are in WIDE-STREAM PIXELS (≈1920x1080 — an
        app ROI of x=975,w=382 gives x+w=1357 > 1280, so the space is NOT
        1280x720). The decoded RTSP frame is that same space, so scale by fw/fh.
        """
        self.canvas.delete("trackbox")
        if self.ctl is None or fw == 0:
            return
        box = self.ctl.state.get("track_box")
        ts = self.ctl.state.get("track_box_ts", 0.0)
        if (not box or box[0] <= TRACK_NO_TARGET or box[1] <= TRACK_NO_TARGET
                or (time.time() - ts) > BOX_STALE_S):
            return
        bx, by, bw, bh = box
        sx = ox + bx / fw * dw
        sy = oy + by / fh * dh
        ex = ox + (bx + bw) / fw * dw
        ey = oy + (by + bh) / fh * dh
        self.canvas.create_rectangle(sx, sy, ex, ey, outline="#39ff14",
                                     width=2, tags="trackbox")

    @staticmethod
    def _interpret_multi(d, idx):
        """Best-effort map of a parsed multi-track sub-message to (id,x,y,w,h) in
        TRACK_REF space. UNVERIFIED field layout — refine when a populated
        15238/15251 sample is captured."""
        if all(k in d for k in (1, 2, 3, 4, 5)):
            return d[1], d[2], d[3], d[4], d[5]          # {id,x,y,w,h}
        if all(k in d for k in (1, 2, 3, 4)):
            return idx, d[1], d[2], d[3], d[4]           # {x,y,w,h}, id=index
        return None

    def _draw_multi_boxes(self, ox, oy, dw, dh, fw, fh) -> None:
        """Draw MOT-detected objects (cyan, clickable) from state['multi_boxes'].
        Coords assumed in stream pixels (same space as the single-track box)."""
        self.canvas.delete("multibox")
        self._multi_disp = []
        if self.ctl is None or not self._mot or fw == 0:
            return
        boxes = self.ctl.state.get("multi_boxes") or []
        ts = self.ctl.state.get("multi_boxes_ts", 0.0)
        if (time.time() - ts) > BOX_STALE_S:
            return
        for idx, d in enumerate(boxes):
            parsed = self._interpret_multi(d, idx)
            if not parsed:
                continue
            oid, x, y, w, h = parsed
            sx = ox + x / fw * dw
            sy = oy + y / fh * dh
            ex = ox + (x + w) / fw * dw
            ey = oy + (y + h) / fh * dh
            self.canvas.create_rectangle(sx, sy, ex, ey, outline="#22d3ee",
                                         width=2, tags="multibox")
            self.canvas.create_text(sx + 3, sy + 3, anchor=tk.NW, fill="#22d3ee",
                                     text=f"id {oid}", tags="multibox")
            self._multi_disp.append((oid, sx, sy, ex, ey))

    def _select_multi_at(self, x, y) -> bool:
        """If (x,y) hits a detected box, lock it via MOT_WIDE_TRACK_ONE."""
        for oid, sx, sy, ex, ey in self._multi_disp:
            if sx <= x <= ex and sy <= y <= ey:
                if self.ctl is not None:
                    self.ctl.mot_wide_track_one(oid)
                    self._tracking = True
                    self._status(f"Locking detected object id {oid} "
                                 "(MOT_WIDE_TRACK_ONE).")
                return True
        return False

    # ── ROI mouse handlers ───────────────────────────────────────────────────
    def _in_disp(self, x: int, y: int) -> bool:
        ox, oy, dw, dh = self._disp
        return ox <= x <= ox + dw and oy <= y <= oy + dh

    def _to_frame(self, x: int, y: int) -> tuple[int, int]:
        ox, oy, dw, dh = self._disp
        fw, fh = self._frame_wh
        if dw == 0 or dh == 0:
            return 0, 0
        fx = (x - ox) / dw * fw
        fy = (y - oy) / dh * fh
        fx = max(0, min(fw - 1, fx))
        fy = max(0, min(fh - 1, fy))
        return int(fx), int(fy)

    def _on_press(self, e) -> None:
        # In MOT mode, a click on a detected (cyan) box locks that object instead
        # of starting a drag-selection.
        if self._mot and self._select_multi_at(e.x, e.y):
            return
        if not self._in_disp(e.x, e.y):
            return
        self._drag_start = (e.x, e.y)
        if self._rect_id is not None:
            self.canvas.delete(self._rect_id)
        self._rect_id = self.canvas.create_rectangle(
            e.x, e.y, e.x, e.y, outline="#22d3ee", width=2, dash=(5, 3))

    def _on_drag(self, e) -> None:
        if self._drag_start is None or self._rect_id is None:
            return
        x0, y0 = self._drag_start
        ox, oy, dw, dh = self._disp
        x = min(max(e.x, ox), ox + dw)
        y = min(max(e.y, oy), oy + dh)
        self.canvas.coords(self._rect_id, x0, y0, x, y)

    def _on_release(self, e) -> None:
        if self._drag_start is None:
            return
        x0, y0 = self._drag_start
        self._drag_start = None
        ox, oy, dw, dh = self._disp
        x1 = min(max(e.x, ox), ox + dw)
        y1 = min(max(e.y, oy), oy + dh)
        cx0, cy0 = min(x0, x1), min(y0, y1)
        cx1, cy1 = max(x0, x1), max(y0, y1)
        if cx1 - cx0 < 5 or cy1 - cy0 < 5:
            self._clear_roi()
            return
        fx0, fy0 = self._to_frame(cx0, cy0)
        fx1, fy1 = self._to_frame(cx1, cy1)
        x, y, w, h = fx0, fy0, fx1 - fx0, fy1 - fy0
        if w < 5 or h < 5:
            self._clear_roi()
            return
        self.roi = (x, y, w, h)
        if self.ctl is not None:
            self._start_track()   # auto-start tracking as soon as the box is set
        else:
            self._status(f"ROI selected: x={x} y={y} w={w} h={h} — "
                         "Connect first to start tracking.")

    def _clear_roi(self) -> None:
        self.roi = None
        if self._rect_id is not None:
            self.canvas.delete(self._rect_id)
            self._rect_id = None
        self._status("ROI cleared.")

    def _hide_roi_rect(self) -> None:
        """Remove the blue dashed selection rectangle from the canvas but keep
        self.roi (used by Center-on-ROI). Called right after a track command so
        the device's response box (green overlay / any burned-in box) is what you
        see, not your own selection."""
        if self._rect_id is not None:
            self.canvas.delete(self._rect_id)
            self._rect_id = None

    # ── ROI nudge (WASD) ─────────────────────────────────────────────────────
    def _roi_nudge_break(self, key: str) -> str:
        """Same as _roi_nudge, but returns 'break' so a focused widget's own
        bindings (e.g. Combobox type-ahead) don't also fire."""
        self._roi_nudge(key)
        return "break"

    def _roi_nudge(self, key: str) -> None:
        if self.roi is None:
            return
        fw, fh = self._frame_wh
        if fw == 0:
            return
        # Sync to the firmware's own live tracked box first, if fresh — nudge
        # should correct the ACTUAL current lock, not the stale mouse-drawn
        # rectangle from whenever the box was first drawn.
        if self.ctl is not None:
            box = self.ctl.state.get("track_box")
            ts = self.ctl.state.get("track_box_ts", 0.0)
            if (box and box[0] > TRACK_NO_TARGET and box[1] > TRACK_NO_TARGET
                    and (time.time() - ts) <= BOX_STALE_S):
                self.roi = box
        x, y, w, h = self.roi
        dx = {"a": -1, "d": 1}.get(key, 0) * ROI_NUDGE_STEP_PX
        dy = {"w": -1, "s": 1}.get(key, 0) * ROI_NUDGE_STEP_PX
        x = max(0, min(fw - w, x + dx))
        y = max(0, min(fh - h, y + dy))
        self.roi = (x, y, w, h)
        ox, oy, dw, dh = self._disp
        sx = ox + x / fw * dw
        sy = oy + y / fh * dh
        ex = ox + (x + w) / fw * dw
        ey = oy + (y + h) / fh * dh
        if self._rect_id is not None:
            self.canvas.coords(self._rect_id, sx, sy, ex, ey)
        else:
            self._rect_id = self.canvas.create_rectangle(
                sx, sy, ex, ey, outline="#22d3ee", width=2, dash=(5, 3))
        self._status(f"ROI nudged: x={x} y={y} w={w} h={h}")
        # Debounce the actual retrack so holding a key doesn't flood the
        # device with a track-start command on every OS key-repeat tick.
        if self._roi_nudge_after_id is not None:
            self.root.after_cancel(self._roi_nudge_after_id)
        self._roi_nudge_after_id = self.root.after(
            ROI_NUDGE_DEBOUNCE_MS, self._roi_nudge_retrack)

    def _roi_nudge_retrack(self) -> None:
        self._roi_nudge_after_id = None
        if self.ctl is not None and self.roi is not None:
            self._start_track()

    # ── tracking ─────────────────────────────────────────────────────────────
    def _start_track(self) -> None:
        if self.ctl is None or self.roi is None:
            self._status("Select a ROI first (drag on the video).")
            return
        # self.roi is already in stream-pixel space (the decoded RTSP frame IS the
        # wide stream, ~1920x1080). start_track_roi() appends the CAPTURE-VERIFIED
        # 5th field (=1) the app sends; together with the V3 camera bring-up done
        # on connect, this is the recipe that actually locks.
        x, y, w, h = self.roi
        ok = self.ctl.start_track_roi(x, y, w, h)   # 5th field added in controller
        self._tracking = ok
        if ok:
            self._hide_roi_rect()   # clear the blue dashed box so the response box shows
        self._status(
            f"Track ROI sent ({x},{y},{w},{h}) +f5=1. Blue cleared; watch for green."
            if ok else "Send failed — not connected.")

    def _start_ai_track(self) -> None:
        """EXPERIMENTAL DWARF3 V3 MOT path (the real subject-tracking pipeline).

        Enables the 30-class object detector (CMD_WIDE_TELE_TRACK_SWITCH 14809)
        and multi-object tracking (CMD_MOT_START 14804). The device then streams
        detected objects on 15238/15251 — drawn here in CYAN with their id. Click
        a cyan box to lock that object (CMD_MOT_WIDE_TRACK_ONE 14808 {id}); the
        locked target then reports on 15252 (green box) and auto-center drives it.

        UNVERIFIED end-to-end: the lab device went offline during development, and
        the 15238/15251 box/id field layout is best-effort (see
        dwarflab_controller._parse_multi_track). Drive deliberately and watch the
        hardware — sending V3 commands out of order destabilised the device once.
        """
        if self.ctl is None:
            self._status("Connect first.")
            return
        self.ctl.set_master_lock(True)
        cam = CAMERAS.get(self.cam_var.get())
        self.ctl.wide_tele_track_switch(1 if cam == "ch1" else 0)  # enable detector
        self.ctl.start_mot()                                       # start MOT
        self._mot = True
        self._hide_roi_rect()
        self._status("MOT detector started — click a CYAN detected box to lock it.")

    def _stop_track(self) -> None:
        self._tracking = False
        self._mot = False
        self._multi_disp = []
        self.canvas.delete("multibox")
        if self.ctl is not None:
            if self._moving:
                self.ctl.joystick_stop()
                self._moving = False
            self.ctl.stop_tracking()
            self._status("Stop-track sent; motors stopped.")

    def _on_loop_toggle(self) -> None:
        if not self.loop_var.get() and self.ctl is not None and self._moving:
            self.ctl.joystick_stop()
            self._moving = False
            self._status("Auto-center off — motors stopped (tracker still running).")

    # ── firmware auto-track modes (bird / airplane / UFO) ────────────────────
    def _start_auto_mode(self, name: str) -> None:
        if self.ctl is None:
            self._status("Connect first.")
            return
        spec = AUTO_MODES[name]
        # stop any software-driven motion so we don't fight the firmware
        self._tracking = False
        if self._moving:
            self.ctl.joystick_stop()
            self._moving = False
        # (re)assert host lock, then start the firmware auto-track profile
        self.ctl.set_master_lock(True)
        if spec["kind"] == "ufo":
            self.ctl.set_ufo_hand_auto(True)        # automatic target selection
            ok = self.ctl.start_ufo_mode(spec["mode"])
        else:
            ok = self.ctl.start_sentry_mode(spec["mode"])
        self.mode_var.set(name if ok else "")
        for n, b in self.mode_btns.items():
            b.config(relief=(tk.SUNKEN if n == name and ok else tk.RAISED))
        self._status(
            f"{name} auto-track started (mode={spec['mode']}). Firmware is "
            "scanning; it will slew to a detected target on its own."
            if ok else "Send failed — not connected.")

    def _stop_auto_mode(self) -> None:
        if self.ctl is None:
            return
        self.ctl.stop_ufo_mode()
        self.ctl.stop_sentry_mode()
        self.mode_var.set("")
        for b in self.mode_btns.values():
            b.config(relief=tk.RAISED)
        self._status("Auto-track stopped.")

    # ── focus controls (operate on the currently selected camera) ────────────
    def _auto_focus(self) -> None:
        if self.ctl is None:
            return
        cam = self.cam_var.get()
        wide = CAMERAS.get(cam) == "ch1"
        # Area-focus on the selected ROI centre (in the current feed's frame
        # pixel coords) so autofocus targets what you're looking at; fall back
        # to global focus when no ROI is selected.
        if self.roi is not None:
            x, y, w, h = self.roi
            cx, cy = int(x + w / 2), int(y + h / 2)
            self.ctl.auto_focus(cx, cy)
            where = f"area @ ({cx},{cy})"
        else:
            self.ctl.auto_focus()
            where = "global"
        if wide:
            self._status(
                f"Auto focus ({where}) sent — note: the wide-angle lens is "
                "fixed-focus; the focuser only moves the telephoto optics.")
        else:
            self._status(f"Auto focus ({where}) triggered on {cam}.")

    def _focus_step(self, direction: int) -> None:
        if self.ctl is None:
            return
        self.ctl.focus_step(direction)
        self._status(f"Focus nudged {'far' if direction > 0 else 'near'} "
                     f"(1 step) on {self.cam_var.get()}.")

    def _focus_near_press(self, _e=None) -> None:
        if self.ctl is None:
            return
        self.ctl.focus_in()                       # continuous toward near
        self._status("Focusing near… release to stop.")

    def _focus_far_press(self, _e=None) -> None:
        if self.ctl is None:
            return
        self.ctl.focus_out()                      # continuous toward far
        self._status("Focusing far… release to stop.")

    def _focus_release(self, _e=None) -> None:
        if self.ctl is None:
            return
        self.ctl.focus_stop()
        self._status("Focus stopped.")

    # ── direct slew (open-loop Center-on-ROI) ────────────────────────────────
    def _center_on_roi(self) -> None:
        """Slew the mount so the selected ROI centre moves to the frame centre.

        Open-loop: drives the joystick in the offset direction for a time
        proportional to how far off-centre the ROI is, then stops. Does not
        depend on the firmware visual tracker, so it always moves the motors.
        """
        if self.ctl is None:
            self._status("Connect first.")
            return
        if self.roi is None:
            self._status("Drag a box on the video first, then Center on ROI.")
            return
        if self._slewing:
            return
        fw, fh = self._frame_wh
        if fw == 0:
            self._status("No video yet — wait for the live feed.")
            return
        x, y, w, h = self.roi
        nx = (x + w / 2.0 - fw / 2.0) / (fw / 2.0)   # -1..1, +right
        ny = (y + h / 2.0 - fh / 2.0) / (fh / 2.0)   # -1..1, +down
        mag = math.hypot(nx, ny)
        if mag < SLEW_MIN_OFF:
            self._status("ROI already centred — nothing to slew.")
            return
        is_wide = CAMERAS.get(self.cam_var.get()) == "ch1"
        if is_wide:
            speed_scale = dur_scale = 1.0
        else:
            speed_scale = TELE_SLEW_SCALE ** TELE_SLEW_SPEED_EXP
            dur_scale = TELE_SLEW_SCALE ** (1.0 - TELE_SLEW_SPEED_EXP)
        speed = SLEW_SPEED * speed_scale
        jx = AZ_SIGN * (nx / mag) * speed
        jy = ALT_SIGN * (ny / mag) * speed
        dur = SLEW_TIME_FULL * min(1.0, mag) * dur_scale  # seconds
        # stop any auto-center activity while we slew
        if self._moving:
            self.ctl.joystick_stop()
            self._moving = False
        self._slewing = True
        self._status(
            f"Slewing to ROI: dir({nx:+.2f},{ny:+.2f}) "
            f"joy({jx:+.0f},{jy:+.0f}) for {dur:.2f}s"
            + ("" if is_wide else
               f" (tele-scaled speed x{speed_scale:.3f}, dur x{dur_scale:.3f})"))
        # Soft-start: ramp 1..SLEW_RAMP_STEPS/SLEW_RAMP_STEPS of full magnitude
        # instead of snapping straight to it (matches the real app's own
        # joystick stream, which never jumps to full speed in one message).
        # Cap the ramp itself to a fraction of dur so short tele pulses don't
        # spend their *entire* duration still ramping up.
        ramp_total_ms = min(SLEW_RAMP_STEPS * SLEW_RAMP_STEP_MS, int(dur * 1000 * 0.6))
        ramp_step_ms = max(10, ramp_total_ms // SLEW_RAMP_STEPS)
        self._slew_ramp(jx, jy, 1, dur, ramp_step_ms)

    def _slew_ramp(self, jx: float, jy: float, step: int, dur: float,
                   ramp_step_ms: int) -> None:
        frac = min(1.0, step / SLEW_RAMP_STEPS)
        self.ctl.joystick(jx * frac, jy * frac)
        if step < SLEW_RAMP_STEPS:
            self.root.after(ramp_step_ms,
                             lambda: self._slew_ramp(jx, jy, step + 1, dur, ramp_step_ms))
        else:
            remaining = max(0, int(dur * 1000) - SLEW_RAMP_STEPS * ramp_step_ms)
            self.root.after(remaining, self._end_slew)

    def _end_slew(self) -> None:
        if self.ctl is not None:
            self.ctl.joystick_stop()
        self._slewing = False
        self._status("Slew complete. Drag a new box and Center on ROI to refine.")

    # ── diagnostics (raw device notify stream) ───────────────────────────────
    def _on_notify(self, pkt) -> None:
        """Runs in the WS thread; only does cheap dict updates (thread-safe)."""
        cmd = pkt.get("cmd")
        data = pkt.get("data", b"")
        d = self._diag
        d["counts"][cmd] += 1
        src = _BOX_CMDS.get(cmd)
        if src is not None:
            try:
                f = _parse_varint_fields(data)
                d["last_box"][src] = (f.get(1, -100), f.get(2, -100),
                                      f.get(3, 0), f.get(4, 0))
            except Exception:
                pass
        elif cmd == CMD_NOTIFY_MULTI_TRACK_RESULT:
            d["multi"]["tele"] += 1
        elif cmd == CMD_NOTIFY_WIDE_MULTI_TRACK_RESULT:
            d["multi"]["wide"] += 1
        elif cmd == CMD_NOTIFY_SENTRY_MODE_STATE:
            try:
                f = _parse_varint_fields(data)
                d["sentry_state"] = _SENTRY_STATE.get(f.get(1, 0), f.get(1, 0))
            except Exception:
                pass
        elif cmd == CMD_NOTIFY_UFO_MODE_STATE:
            try:
                f = _parse_varint_fields(data)
                d["ufo_state"] = _SENTRY_STATE.get(f.get(1, 0), f.get(1, 0))
            except Exception:
                pass
        elif cmd == CODE_TRACK_TRACKER_INITING:
            d["tracker"] = "INITING"
        elif cmd == CODE_TRACK_TRACKER_FAILED:
            d["tracker"] = "FAILED"

    def _diag_tick(self) -> None:
        if self.ctl is None:
            self.diag_var.set("diag: (not connected)")
        else:
            d = self._diag
            c = d["counts"]
            tele = d["last_box"]["tele"]
            wide = d["last_box"]["wide"]
            sen = d["last_box"]["sentry"]

            def fmt(b):
                if b is None:
                    return "—"
                if b[0] <= -100 or b[1] <= -100:
                    return "none"
                return f"{b[0]},{b[1]},{b[2]},{b[3]}"

            parts = [
                f"tele#{c[CMD_NOTIFY_TRACK_RESULT]}={fmt(tele)}",
                f"wide#{c[CMD_NOTIFY_WIDE_TRACK_RESULT]}={fmt(wide)}",
                f"sentry#{c[CMD_NOTIFY_SENTRY_MODE_TRACK_RESULT]}={fmt(sen)}",
                f"multi(t/w)={d['multi']['tele']}/{d['multi']['wide']}",
            ]
            if d["sentry_state"] is not None:
                parts.append(f"sentry-state={d['sentry_state']}")
            if d["ufo_state"] is not None:
                parts.append(f"ufo={d['ufo_state']}")
            if d["tracker"] is not None:
                parts.append(f"tracker={d['tracker']}")
            tstate = self.ctl.state.get("track_state")
            if tstate is not None:
                parts.append(f"wtrack_state={tstate}")   # cmd 15284 (capture-found)
            ws = "up" if self.ctl.state.get("connected") else "DOWN"
            self.diag_var.set(f"diag[ws={ws}]: " + "  ".join(parts))
        self.root.after(300, self._diag_tick)

    # ── closed-loop motor control ─────────────────────────────────────────
    @staticmethod
    def _axis_speed(n: float, gain_scale: float = 1.0) -> float:
        """Proportional speed for one axis from normalised error n in [-1, 1].

        gain_scale shrinks GAIN/MAX_SPEED for tele's narrow FOV (same ratio as
        the Center-on-ROI slew). MIN_SPEED is NOT scaled — it's the mount's
        real mechanical stiction floor, independent of which camera is
        selected. Never scaling it down means every non-deadzone tele error
        still gets a full-strength MIN_SPEED pulse; _drive_to_center handles
        that by only holding it for a fraction of the tick (duty-cycling)
        instead of the whole LOOP_MS, mirroring the slew's speed/duration split.
        """
        if abs(n) < DEADZONE:
            return 0.0
        max_speed = MAX_SPEED * gain_scale
        s = max(-max_speed, min(max_speed, GAIN * gain_scale * n))
        if 0 < abs(s) < MIN_SPEED:
            s = MIN_SPEED if s > 0 else -MIN_SPEED
        return s

    def _control_tick(self) -> None:
        if self._slewing or self._keys_down:
            self.root.after(LOOP_MS, self._control_tick)
            return
        active = (self._tracking and self.loop_var.get()
                  and self.ctl is not None)
        if active:
            self._drive_to_center()
        elif self._moving and self.ctl is not None:
            self.ctl.joystick_stop()
            self._moving = False
        self.root.after(LOOP_MS, self._control_tick)

    # ── keyboard jog (arrow keys) ────────────────────────────────────────────
    def _on_arrow_press(self, key: str) -> None:
        if key in self._keys_down:
            return   # ignore OS key-repeat while held
        self._keys_down.add(key)
        self._apply_arrow_joystick()

    def _on_arrow_release(self, key: str) -> None:
        self._keys_down.discard(key)
        self._apply_arrow_joystick()

    def _jog_press_break(self, key: str) -> str:
        """Same as _on_arrow_press, but returns 'break' to stop the event
        reaching a focused widget's own arrow-key bindings (e.g. Combobox)."""
        self._on_arrow_press(key)
        return "break"

    def _jog_release_break(self, key: str) -> str:
        self._on_arrow_release(key)
        return "break"

    def _on_arrow_focus_lost(self) -> None:
        if self._keys_down:
            self._keys_down.clear()
            self._apply_arrow_joystick()

    def _apply_arrow_joystick(self) -> None:
        if self.ctl is None:
            self._keys_down.clear()
            return
        jx = jy = 0.0
        if "Right" in self._keys_down: jx += AZ_SIGN * ARROW_KEY_SPEED
        if "Left" in self._keys_down:  jx -= AZ_SIGN * ARROW_KEY_SPEED
        if "Up" in self._keys_down:    jy += ALT_SIGN * ARROW_KEY_SPEED
        if "Down" in self._keys_down:  jy -= ALT_SIGN * ARROW_KEY_SPEED
        if jx == 0.0 and jy == 0.0:
            self.ctl.joystick_stop()
            self._moving = False
            self._status("Keyboard jog stopped.")
        else:
            self.ctl.joystick(jx, jy)
            self._moving = True
            self._status(f"Keyboard jog: joy({jx:+.0f},{jy:+.0f})")

    def _drive_to_center(self) -> None:
        box = self.ctl.state.get("track_box")
        ts = self.ctl.state.get("track_box_ts", 0.0)
        fw, fh = self._frame_wh
        stale = (time.time() - ts) > BOX_STALE_S
        lost = (not box or fw == 0 or box[0] <= TRACK_NO_TARGET
                or box[1] <= TRACK_NO_TARGET or stale)
        if lost:
            if self._moving:
                self.ctl.joystick_stop()
                self._moving = False
            self._status("tracking: searching for target (no lock) — motors idle.")
            return
        # Box coords are in stream pixels (== decoded frame size), so normalise
        # the centring error against fw/fh.
        x, y, w, h = box
        bx, by = x + w / 2.0, y + h / 2.0
        nx = (bx - fw / 2.0) / (fw / 2.0)
        ny = (by - fh / 2.0) / (fh / 2.0)
        is_wide = CAMERAS.get(self.cam_var.get()) == "ch1"
        gain_scale = 1.0 if is_wide else TELE_SLEW_SCALE ** TELE_SLEW_SPEED_EXP
        hold_scale = 1.0 if is_wide else TELE_SLEW_SCALE ** (1.0 - TELE_SLEW_SPEED_EXP)
        jx = AZ_SIGN * self._axis_speed(nx, gain_scale)
        jy = ALT_SIGN * self._axis_speed(ny, gain_scale)
        if jx == 0.0 and jy == 0.0:
            if self._moving:
                self.ctl.joystick_stop()
                self._moving = False
            self._status(f"tracking: centred (err {nx:+.2f},{ny:+.2f}) — holding.")
        else:
            self.ctl.joystick(jx, jy)
            self._moving = True
            if is_wide:
                self._status(
                    f"tracking: correcting err({nx:+.2f},{ny:+.2f}) "
                    f"joy({jx:+.0f},{jy:+.0f})")
            else:
                # Duty-cycle: MIN_SPEED isn't scaled down (it's the real
                # stiction floor), so holding it for the full LOOP_MS tick
                # would still overshoot tele's narrow FOV — only hold for a
                # tele-scaled fraction of the tick, then stop early.
                hold_ms = max(10, int(LOOP_MS * hold_scale))
                self.root.after(hold_ms, self._tele_tick_stop)
                self._status(
                    f"tracking: correcting err({nx:+.2f},{ny:+.2f}) "
                    f"joy({jx:+.0f},{jy:+.0f}) tele-hold {hold_ms}ms")

    def _tele_tick_stop(self) -> None:
        if self.ctl is not None and self._moving:
            self.ctl.joystick_stop()
            self._moving = False

    # ── capture (telephoto photo + IMU attitude snapshot) ────────────────────
    def _capture_photo(self) -> None:
        """Trigger a telephoto still on the DWARF, download it off the SD card
        via FTP (vsFTPd runs on :21, no credentials needed — confirmed by
        direct test), and log it with the mount attitude at the shutter instant.

        The photo command (CMD_CAMERA_TELE_PHOTOGRAPH / 10002) is telephoto-only,
        so it captures the tele lens regardless of which feed is previewed.
        Runs in a background thread (FTP + file I/O); reads the wit_attach Tk
        variable here on the main thread first since Python 3.12 raises if a
        background thread touches Tk state directly (see _status's comment).
        """
        if self.ctl is None:
            self._status("Connect to the DWARF first.")
            return
        attach_imu = self.wit_attach.get()
        self._status("Taking tele photo…")
        threading.Thread(target=self._capture_photo_worker, args=(attach_imu,),
                         daemon=True).start()

    def _capture_photo_worker(self, attach_imu: bool) -> None:
        ftp = None
        before = None
        try:
            ftp = ftplib.FTP()
            ftp.connect(self.ip, 21, timeout=8)
            ftp.login()
            ftp.cwd("Normal_Photos")
            before = set(ftp.nlst())
        except Exception as exc:
            self._status(f"Photo: FTP unavailable ({exc}) — taking shutter only.")

        self.ctl.take_photo()                       # CMD_CAMERA_TELE_PHOTOGRAPH
        shutter_time = datetime.now(timezone.utc)

        new_name = None
        local_path = None
        if ftp is not None and before is not None:
            deadline = time.time() + 12.0
            while time.time() < deadline and new_name is None:
                time.sleep(1.0)
                try:
                    added = set(ftp.nlst()) - before
                except Exception:
                    break
                if added:
                    new_name = sorted(added)[-1]   # newest if several appeared
            if new_name:
                try:
                    os.makedirs(self.photo_dir, exist_ok=True)
                    local_path = os.path.join(self.photo_dir, new_name)
                    with open(local_path, "wb") as fh:
                        ftp.retrbinary(f"RETR {new_name}", fh.write)
                    self.last_photo_path = local_path
                except Exception as exc:
                    self._status(f"Photo taken, but FTP download failed: {exc}")
                    local_path = None
            try:
                ftp.quit()
            except Exception:
                pass

        rec = {
            "timestamp": shutter_time.isoformat(timespec="milliseconds"),
            "camera": "tele",
            "cmd": "CMD_CAMERA_TELE_PHOTOGRAPH(10002)",
        }
        if new_name:
            rec["device_filename"] = new_name
        if local_path:
            rec["local_path"] = local_path
        att = None
        if attach_imu and self.wit is not None and self.wit.connected:
            att = self.wit.snapshot()
            if att is not None:
                rec["imu"] = att
        self._log_capture(rec)

        if local_path:
            base = f"📷 Tele photo saved: {local_path}"
        elif new_name:
            base = f"📷 Tele photo taken ({new_name}) — FTP download failed, still on device."
        else:
            base = ("📷 Tele photo taken (saved on device) — couldn't find the "
                    "new file over FTP within 12s.")
        if att is not None and att.get("calibrated"):
            self._status(f"{base}  IMU alt={att['altitude']:+.3f}° "
                         f"az={att['azimuth']:.3f}°")
        elif att is not None:
            self._status(f"{base}  IMU roll={att['roll']:+.3f} "
                         f"pitch={att['pitch']:+.3f} yaw={att['yaw']:+.3f}")
        else:
            self._status(f"{base} (no IMU attitude — connect WIT IMU to capture angles).")

    def _log_capture(self, rec: dict) -> None:
        try:
            with open(self.capture_log, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(rec) + "\n")
        except Exception as exc:
            self._status(f"Photo taken but capture-log write failed: {exc}")

    # ── photo matching (sub-pixel registration vs a reference photo) ─────────
    def _find_reference_photo(self) -> str | None:
        for c in (self.reference_photo,
                 os.path.join(self.photo_dir, self.reference_photo)):
            if os.path.isfile(c):
                return c
        return None

    def _find_latest_photo(self) -> str | None:
        try:
            files = [os.path.join(self.photo_dir, f)
                    for f in os.listdir(self.photo_dir)
                    if f.lower().endswith((".jpg", ".jpeg", ".png"))
                    and f != "match_result.jpg"]
        except FileNotFoundError:
            return None
        return max(files, key=os.path.getmtime) if files else None

    def _match_photos(self) -> None:
        """Sub-pixel registration: reference.jpg vs the most recent capture,
        via SIFT features + a RANSAC-fit similarity transform (translation +
        rotation + uniform scale). Local-file-only — works with or without
        the DWARF connected. Runs in a background thread (SIFT on a 4K image
        isn't instant)."""
        ref_path = self._find_reference_photo()
        if ref_path is None:
            self._status(f"Match: no reference photo — place one at "
                         f"'{self.reference_photo}' (repo dir or "
                         f"{self.photo_dir}/) first.")
            return
        latest_path = self.last_photo_path
        if latest_path is None or not os.path.isfile(latest_path):
            latest_path = self._find_latest_photo()
        if latest_path is None:
            self._status(f"Match: no photo found in '{self.photo_dir}/' — "
                         "take one with Capture first.")
            return
        if os.path.abspath(latest_path) == os.path.abspath(ref_path):
            self._status("Match: latest capture IS the reference photo — "
                         "take a new one first.")
            return
        self._status(f"Matching {os.path.basename(latest_path)} against "
                     f"{os.path.basename(ref_path)}…")
        threading.Thread(target=self._match_photos_worker,
                         args=(ref_path, latest_path), daemon=True).start()

    def _match_photos_worker(self, ref_path: str, latest_path: str) -> None:
        ref = cv2.imread(ref_path, cv2.IMREAD_GRAYSCALE)
        cur = cv2.imread(latest_path, cv2.IMREAD_GRAYSCALE)
        if ref is None or cur is None:
            self._status("Match: failed to load one of the images.")
            return

        sift = cv2.SIFT_create()
        kp1, des1 = sift.detectAndCompute(ref, None)
        kp2, des2 = sift.detectAndCompute(cur, None)
        if des1 is None or des2 is None or len(kp1) < 4 or len(kp2) < 4:
            self._status("Match: not enough SIFT features detected "
                         "(low-contrast or featureless image?).")
            return

        bf = cv2.BFMatcher(cv2.NORM_L2)
        knn = bf.knnMatch(des1, des2, k=2)
        good = [m for m, n in knn if m.distance < 0.75 * n.distance]  # Lowe's ratio
        if len(good) < 4:
            self._status(f"Match: only {len(good)} good feature matches "
                         "(need >= 4 for a fit) — images may not overlap.")
            return

        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        # RANSAC-fit similarity transform (translation + rotation + uniform
        # scale — no perspective term, since both photos are from the same
        # fixed optical setup). Fitting over many correspondences yields a
        # sub-pixel-accurate translation even though each individual SIFT
        # keypoint is only pixel-ish precise — the same principle astrometric
        # plate-solving relies on for centroid accuracy.
        M, inlier_mask = cv2.estimateAffinePartial2D(
            src_pts, dst_pts, method=cv2.RANSAC, ransacReprojThreshold=3.0,
            maxIters=5000, confidence=0.995)
        if M is None:
            self._status("Match: RANSAC found no consistent transform "
                         "(images may not actually overlap).")
            return

        n_inliers = int(inlier_mask.sum()) if inlier_mask is not None else 0
        dx, dy = float(M[0, 2]), float(M[1, 2])
        scale = float(math.hypot(M[0, 0], M[1, 0]))
        rot_deg = float(math.degrees(math.atan2(M[1, 0], M[0, 0])))

        lines_path, overlay_path = self._save_match_visuals(
            ref_path, latest_path, ref, cur, kp1, kp2, good, inlier_mask, M)

        rec = {
            "timestamp": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "reference": ref_path, "latest": latest_path,
            "matches_total": len(good), "inliers": n_inliers,
            "dx_px": round(dx, 4), "dy_px": round(dy, 4),
            "rotation_deg": round(rot_deg, 5), "scale": round(scale, 6),
            "match_lines_image": lines_path, "overlay_image": overlay_path,
        }
        try:
            with open(self.match_log, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(rec) + "\n")
        except Exception:
            pass

        self._status(
            f"🎯 Match: dx={dx:+.3f}px dy={dy:+.3f}px rot={rot_deg:+.4f}° "
            f"scale={scale:.5f}  ({n_inliers}/{len(good)} inliers)"
            + (f"  → {overlay_path}" if overlay_path else ""))

    def _save_match_visuals(self, ref_path, latest_path, ref, cur, kp1, kp2,
                            good, inlier_mask, M):
        """Two diagnostic images, both downscaled to something actually
        legible (the raw full-res drawMatches plot with 1000+ lines is just
        visual noise):
          match_lines.jpg   — a small SAMPLE of inlier match lines only.
          match_overlay.jpg — reference in the red channel, the latest photo
                               WARPED onto the reference's frame in the green
                               channel. Well-aligned content reads as
                               grey/yellow; misaligned edges fringe red/green
                               — much more direct evidence of fit quality
                               than a line plot.
        """
        os.makedirs(self.photo_dir, exist_ok=True)
        lines_path = overlay_path = None

        try:
            mask = inlier_mask.ravel().tolist() if inlier_mask is not None else None
            inliers = [m for m, keep in zip(good, mask) if keep] if mask else good
            sample = inliers[:: max(1, len(inliers) // 40)][:40]   # ~40 lines, evenly spread
            vis = cv2.drawMatches(
                ref, kp1, cur, kp2, sample, None,
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            vh, vw = vis.shape[:2]
            target_w = 1600
            if vw > target_w:
                vis = cv2.resize(vis, (target_w, int(vh * target_w / vw)))
            lines_path = os.path.join(self.photo_dir, "match_lines.jpg")
            cv2.imwrite(lines_path, vis)
        except Exception:
            lines_path = None

        try:
            M_inv = cv2.invertAffineTransform(M)
            warped_cur = cv2.warpAffine(cur, M_inv, (ref.shape[1], ref.shape[0]))
            overlay = np.zeros((*ref.shape, 3), dtype=np.uint8)
            overlay[..., 2] = ref          # red   = reference
            overlay[..., 1] = warped_cur   # green = latest, warped to align
            oh, ow = overlay.shape[:2]
            target_w = 1600
            if ow > target_w:
                overlay = cv2.resize(overlay, (target_w, int(oh * target_w / ow)))
            overlay_path = os.path.join(self.photo_dir, "match_overlay.jpg")
            cv2.imwrite(overlay_path, overlay)
        except Exception:
            overlay_path = None

        return lines_path, overlay_path

    # ── live matching (tele feed vs reference.jpg, continuous) ───────────────
    def _set_live_status(self, msg: str) -> None:
        # Called from the background match thread — Python 3.12 raises if Tk
        # state is touched off the main thread, same as _status().
        self.root.after(0, lambda: self.live_match_status_var.set(msg))

    def _toggle_live_match(self) -> None:
        if not self.live_match_var.get():
            self.live_match_status_var.set("Live match: off")
            self._clear_live_marker()
            self._live_ref_kp = None
            self._live_ref_des = None
            self._live_ref_shape = None
            self._live_ref_gray = None
            return
        ref_path = self._find_reference_photo()
        if ref_path is None:
            self.live_match_var.set(False)
            self._status(f"Live match: no reference photo — place one at "
                         f"'{self.reference_photo}' first.")
            return
        ref = cv2.imread(ref_path, cv2.IMREAD_GRAYSCALE)
        if ref is None:
            self.live_match_var.set(False)
            self._status(f"Live match: failed to load {ref_path}.")
            return
        sift = cv2.SIFT_create()
        kp, des = sift.detectAndCompute(ref, None)
        if des is None or len(kp) < 4:
            self.live_match_var.set(False)
            self._status("Live match: not enough SIFT features in the "
                         "reference photo.")
            return
        # Cached once here — only the live frame needs re-detecting each tick.
        self._live_ref_kp = kp
        self._live_ref_des = des
        self._live_ref_shape = ref.shape   # (h, w)
        self._live_ref_gray = ref
        self.live_match_status_var.set(
            f"Live match: on ({os.path.basename(ref_path)}) — searching…")
        self._ensure_match_window()

    def _get_tele_frame(self) -> tuple[np.ndarray | None, bool]:
        """Always returns the TELE stream's frame, regardless of which camera
        is currently selected as primary — live match targets tele always.
        Returns (frame, is_primary) so the crosshair lands on whichever
        canvas is actually showing tele right now (primary or the secondary
        preview pane)."""
        if self._is_wide_selected():
            return self._latest_frame2, False   # tele is the secondary pane
        return self._latest_frame, True         # tele is the primary pane

    def _live_match_tick(self) -> None:
        tele_frame, is_primary = self._get_tele_frame()
        ready = (self.live_match_var.get() and not self._live_match_busy
                and self._live_ref_des is not None and tele_frame is not None)
        if ready:
            self._live_match_busy = True
            threading.Thread(target=self._live_match_worker,
                             args=(tele_frame, is_primary), daemon=True).start()
        self.root.after(LIVE_MATCH_INTERVAL_MS, self._live_match_tick)

    def _live_match_worker(self, frame: np.ndarray, is_primary: bool) -> None:
        t0 = time.time()

        def elapsed_ms() -> float:
            return (time.time() - t0) * 1000.0

        try:
            cur = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            sift = cv2.SIFT_create()
            kp2, des2 = sift.detectAndCompute(cur, None)
            if des2 is None or len(kp2) < 4:
                self._set_live_status(
                    f"Live match: no features in current frame ({elapsed_ms():.0f}ms)")
                self.root.after(0, self._clear_live_marker)
                return

            bf = cv2.BFMatcher(cv2.NORM_L2)
            knn = bf.knnMatch(self._live_ref_des, des2, k=2)
            good = [m for m, n in knn if m.distance < 0.75 * n.distance]
            if len(good) < 4:
                self._set_live_status(
                    f"Live match: only {len(good)} matches — no lock "
                    f"({elapsed_ms():.0f}ms)")
                self.root.after(0, self._clear_live_marker)
                return

            src_pts = np.float32(
                [self._live_ref_kp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32(
                [kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
            M, inlier_mask = cv2.estimateAffinePartial2D(
                src_pts, dst_pts, method=cv2.RANSAC, ransacReprojThreshold=3.0,
                maxIters=2000, confidence=0.99)
            if M is None:
                self._set_live_status(
                    f"Live match: RANSAC found no consistent transform — "
                    f"no lock ({elapsed_ms():.0f}ms)")
                self.root.after(0, self._clear_live_marker)
                return

            n_inliers = int(inlier_mask.sum()) if inlier_mask is not None else 0
            dx, dy = float(M[0, 2]), float(M[1, 2])
            scale = float(math.hypot(M[0, 0], M[1, 0]))
            rot_deg = float(math.degrees(math.atan2(M[1, 0], M[0, 0])))

            # Where does the reference frame's CENTER land in the current
            # live frame? That's the crosshair target to slew/nudge toward.
            ref_h, ref_w = self._live_ref_shape
            ref_cx, ref_cy = ref_w / 2.0, ref_h / 2.0
            cur_x = M[0, 0] * ref_cx + M[0, 1] * ref_cy + M[0, 2]
            cur_y = M[1, 0] * ref_cx + M[1, 1] * ref_cy + M[1, 2]

            total_ms = elapsed_ms()
            self._set_live_status(
                f"Live match: dx={dx:+.2f}px dy={dy:+.2f}px "
                f"rot={rot_deg:+.3f}° scale={scale:.4f} "
                f"({n_inliers}/{len(good)} inliers, {total_ms:.0f}ms)")
            self.root.after(0, lambda: self._draw_live_marker(cur_x, cur_y, is_primary))

            # Small sampled match-lines image for the popup window — built
            # here (background thread; pure numpy/cv2, no Tk) then handed to
            # the main thread to convert to a PhotoImage and display.
            vis = self._build_match_lines_thumb(
                self._live_ref_gray, self._live_ref_kp, cur, kp2, good, inlier_mask)
            self.root.after(0, lambda: self._update_match_window(
                vis, total_ms, rot_deg, dx, dy, n_inliers, len(good)))
        except Exception as exc:
            self._set_live_status(f"Live match: error ({exc}) ({elapsed_ms():.0f}ms)")
        finally:
            self._live_match_busy = False

    def _build_match_lines_thumb(self, ref, ref_kp, cur, cur_kp, good, inlier_mask):
        """Small in-memory (no disk write — this runs every ~600ms) match-lines
        image: reference | current, with a sampled set of inlier lines."""
        mask = inlier_mask.ravel().tolist() if inlier_mask is not None else None
        inliers = [m for m, keep in zip(good, mask) if keep] if mask else good
        sample = inliers[:: max(1, len(inliers) // 30)][:30]
        vis = cv2.drawMatches(ref, ref_kp, cur, cur_kp, sample, None,
                              flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        vh, vw = vis.shape[:2]
        target_w = 760
        if vw > target_w:
            vis = cv2.resize(vis, (target_w, int(vh * target_w / vw)))
        return vis

    def _draw_live_marker(self, fx: float, fy: float, is_primary: bool) -> None:
        """Crosshair, in canvas coords, at the live-frame pixel where the
        reference photo's center currently falls — main-thread only (canvas).
        Drawn on whichever canvas is actually showing tele right now."""
        canvas = self.canvas if is_primary else self.canvas2
        fw, fh = self._frame_wh if is_primary else self._frame_wh2
        disp = self._disp if is_primary else self._disp2
        canvas.delete("livematch")
        if fw == 0:
            return
        ox, oy, dw, dh = disp
        sx = ox + fx / fw * dw
        sy = oy + fy / fh * dh
        r = 14
        canvas.create_line(sx - r, sy, sx + r, sy, fill="#22d3ee",
                           width=2, tags="livematch")
        canvas.create_line(sx, sy - r, sx, sy + r, fill="#22d3ee",
                           width=2, tags="livematch")
        canvas.create_oval(sx - r, sy - r, sx + r, sy + r,
                           outline="#22d3ee", width=2, tags="livematch")

    def _clear_live_marker(self) -> None:
        self.canvas.delete("livematch")
        self.canvas2.delete("livematch")

    # ── match details popup window ────────────────────────────────────────
    def _ensure_match_window(self) -> None:
        if self.match_win is not None and self.match_win.winfo_exists():
            return
        win = tk.Toplevel(self.root)
        win.title("Match Details")
        win.configure(bg="#1a1a1f")
        win.geometry("800x540")
        self.match_win = win
        self.match_win_canvas = tk.Canvas(win, bg="#0e0e12", highlightthickness=0)
        self.match_win_canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True,
                                   padx=6, pady=6)
        info = tk.Frame(win, bg="#1a1a1f")
        info.pack(side=tk.TOP, fill=tk.X, padx=6, pady=(0, 8))
        self.match_win_time_var = tk.StringVar(value="Match time: —")
        self.match_win_angle_var = tk.StringVar(value="Relative angle: —")
        self.match_win_offset_var = tk.StringVar(value="Offset: —")
        for var in (self.match_win_time_var, self.match_win_angle_var,
                   self.match_win_offset_var):
            tk.Label(info, textvariable=var, fg="#ddd", bg="#1a1a1f",
                    font=("Consolas", 10)).pack(side=tk.LEFT, padx=(0, 20))
        self._match_win_photo = None   # keep a reference alive

        def _on_close():
            win.destroy()
            self.match_win = None

        win.protocol("WM_DELETE_WINDOW", _on_close)

    def _update_match_window(self, vis_bgr, elapsed_ms, rot_deg, dx, dy,
                             n_inliers, n_good) -> None:
        self._ensure_match_window()
        rgb = cv2.cvtColor(vis_bgr, cv2.COLOR_BGR2RGB)
        self._match_win_photo = ImageTk.PhotoImage(Image.fromarray(rgb))
        self.match_win_canvas.delete("vis")
        self.match_win_canvas.create_image(0, 0, anchor=tk.NW,
                                           image=self._match_win_photo, tags="vis")
        self.match_win_time_var.set(f"Match time: {elapsed_ms:.0f} ms")
        self.match_win_angle_var.set(f"Relative angle: {rot_deg:+.4f}°")
        self.match_win_offset_var.set(
            f"Offset: dx={dx:+.2f}px dy={dy:+.2f}px  ({n_inliers}/{n_good} inliers)")

    # ── WitMotion IMU (attitude capture) ─────────────────────────────────────
    def _toggle_wit(self) -> None:
        if self.wit is not None and self.wit.active:
            self.wit.stop()
            self.wit = None
            self.btn_wit.config(text="Connect WIT IMU")
            self.wit_var.set("IMU: (disconnected)")
            self._status("WIT IMU disconnected.")
            return
        self.wit = WitMonitor(self._status, address=self.wit_address,
                              name=self.wit_name)
        self.wit.start()
        self.btn_wit.config(text="Disconnect WIT")
        self._status("Connecting to WIT IMU over Bluetooth…")

    def _wit_tick(self) -> None:
        w = self.wit
        if w is None:
            self.wit_var.set("IMU: (not connected)")
        elif not w.active and not w.connected:
            # background thread ended (connect failed, missing bleak, or stopped)
            self.wit_var.set(f"IMU: {w.error or 'disconnected'}")
            self.btn_wit.config(text="Connect WIT IMU")
            self.wit = None
        elif not w.connected:
            self.wit_var.set("IMU: connecting…")
        else:
            att = w.snapshot()
            if att is None:
                self.wit_var.set("IMU: connected — waiting for data…")
            else:
                cal = "cal" if att.get("calibrated") else "raw"
                line = (f"IMU[{cal}]: roll={att['roll']:+.3f}  "
                        f"pitch={att['pitch']:+.3f}  yaw={att['yaw']:+.3f}  "
                        f"(age {att['age_s']:.1f}s)")
                if att.get("calibrated"):
                    line += (f"   →   alt={att['altitude']:+.3f}°  "
                             f"az={att['azimuth']:.3f}°")
                self.wit_var.set(line)
        self.root.after(200, self._wit_tick)

    def _on_close(self) -> None:
        if self.wit is not None:
            self.wit.stop()
        self._disconnect()
        self.root.destroy()


# ── Listen-only dual-pane viewer ───────────────────────────────────────────────
class ListenOnlyApp:
    """Pure video viewer: both RTSP streams (tele + wide) side by side, no WS
    connection at all. The control WebSocket (:9900) is exclusive at the
    transport level — field-tested: even connecting without requesting the
    host lock or sending any command still gets DEVICE_OCCUPIED-bumped while
    another client (e.g. the phone app) holds it. RTSP video has no such
    exclusivity (confirmed by direct probing), so this never competes with
    whatever else is driving the mount.
    """

    PANES = (("ch0", "Tele"), ("ch1", "Wide"))

    def __init__(self, root: tk.Tk, ip: str) -> None:
        self.root = root
        self.ip = ip
        self.grabbers: dict[str, FrameGrabber] = {}
        self.queues: dict[str, "queue.Queue[np.ndarray]"] = {}
        self.canvases: dict[str, tk.Canvas] = {}
        self.photos: dict[str, ImageTk.PhotoImage | None] = {}
        self.statuses: dict[str, str] = {k: "" for k, _ in self.PANES}

        root.title(f"DWARF 3 — Listen Only ({ip})")
        root.geometry("1600x720")
        root.configure(bg="#1a1a1f")

        tk.Label(root, text=f"{ip} — video only, no commands sent "
                             "(drive the mount from the phone app)",
                 fg="#ddd", bg="#1a1a1f").pack(side=tk.TOP, anchor=tk.W, padx=8, pady=6)

        panes = tk.Frame(root, bg="#1a1a1f")
        panes.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4, pady=4)

        for key, label in self.PANES:
            pane = tk.Frame(panes, bg="#1a1a1f")
            pane.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)
            tk.Label(pane, text=label, fg="#ddd", bg="#1a1a1f").pack(side=tk.TOP, anchor=tk.W)
            cv = tk.Canvas(pane, bg="#0e0e12", highlightthickness=0)
            cv.pack(fill=tk.BOTH, expand=True)
            self.canvases[key] = cv
            self.photos[key] = None
            self.queues[key] = queue.Queue(maxsize=2)

        self.status_var = tk.StringVar(value="connecting...")
        tk.Label(root, textvariable=self.status_var, fg="#9cf", bg="#1a1a1f",
                 anchor=tk.W).pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=(0, 6))

        # Start the grabber threads only after status_var exists — their
        # status callback fires from another thread almost immediately.
        for key, _label in self.PANES:
            url = f"rtsp://{ip}/{key}/stream0"
            grabber = FrameGrabber(url, self.queues[key],
                                    lambda s, k=key: self._on_status(k, s))
            grabber.start()
            self.grabbers[key] = grabber

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._poll()

    def _on_status(self, key: str, s: str) -> None:
        # Called from a FrameGrabber worker thread — marshal onto the Tk
        # main thread (Python 3.12 raises if Tcl is touched from elsewhere).
        self.root.after(0, self._apply_status, key, s)

    def _apply_status(self, key: str, s: str) -> None:
        self.statuses[key] = s
        self.status_var.set("   |   ".join(
            f"{dict(self.PANES)[k]}: {v}" for k, v in self.statuses.items()))

    def _poll(self) -> None:
        for key, cv in self.canvases.items():
            try:
                frame = self.queues[key].get_nowait()
            except queue.Empty:
                frame = None
            if frame is not None:
                self._show(key, cv, frame)
        self.root.after(30, self._poll)

    def _show(self, key: str, cv: tk.Canvas, frame: np.ndarray) -> None:
        cw = max(1, cv.winfo_width())
        ch = max(1, cv.winfo_height())
        fh, fw = frame.shape[:2]
        scale = min(cw / fw, ch / fh)
        dw, dh = max(1, int(fw * scale)), max(1, int(fh * scale))
        ox, oy = (cw - dw) // 2, (ch - dh) // 2
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        photo = ImageTk.PhotoImage(Image.fromarray(rgb).resize((dw, dh)))
        self.photos[key] = photo   # keep a reference alive
        cv.delete("video")
        cv.create_image(ox, oy, anchor=tk.NW, image=photo, tags="video")

    def _on_close(self) -> None:
        for g in self.grabbers.values():
            g.stop()
        self.root.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description="DWARF 3 live ROI tracker GUI")
    parser.add_argument("--ip", default="192.168.1.102", help="DWARF 3 IP address")
    parser.add_argument("--wit-address", default=None,
                        help="WitMotion IMU BLE address (else auto-scan on connect)")
    parser.add_argument("--wit-name", default=None,
                        help="WitMotion IMU BLE name substring to match")
    parser.add_argument("--listen-only", action="store_true",
                         help="Video-only dual-pane viewer (tele + wide side by "
                              "side): never connects the control WebSocket, sends "
                              "no commands, and cannot conflict with another "
                              "client (e.g. the phone app) driving the mount.")
    args = parser.parse_args()

    root = tk.Tk()
    if args.listen_only:
        ListenOnlyApp(root, args.ip)
    else:
        App(root, args.ip, wit_address=args.wit_address, wit_name=args.wit_name)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

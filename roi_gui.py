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
import math
import os
import queue
import threading
import time
import tkinter as tk
from collections import defaultdict
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


# ── Main application ──────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk, ip: str) -> None:
        self.root = root
        self.ip = ip
        self.ctl: DwarfLab | None = None
        self.grabber: FrameGrabber | None = None
        self.frame_q: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=2)

        self.roi: tuple[int, int, int, int] | None = None   # frame-pixel ROI
        self._drag_start: tuple[int, int] | None = None      # canvas coords
        self._rect_id: int | None = None
        self._photo: ImageTk.PhotoImage | None = None
        self._tracking = False   # closed loop requested (Start Track pressed)
        self._mot = False        # MOT detector running (AI Track pressed)
        self._multi_disp = []    # [(id, sx, sy, ex, ey)] clickable detected boxes
        self._moving = False     # joystick currently commanded non-zero
        self._slewing = False    # a manual Center-on-ROI slew is in progress
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

        # -- video canvas --
        self.canvas = tk.Canvas(root, bg="#0e0e12", highlightthickness=0,
                                cursor="crosshair")
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=4)
        self.canvas.bind("<ButtonPress-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.create_text(
            10, 10, anchor=tk.NW, fill="#888", tags="hint",
            text="No video — click Connect, then drag a box over the feed.")

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

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(30, self._poll_frame)
        self.root.after(LOOP_MS, self._control_tick)
        self.root.after(300, self._diag_tick)

    # -- status (updates a Tk variable; safe to call from worker threads) --
    def _status(self, msg: str) -> None:
        self.status_var.set(msg)

    def _rtsp_url(self) -> str:
        return f"rtsp://{self.ip}/{CAMERAS[self.cam_var.get()]}/stream0"

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
        self.ctl.v3_mode_switch(1)     # 16404 {3:{1:1}}
        self.ctl.v3_open_tele(1)       # 10050 {1:1}
        self.ctl.v3_open_wide()        # 12036 (empty) = open

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

    def _stop_stream(self) -> None:
        if self.grabber is not None:
            self.grabber.stop()
            self.grabber = None

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
        self.root.after(30, self._poll_frame)

    def _show_frame(self, frame: np.ndarray) -> None:
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
        self._status(f"ROI selected: x={x} y={y} w={w} h={h} — "
                     "press 'Center on ROI' to slew the mount there.")

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
    def _axis_speed(n: float) -> float:
        """Proportional speed for one axis from normalised error n in [-1, 1]."""
        if abs(n) < DEADZONE:
            return 0.0
        s = max(-MAX_SPEED, min(MAX_SPEED, GAIN * n))
        if 0 < abs(s) < MIN_SPEED:
            s = MIN_SPEED if s > 0 else -MIN_SPEED
        return s

    def _control_tick(self) -> None:
        if self._slewing:
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
        jx = AZ_SIGN * self._axis_speed(nx)
        jy = ALT_SIGN * self._axis_speed(ny)
        if jx == 0.0 and jy == 0.0:
            if self._moving:
                self.ctl.joystick_stop()
                self._moving = False
            self._status(f"tracking: centred (err {nx:+.2f},{ny:+.2f}) — holding.")
        else:
            self.ctl.joystick(jx, jy)
            self._moving = True
            self._status(
                f"tracking: correcting err({nx:+.2f},{ny:+.2f}) "
                f"joy({jx:+.0f},{jy:+.0f})")

    def _on_close(self) -> None:
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
        App(root, args.ip)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

# DWARF 3 tracking protocol — capture-verified findings

This documents what a **live packet capture** of the iOS app ↔ DWARF 3 session
proved about object tracking, and where the APK-reconstructed model in
`dwarflab_controller.py` was incomplete. Re-run the analysis on any capture with:

```
python3 dwarf_capture_decode.py capture.pcapng --boxes-csv boxes.csv
```

The capture was taken on the PC's Wi-Fi hotspot (the DWARF and the phone were
both clients), so all app↔device traffic traversed the host and was recorded with
Windows `pktmon`, then converted with `pktmon etl2pcap`.

## Transport (confirmed)

| Channel | Port | Direction | Contents |
|---|---|---|---|
| RTSP/RTP video | TCP 554 | device → phone | live H.26x stream (`ch0`=tele, `ch1`=wide) |
| Control | TCP **9900** | both | **WebSocket**; binary frames carry the WsCmd protobuf |
| Heartbeat | UDP **9900** | phone → device | protobuf `{1:1, 2:<unix_ms>, 3:"txtl"}` |

The control channel is a **WebSocket**, exactly as `dwarflab_controller`
assumes. A raw capture shows the WS framing directly: server→client frames are
unmasked (`0x82` = FIN+binary), client→server frames are masked.

## The tracking message (confirmed, matches the controller)

Each box update is `CMD_NOTIFY_WIDE_TRACK_RESULT (15252)`, module
`MODULE_NOTIFY (9)`, with the WsCmd envelope the controller already decodes:

```
f1 major=1  f2 minor=8  f3 device_id=2  f4 module=9  f5 cmd=15252  f6 type=2
f7 data = { 1:x, 2:y, 3:w, 4:h }      <- the bounding box
f8 client_id = "<uuid>.<unix_ms>.iOS"
```

In the analysed session the device pushed **417** of these. `_parse_varint_fields`
decodes `data` to `(x, y, w, h)` correctly.

### How it gets overlaid on screen
The device's onboard tracker computes the box and streams it; the phone draws it
over the RTSP video. There is **no phone-side detection** — the phone is a
display + motor-command client.

## What the repo failed to identify

### 1. `cmd 15284` — an unlisted NOTIFY (`NOTIFY_WIDE_TRACK_STATE`)
The firmware emits `cmd 15284` during a wide track, payload `{1:1, 2:1}`. It is
**absent from the WsCmd table** in the decompiled-APK model. Added as
`CMD_NOTIFY_WIDE_TRACK_STATE` with a handler that sets `state["track_state"]`.

### 2. App-level keepalives the controller never sends
The app keeps the session alive with **two** heartbeats beyond websocket-client's
protocol-level PING:
* a WebSocket **TEXT `"ping"`** frame, and
* the **UDP :9900** `{1:1, 2:<unix_ms>, 3:"txtl"}` protobuf.

If the firmware ever drops the tracker / host-lock when it only sees protocol
pings, call `DwarfLab.start_app_keepalive()` to mimic the app exactly.

### 3. Box coordinate space is a FIXED reference, not the video size
Across the session the box edges approached `x+w ≈ 1280` and `y+h ≈ 720`
(measured extents: x∈[828,1017], y∈[280,529], x+w max **1271**, y+h max **757**).
So boxes are **top-left `(x,y)` + `(w,h)` in a fixed ≈1280×720 reference**, not
normalised and not the decoded RTSP frame size. `roi_gui.py` previously scaled by
the decoded frame dimensions (`fw,fh`), which is only correct when the wide
stream is exactly 1280×720. Now it scales by `TRACK_REF_W/H` for both the overlay
and the closed-loop centring, and converts the manual ROI to the same reference
on send.

### 4. The `-100` "no-target" sentinel
When the tracker has no lock it sends `x = y = -100` (a negative varint, i.e.
`0xFFFF…FF9C`). The controller already handles this via `_to_signed`; documented
as `TRACK_NO_TARGET` and respected by the GUI.

### 5. CORRECTION — `11043` is NOT a track command (authoritative proto)
An earlier version of this doc claimed cmd `11043` (payload `{1,2}`) was a no-ROI
"wide AI track start". **That was wrong.** Cross-checking the authoritative
`dwarfAlp` protobufs + `DWARF API2.txt`:

| cmd | real meaning |
|---|---|
| `10050` | `V3ReqOpenTeleCamera {action:1=open}` |
| `12036` | `V3ReqOpenWideCamera {action:0=open,1=close}` |
| `11040` | `V3_ASTRO_GET_PARAMS` |
| `11043` | `V3_ASTRO_GET_PRESETS` (exposure presets) |
| `15267` | `V3ResNotifyModeChange {changing,mode,sub_mode}` |
| `15292` | `V3_TEMPERATURE2` |

So the commands the app sent at "create tracking" were **V3 camera bring-up +
astro param/preset reads** — the box stream was already running, which is why they
*looked* like a start. The mislabelled constant/method were removed.

### 6. Why tracking fails on the Windows host (root cause)
DWARF 3 runs **V3 firmware** and has **two different trackers**; our tools used
the wrong one without the V3 setup:

* **Wrong tracker.** `start_track_roi()` → `CMD_TRACK_START_TRACK (14800)`
  `ReqStartTrack{x,y,w,h}` is the **basic correlation tracker**. Per the vendor
  doc it locks only onto a **distinct/MOVING** target in the ROI; on a static or
  low-contrast scene it returns `CODE_TRACK_TRACKER_FAILED (-14901)` and boxes of
  `-100`. That is exactly the transient-lock-then-`-100` behaviour observed.
* **The app uses the AI MOT pipeline instead:** enable the 30-class detector
  (`CMD_WIDE_TELE_TRACK_SWITCH 14809`), `CMD_MOT_START 14804` (device auto-detects
  and streams boxes+ids on `15238`/`15251`), then `CMD_MOT_WIDE_TRACK_ONE 14808
  {id}` to lock the chosen object. This recognises people/pets/objects and locks
  reliably.
* **Missing V3 mode/camera setup.** The app opens the camera with the **V3**
  commands (`10050`/`12036`) and switches V3 mode (`16403`/`16404`, producing the
  `15267` mode-change). The repo opened the legacy `10000`/`12000` and never
  switched mode, so the AI detection pipeline that feeds tracking was never armed.

**Implemented (untested — see caution):** removed the bogus `11043` mapping;
added `CMD_V3_CAMERA_TELE/WIDE_OPEN_CAMERA`, `v3_open_tele/wide()`,
`wide_tele_track_switch()`, `start_mot()`, `mot_wide_track_one()`,
`mot_tele_track_one()`, a best-effort `_parse_multi_track()` + `multi_boxes`
state, and a GUI **"AI Track (MOT)"** button that arms the detector and lets you
click a detected (cyan) box to lock it.

> ⚠️ **HARDWARE CAUTION.** While probing the MOT/V3 sequence live, sending V3
> open-camera commands out of the app's exact order made the device emit
> `15229` (POWER_OFF) and drop off the network. **Do not blind-probe the
> hardware.** The safe way to finalise a working track is to capture the official
> app performing a *complete successful* select-and-track and replay it byte-exact
> with `dwarf_capture_decode.py`. The `15238/15251` box/id field layout is still
> unverified and must be confirmed from a populated capture.

`dwarfAlp` itself does **not** implement subject tracking (it is an ASCOM/astro
bridge; its "track" = sidereal astro tracking). Its `.proto` + `DWARF API2.txt`
are the authoritative reference used for these corrections.

### 7. CONFIRMED WORKING RECIPE (captured a 2592/2592-valid app lock)
A clean lossless capture of the official app getting a **perfect lock** (2592 of
2592 boxes valid, zero `-100`) settles it. The app uses `14800`, NOT MOT — but
with setup the repo was missing:

1. **V3 camera bring-up** (not legacy `10000`/`12000`):
   - `16404` mode switch, payload `{3:{1:1}}`  (`v3_mode_switch()`)
   - `10050` V3 open tele, `{1:1}`             (`v3_open_tele(1)`)
   - `12036` V3 open wide, **EMPTY payload** = open (`v3_open_wide()`).
     Sending `{1:1}` here = CLOSE and knocked the device offline (the POWER_OFF).
2. **`14800 ReqStartTrack` with a FIFTH field**: `{x, y, w, h, 5:1}`. The public
   proto documents only 4 fields; the app always sends `f5=1`. The 4-field form
   runs the basic correlation tracker that fails to lock.

**Coordinate space CORRECTION:** boxes/ROI are in **wide-stream pixels
(≈1920×1080)**, NOT 1280×720. Proof: the app's ROI `x=975, w=382 → x+w=1357`,
which can't fit a 1280-wide frame. The earlier "≈1280×720" reading was an
artifact of where those subjects sat. The GUI now scales the box by the **decoded
frame size (fw/fh)** — i.e. the original repo behaviour was right on coords; the
failure was the missing V3 setup + 5th field, not the scaling.

**Implemented:** `start_track_roi(x,y,w,h,field5=1)`, `v3_mode_switch()`,
`v3_open_tele()`, `v3_open_wide()` (empty=open); `roi_gui` `_open_camera()` now
does the V3 bring-up and `_start_track()` sends stream-pixel ROI + `f5=1`; box
overlay/centring reverted to `fw/fh`. Verified the wire bytes match the captured
app exactly. (Still untested live against the device after these fixes.)

### 6. Live-session confirmations (official app + exclusive-control probing)
Captured the official iOS app starting a wide track, and probed the device with
exclusive control after the app was closed:

* **Start is a SEQUENCE, not one command.** The app enables detection across both
  cameras plus the track mode, each payload `{1:1}`:
  `10050` (tele, module 1) + `12036` (wide, module 2) + `11040` + `11043`
  (module 3). `11043` alone did not engage the tracker in isolation; `14800`
  (manual ROI) did. The wide camera produces the `15252` boxes.
* **Box is NOT in the video** — user observed the GUI's green overlay only, never
  an orange box → the box is drawn from the `15252` protobuf, not burned into
  H.265. (Hypothesis tested and rejected.)
* **Control channel is single-client.** A second controller is rejected with a
  WebSocket CLOSE carrying `DEVICE_OCCUPIED`; you must close the app to take over.
* **Message `type` field:** `2` = notify, `3` = command-ack (the controller's
  0=request/1=response model is incomplete).
* **Tracker state / LED:** `WIDE_TRACK_STATE (15284) {1:state}` drives the ring
  LED — searching/lost = red/off, locked = solid green; values `{1}` (init) →
  `{3}` (tracking) observed. `15267 {2:1,3:1}` is a related camera/track sub-state.
* **Sensor parameters:** the APK's `GET_ALL_PARAMS (10036)` returns nothing on
  DWARF 3 firmware 1.5.0.1 — the parameter query/response differs from the
  decompiled-APK model and was NOT resolved. To capture real shutter/ISO/exposure
  values, record the official app while opening its camera/exposure settings panel
  (the app issues the correct query) and decode with `dwarf_capture_decode.py`.

### Why "the box might be in the video" is plausible (open question)
Because the start command carries **no ROI**, the device alone decides the
subject and draws the box. The on-screen **orange** box may therefore be **burned
into the H.265 video** (an OSD rendered before encoding), with `15252` being just
the machine-readable copy used for motor centring. This was NOT yet pixel-tested
(no H.265 decoder was available on the capture host). To settle it:

* **Decode test:** extract the wide RTSP H.265 from TCP 554 (use the full-packet
  capture, not a 512-byte-truncated one), decode one frame with ffmpeg, and check
  for an orange rectangle at the `15252` `{x,y,w,h}`. Box present in pixels →
  burned into video; clean frame → app-drawn from protobuf.
* **Quick experiment:** run `roi_gui.py` (which draws the box GREEN from `15252`)
  against the live feed. If you also see an ORANGE box → it is in the video. If
  you only see green → the app draws it from the protobuf.

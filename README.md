# dwarf3toStellarium
A simple python bridge server allowing to bridge and remote control Dwarf3 smart telescopes with Stellarium.
I created dwarf3tostellarium because due to changes in the latest dwarf3 firmware, the great dwarfium software doesn't work anymore.

Disclaimer: The server is entirely vibe-coded, therefore use with care ^^. Also I do not recommend to run this server on public networks or the internet as it doesn't have any security built in. Run it on a private WiFi.

---

Server must be on the same WiFi network as the dwarf 3. Usage of server is simple:

First install the three needed dependencies with pip:
pip install flask requests websocket-client

Then the server can be started with
python3 server.py IP-ADDESS-DWARF --lat YOUR_LAT --lon YOUR_LON --alt YOUR_ALT

Setting the lattitude/longitude is necessary for dwarf3 as this is normally done via app. You can find the lattitude/longitude of your location e.g. via Google Maps.
For example, if the dwarf3 is on its standard IP 192.168.88.1 and you're in Berlin:

python3 server.py 192.168.88.1 --lat 52.5200 --lon 13.4050 --alt 34

After starting, you should be able to check the telescope state via browser on http://localhost:5002/api/status or the dashboard at http://localhost:5002 

In case you want to set/change the location at runtime after starting:
  curl -s -X POST http://localhost:5002/api/location \
    -H 'Content-Type: application/json' \-----
    -d '{"lat": 52.5200, "lon": 13.4050, "alt": 34}'-

---
Stellarium Setup

Prerequisites:
- Stellarium 24.x or later with the Telescope Control plugin
- The bridge server running: python3 server.py <DWARF3_IP> --lat <LAT> --lon <LON>

Steps:
  1. Enable the plugin (first time only)
    - Open Stellarium → click the wrench icon (bottom toolbar) → Plugins tab
    - Find Telescope Control in the list → tick Load at startup → click Configure
    - Restart Stellarium
  2. Add the telescope
    - Press F2 (or go to Configuration Window) → Plugins tab → Telescope Control → click Configure
    - Click Add a new telescope
    - Set Name to anything, e.g. Dwarf3
    - Set Telescope controlled by → External software or remote computer
    - Set Host to localhost (or the IP of the machine running the bridge)
    - Set TCP port to 10001 (standard is 10001 or whatever --tcp-port you chose)
    - Leave Start/connect at startup ticked if desired
    - Click OK
  3. Connect
    - In the Telescope Control window, select Dwarf3 → click Connect
    - The status indicator turns green when the bridge accepts the connection
  4. Slew the telescope
    - Click any object in the sky
    - Press Ctrl+1 (or right-click → Slew telescope to → Dwarf3)
    - The bridge logs GOTO → RA … Dec … and the Dwarf3 starts its one-click goto sequence (plate-solve → slew → track)

---
Live ROI GUI — photo capture & IMU attitude

`roi_gui.py` is the Tkinter live-video window (drag a box to track). It also
lets you trigger a still and record the mount attitude:

    pip install opencv-python numpy Pillow bleak
    python3 roi_gui.py --ip 192.168.1.102

- **📷 Photo (Tele)** — takes a telephoto still on the DWARF
  (`CMD_CAMERA_TELE_PHOTOGRAPH`). The JPEG is saved to the device's own storage;
  the button also writes a timestamped line to `captures.jsonl`. This is the
  telephoto lens regardless of which feed (wide/tele) is being previewed.
- **Connect WIT IMU** — connects to the WitMotion BLE IMU independently of the
  DWARF and shows a live roll / pitch / yaw readout at the bottom of the window.
  If a `wit_calibration.json` exists (see below) it also shows corrected
  altitude / azimuth.
- **Record IMU attitude with photo** (checkbox, on by default) — when a photo is
  taken and the IMU is connected, the current attitude (angles, and calibrated
  alt/az if available) is stored in the `captures.jsonl` record for that photo.

- **Tele CV Track** — fine target-centring on the **tele** feed. The firmware's
  detector only runs on the wide camera (the tele notify stays empty), so its
  "tele tracking" is really wide tracking and can only centre to the wide
  camera's coarse resolution — the wide dead-zone (±1.8°) is wider than the whole
  tele FOV (±1.7°), so the target can sit at the tele edge and still be called
  "centred". This button instead runs an OpenCV object tracker on the tele image:
  switch to Tele (ch0), drag a box over the target, click **Tele CV Track**, and
  the mount is driven from tele-pixel error (~17× finer). The tracked box is
  drawn in magenta. Stop Track (or leaving the tele feed) ends it. Needs an
  OpenCV build with the object trackers (CSRT/KCF/MIL — in `opencv-python`
  ≥4.5.1). The `CV_*` tuning constants at the top of `roi_gui.py` are best-effort.

Optionally pin the sensor so it connects without scanning:

    python3 roi_gui.py --ip 192.168.1.102 --wit-address AA:BB:CC:DD:EE:FF

Each `captures.jsonl` line looks like:

    {"timestamp": "2026-07-24T21:05:12.482+00:00", "camera": "tele",
     "cmd": "CMD_CAMERA_TELE_PHOTOGRAPH(10002)",
     "imu": {"roll": 1.23, "pitch": 48.11, "yaw": 271.4, "altitude": 48.9,
             "azimuth": 271.4, "calibrated": true, "age_s": 0.05}}

`bleak` is optional here — without it the video, tracking and photo button still
work; only the WIT IMU button is unavailable.

---
WitMotion IMU (polar alignment helper)

`wit_imu.py` is a standalone helper for a WitMotion BLE IMU (WT901BLE / BWT901BLE
class sensor), ported from the dwarfium project. Attach the sensor to the mount
and it streams the tilt angle over Bluetooth in real time. The pitch angle (Y) is
shown as "altitude": tilt the mount until it equals your latitude and the mount
axis points at the celestial pole.

It needs one extra dependency:

  pip install bleak

Sun/Moon calibration also uses `skyfield` (a real JPL DE421 ephemeris) if it's
installed, for sub-arcsecond accuracy:

  pip install skyfield

This is optional — skyfield downloads a ~17MB ephemeris file into
`.skyfield_cache/` on first use (needs network once, then works offline), and
if it isn't installed or the download can't complete, calibration falls back
automatically to a self-contained low-precision formula (verified to within
~0.01-0.02° of true position, still plenty for a mount offset).

Usage:

  # list nearby BLE devices to find the sensor's name/address
  python3 wit_imu.py --scan

  # connect to the first WitMotion sensor and print a live altitude readout
  python3 wit_imu.py

  # target a specific device and compare the pitch against your latitude
  python3 wit_imu.py --address AA:BB:CC:DD:EE:FF --latitude 52.5200

Sun/Moon calibration (finding the mount offset):

The IMU only knows its own tilt, which differs from the telescope's true pointing
by a fixed mounting offset. To measure that offset, no camera view of the Sun or
Moon is needed — the Dwarf's own tracking is trusted, and the true position comes
from ephemeris instead of an on-screen fix. Switch the Dwarf to track the Sun or
Moon, keep the sensor strapped on, then run:

  python3 wit_imu.py --calibrate-sun  --lat 52.5200 --lon 13.4050
  python3 wit_imu.py --calibrate-moon --lat 52.5200 --lon 13.4050

This computes the body's true altitude/azimuth from your location and the current
time (system clock, UTC), averages the IMU reading for a few seconds, and saves
the altitude/azimuth offset to `wit_calibration.json`. After that, plain
`python3 wit_imu.py` shows corrected altitude/azimuth automatically.

Notes:
- Altitude calibration is gravity-referenced and reliable. Azimuth calibration
  is only meaningful when the sensor outputs an absolute heading (9-DOF /
  magnetometer mode) — in 6-DOF the yaw drifts.
- If you'd rather use the Sun position the Dwarf reports (instead of computing
  it), pass `--sun-alt` and `--sun-az` directly.

The `WitIMU` class, `decode_frame()`, `sun_altaz()` and `Calibration` can also be
imported into other scripts. Bluetooth only for now (the dwarfium USB/serial
fallback was not ported).

#!/usr/bin/env python3
"""
wit_imu.py — standalone WitMotion BLE IMU reader for Dwarf3 polar alignment.

Ports the WitMotion Bluetooth IMU support found in the `dwarfium` project
(src/lib/witmotion/*) from the browser Web Bluetooth API to Python + bleak, so
it can be used alongside this Stellarium bridge.

The sensor is a WitMotion BLE 5.0 IMU (WT901BLE / BWT901BLE class).  Attach it
to the telescope mount and it streams the tilt angle in real time.  During
polar alignment the interesting figure is the **pitch angle (Y)**, shown here
as "altitude": tilt the mount until that reading equals your latitude and the
mount axis points at the celestial pole.

BLE protocol (identical to dwarfium's ConnectToBluetooth.ts)
-----------------------------------------------------------
  Service          0000ffe5-0000-1000-8000-00805f9a34fb
  Read / notify    0000ffe4-0000-1000-8000-00805f9a34fb
  Write (commands) 0000ffe9-0000-1000-8000-00805f9a34fb

  Data frame (20 bytes, notified on the read characteristic):
    byte 0-1   header 0x55 0x61
    byte 2-19  nine little-endian int16:
                 [0..2] acceleration  -> value/32768 * 16    (g)
                 [3..5] angular vel.   -> value/32768 * 2000  (deg/s)
                 [6..8] angle          -> value/32768 * 180   (deg)  (roll,pitch,yaw)

  Command frame (5 bytes, written to the write characteristic):
    0xFF 0xAA <register> <value_lo> <value_hi>

Usage
-----
  pip install bleak

  # list nearby BLE devices so you can find the sensor's name/address
  python3 wit_imu.py --scan

  # connect (first matching WitMotion service) and print a live altitude readout
  python3 wit_imu.py

  # connect to a specific device and compare against your latitude
  python3 wit_imu.py --address AA:BB:CC:DD:EE:FF --latitude 48.8566

  # set the output rate to 10 Hz on connect
  python3 wit_imu.py --rate 10

Sun / Moon calibration
----------------------
The IMU reads its own tilt, which differs from the telescope's true pointing by
a fixed mounting offset. To measure that offset, no camera view of the target is
needed at all: the Dwarf's own tracking is trusted as ground truth, and the
target's true altitude/azimuth is computed from ephemeris instead of being read
off a live image. Start the Dwarf tracking the Sun or Moon, strap on the sensor,
then run:

  python3 wit_imu.py --calibrate-sun  --lat 48.8566 --lon 2.3522
  python3 wit_imu.py --calibrate-moon --lat 48.8566 --lon 2.3522

This computes the body's true altitude/azimuth from your location and the
current time (Sun: NOAA solar position algorithm; Moon: a Schlyter-style low-
precision lunar theory, good to about 1 arcminute geocentric, plus topocentric
parallax — significant for the Moon, ~1° near the horizon, unlike the Sun's
~9 arcsec which is negligible), averages the IMU reading over --duration
seconds, and saves the offset to wit_calibration.json. Afterwards
`python3 wit_imu.py` shows corrected altitude/azimuth automatically.

Both commands refuse to run if the target is below the horizon (the Dwarf
would not actually be tracking it), and both accept a directly-supplied
--sun-alt/--sun-az or --moon-alt/--moon-az to skip the ephemeris computation
(e.g. if you already have the Dwarf's own reported target position).
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import struct
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # pragma: no cover - dependency hint
    print(
        "wit_imu.py requires the 'bleak' package for Bluetooth access.\n"
        "Install it with:  pip install bleak",
        file=sys.stderr,
    )
    raise

# ── logging ───────────────────────────────────────────────────────────────────
log = logging.getLogger("wit_imu")

# ── BLE identifiers ───────────────────────────────────────────────────────────
SERVICE_UUID = "0000ffe5-0000-1000-8000-00805f9a34fb"
READ_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"
WRITE_UUID = "0000ffe9-0000-1000-8000-00805f9a34fb"

# ── frame decoding constants ──────────────────────────────────────────────────
FRAME_LEN = 20
FRAME_HEADER = (0x55, 0x61)  # data (accel/gyro/angle) frame
_DIVIDER = 32768.0
_ACC_SCALE = 16.0    # -> g
_GYRO_SCALE = 2000.0  # -> deg/s
_ANGLE_SCALE = 180.0  # -> deg

# ── device commands (0xFF 0xAA <reg> <lo> <hi>) ───────────────────────────────
# WT9011DCL firmware locks its config registers; unlock must precede any write
# to a config/calibration register (verified in wit_ble_scratch/wt9011dcl_ble_reader.py).
CMD_UNLOCK = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])
CMD_ACCEL_CAL = bytes([0xFF, 0xAA, 0x01, 0x01, 0x00])
CMD_MAG_CAL = bytes([0xFF, 0xAA, 0x01, 0x07, 0x00])
CMD_EXIT_CAL = bytes([0xFF, 0xAA, 0x01, 0x00, 0x00])
CMD_DOF_6 = bytes([0xFF, 0xAA, 0x24, 0x01, 0x00])
CMD_DOF_9 = bytes([0xFF, 0xAA, 0x24, 0x00, 0x00])
CMD_SAVE = bytes([0xFF, 0xAA, 0x00, 0x00, 0x00])

# output rate in Hz -> command byte
_RATE_CMDS = {
    0.2: bytes([0xFF, 0xAA, 0x03, 0x01, 0x00]),
    0.5: bytes([0xFF, 0xAA, 0x03, 0x02, 0x00]),
    1: bytes([0xFF, 0xAA, 0x03, 0x03, 0x00]),
    2: bytes([0xFF, 0xAA, 0x03, 0x04, 0x00]),
    5: bytes([0xFF, 0xAA, 0x03, 0x05, 0x00]),
    10: bytes([0xFF, 0xAA, 0x03, 0x06, 0x00]),
    20: bytes([0xFF, 0xAA, 0x03, 0x07, 0x00]),
    50: bytes([0xFF, 0xAA, 0x03, 0x08, 0x00]),
}
SUPPORTED_RATES = sorted(_RATE_CMDS)


@dataclass
class Vec3:
    x: float
    y: float
    z: float


@dataclass
class SensorData:
    """One decoded IMU sample.  Angles are in degrees; ``angle.y`` is pitch."""

    acceleration: Vec3  # g
    angular_velocity: Vec3  # deg/s
    angle: Vec3  # deg (roll=x, pitch=y, yaw=z)

    @property
    def altitude(self) -> float:
        """Pitch angle used as the mount altitude during polar alignment."""
        return self.angle.y


def decode_frame(raw: bytes) -> SensorData | None:
    """Decode a 20-byte WitMotion data frame, or ``None`` if it is not one.

    Mirrors dwarfium's extractDataFromRaw(): the two header bytes are skipped
    and the remaining nine int16 little-endian values are scaled.  Non-data
    frames (e.g. register-read replies with a different header) are ignored.
    """
    if len(raw) < FRAME_LEN or (raw[0], raw[1]) != FRAME_HEADER:
        return None

    ax, ay, az, gx, gy, gz, rx, ry, rz = struct.unpack_from("<9h", raw, 2)

    def acc(v: int) -> float:
        return v / _DIVIDER * _ACC_SCALE

    def gyro(v: int) -> float:
        return v / _DIVIDER * _GYRO_SCALE

    def ang(v: int) -> float:
        return v / _DIVIDER * _ANGLE_SCALE

    return SensorData(
        acceleration=Vec3(acc(ax), acc(ay), acc(az)),
        angular_velocity=Vec3(gyro(gx), gyro(gy), gyro(gz)),
        angle=Vec3(ang(rx), ang(ry), ang(rz)),
    )


# ── solar position (NOAA algorithm) ───────────────────────────────────────────
def _julian_day(dt_utc: datetime) -> float:
    """Julian Day for a UTC datetime."""
    year, month = dt_utc.year, dt_utc.month
    day = dt_utc.day + (dt_utc.hour + dt_utc.minute / 60 + dt_utc.second / 3600) / 24
    if month <= 2:
        year -= 1
        month += 12
    a = year // 100
    b = 2 - a + a // 4
    return (
        int(365.25 * (year + 4716))
        + int(30.6001 * (month + 1))
        + day + b - 1524.5
    )


def refraction(true_alt_deg: float) -> float:
    """Atmospheric refraction (deg) to add to a true altitude to get the
    apparent altitude a telescope actually centres on (Saemundsson's formula)."""
    if true_alt_deg < -1.0:
        return 0.0
    r_arcmin = 1.02 / math.tan(
        math.radians(true_alt_deg + 10.3 / (true_alt_deg + 5.11))
    )
    return r_arcmin / 60.0


def sun_altaz(lat_deg: float, lon_deg: float, when_utc: datetime | None = None,
              apply_refraction: bool = True) -> tuple[float, float]:
    """Sun altitude and azimuth for an observer, using the NOAA algorithm.

    Longitude is east-positive. Azimuth is measured from true North, clockwise
    (0 = N, 90 = E, 180 = S, 270 = W). Altitude includes atmospheric refraction
    by default, so it matches where the telescope mechanically points when the
    Sun is centred. Returns ``(altitude_deg, azimuth_deg)``.
    """
    if when_utc is None:
        when_utc = datetime.now(timezone.utc)

    jd = _julian_day(when_utc)
    t = (jd - 2451545.0) / 36525.0  # Julian centuries since J2000.0

    l0 = (280.46646 + t * (36000.76983 + t * 0.0003032)) % 360  # mean longitude
    m = 357.52911 + t * (35999.05029 - 0.0001537 * t)           # mean anomaly
    e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t)      # eccentricity
    m_rad = math.radians(m)

    c = (
        (1.914602 - t * (0.004817 + 0.000014 * t)) * math.sin(m_rad)
        + (0.019993 - 0.000101 * t) * math.sin(2 * m_rad)
        + 0.000289 * math.sin(3 * m_rad)
    )
    true_long = l0 + c
    omega = 125.04 - 1934.136 * t
    lambda_sun = true_long - 0.00569 - 0.00478 * math.sin(math.radians(omega))

    seconds = 21.448 - t * (46.8150 + t * (0.00059 - t * 0.001813))
    e0 = 23 + (26 + seconds / 60) / 60
    obliq = e0 + 0.00256 * math.cos(math.radians(omega))

    decl = math.degrees(
        math.asin(math.sin(math.radians(obliq)) * math.sin(math.radians(lambda_sun)))
    )

    y = math.tan(math.radians(obliq / 2)) ** 2
    l0_rad = math.radians(l0)
    eot = 4 * math.degrees(
        y * math.sin(2 * l0_rad)
        - 2 * e * math.sin(m_rad)
        + 4 * e * y * math.sin(m_rad) * math.cos(2 * l0_rad)
        - 0.5 * y * y * math.sin(4 * l0_rad)
        - 1.25 * e * e * math.sin(2 * m_rad)
    )  # equation of time, minutes

    minutes_utc = when_utc.hour * 60 + when_utc.minute + when_utc.second / 60
    true_solar_time = (minutes_utc + eot + 4 * lon_deg) % 1440
    ha = true_solar_time / 4 - 180  # hour angle, deg

    lat_rad = math.radians(lat_deg)
    decl_rad = math.radians(decl)
    ha_rad = math.radians(ha)

    cos_zenith = (
        math.sin(lat_rad) * math.sin(decl_rad)
        + math.cos(lat_rad) * math.cos(decl_rad) * math.cos(ha_rad)
    )
    cos_zenith = max(-1.0, min(1.0, cos_zenith))
    zenith = math.degrees(math.acos(cos_zenith))
    altitude = 90 - zenith

    denom = math.cos(lat_rad) * math.sin(math.radians(zenith))
    if abs(denom) > 1e-9:
        az_cos = (math.sin(lat_rad) * cos_zenith - math.sin(decl_rad)) / denom
        az_cos = max(-1.0, min(1.0, az_cos))
        azimuth = math.degrees(math.acos(az_cos))
        azimuth = (azimuth + 180) % 360 if ha > 0 else (540 - azimuth) % 360
    else:
        azimuth = 180.0 if lat_deg > decl else 0.0

    if apply_refraction:
        altitude += refraction(altitude)

    return altitude, azimuth % 360


# ── lunar position (Schlyter low-precision lunar theory) ──────────────────────
# CAPTURE-VERIFIED 2026-07-23 against the USNO celestial-navigation API for
# Paris (48.8566N, 2.3522E) at 2026-07-23T20:42:13Z: this reproduces the
# reference Dec/GHA to within ~0.02 deg (~1 arcmin) and, via the shared
# RA/Dec -> alt/az conversion below, the Sun's alt/az/dec to within 0.01 deg —
# both well inside the accuracy needed for a mount-offset calibration.
def _gmst_deg(jd: float) -> float:
    """Greenwich Mean Sidereal Time (deg) for a Julian Day."""
    t = (jd - 2451545.0) / 36525.0
    g = (280.46061837 + 360.98564736629 * (jd - 2451545.0)
         + 0.000387933 * t * t - t ** 3 / 38710000.0)
    return g % 360.0


def _radec_to_altaz(ra_deg: float, dec_deg: float, lat_deg: float, lon_deg: float,
                     jd: float) -> tuple[float, float]:
    """Geocentric RA/Dec (deg) -> (altitude, azimuth) for an observer, via the
    Greenwich Hour Angle. Longitude is east-positive; azimuth from true North,
    clockwise, matching sun_altaz's convention."""
    lst = (_gmst_deg(jd) + lon_deg) % 360.0
    ha = (lst - ra_deg + 180) % 360 - 180

    lat_r, dec_r, ha_r = math.radians(lat_deg), math.radians(dec_deg), math.radians(ha)
    sin_alt = (math.sin(lat_r) * math.sin(dec_r)
               + math.cos(lat_r) * math.cos(dec_r) * math.cos(ha_r))
    sin_alt = max(-1.0, min(1.0, sin_alt))
    alt_r = math.asin(sin_alt)

    denom = math.cos(lat_r) * math.cos(alt_r)
    if abs(denom) > 1e-9:
        cos_az = (math.sin(dec_r) - math.sin(lat_r) * sin_alt) / denom
        cos_az = max(-1.0, min(1.0, cos_az))
        az = math.degrees(math.acos(cos_az))
        if math.sin(ha_r) > 0:
            az = 360 - az
    else:
        az = 180.0 if lat_deg > dec_deg else 0.0

    return math.degrees(alt_r), az % 360


def _ecliptic_to_equatorial(lon_deg: float, lat_deg: float,
                            jd: float) -> tuple[float, float]:
    """Geocentric ecliptic (lon, lat, deg) -> equatorial (RA, Dec, deg), using
    the mean obliquity (no nutation — negligible at this accuracy target)."""
    t = (jd - 2451545.0) / 36525.0
    obliq = 23.439291 - 0.0130042 * t
    lon_r, lat_r, ob_r = math.radians(lon_deg), math.radians(lat_deg), math.radians(obliq)
    ra = math.degrees(math.atan2(
        math.sin(lon_r) * math.cos(ob_r) - math.tan(lat_r) * math.sin(ob_r),
        math.cos(lon_r),
    )) % 360.0
    dec = math.degrees(math.asin(
        math.sin(lat_r) * math.cos(ob_r) + math.cos(lat_r) * math.sin(ob_r) * math.sin(lon_r)
    ))
    return ra, dec


def _moon_geocentric(jd: float) -> tuple[float, float, float]:
    """Geocentric ecliptic (lon, lat, deg) and distance (Earth radii) of the
    Moon, via Paul Schlyter's low-precision lunar theory (orbital elements +
    the dozen largest perturbation terms; accurate to roughly 1 arcminute in
    longitude/latitude, which is what the Dec/GHA cross-check above confirms)."""
    d = jd - 2451543.5  # days since epoch 1999-12-31 00:00 UT

    n = (125.1228 - 0.0529538083 * d) % 360.0     # longitude of ascending node
    incl = 5.1454                                  # inclination
    w = (318.0634 + 0.1643573223 * d) % 360.0     # argument of perigee
    a = 60.2666                                    # mean distance, Earth radii
    e = 0.054900                                   # eccentricity
    m = (115.3654 + 13.0649929509 * d) % 360.0    # mean anomaly

    m_r = math.radians(m)
    ecc = m + math.degrees(e * math.sin(m_r) * (1 + e * math.cos(m_r)))
    for _ in range(8):
        ecc_r = math.radians(ecc)
        nxt = ecc - (ecc - math.degrees(e * math.sin(ecc_r)) - m) / (1 - e * math.cos(ecc_r))
        if abs(nxt - ecc) < 1e-8:
            ecc = nxt
            break
        ecc = nxt

    ecc_r = math.radians(ecc)
    xv = a * (math.cos(ecc_r) - e)
    yv = a * (math.sqrt(1 - e * e) * math.sin(ecc_r))
    r = math.hypot(xv, yv)
    v = math.degrees(math.atan2(yv, xv))

    n_r, i_r, w_r, v_r = (math.radians(x) for x in (n, incl, w, v))
    xh = r * (math.cos(n_r) * math.cos(v_r + w_r) - math.sin(n_r) * math.sin(v_r + w_r) * math.cos(i_r))
    yh = r * (math.sin(n_r) * math.cos(v_r + w_r) + math.cos(n_r) * math.sin(v_r + w_r) * math.cos(i_r))
    zh = r * (math.sin(v_r + w_r) * math.sin(i_r))

    lon_ecl = math.degrees(math.atan2(yh, xh))
    lat_ecl = math.degrees(math.atan2(zh, math.hypot(xh, yh)))

    # Perturbations (Sun's pull on the Moon's orbit) — the dozen largest terms.
    t = (jd - 2451545.0) / 36525.0
    ms = (357.5291 + 35999.0503 * t) % 360.0      # Sun's mean anomaly
    ws = 282.9404 + 4.70935e-5 * d                 # Sun's argument of perihelion
    ls = (ws + ms) % 360.0                         # Sun's mean longitude
    lm = (n + w + m) % 360.0                       # Moon's mean longitude
    D = (lm - ls) % 360.0                          # Moon's mean elongation
    F = (lm - n) % 360.0                           # Moon's argument of latitude
    mm_r, d_r, ms_r, f_r = (math.radians(x) for x in (m, D, ms, F))

    lon_ecl += (
        -1.274 * math.sin(mm_r - 2 * d_r) + 0.658 * math.sin(2 * d_r)
        - 0.186 * math.sin(ms_r) - 0.059 * math.sin(2 * mm_r - 2 * d_r)
        - 0.057 * math.sin(mm_r - 2 * d_r + ms_r) + 0.053 * math.sin(mm_r + 2 * d_r)
        + 0.046 * math.sin(2 * d_r - ms_r) + 0.041 * math.sin(mm_r - ms_r)
        - 0.035 * math.sin(d_r) - 0.031 * math.sin(mm_r + ms_r)
        - 0.015 * math.sin(2 * f_r - 2 * d_r) + 0.011 * math.sin(mm_r - 4 * d_r)
    )
    lat_ecl += (
        -0.173 * math.sin(f_r - 2 * d_r) - 0.055 * math.sin(mm_r - f_r - 2 * d_r)
        - 0.046 * math.sin(mm_r + f_r - 2 * d_r) + 0.033 * math.sin(f_r + 2 * d_r)
        + 0.017 * math.sin(2 * mm_r + f_r)
    )
    r += -0.58 * math.cos(mm_r - 2 * d_r) - 0.46 * math.cos(2 * d_r)

    return lon_ecl % 360.0, lat_ecl, r


def moon_altaz(lat_deg: float, lon_deg: float, when_utc: datetime | None = None,
               apply_refraction: bool = True, apply_parallax: bool = True,
               ) -> tuple[float, float]:
    """Moon altitude and azimuth for an observer (same convention as sun_altaz).

    Unlike the Sun, the Moon's parallax is large enough to matter for pointing
    (up to ~1 deg near the horizon), so it's applied by default alongside
    refraction to give the *apparent* altitude — where the telescope actually
    ends up pointing when centred on the Moon.
    """
    if when_utc is None:
        when_utc = datetime.now(timezone.utc)

    jd = _julian_day(when_utc)
    lon_ecl, lat_ecl, dist_er = _moon_geocentric(jd)
    ra, dec = _ecliptic_to_equatorial(lon_ecl, lat_ecl, jd)
    altitude, azimuth = _radec_to_altaz(ra, dec, lat_deg, lon_deg, jd)

    if apply_parallax:
        dist_km = dist_er * 6378.14
        horiz_parallax = math.degrees(math.asin(6378.14 / dist_km))
        altitude -= horiz_parallax * math.cos(math.radians(altitude))

    if apply_refraction:
        altitude += refraction(altitude)

    return altitude, azimuth % 360


# ── calibration ───────────────────────────────────────────────────────────────
def _wrap180(angle: float) -> float:
    """Wrap an angle to the (-180, 180] range."""
    return (angle + 180) % 360 - 180


def _circular_mean(degrees_list: list[float]) -> float:
    """Mean of angles in degrees, handling the 0/360 wrap; result in [0, 360)."""
    s = sum(math.sin(math.radians(d)) for d in degrees_list)
    c = sum(math.cos(math.radians(d)) for d in degrees_list)
    return math.degrees(math.atan2(s, c)) % 360


@dataclass
class Calibration:
    """Fixed mounting offset between the IMU angles and the sky.

    ``alt_offset`` is added to the IMU pitch to give true altitude.
    ``az_offset`` is added to the IMU yaw to give true azimuth. Offsets are a
    single-point fit (``true = raw + offset``); they assume the IMU axis moves
    the same direction and scale as the telescope, which holds for a rigidly
    strapped-on sensor. Azimuth is only meaningful if the sensor outputs an
    absolute heading (9-DOF / magnetometer mode).
    """

    alt_offset: float
    az_offset: float
    reference: str = "sun"  # which body this was fit against: "sun" or "moon"
    sun_alt: float | None = None  # reference body's true alt/az (field name kept
    sun_az: float | None = None   # for wit_calibration.json back-compat; see `reference`
    imu_pitch: float | None = None
    imu_yaw: float | None = None
    samples: int = 0
    timestamp: str = ""

    def apply(self, sample: "SensorData") -> tuple[float, float]:
        """Return ``(altitude, azimuth)`` for a sample, corrected by the offsets."""
        altitude = sample.angle.y + self.alt_offset
        azimuth = (sample.angle.z + self.az_offset) % 360
        return altitude, azimuth

    def save(self, path: Path) -> None:
        path.write_text(json.dumps(asdict(self), indent=2))

    @classmethod
    def load(cls, path: Path) -> "Calibration":
        data = json.loads(Path(path).read_text())
        return cls(**data)


def compute_calibration(pitches: list[float], yaws: list[float],
                        ref_alt: float, ref_az: float, reference: str) -> Calibration:
    """Build a Calibration from averaged IMU samples and the true position of
    whichever body (``reference``: "sun" or "moon") the Dwarf is tracking."""
    mean_pitch = sum(pitches) / len(pitches)
    mean_yaw = _circular_mean(yaws)
    return Calibration(
        alt_offset=ref_alt - mean_pitch,
        az_offset=_wrap180(ref_az - mean_yaw),
        reference=reference,
        sun_alt=round(ref_alt, 4),
        sun_az=round(ref_az, 4),
        imu_pitch=round(mean_pitch, 4),
        imu_yaw=round(mean_yaw, 4),
        samples=len(pitches),
        timestamp=datetime.now(timezone.utc).isoformat(timespec="seconds"),
    )


class WitIMU:
    """Async connection to a WitMotion BLE IMU.

    Example
    -------
        def on_data(sample):
            print(sample.altitude)

        imu = WitIMU(on_data=on_data)
        await imu.connect(address="AA:BB:CC:DD:EE:FF")
        ...
        await imu.disconnect()
    """

    def __init__(self, on_data=None):
        # on_data: callable invoked with a SensorData for every decoded frame.
        self.on_data = on_data
        self._client: BleakClient | None = None

    @staticmethod
    async def scan(timeout: float = 5.0):
        """Return a list of ``(address, name)`` for nearby BLE devices."""
        devices = await BleakScanner.discover(timeout=timeout)
        return [(d.address, d.name or "?") for d in devices]

    @staticmethod
    async def find_device(name: str | None = None, timeout: float = 8.0):
        """Find a WitMotion sensor by advertised service UUID (or name substring)."""
        def match(dev, adv):
            if name is not None:
                return dev.name is not None and name.lower() in dev.name.lower()
            uuids = [u.lower() for u in (adv.service_uuids or [])]
            return SERVICE_UUID.lower() in uuids

        return await BleakScanner.find_device_by_filter(match, timeout=timeout)

    async def connect(self, address: str | None = None, name: str | None = None,
                      timeout: float = 10.0) -> None:
        """Connect to the sensor and subscribe to the data notifications."""
        target: object = address
        if target is None:
            log.info("Scanning for a WitMotion sensor...")
            device = await self.find_device(name=name, timeout=timeout)
            if device is None:
                raise RuntimeError(
                    "No WitMotion sensor found. Use --scan to list devices, or "
                    "pass --address / --name."
                )
            log.info("Found %s (%s)", device.name or "?", device.address)
            target = device

        self._client = BleakClient(target, timeout=timeout)
        await self._client.connect()
        log.info("Connected.")

        def _handler(_char, data: bytearray) -> None:
            sample = decode_frame(bytes(data))
            if sample is not None and self.on_data is not None:
                self.on_data(sample)

        await self._client.start_notify(READ_UUID, _handler)

    async def disconnect(self) -> None:
        if self._client is not None:
            try:
                await self._client.stop_notify(READ_UUID)
            except Exception:  # noqa: BLE001 - already disconnecting
                pass
            await self._client.disconnect()
            self._client = None
            log.info("Disconnected.")

    @property
    def is_connected(self) -> bool:
        return self._client is not None and self._client.is_connected

    # ── device configuration / calibration ───────────────────────────────────
    async def _write(self, payload: bytes) -> None:
        if self._client is None:
            raise RuntimeError("Not connected.")
        # response=False matches the write-without-response used over BLE here.
        await self._client.write_gatt_char(WRITE_UUID, payload, response=False)

    async def _unlock(self) -> None:
        """Unlock the config registers; required before any config-register write."""
        await self._write(CMD_UNLOCK)

    async def set_rate(self, hz: float) -> None:
        """Set the output data rate (one of SUPPORTED_RATES) and save it."""
        if hz not in _RATE_CMDS:
            raise ValueError(f"Unsupported rate {hz}; choose from {SUPPORTED_RATES}")
        await self._unlock()
        await self._write(_RATE_CMDS[hz])
        await self._write(CMD_SAVE)

    async def set_dof(self, dof: int) -> None:
        """Select the fusion algorithm: 6 (no magnetometer) or 9 DOF."""
        if dof not in (6, 9):
            raise ValueError("dof must be 6 or 9")
        await self._unlock()
        await self._write(CMD_DOF_6 if dof == 6 else CMD_DOF_9)
        await self._write(CMD_SAVE)

    async def calibrate_accelerometer(self) -> None:
        """Run the accelerometer calibration (keep the sensor level and still)."""
        await self._unlock()
        await self._write(CMD_ACCEL_CAL)
        await asyncio.sleep(3.1)
        await self._write(CMD_EXIT_CAL)

    async def start_magnetometer_calibration(self) -> None:
        """Enter magnetometer calibration; rotate the sensor through all axes."""
        await self._unlock()
        await self._write(CMD_MAG_CAL)

    async def stop_magnetometer_calibration(self) -> None:
        await self._unlock()
        await self._write(CMD_EXIT_CAL)


# ── CLI ───────────────────────────────────────────────────────────────────────
async def _cmd_scan(args) -> int:
    print(f"Scanning for {args.timeout:.0f}s...")
    devices = await WitIMU.scan(timeout=args.timeout)
    if not devices:
        print("No BLE devices found.")
        return 1
    for address, name in devices:
        print(f"  {address}  {name}")
    return 0


def _load_calibration(args) -> "Calibration | None":
    path = Path(args.calibration_file)
    if not path.exists():
        return None
    try:
        cal = Calibration.load(path)
        log.info(
            "Loaded calibration from %s (alt_offset=%.2f° az_offset=%.2f°)",
            path, cal.alt_offset, cal.az_offset,
        )
        return cal
    except Exception as exc:  # noqa: BLE001 - bad/old file, just skip it
        log.warning("Could not read calibration %s: %s", path, exc)
        return None


async def _sample_average(imu: "WitIMU", latest: dict, duration: float):
    """Collect pitch/yaw samples for ``duration`` seconds; return the two lists."""
    pitches: list[float] = []
    yaws: list[float] = []
    deadline = asyncio.get_running_loop().time() + duration
    last_id = None
    while asyncio.get_running_loop().time() < deadline:
        await asyncio.sleep(0.05)
        sample = latest.get("sample")
        if sample is None or id(sample) == last_id:
            continue
        last_id = id(sample)
        pitches.append(sample.angle.y)
        yaws.append(sample.angle.z)
    return pitches, yaws


async def _cmd_read(args) -> int:
    latest: dict[str, SensorData] = {}

    def on_data(sample: SensorData) -> None:
        latest["sample"] = sample

    calibration = _load_calibration(args)

    imu = WitIMU(on_data=on_data)
    try:
        await imu.connect(address=args.address, name=args.name, timeout=args.timeout)
    except Exception as exc:  # noqa: BLE001 - surface a clean CLI error
        # str(exc) is empty for bare TimeoutError etc.; always show the type too.
        log.error("%s: %s", type(exc).__name__, str(exc) or "(no further detail)")
        return 1

    if args.rate is not None:
        log.info("Setting output rate to %g Hz", args.rate)
        await imu.set_rate(args.rate)

    print("Reading IMU. Pitch (Y) is shown as altitude. Press Ctrl+C to stop.")
    try:
        while True:
            await asyncio.sleep(0.25)
            sample = latest.get("sample")
            if sample is None:
                continue
            line = (
                f"altitude(pitch Y): {sample.angle.y:7.2f}°   "
                f"roll(X): {sample.angle.x:7.2f}°   yaw(Z): {sample.angle.z:7.2f}°"
            )
            if calibration is not None:
                alt, az = calibration.apply(sample)
                line += f"   →  corrected alt: {alt:7.2f}°  az: {az:7.2f}°"
            elif args.latitude is not None:
                delta = sample.angle.y - args.latitude
                line += f"   Δlat: {delta:+6.2f}°"
            print("\r" + line + "  ", end="", flush=True)
    except (KeyboardInterrupt, asyncio.CancelledError):
        print()
    finally:
        await imu.disconnect()
    return 0


_BODY_ALTAZ = {"sun": sun_altaz, "moon": moon_altaz}


async def _cmd_calibrate(args, body: str) -> int:
    """Calibrate the IMU offset against the Sun or Moon.

    Point the Dwarf at the body (it must already be tracking) with the IMU
    strapped on, then run this. The body's true altitude/azimuth (from
    ephemeris, or --sun-alt/--sun-az / --moon-alt/--moon-az) minus the
    averaged IMU reading gives the mounting offset, saved to disk. No camera
    view of the body is needed — the Dwarf's own tracking is trusted, and the
    true position comes from ephemeris rather than an on-screen fix.
    """
    alt_override = args.sun_alt if body == "sun" else args.moon_alt
    az_override = args.sun_az if body == "sun" else args.moon_az

    if alt_override is not None and az_override is not None:
        ref_alt, ref_az = alt_override, az_override
        log.info("Using supplied %s position: alt=%.3f° az=%.3f°", body, ref_alt, ref_az)
    else:
        if args.lat is None or args.lon is None:
            log.error(
                "%s calibration needs --lat and --lon (or --%s-alt and --%s-az).",
                body.capitalize(), body, body,
            )
            return 2
        kwargs = {"apply_refraction": not args.no_refraction}
        if body == "moon":
            kwargs["apply_parallax"] = not args.no_refraction
        ref_alt, ref_az = _BODY_ALTAZ[body](args.lat, args.lon, **kwargs)
        log.info(
            "Computed %s position now: alt=%.3f° az=%.3f° (lat=%.4f lon=%.4f)",
            body.capitalize(), ref_alt, ref_az, args.lat, args.lon,
        )

    if ref_alt < 0:
        log.error("The %s is below the horizon (alt=%.2f°) — cannot calibrate.",
                  body.capitalize(), ref_alt)
        return 1

    latest: dict[str, SensorData] = {}

    def on_data(sample: SensorData) -> None:
        latest["sample"] = sample

    imu = WitIMU(on_data=on_data)
    try:
        await imu.connect(address=args.address, name=args.name, timeout=args.timeout)
    except Exception as exc:  # noqa: BLE001 - surface a clean CLI error
        log.error("%s: %s", type(exc).__name__, str(exc) or "(no further detail)")
        return 1

    try:
        log.info("Collecting IMU samples for %.0fs — keep the mount tracking the %s...",
                 args.duration, body.capitalize())
        pitches, yaws = await _sample_average(imu, latest, args.duration)
    finally:
        await imu.disconnect()

    if not pitches:
        log.error("No IMU data received during calibration.")
        return 1

    cal = compute_calibration(pitches, yaws, ref_alt, ref_az, body)
    path = Path(args.calibration_file)
    cal.save(path)

    print()
    print(f"  Samples averaged : {cal.samples}")
    print(f"  {body.capitalize()} (reference) : alt {cal.sun_alt:.3f}°   az {cal.sun_az:.3f}°")
    print(f"  IMU (raw mean)   : pitch {cal.imu_pitch:.3f}°   yaw {cal.imu_yaw:.3f}°")
    print(f"  Altitude offset  : {cal.alt_offset:+.3f}°")
    print(f"  Azimuth offset   : {cal.az_offset:+.3f}°  "
          f"(only valid in 9-DOF/absolute-heading mode)")
    print(f"  Saved to         : {path}")
    print("\nRun 'python3 wit_imu.py' to see corrected alt/az using this calibration.")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="WitMotion BLE IMU reader for Dwarf3 polar alignment",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--scan", action="store_true",
        help="List nearby BLE devices and exit (find the sensor's name/address)",
    )
    parser.add_argument(
        "--calibrate-sun", action="store_true",
        help="Calibrate the IMU offset against the Sun (the mount must already "
             "be tracking it), then save it to the calibration file",
    )
    parser.add_argument(
        "--calibrate-moon", action="store_true",
        help="Calibrate the IMU offset against the Moon (the mount must already "
             "be tracking it), then save it to the calibration file",
    )
    parser.add_argument(
        "--address", default=None, metavar="ADDR",
        help="BLE address (or macOS UUID) of the sensor to connect to",
    )
    parser.add_argument(
        "--name", default=None, metavar="SUBSTR",
        help="Connect to the first device whose name contains this substring",
    )
    parser.add_argument(
        "--rate", type=float, default=None, metavar="HZ",
        choices=SUPPORTED_RATES,
        help=f"Output data rate to set on connect (one of {SUPPORTED_RATES})",
    )
    parser.add_argument(
        "--latitude", type=float, default=None, metavar="DEG",
        help="Observer latitude; shows Δlat = pitch − latitude for polar alignment "
             "(read mode, when no calibration file is present)",
    )
    parser.add_argument(
        "--timeout", type=float, default=8.0, metavar="SEC",
        help="Scan/connect timeout in seconds",
    )

    cal_group = parser.add_argument_group(
        "Sun/Moon calibration (--calibrate-sun / --calibrate-moon)")
    cal_group.add_argument(
        "--lat", type=float, default=None, metavar="DEG",
        help="Observer latitude for computing the target's position",
    )
    cal_group.add_argument(
        "--lon", type=float, default=None, metavar="DEG",
        help="Observer longitude (east-positive) for computing the target's position",
    )
    cal_group.add_argument(
        "--sun-alt", type=float, default=None, metavar="DEG",
        help="Override the reference Sun altitude instead of computing it "
             "(e.g. from the Dwarf's own report)",
    )
    cal_group.add_argument(
        "--sun-az", type=float, default=None, metavar="DEG",
        help="Override the reference Sun azimuth instead of computing it",
    )
    cal_group.add_argument(
        "--moon-alt", type=float, default=None, metavar="DEG",
        help="Override the reference Moon altitude instead of computing it",
    )
    cal_group.add_argument(
        "--moon-az", type=float, default=None, metavar="DEG",
        help="Override the reference Moon azimuth instead of computing it",
    )
    cal_group.add_argument(
        "--duration", type=float, default=5.0, metavar="SEC",
        help="How long to average IMU samples during calibration",
    )
    cal_group.add_argument(
        "--no-refraction", action="store_true",
        help="Do not add atmospheric refraction (or, for the Moon, parallax) "
             "to the computed altitude",
    )
    parser.add_argument(
        "--calibration-file", default="wit_calibration.json", metavar="PATH",
        help="Where the Sun calibration is saved/loaded",
    )

    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable debug logging",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )

    if args.calibrate_sun and args.calibrate_moon:
        parser.error("--calibrate-sun and --calibrate-moon are mutually exclusive")

    if args.scan:
        coro = _cmd_scan(args)
    elif args.calibrate_sun:
        coro = _cmd_calibrate(args, "sun")
    elif args.calibrate_moon:
        coro = _cmd_calibrate(args, "moon")
    else:
        coro = _cmd_read(args)
    try:
        return asyncio.run(coro)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())

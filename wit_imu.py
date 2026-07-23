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

Sun calibration
---------------
The IMU reads its own tilt, which differs from the telescope's true pointing by
a fixed mounting offset. To measure that offset, point the Dwarf at the Sun
(solar tracking) with the sensor strapped on, then run:

  python3 wit_imu.py --calibrate-sun --lat 48.8566 --lon 2.3522

This computes the Sun's true altitude/azimuth from your location and the current
time, averages the IMU reading, and saves the offset to wit_calibration.json.
Afterwards `python3 wit_imu.py` shows corrected altitude/azimuth automatically.
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
    reference: str = "sun"
    sun_alt: float | None = None
    sun_az: float | None = None
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


def compute_sun_calibration(pitches: list[float], yaws: list[float],
                            sun_alt: float, sun_az: float) -> Calibration:
    """Build a Calibration from averaged IMU samples and the true Sun position."""
    mean_pitch = sum(pitches) / len(pitches)
    mean_yaw = _circular_mean(yaws)
    return Calibration(
        alt_offset=sun_alt - mean_pitch,
        az_offset=_wrap180(sun_az - mean_yaw),
        reference="sun",
        sun_alt=round(sun_alt, 4),
        sun_az=round(sun_az, 4),
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
        log.error("%s", exc)
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


async def _cmd_calibrate_sun(args) -> int:
    """Calibrate the IMU offset against the Sun.

    Point the Dwarf at the Sun (solar tracking) with the IMU strapped on, then
    run this. The true Sun altitude/azimuth (from ephemeris, or --sun-alt/--sun-az)
    minus the averaged IMU reading gives the mounting offset, saved to disk.
    """
    if args.sun_alt is not None and args.sun_az is not None:
        sun_alt, sun_az = args.sun_alt, args.sun_az
        log.info("Using supplied Sun position: alt=%.3f° az=%.3f°", sun_alt, sun_az)
    else:
        if args.lat is None or args.lon is None:
            log.error(
                "Sun calibration needs --lat and --lon (or --sun-alt and --sun-az)."
            )
            return 2
        sun_alt, sun_az = sun_altaz(
            args.lat, args.lon, apply_refraction=not args.no_refraction
        )
        log.info(
            "Computed Sun position now: alt=%.3f° az=%.3f° (lat=%.4f lon=%.4f)",
            sun_alt, sun_az, args.lat, args.lon,
        )

    if sun_alt < 0:
        log.error("The Sun is below the horizon (alt=%.2f°) — cannot calibrate.", sun_alt)
        return 1

    latest: dict[str, SensorData] = {}

    def on_data(sample: SensorData) -> None:
        latest["sample"] = sample

    imu = WitIMU(on_data=on_data)
    try:
        await imu.connect(address=args.address, name=args.name, timeout=args.timeout)
    except Exception as exc:  # noqa: BLE001 - surface a clean CLI error
        log.error("%s", exc)
        return 1

    try:
        log.info("Collecting IMU samples for %.0fs — keep the mount tracking the Sun...",
                 args.duration)
        pitches, yaws = await _sample_average(imu, latest, args.duration)
    finally:
        await imu.disconnect()

    if not pitches:
        log.error("No IMU data received during calibration.")
        return 1

    cal = compute_sun_calibration(pitches, yaws, sun_alt, sun_az)
    path = Path(args.calibration_file)
    cal.save(path)

    print()
    print(f"  Samples averaged : {cal.samples}")
    print(f"  Sun (reference)  : alt {cal.sun_alt:.3f}°   az {cal.sun_az:.3f}°")
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
        help="Calibrate the IMU offset against the Sun (point the mount at the "
             "Sun first), then save it to the calibration file",
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

    cal_group = parser.add_argument_group("Sun calibration (--calibrate-sun)")
    cal_group.add_argument(
        "--lat", type=float, default=None, metavar="DEG",
        help="Observer latitude for computing the Sun's position",
    )
    cal_group.add_argument(
        "--lon", type=float, default=None, metavar="DEG",
        help="Observer longitude (east-positive) for computing the Sun's position",
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
        "--duration", type=float, default=5.0, metavar="SEC",
        help="How long to average IMU samples during calibration",
    )
    cal_group.add_argument(
        "--no-refraction", action="store_true",
        help="Do not add atmospheric refraction to the computed Sun altitude",
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

    if args.scan:
        coro = _cmd_scan(args)
    elif args.calibrate_sun:
        coro = _cmd_calibrate_sun(args)
    else:
        coro = _cmd_read(args)
    try:
        return asyncio.run(coro)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())

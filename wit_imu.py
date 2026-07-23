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
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import struct
import sys
from dataclasses import dataclass

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

    async def set_rate(self, hz: float) -> None:
        """Set the output data rate (one of SUPPORTED_RATES) and save it."""
        if hz not in _RATE_CMDS:
            raise ValueError(f"Unsupported rate {hz}; choose from {SUPPORTED_RATES}")
        await self._write(_RATE_CMDS[hz])
        await self._write(CMD_SAVE)

    async def set_dof(self, dof: int) -> None:
        """Select the fusion algorithm: 6 (no magnetometer) or 9 DOF."""
        if dof == 6:
            await self._write(CMD_DOF_6)
        elif dof == 9:
            await self._write(CMD_DOF_9)
        else:
            raise ValueError("dof must be 6 or 9")
        await self._write(CMD_SAVE)

    async def calibrate_accelerometer(self) -> None:
        """Run the accelerometer calibration (keep the sensor level and still)."""
        await self._write(CMD_ACCEL_CAL)
        await asyncio.sleep(3.1)
        await self._write(CMD_EXIT_CAL)

    async def start_magnetometer_calibration(self) -> None:
        """Enter magnetometer calibration; rotate the sensor through all axes."""
        await self._write(CMD_MAG_CAL)

    async def stop_magnetometer_calibration(self) -> None:
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


async def _cmd_read(args) -> int:
    loop = asyncio.get_running_loop()
    latest: dict[str, SensorData] = {}

    def on_data(sample: SensorData) -> None:
        latest["sample"] = sample

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
            if args.latitude is not None:
                delta = sample.angle.y - args.latitude
                line += f"   Δlat: {delta:+6.2f}°"
            print("\r" + line + "  ", end="", flush=True)
    except (KeyboardInterrupt, asyncio.CancelledError):
        print()
    finally:
        await imu.disconnect()
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
        help="Observer latitude; shows Δlat = pitch − latitude for polar alignment",
    )
    parser.add_argument(
        "--timeout", type=float, default=8.0, metavar="SEC",
        help="Scan/connect timeout in seconds",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable debug logging",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )

    coro = _cmd_scan(args) if args.scan else _cmd_read(args)
    try:
        return asyncio.run(coro)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())

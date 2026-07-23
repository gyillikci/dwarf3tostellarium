#!/usr/bin/env python3
"""
Live stream reader for a WitMotion WT9011DCL BT5.0 IMU, connecting directly
over the PC's own Bluetooth LE radio via GATT (bypasses the nRF52 USB dongle,
whose auto-pairing to the sensor was unreliable).

GATT layout (WitMotion "BLE5.0" custom UART-like service):
  service    0000ffe5-0000-1000-8000-00805f9a34fb
  notify char 0000ffe4-0000-1000-8000-00805f9a34fb  <- sensor data streams here
  write  char 0000ffe9-0000-1000-8000-00805f9a34fb  <- register commands go here

Streaming data notification: fixed 20 bytes, no checksum (BLE already
guarantees payload integrity at the link layer):
  byte 0    : 0x55            header
  byte 1    : 0x61            flag = combined IMU stream
  bytes 2-19: 9 x int16 LE    ax, ay, az, wx, wy, wz, roll, pitch, yaw

Scaling:
  acceleration (g)       = raw / 32768 * 16
  angular velocity (dps) = raw / 32768 * 2000
  angle (deg)            = raw / 32768 * 180

Register write command format (0xFF 0xAA <reg> <dataLow> <dataHigh>), e.g.
unlock=FF AA 69 88 B5, save=FF AA 00 00 00 -- not needed just to stream data,
included here (send_reg_write) for reference/future config use.
"""

import argparse
import asyncio
import csv
import os
import struct
import time
from datetime import datetime, timezone

from bleak import BleakClient, BleakScanner

SERVICE_UUID = "0000ffe5-0000-1000-8000-00805f9a34fb"
NOTIFY_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"
WRITE_UUID = "0000ffe9-0000-1000-8000-00805f9a34fb"

HEADER = 0x55
FLAG_IMU = 0x61


def parse_imu_packet(data: bytes):
    if len(data) != 20 or data[0] != HEADER or data[1] != FLAG_IMU:
        return None
    vals = struct.unpack("<9h", data[2:20])
    ax, ay, az, wx, wy, wz, roll, pitch, yaw = vals
    return {
        "ax_g": ax / 32768 * 16,
        "ay_g": ay / 32768 * 16,
        "az_g": az / 32768 * 16,
        "wx_dps": wx / 32768 * 2000,
        "wy_dps": wy / 32768 * 2000,
        "wz_dps": wz / 32768 * 2000,
        "roll_deg": roll / 32768 * 180,
        "pitch_deg": pitch / 32768 * 180,
        "yaw_deg": yaw / 32768 * 180,
    }


async def send_reg_write(client: BleakClient, addr: int, data: int):
    payload = bytes([0xFF, 0xAA, addr & 0xFF, data & 0xFF, (data >> 8) & 0xFF])
    await client.write_gatt_char(WRITE_UUID, payload, response=False)


CSV_FIELDS = [
    "timestamp", "ax_g", "ay_g", "az_g",
    "wx_dps", "wy_dps", "wz_dps",
    "roll_deg", "pitch_deg", "yaw_deg",
]


async def find_device(name_filter: str):
    print(f"Scanning for a device matching {name_filter!r} ...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, adv: bool((d.name or adv.local_name or "").startswith(name_filter)),
        timeout=10.0,
    )
    return device


async def stream_once(device, seconds, csv_writer, log_file, print_every: float):
    disconnected = asyncio.Event()
    last_print = 0.0

    def on_disconnect(_client):
        disconnected.set()

    def on_notify(_handle, data: bytearray):
        nonlocal last_print
        fields = parse_imu_packet(bytes(data))
        if not fields:
            return
        now = time.time()

        if csv_writer:
            csv_writer.writerow(
                [datetime.now(timezone.utc).isoformat()] + [f"{v:.5f}" for v in fields.values()]
            )
            log_file.flush()

        if now - last_print >= print_every:
            last_print = now
            print(
                f"acc(g)=({fields['ax_g']:+.3f},{fields['ay_g']:+.3f},{fields['az_g']:+.3f})  "
                f"gyro(dps)=({fields['wx_dps']:+7.2f},{fields['wy_dps']:+7.2f},{fields['wz_dps']:+7.2f})  "
                f"angle(deg)=({fields['roll_deg']:+7.2f},{fields['pitch_deg']:+7.2f},{fields['yaw_deg']:+7.2f})"
            )

    async with BleakClient(device, disconnected_callback=on_disconnect) as client:
        print(f"Connected: {client.is_connected}. Streaming (Ctrl+C to stop)...")
        await client.start_notify(NOTIFY_UUID, on_notify)
        try:
            waiters = [asyncio.create_task(disconnected.wait())]
            if seconds:
                waiters.append(asyncio.create_task(asyncio.sleep(seconds)))
            done, pending = await asyncio.wait(waiters, return_when=asyncio.FIRST_COMPLETED)
            for p in pending:
                p.cancel()
        finally:
            try:
                await client.stop_notify(NOTIFY_UUID)
            except Exception:
                pass
    return disconnected.is_set()


async def run(name_filter: str, seconds: float | None, log_path: str | None, reconnect: bool, print_every: float):
    log_file = None
    csv_writer = None
    if log_path:
        is_new = not os.path.exists(log_path)
        log_file = open(log_path, "a", newline="")
        csv_writer = csv.writer(log_file)
        if is_new:
            csv_writer.writerow(CSV_FIELDS)
        print(f"Logging to {log_path}")

    try:
        while True:
            device = await find_device(name_filter)
            if device is None:
                print("Device not found. Make sure the sensor is powered on and nearby.")
                if not reconnect:
                    return
                await asyncio.sleep(3)
                continue

            print(f"Found {device.address} ({device.name}). Connecting...")
            try:
                was_disconnect = await stream_once(device, seconds, csv_writer, log_file, print_every)
            except KeyboardInterrupt:
                raise
            except Exception as e:
                print(f"Connection error: {e}")
                was_disconnect = True

            if not reconnect or seconds:
                return
            if was_disconnect:
                print("Disconnected. Reconnecting...")
                await asyncio.sleep(2)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if log_file:
            log_file.close()


def main():
    ap = argparse.ArgumentParser(description="Stream (and optionally log) a WT9011DCL IMU directly over BLE GATT.")
    ap.add_argument("--name", default="WT901BLE", help="BLE advertised name prefix to match (default: WT901BLE)")
    ap.add_argument("--seconds", type=float, default=None, help="Stop after N seconds (default: run until Ctrl+C)")
    ap.add_argument("--log", metavar="PATH", default=None, help="Append timestamped CSV rows to this file")
    ap.add_argument("--no-reconnect", action="store_true", help="Don't auto-reconnect if the BLE link drops")
    ap.add_argument("--print-every", type=float, default=0.2, help="Console print throttle in seconds (default: 0.2)")
    args = ap.parse_args()
    asyncio.run(run(args.name, args.seconds, args.log, not args.no_reconnect, args.print_every))


if __name__ == "__main__":
    main()

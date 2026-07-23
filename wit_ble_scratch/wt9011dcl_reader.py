#!/usr/bin/env python3
"""
Stream reader for the WitMotion WT9011DCL BT5.0 IMU via its BLE dongle
(Nordic nRF52 "USB CDC BLE Demo" virtual COM port, 115200 8N1).

Wire protocol (WitMotion "0x55" packet format, shared across their WTxxx line):
  byte 0      : 0x55                     header
  byte 1      : flag                     packet type (see FLAG_* below)
  bytes 2..N  : payload, int16 LE fields
  byte N+1    : checksum = (sum of all preceding bytes) & 0xFF

Standard single-quantity packets (11 bytes total: header+flag+8 data+checksum):
  0x51 acceleration : ax, ay, az, temperature
  0x52 angular vel  : wx, wy, wz, temperature
  0x53 angle        : roll, pitch, yaw, temperature
  0x54 magnetometer : hx, hy, hz, temperature

Combined fast-stream packet used by the BLE dongle by default (21 bytes total,
header+flag+18 data+checksum):
  0x61 : ax, ay, az, wx, wy, wz, roll, pitch, yaw   (9 int16 fields, no temp)

Scaling:
  acceleration (g)      = raw / 32768 * 16
  angular velocity(deg/s)= raw / 32768 * 2000
  angle (deg)            = raw / 32768 * 180
  magnetometer (raw LSB) = raw  (no documented scale, relative units)
"""

import argparse
import struct
import sys
import time

import serial

HEADER = 0x55

FLAG_ACCEL = 0x51
FLAG_GYRO = 0x52
FLAG_ANGLE = 0x53
FLAG_MAG = 0x54
FLAG_COMBINED = 0x61

PAYLOAD_LEN = {
    FLAG_ACCEL: 8,
    FLAG_GYRO: 8,
    FLAG_ANGLE: 8,
    FLAG_MAG: 8,
    FLAG_COMBINED: 18,
}


def s16(lo, hi):
    return struct.unpack("<h", bytes([lo, hi]))[0]


def parse_payload(flag, data):
    vals = [s16(data[i], data[i + 1]) for i in range(0, len(data), 2)]

    if flag == FLAG_ACCEL:
        ax, ay, az, t = vals
        return "ACCEL", {
            "ax_g": ax / 32768 * 16,
            "ay_g": ay / 32768 * 16,
            "az_g": az / 32768 * 16,
            "temp_C": t / 100,
        }
    if flag == FLAG_GYRO:
        wx, wy, wz, t = vals
        return "GYRO", {
            "wx_dps": wx / 32768 * 2000,
            "wy_dps": wy / 32768 * 2000,
            "wz_dps": wz / 32768 * 2000,
            "temp_C": t / 100,
        }
    if flag == FLAG_ANGLE:
        roll, pitch, yaw, t = vals
        return "ANGLE", {
            "roll_deg": roll / 32768 * 180,
            "pitch_deg": pitch / 32768 * 180,
            "yaw_deg": yaw / 32768 * 180,
            "temp_C": t / 100,
        }
    if flag == FLAG_MAG:
        hx, hy, hz, t = vals
        return "MAG", {"hx": hx, "hy": hy, "hz": hz, "temp_C": t / 100}
    if flag == FLAG_COMBINED:
        ax, ay, az, wx, wy, wz, roll, pitch, yaw = vals
        return "IMU", {
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
    return None, None


def read_stream(port, baud):
    ser = serial.Serial(port, baud, timeout=1)
    print(f"Opened {port} @ {baud} baud. Reading WT9011DCL stream (Ctrl+C to stop)...")

    buf = bytearray()
    last_print = 0.0
    latest = {}

    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)

            # Resync to header, then try to parse whatever packet type follows.
            while len(buf) >= 2:
                if buf[0] != HEADER:
                    buf.pop(0)
                    continue

                flag = buf[1]
                plen = PAYLOAD_LEN.get(flag)
                if plen is None:
                    # Unknown flag right after a header byte; drop and resync.
                    buf.pop(0)
                    continue

                total = 2 + plen + 1  # header + flag + payload + checksum
                if len(buf) < total:
                    break  # wait for more bytes

                packet = buf[:total]
                data = packet[2:2 + plen]
                checksum = packet[-1]
                calc = sum(packet[:-1]) & 0xFF

                if checksum != calc:
                    buf.pop(0)  # bad frame, resync one byte at a time
                    continue

                del buf[:total]

                label, fields = parse_payload(flag, data)
                if label:
                    latest[label] = fields

            now = time.time()
            if now - last_print > 0.2 and latest:
                parts = []
                if "IMU" in latest:
                    f = latest["IMU"]
                    parts.append(
                        f"acc(g)=({f['ax_g']:+.3f},{f['ay_g']:+.3f},{f['az_g']:+.3f}) "
                        f"gyro(dps)=({f['wx_dps']:+7.2f},{f['wy_dps']:+7.2f},{f['wz_dps']:+7.2f}) "
                        f"angle(deg)=({f['roll_deg']:+7.2f},{f['pitch_deg']:+7.2f},{f['yaw_deg']:+7.2f})"
                    )
                else:
                    if "ACCEL" in latest:
                        f = latest["ACCEL"]
                        parts.append(f"acc(g)=({f['ax_g']:+.3f},{f['ay_g']:+.3f},{f['az_g']:+.3f})")
                    if "GYRO" in latest:
                        f = latest["GYRO"]
                        parts.append(f"gyro(dps)=({f['wx_dps']:+7.2f},{f['wy_dps']:+7.2f},{f['wz_dps']:+7.2f})")
                    if "ANGLE" in latest:
                        f = latest["ANGLE"]
                        parts.append(f"angle(deg)=({f['roll_deg']:+7.2f},{f['pitch_deg']:+7.2f},{f['yaw_deg']:+7.2f})")
                if parts:
                    print("  ".join(parts))
                last_print = now
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


def main():
    ap = argparse.ArgumentParser(description="Read live stream from a WT9011DCL BLE IMU dongle.")
    ap.add_argument("--port", default="COM4", help="Serial port for the BLE dongle (default: COM4)")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = ap.parse_args()

    try:
        read_stream(args.port, args.baud)
    except serial.SerialException as e:
        print(f"Could not open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

import asyncio
import sys
from bleak import BleakClient, BleakScanner

NAME_FILTER = sys.argv[1] if len(sys.argv) > 1 else "WT901BLE"
NOTIFY_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0

count = 0


def on_notify(_handle, data: bytearray):
    global count
    count += 1
    print(data.hex(" "))


async def main():
    print(f"Scanning for a device matching {NAME_FILTER!r} ...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, adv: bool((d.name or adv.local_name or "").startswith(NAME_FILTER)),
        timeout=10.0,
    )
    if device is None:
        print("Device not found.")
        return

    print(f"Found {device.address} ({device.name}). Connecting...")
    async with BleakClient(device) as client:
        print(f"Connected: {client.is_connected}")
        for svc in client.services:
            print(f"service {svc.uuid}")
            for ch in svc.characteristics:
                print(f"   char {ch.uuid} props={ch.properties}")
        await client.start_notify(NOTIFY_UUID, on_notify)
        await asyncio.sleep(SECONDS)
        await client.stop_notify(NOTIFY_UUID)
    print(f"Total notifications: {count}")


asyncio.run(main())

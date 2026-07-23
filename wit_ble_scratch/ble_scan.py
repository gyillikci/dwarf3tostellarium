import asyncio
from bleak import BleakScanner


async def main():
    print("Scanning for 10s...")
    devices = await BleakScanner.discover(timeout=10.0, return_adv=True)
    for addr, (dev, adv) in devices.items():
        name = dev.name or adv.local_name or "(no name)"
        print(f"{addr}  {name!r}  rssi={adv.rssi}")


asyncio.run(main())

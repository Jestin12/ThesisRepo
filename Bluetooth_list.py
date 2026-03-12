from bleak import BleakScanner
import asyncio

async def scan():
    print("scanning...")
    devices = await BleakScanner.discover()
    for d in devices:
        print(d.address, d.name)

asyncio.run(scan())
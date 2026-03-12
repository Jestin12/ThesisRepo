import asyncio
from bleak import BleakScanner, BleakClient

DEVICE_NAME      = "ESP32S3-Zero-BLE"

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_CHAR_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # notify
UART_CHAR_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # write

received_event = asyncio.Event()
last_payload = None

def notify_handler(sender, data: bytearray):
    global last_payload
    last_payload = data.decode(errors="ignore")
    print("received data")
    print(last_payload)
    received_event.set()

async def main():
    print("looking for ESP32")
    device = None
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == DEVICE_NAME:
            device = d
            break

    if device is None:
        print("ESP32 not found")
        return

    print(f"found {DEVICE_NAME} at {device.address}")

    async with BleakClient(device) as client:
        print("connected to ESP32")

        # Subscribe to TX notifications
        await client.start_notify(UART_CHAR_TX_UUID, notify_handler)

        # Send GET command over RX
        print("requested data")
        await client.write_gatt_char(UART_CHAR_RX_UUID, b"GET", response=True)

        # Wait for one notification with JSON
        try:
            await asyncio.wait_for(received_event.wait(), timeout=5.0)
        except asyncio.TimeoutError:
            print("timeout waiting for data")
        finally:
            await client.stop_notify(UART_CHAR_TX_UUID)

if __name__ == "__main__":
    asyncio.run(main())

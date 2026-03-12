import asyncio
from bleak import BleakClient, BleakScanner

# Must match the name used in your Arduino sketch
DEVICE_NAME = "ESP32S3-Zero-BLE"

# Nordic UART Service UUIDs (same as in the Arduino code)
UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_CHAR_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify (ESP32 -> PC)
UART_CHAR_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write (PC -> ESP32)


async def find_device():
    print(f"Scanning for BLE device named '{DEVICE_NAME}'...")
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == DEVICE_NAME:
            print(f"Found device: {d.name} [{d.address}]")
            return d
    print("Device not found.")
    return None


def handle_notification(sender, data: bytearray):
    # data is a bytes-like object; decode and print
    try:
        text = data.decode("utf-8", errors="replace")
    except Exception:
        text = repr(data)
    print(f"[BLE] {text}", end="")  # Arduino example already includes \r\n


async def run():
    device = await find_device()
    if device is None:
        return

    async with BleakClient(device) as client:
        print("Connected to device.")

        # Start notifications on TX characteristic
        await client.start_notify(UART_CHAR_TX_UUID, handle_notification)
        print("Notifications started. Waiting for data... (Ctrl+C to quit)")

        # Optional: send something once (not required for your current sketch)
        # msg = "Hello from PC\n"
        # await client.write_gatt_char(UART_CHAR_RX_UUID, msg.encode("utf-8"))

        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping notifications and disconnecting...")
        finally:
            await client.stop_notify(UART_CHAR_TX_UUID)


if __name__ == "__main__":
    asyncio.run(run())

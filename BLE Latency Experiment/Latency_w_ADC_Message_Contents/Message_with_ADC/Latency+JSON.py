import asyncio
import json
import time
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "ESP32S3-Zero-BLE"

UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_CHAR_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify
UART_CHAR_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write

pending_start = None
buffer = ""
latest_json = None  # store last parsed JSON dict


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
    """Called when ESP32 sends a notification with JSON."""
    global buffer, pending_start, latest_json
    try:
        text = data.decode("utf-8", errors="replace")
    except Exception:
        text = repr(data)

    buffer += text

    # Process complete lines
    while "\n" in buffer:
        line, buffer_rest = buffer.split("\n", 1)
        buffer = buffer_rest
        line = line.rstrip("\r")

        now = time.perf_counter()
        if pending_start is not None:
            rtt_ms = (now - pending_start) * 1000.0
            print(f"<- {line}   RTT={rtt_ms:.2f} ms")
            pending_start = None
        else:
            print(f"<- {line} (no pending request)")

        # Try to parse JSON
        try:
            latest_json = json.loads(line)
            if "adc" in latest_json and isinstance(latest_json["adc"], list):
                print(f"   adc[0..{len(latest_json['adc'])-1}] = {latest_json['adc']}")
        except json.JSONDecodeError:
            print("   (Warning: received line is not valid JSON)")


async def run():
    global pending_start

    device = await find_device()
    if device is None:
        return

    async with BleakClient(device) as client:
        print("Connected to device.")

        await client.start_notify(UART_CHAR_TX_UUID, handle_notification)
        print("Notifications started. Requesting ADC JSON... (Ctrl+C to stop)")

        try:
            count = 0
            while True:
                # Send GET request
                msg = "GET\n"
                pending_start = time.perf_counter()
                await client.write_gatt_char(UART_CHAR_RX_UUID, msg.encode("utf-8"))
                print(f"-> {msg.strip()} (request {count})")

                # Wait for response
                while pending_start is not None:
                    await asyncio.sleep(0.001)

                count += 1
                # Wait a bit before next measurement
                await asyncio.sleep(0.5)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            await client.stop_notify(UART_CHAR_TX_UUID)


if __name__ == "__main__":
    asyncio.run(run())

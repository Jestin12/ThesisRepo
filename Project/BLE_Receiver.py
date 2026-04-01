#!/usr/bin/env python3
import asyncio
from bleak import BleakScanner, BleakClient
import json
import pandas as pd
from datetime import datetime
from pathlib import Path

DEVICE_NAME      = "ESP32S3-Zero-BLE"

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_CHAR_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # notify
UART_CHAR_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # write (unused)

# Global state
current_buffer = ""
rows = []
run_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def flatten_packet_to_wide(packet: dict, run_ts: str, read_time: str) -> dict:
    """
    Convert one JSON packet into a single 'wide' row:
    columns like index_pitch_prox, index_roll_mid, ring_flex_mcp, etc.
    """
    row = {
        "run_timestamp": run_ts,
        "read_time": read_time,
        "hand": packet.get("Hand"),
        "esp_time_ms": packet.get("Time"),
    }

    data = packet.get("Data", {})
    for finger_name, finger_data in data.items():
        prefix = finger_name.lower()  # "Index" -> "index"
        for metric_key, value in finger_data.items():
            col_name = f"{prefix}_{metric_key}"  # "index_pitch_prox"
            row[col_name] = value

    return row


def notify_handler(sender, data: bytearray):
    """
    Accumulate chunks in current_buffer, split on newline, and parse
    each complete JSON line independently.
    """
    global current_buffer, rows, run_timestamp

    chunk = data.decode(errors="ignore")
    current_buffer += chunk

    # Process all complete lines
    while "\n" in current_buffer:
        line, current_buffer = current_buffer.split("\n", 1)
        line = line.strip()
        if not line:
            continue

        print("Received JSON line:")
        print(line)

        try:
            packet = json.loads(line)
        except json.JSONDecodeError as e:
            print("JSON decode failed for line, skipping:", e)
            continue

        read_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        row = flatten_packet_to_wide(packet, run_timestamp, read_time)
        rows.append(row)
        print("Packet flattened and stored; total rows:", len(rows))


async def main():
    print("Looking for ESP32...")
    device = None
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == DEVICE_NAME:
            device = d
            break

    if device is None:
        print("ESP32 not found")
        return

    print(f"Found {DEVICE_NAME} at {device.address}")

    async with BleakClient(device) as client:
        print("Connected to ESP32")

        await client.start_notify(UART_CHAR_TX_UUID, notify_handler)
        print("Listening for JSON notifications. Press Ctrl+C to stop and save CSV.")

        try:
            while True:
                await asyncio.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopping (Ctrl+C pressed) inside main...")
        finally:
            await client.stop_notify(UART_CHAR_TX_UUID)
            print("Notifications stopped")


def save_csv():
    if not rows:
        print("No packets received; nothing to save.")
        return

    df = pd.DataFrame(rows)
    script_dir = Path(__file__).resolve().parent
    ts_for_filename = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = script_dir / f"glove_data_ble_{ts_for_filename}.csv"
    df.to_csv(csv_path, index=False)
    print(f"Saved CSV to: {csv_path}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C) in outer scope...")

    # Always attempt to save whatever we collected
    save_csv()
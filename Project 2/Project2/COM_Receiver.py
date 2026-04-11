#!/usr/bin/env python3
import serial
import json
from datetime import datetime
import pandas as pd
from pathlib import Path

# ===== USER SETTINGS =====
SERIAL_PORT  = "/dev/ttyACM0"        # Linux: "/dev/ttyUSB0" or "/dev/ttyACM0"
BAUD_RATE    = 115200
# =========================

def flatten_packet_to_wide(packet: dict, run_timestamp: str, read_time: str) -> dict:
    """
    Convert one JSON packet into a single 'wide' row:
    columns like index_pitch_prox, index_roll_mid, ring_flex_mcp, etc.
    """
    row = {
        "run_timestamp": run_timestamp,
        "read_time": read_time,
        "hand": packet.get("Hand"),
        "esp_time_ms": packet.get("Time"),
    }

    data = packet.get("Data", {})
    for finger_name, finger_data in data.items():
        prefix = finger_name.lower()  # e.g. 'Index' -> 'index'
        for metric_key, value in finger_data.items():
            col_name = f"{prefix}_{metric_key}"  # e.g. 'index_pitch_prox'
            row[col_name] = value

    return row

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud")
    print("Press Ctrl+C to stop and save to CSV.")

    all_rows = []
    run_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            print("Serial:", line)

            read_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            try:
                packet = json.loads(line)
            except json.JSONDecodeError:
                print("Skipped non-JSON line")
                continue

            row = flatten_packet_to_wide(packet, run_timestamp, read_time)
            all_rows.append(row)

    except KeyboardInterrupt:
        print("\nStopping logging (Ctrl+C pressed)...")
    finally:
        ser.close()
        print("Serial port closed")

    if not all_rows:
        print("No data received; nothing to save.")
        return

    df = pd.DataFrame(all_rows)

    script_dir = Path(__file__).resolve().parent
    ts_for_filename = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = script_dir / f"glove_data_{ts_for_filename}.csv"

    df.to_csv(csv_path, index=False)
    print(f"Saved CSV to: {csv_path}")

if __name__ == "__main__":
    main()
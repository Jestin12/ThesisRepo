#!/usr/bin/env python3
import socket
import json
import pandas as pd
from datetime import datetime
from pathlib import Path

HOST = "0.0.0.0"   # listen on all interfaces
PORT = 5000        # must match TCP_PORT on ESP32

rows = []
run_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def flatten_packet_to_wide(packet: dict, run_ts: str, read_time: str) -> dict:
    row = {
        "run_timestamp": run_ts,
        "read_time": read_time,
        "hand": packet.get("Hand"),
        "esp_time_ms": packet.get("Time"),
    }
    data = packet.get("Data", {})
    for finger_name, finger_data in data.items():
        prefix = finger_name.lower()
        for metric_key, value in finger_data.items():
            col_name = f"{prefix}_{metric_key}"
            row[col_name] = value
    return row

def save_csv():
    if not rows:
        print("No packets received; nothing to save.")
        return
    df = pd.DataFrame(rows)
    script_dir = Path(__file__).resolve().parent
    ts_for_filename = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = script_dir / f"glove_data_tcp_{ts_for_filename}.csv"
    df.to_csv(csv_path, index=False)
    print(f"Saved CSV to: {csv_path}")

def main():
    global rows

    print(f"Starting TCP server on {HOST}:{PORT}")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print("Waiting for ESP32 connection...")
        conn, addr = s.accept()
        print(f"Connected by {addr}")

        with conn:
            buffer = ""
            try:
                while True:
                    data = conn.recv(4096)
                    if not data:
                        print("Client disconnected.")
                        break
                    buffer += data.decode(errors="ignore")

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
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
            except KeyboardInterrupt:
                print("\nStopping (Ctrl+C)...")

if __name__ == "__main__":
    try:
        main()
    finally:
        save_csv()
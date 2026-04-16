import socket
import json
import time
import re
from datetime import datetime
from pathlib import Path
import pandas as pd


HOST = "0.0.0.0"
PORT = 5000
LISTEN_SECONDS = 10
FILE_PREFIX = "glove_data_rock"


def get_next_run_index():
    pattern = re.compile(rf"^{FILE_PREFIX}_(\d+)_\d{{4}}-\d{{2}}-\d{{2}}_\d{{2}}-\d{{2}}-\d{{2}}\.csv$")
    max_index = 0

    for path in Path(".").glob(f"{FILE_PREFIX}_*.csv"):
        match = pattern.match(path.name)
        if match:
            idx = int(match.group(1))
            if idx > max_index:
                max_index = idx

    return max_index + 1


RUN_INDEX = get_next_run_index()
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
OUTPUT_CSV = f"{FILE_PREFIX}_{RUN_INDEX}_{timestamp}.csv"


def flatten_glove_json(msg, base_time_ms):
    current_time_ms = msg.get("Time")

    row = {
        "run_index": RUN_INDEX,
        "hand": msg.get("Hand"),
        "glove_time_ms": msg.get("glove_time_ms"),
        "time": (current_time_ms - base_time_ms) if current_time_ms is not None and base_time_ms is not None else None,
    }

    data = msg.get("Data", {})

    for sensor_name, sensor_values in data.items():
        sensor = sensor_name.lower()

        for key, value in sensor_values.items():
            if key.startswith("flex_"):
                joint = key.split("_", 1)[1]
                col = f"{sensor}_{joint}_flex"
            elif "_" in key:
                metric, joint = key.split("_", 1)
                col = f"{sensor}_{joint}_{metric}"
            else:
                col = f"{sensor}_{key}"

            row[col] = value

    return row


def main():
    rows = []
    start_time = time.time()
    base_time_ms = None

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen()
        server.settimeout(0.5)

        print(f"Listening on {HOST}:{PORT} for {LISTEN_SECONDS} seconds...")
        print(f"Run index: {RUN_INDEX}")
        print(f"Output file will be: {OUTPUT_CSV}")

        while time.time() - start_time < LISTEN_SECONDS:
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue

            with conn:
                print(f"Connected by {addr}")
                buffer = ""

                while time.time() - start_time < LISTEN_SECONDS:
                    try:
                        data = conn.recv(4096)
                    except ConnectionResetError:
                        break

                    if not data:
                        break

                    buffer += data.decode("utf-8")

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue

                        try:
                            obj = json.loads(line)

                            if base_time_ms is None:
                                base_time_ms = obj.get("Time")

                            rows.append(flatten_glove_json(obj, base_time_ms))
                            print("Received one JSON object")

                        except json.JSONDecodeError as e:
                            print(f"Skipping invalid JSON: {e}")

                leftover = buffer.strip()
                if leftover:
                    try:
                        obj = json.loads(leftover)

                        if base_time_ms is None:
                            base_time_ms = obj.get("Time")

                        rows.append(flatten_glove_json(obj, base_time_ms))
                        print("Received one final JSON object")

                    except json.JSONDecodeError:
                        pass

    if rows:
        df = pd.DataFrame(rows)
        df.to_csv(OUTPUT_CSV, index=False)
        print(f"Saved {len(df)} row(s) to {OUTPUT_CSV}")
    else:
        print("No valid JSON data received. No CSV file created.")


if __name__ == "__main__":
    main()
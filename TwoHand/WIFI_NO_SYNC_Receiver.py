import socket
import json
import time
import re
from datetime import datetime
from pathlib import Path
import pandas as pd
import threading

HOST = "0.0.0.0"
LEFT_PORT = 5001
RIGHT_PORT = 5000
LISTEN_SECONDS = 5
FILE_PREFIX = "glove_data_rock"

def get_next_run_index():
    pattern = re.compile(
        rf"^{FILE_PREFIX}_(left_hand|right_hand)_(\d+)_\d{{4}}-\d{{2}}-\d{{2}}_\d{{2}}-\d{{2}}-\d{{2}}\.csv$"
    )
    max_index = 0

    for path in Path(".").glob(f"{FILE_PREFIX}_*.csv"):
        match = pattern.match(path.name)
        if match:
            idx = int(match.group(2))
            if idx > max_index:
                max_index = idx

    return max_index + 1

RUN_INDEX = get_next_run_index()
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

LEFT_OUTPUT_CSV = f"{FILE_PREFIX}_left_hand_{RUN_INDEX}_{timestamp}.csv"
RIGHT_OUTPUT_CSV = f"{FILE_PREFIX}_right_hand_{RUN_INDEX}_{timestamp}.csv"

def flatten_glove_json(msg, base_time_ms):
    current_time_ms = msg.get("Time")

    row = {
        "run_index": RUN_INDEX,
        "hand": msg.get("Hand"),
        "glove_time_ms": msg.get("glove_time_ms"),
        "time": (current_time_ms - base_time_ms)
        if current_time_ms is not None and base_time_ms is not None
        else None,
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

def listen_on_port(port, expected_hand, rows_out):
    start_time = time.time()
    base_time_ms = None

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, port))
        server.listen()
        server.settimeout(0.5)

        print(f"[{expected_hand}] Listening on {HOST}:{port} for {LISTEN_SECONDS} seconds...")

        while time.time() - start_time < LISTEN_SECONDS:
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue

            with conn:
                print(f"[{expected_hand}] Connected by {addr}")
                buffer = ""

                while time.time() - start_time < LISTEN_SECONDS:
                    try:
                        data = conn.recv(4096)
                    except ConnectionResetError:
                        print(f"[{expected_hand}] Connection reset by peer")
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

                            actual_hand = str(obj.get("Hand", "")).strip().lower()
                            expected_hand_normalized = expected_hand.replace("_", " ").lower()

                            if actual_hand and actual_hand != expected_hand_normalized:
                                print(f"[{expected_hand}] Warning: received Hand='{obj.get('Hand')}', expected '{expected_hand_normalized}'")

                            rows_out.append(flatten_glove_json(obj, base_time_ms))
                            print(f"[{expected_hand}] Received one JSON object")

                        except json.JSONDecodeError as e:
                            print(f"[{expected_hand}] Skipping invalid JSON: {e}")

                leftover = buffer.strip()
                if leftover:
                    try:
                        obj = json.loads(leftover)

                        if base_time_ms is None:
                            base_time_ms = obj.get("Time")

                        actual_hand = str(obj.get("Hand", "")).strip().lower()
                        expected_hand_normalized = expected_hand.replace("_", " ").lower()

                        if actual_hand and actual_hand != expected_hand_normalized:
                            print(f"[{expected_hand}] Warning: received Hand='{obj.get('Hand')}', expected '{expected_hand_normalized}'")

                        rows_out.append(flatten_glove_json(obj, base_time_ms))
                        print(f"[{expected_hand}] Received one final JSON object")

                    except json.JSONDecodeError:
                        pass

def main():
    print(f"Run index: {RUN_INDEX}")
    print(f"Left hand file:  {LEFT_OUTPUT_CSV}")
    print(f"Right hand file: {RIGHT_OUTPUT_CSV}")

    left_rows = []
    right_rows = []

    left_thread = threading.Thread(
        target=listen_on_port,
        args=(LEFT_PORT, "left_hand", left_rows),
        daemon=True
    )
    right_thread = threading.Thread(
        target=listen_on_port,
        args=(RIGHT_PORT, "right_hand", right_rows),
        daemon=True
    )

    left_thread.start()
    right_thread.start()

    left_thread.join()
    right_thread.join()

    if left_rows:
        df_left = pd.DataFrame(left_rows)
        df_left.to_csv(LEFT_OUTPUT_CSV, index=False)
        print(f"Saved {len(df_left)} row(s) to {LEFT_OUTPUT_CSV}")
    else:
        print("No valid LEFT hand JSON data received. No left-hand CSV created.")

    if right_rows:
        df_right = pd.DataFrame(right_rows)
        df_right.to_csv(RIGHT_OUTPUT_CSV, index=False)
        print(f"Saved {len(df_right)} row(s) to {RIGHT_OUTPUT_CSV}")
    else:
        print("No valid RIGHT hand JSON data received. No right-hand CSV created.")

if __name__ == "__main__":
    main()
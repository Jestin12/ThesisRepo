import socket
import json
import time
import re
import threading
from datetime import datetime
from pathlib import Path
import os
import pandas as pd

HOST = "0.0.0.0"
LEFT_PORT = 5000
RIGHT_PORT = 5001
RUN_SECONDS = 20
PAIR_TOLERANCE_MS = 50          # rows within this window are considered paired
FILE_PREFIX = "glove_data_rock"
OUTPUT_DIR  = r"C:\Users\YourName\Documents\GloveData"  # ← set your path here

def get_next_run_index():
    pattern = re.compile(
        rf"^{FILE_PREFIX}_(\d+)_\d{{4}}-\d{{2}}-\d{{2}}_\d{{2}}-\d{{2}}-\d{{2}}\.csv$"
    )
    max_index = 0
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    for path in Path(OUTPUT_DIR).glob(f"{FILE_PREFIX}_*.csv"):
        match = pattern.match(path.name)
        if match:
            max_index = max(max_index, int(match.group(1)))
    return max_index + 1

RUN_INDEX = get_next_run_index()
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
OUTPUT_CSV = os.path.join(OUTPUT_DIR, f"{FILE_PREFIX}_{RUN_INDEX}_{timestamp}.csv")

def flatten_glove_json(msg, hand_label):
    row = {
        "run_index": RUN_INDEX,
        f"{hand_label}_hand": msg.get("Hand"),
        f"{hand_label}_glove_time_ms": msg.get("glove_time_ms"),
        f"{hand_label}_recv_time_ms": msg.get("_wall_time_ms"),
    }
    data = msg.get("Data", {})
    for sensor_name, sensor_values in data.items():
        sensor = sensor_name.lower()
        for key, value in sensor_values.items():
            if key.startswith("flex_"):
                joint = key.split("_", 1)[1]
                col = f"{hand_label}_{sensor}_{joint}_flex"
            elif "_" in key:
                metric, joint = key.split("_", 1)
                col = f"{hand_label}_{sensor}_{joint}_{metric}"
            else:
                col = f"{hand_label}_{sensor}_{key}"
            row[col] = value
    return row


class GloveReceiver:
    def __init__(self, label, port):
        self.label = label
        self.port = port
        self.rows = []          # raw flattened rows with wall_time_ms
        self.lock = threading.Lock()

    def run(self, stop_event):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, self.port))
        server.listen(1)
        server.settimeout(0.5)
        print(f"[{self.label}] Listening on {HOST}:{self.port}")

        conn = None
        while not stop_event.is_set():
            if conn is None:
                try:
                    conn, addr = server.accept()
                    conn.settimeout(0.2)
                    print(f"[{self.label}] Connected by {addr}")
                except socket.timeout:
                    continue

            buffer = ""
            try:
                data = conn.recv(4096)
                if not data:
                    print(f"[{self.label}] Disconnected")
                    conn = None
                    continue
                buffer += data.decode("utf-8")
            except socket.timeout:
                continue
            except ConnectionResetError:
                conn = None
                continue

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                    obj["_wall_time_ms"] = time.time() * 1000
                    row = flatten_glove_json(obj, self.label)
                    with self.lock:
                        self.rows.append(row)
                    print(f"[{self.label}] row received (total={len(self.rows)})")
                except json.JSONDecodeError:
                    print(f"[{self.label}] Invalid JSON skipped")

        server.close()


def pair_rows(left_rows, right_rows, tolerance_ms=PAIR_TOLERANCE_MS):
    """
    Greedy nearest-neighbour pairing by wall_time_ms.
    Each left row is matched to the closest unmatched right row
    within tolerance_ms. Unmatched rows are dropped.
    """
    paired = []
    right_remaining = list(right_rows)  # copy so we can pop

    for lrow in left_rows:
        lt = lrow.get("left_recv_time_ms")
        if lt is None:
            continue
        best_idx, best_diff = None, float("inf")
        for i, rrow in enumerate(right_remaining):
            rt = rrow.get("right_recv_time_ms")
            if rt is None:
                continue
            diff = abs(lt - rt)
            if diff < best_diff:
                best_diff = diff
                best_idx = i
        if best_idx is not None and best_diff <= tolerance_ms:
            merged = {}
            merged.update(left_rows[left_rows.index(lrow)])
            merged.update(right_remaining.pop(best_idx))
            merged["pair_delta_ms"] = round(best_diff, 2)
            paired.append(merged)

    return paired


def main():
    stop_event = threading.Event()
    left_glove  = GloveReceiver("left",  LEFT_PORT)
    right_glove = GloveReceiver("right", RIGHT_PORT)

    t_left  = threading.Thread(target=left_glove.run,  args=(stop_event,), daemon=True)
    t_right = threading.Thread(target=right_glove.run, args=(stop_event,), daemon=True)
    t_left.start()
    t_right.start()

    # Wait for both gloves to connect before starting the timer
    print("Waiting for both gloves to connect...")
    while True:
        # Peek: a row arriving means a connection was made
        with left_glove.lock:
            left_up = len(left_glove.rows) > 0
        with right_glove.lock:
            right_up = len(right_glove.rows) > 0
        if left_up and right_up:
            break
        # Alternative: just wait for the accept print and then press Enter
        time.sleep(0.1)

    input("Both gloves streaming. Press Enter to begin recording...")

    # Clear any pre-Enter rows
    with left_glove.lock:
        left_glove.rows.clear()
    with right_glove.lock:
        right_glove.rows.clear()

    print(f"Recording for {RUN_SECONDS}s ...")
    time.sleep(RUN_SECONDS)
    stop_event.set()

    with left_glove.lock:
        left_rows = list(left_glove.rows)
    with right_glove.lock:
        right_rows = list(right_glove.rows)

    print(f"Collected: left={len(left_rows)}, right={len(right_rows)}")

    paired = pair_rows(left_rows, right_rows, PAIR_TOLERANCE_MS)
    print(f"Paired rows: {len(paired)}")

    if paired:
        df = pd.DataFrame(paired)

        # ── Time columns first, then everything else ─────────────────────
        time_cols = [c for c in df.columns if any(t in c for t in (
            "time", "glove_time", "recv_time", "pair_delta"
        ))]
        other_cols = [c for c in df.columns if c not in time_cols and c != "run_index"]
        df = df[["run_index"] + time_cols + other_cols]

        df.to_csv(OUTPUT_CSV, index=False)
        print(f"Saved {len(df)} paired row(s) to {OUTPUT_CSV}")
    else:
        print("No paired rows. No CSV created.")

if __name__ == "__main__":
    main()

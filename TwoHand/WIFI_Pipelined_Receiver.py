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
RUN_SECONDS = 5
REQUEST_PIPELINE_INTERVAL = 0.005   # 5 ms between sends — tune down if gloves keep up
FILE_PREFIX = f"glove_data_L_Point_R_Point_{RUN_SECONDS}s"
OUTPUT_DIR = r"/home/jestin/ThesisRepo/ML/TwoHand_L_Point_R_Point"


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


def flatten_glove_json(msg, base_time_ms, hand_label):
    current_time_ms = msg.get("Time")
    row = {
        "run_index": RUN_INDEX,
        "request_id": msg.get("request_id"),
        "request_ts": msg.get("request_ts"),
        f"{hand_label}_hand": msg.get("Hand"),
        f"{hand_label}_recv_time_ms": msg.get("_recv_time_ms"),
        f"{hand_label}_glove_time_ms": msg.get("glove_time_ms"),
        f"{hand_label}_time": (
            current_time_ms - base_time_ms
            if current_time_ms is not None and base_time_ms is not None
            else None
        ),
    }

    data = msg.get("Data", {})
    for sensor_name, sensor_values in data.items():
        sensor = sensor_name.lower()

        for key, value in sensor_values.items():
            if key.startswith("flex_"):
                joint = key[len("flex_"):]
                col = f"{hand_label}_{sensor}_{joint}_flex"

            elif key.startswith("quat_"):
                parts = key.split("_")
                if len(parts) == 3:
                    _, component, segment = parts
                    col = f"{hand_label}_{sensor}_{segment}_quat_{component}"
                elif sensor == "wrist" and len(parts) == 2:
                    _, component = parts
                    col = f"{hand_label}_{sensor}_quat_{component}"
                else:
                    col = f"{hand_label}_{sensor}_{key}"

            else:
                parts = key.split("_")
                if len(parts) == 2:
                    metric, segment = parts
                    col = f"{hand_label}_{sensor}_{segment}_{metric}"
                else:
                    col = f"{hand_label}_{sensor}_{key}"

            row[col] = value

    return row


def reorder_columns(df):
    base_cols = ["run_index", "request_id"]

    time_cols = [
        c for c in df.columns
        if c not in base_cols and any(t in c for t in (
            "request_ts", "recv_time", "glove_time", "_time"
        ))
    ]

    hand_cols = [c for c in ("left_hand", "right_hand") if c in df.columns]

    hand_order = {"left": 0, "right": 1}
    sensor_order = {
        "palm": 0,
        "thumb": 1,
        "index": 2,
        "middle": 3,
        "ring": 4,
        "pinky": 5,
        "wrist": 6,
    }
    segment_order = {
        "mid": 0,
        "prox": 1,
        "mcp": 2,
        "pip": 3,
    }
    metric_order = {
        "yaw": 0,
        "pitch": 1,
        "roll": 2,
        "quat_w": 3,
        "quat_x": 4,
        "quat_y": 5,
        "quat_z": 6,
        "ax": 7,
        "ay": 8,
        "az": 9,
        "flex": 10,
    }

    wrist_metric_order = {
        "heading": 0,
        "pitch": 1,
        "roll": 2,
        "quat_w": 3,
        "quat_x": 4,
        "quat_y": 5,
        "quat_z": 6,
        "ax": 7,
        "ay": 8,
        "az": 9,
    }

    def sensor_sort_key(col):
        parts = col.split("_")

        hand = parts[0] if len(parts) > 0 else ""
        sensor = parts[1] if len(parts) > 1 else ""

        if sensor == "wrist":
            metric = "_".join(parts[2:]) if len(parts) > 2 else ""
            return (
                hand_order.get(hand, 99),
                sensor_order.get(sensor, 99),
                99,
                wrist_metric_order.get(metric, 99),
                col,
            )

        if len(parts) == 4 and parts[3] == "flex":
            segment = parts[2]
            metric = "flex"
            return (
                hand_order.get(hand, 99),
                sensor_order.get(sensor, 99),
                segment_order.get(segment, 99),
                metric_order.get(metric, 99),
                col,
            )

        if len(parts) == 5 and parts[3] == "quat":
            segment = parts[2]
            metric = f"{parts[3]}_{parts[4]}"
            return (
                hand_order.get(hand, 99),
                sensor_order.get(sensor, 99),
                segment_order.get(segment, 99),
                metric_order.get(metric, 99),
                col,
            )

        if len(parts) == 4:
            segment = parts[2]
            metric = parts[3]
            return (
                hand_order.get(hand, 99),
                sensor_order.get(sensor, 99),
                segment_order.get(segment, 99),
                metric_order.get(metric, 99),
                col,
            )

        return (99, 99, 99, 99, col)

    signal_cols = [
        c for c in df.columns
        if c not in base_cols and c not in time_cols and c not in hand_cols
    ]
    signal_cols = sorted(signal_cols, key=sensor_sort_key)

    ordered_cols = base_cols + time_cols + hand_cols + signal_cols
    return df[ordered_cols]


class GloveConnection:
    def __init__(self, label, port):
        self.label = label
        self.port = port
        self.server = None
        self.conn = None
        self.addr = None
        self.buffer = ""
        self.base_time_ms = None
        self.connected = False

    def setup_server(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((HOST, self.port))
        self.server.listen(1)
        self.server.settimeout(0.5)

    def accept(self):
        while not self.connected:
            try:
                conn, addr = self.server.accept()
                conn.settimeout(0.1)
                self.conn = conn
                self.addr = addr
                self.connected = True
                print(f"[{self.label}] Connected by {addr}")
            except socket.timeout:
                continue

    def send_json(self, obj):
        if not self.conn:
            return
        payload = json.dumps(obj) + "\n"
        try:
            self.conn.sendall(payload.encode("utf-8"))
        except (BrokenPipeError, ConnectionResetError):
            print(f"[{self.label}] Send failed — connection lost")
            self.connected = False

    def drain(self, combined_rows):
        if not self.conn:
            return
        try:
            data = self.conn.recv(4096)
            if not data:
                print(f"[{self.label}] Disconnected during drain")
                self.connected = False
                return
            self.buffer += data.decode("utf-8")
        except socket.timeout:
            pass
        except ConnectionResetError:
            self.connected = False
            return

        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                print(f"[{self.label}] Invalid JSON skipped")
                continue

            if obj.get("type") is not None:
                continue

            obj["_recv_time_ms"] = time.time() * 1000

            if self.base_time_ms is None:
                self.base_time_ms = obj.get("Time")

            req_id = obj.get("request_id")
            if req_id not in combined_rows:
                combined_rows[req_id] = {}

            flattened = flatten_glove_json(obj, self.base_time_ms, self.label)
            combined_rows[req_id].update(flattened)
            print(f"[{self.label}] reply for request_id={req_id}")

    def close(self):
        for s in (self.conn, self.server):
            if s:
                try:
                    s.close()
                except Exception:
                    pass


def accept_worker(glove):
    glove.setup_server()
    print(f"[{glove.label}] Listening on {HOST}:{glove.port}")
    glove.accept()


def main():
    left = GloveConnection("left", LEFT_PORT)
    right = GloveConnection("right", RIGHT_PORT)
    combined_rows = {}

    t_left = threading.Thread(target=accept_worker, args=(left,), daemon=True)
    t_right = threading.Thread(target=accept_worker, args=(right,), daemon=True)
    t_left.start()
    t_right.start()
    t_left.join()
    t_right.join()

    print("Both gloves connected.")
    input("Press Enter to begin requesting data...")

    start = time.time()
    request_id = 0

    while time.time() - start < RUN_SECONDS:
        request_id += 1
        request_ts = datetime.now().isoformat(timespec="milliseconds")

        cmd = {
            "type": "REQUEST_DATA",
            "request_id": request_id,
            "request_ts": request_ts,
        }

        left.send_json(cmd)
        right.send_json(cmd)
        print(f"[SERVER] Sent REQUEST_DATA {request_id}")

        left.drain(combined_rows)
        right.drain(combined_rows)

        time.sleep(REQUEST_PIPELINE_INTERVAL)

    drain_deadline = time.time() + 2.0
    while time.time() < drain_deadline:
        left.drain(combined_rows)
        right.drain(combined_rows)
        time.sleep(0.005)

    complete = {
        rid: row for rid, row in combined_rows.items()
        if "left_hand" in row and "right_hand" in row
    }
    incomplete = len(combined_rows) - len(complete)
    print(f"Total requests: {request_id} | Complete pairs: {len(complete)} | Incomplete: {incomplete}")

    if complete:
        ordered = [complete[k] for k in sorted(complete.keys())]
        df = pd.DataFrame(ordered)
        df = reorder_columns(df)

        df.to_csv(OUTPUT_CSV, index=False)
        print(f"Saved {len(df)} paired row(s) to {OUTPUT_CSV}")
    else:
        print("No complete paired rows. No CSV created.")

    left.close()
    right.close()


if __name__ == "__main__":
    main()
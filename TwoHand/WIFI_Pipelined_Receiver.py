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
REQUEST_PIPELINE_INTERVAL = 0.005   # 5 ms between sends — tune down if gloves keep up
FILE_PREFIX = "glove_data_rock"
OUTPUT_DIR  = r"/home/jestin/ThesisData/"  # ← set your path here

def get_next_run_index():
    pattern = re.compile(
        rf"^{FILE_PREFIX}_(\d+)_\d{{4}}-\d{{2}}-\d{{2}}_\d{{2}}-\d{{2}}-\d{{2}}\.csv$"
    )
    max_index = 0
    for path in Path(".").glob(f"{FILE_PREFIX}_*.csv"):
        match = pattern.match(path.name)
        if match:
            max_index = max(max_index, int(match.group(1)))
    return max_index + 1

RUN_INDEX = get_next_run_index()
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
os.makedirs(OUTPUT_DIR, exist_ok=True)
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
                joint = key.split("_", 1)[1]
                col = f"{hand_label}_{sensor}_{joint}_flex"
            elif "_" in key:
                metric, joint = key.split("_", 1)
                col = f"{hand_label}_{sensor}_{joint}_{metric}"
            else:
                col = f"{hand_label}_{sensor}_{key}"
            row[col] = value
    return row


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
        """
        Non-blocking drain of the receive buffer.
        Appends any complete JSON lines into combined_rows keyed by request_id.
        """
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

            # Ignore control messages echoed back
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
    left  = GloveConnection("left",  LEFT_PORT)
    right = GloveConnection("right", RIGHT_PORT)
    combined_rows = {}

    # Accept both connections in parallel
    t_left  = threading.Thread(target=accept_worker, args=(left,),  daemon=True)
    t_right = threading.Thread(target=accept_worker, args=(right,), daemon=True)
    t_left.start()
    t_right.start()
    t_left.join()
    t_right.join()

    print("Both gloves connected.")
    input("Press Enter to begin requesting data...")

    start      = time.time()
    request_id = 0

    while time.time() - start < RUN_SECONDS:
        request_id += 1
        request_ts = datetime.now().isoformat(timespec="milliseconds")

        cmd = {
            "type": "REQUEST_DATA",
            "request_id": request_id,
            "request_ts": request_ts,
        }

        # Fire-and-forget: send to both, do NOT wait for replies
        left.send_json(cmd)
        right.send_json(cmd)
        print(f"[SERVER] Sent REQUEST_DATA {request_id}")

        # Drain whatever has come back so far (non-blocking)
        left.drain(combined_rows)
        right.drain(combined_rows)

        time.sleep(REQUEST_PIPELINE_INTERVAL)

    # After the timed loop ends, keep draining for up to 2s to catch
    # any in-flight replies for the last few request IDs
    drain_deadline = time.time() + 2.0
    while time.time() < drain_deadline:
        left.drain(combined_rows)
        right.drain(combined_rows)
        time.sleep(0.005)

    # Build output — only keep rows where BOTH gloves replied
    complete = {
        rid: row for rid, row in combined_rows.items()
        if f"left_hand" in str(row) and f"right_hand" in str(row)
    }
    incomplete = len(combined_rows) - len(complete)
    print(f"Total requests: {request_id} | Complete pairs: {len(complete)} | Incomplete: {incomplete}")

    if complete:
        ordered = [complete[k] for k in sorted(complete.keys())]
        df = pd.DataFrame(ordered)

        # ── Time columns first, then everything else ─────────────────────
        time_cols = [c for c in df.columns if any(t in c for t in (
            "time", "glove_time", "recv_time", "request_ts"
        ))]
        other_cols = [c for c in df.columns if c not in time_cols
                      and c not in ("run_index", "request_id")]
        df = df[["run_index", "request_id"] + time_cols + other_cols]

        df.to_csv(OUTPUT_CSV, index=False)
        print(f"Saved {len(df)} paired row(s) to {OUTPUT_CSV}")
    else:
        print("No complete paired rows. No CSV created.")

    left.close()
    right.close()

if __name__ == "__main__":
    main()

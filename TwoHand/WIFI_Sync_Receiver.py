import socket
import json
import time
import re
import threading
from datetime import datetime
from pathlib import Path
import pandas as pd

HOST = "0.0.0.0"
LEFT_PORT = 5000
RIGHT_PORT = 5001
REQUEST_INTERVAL_TIMEOUT = 2.0
RUN_SECONDS = 20
FILE_PREFIX = "glove_data_rock"

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
OUTPUT_CSV = f"{FILE_PREFIX}_{RUN_INDEX}_{timestamp}.csv"

def flatten_glove_json(msg, base_time_ms, hand_label):
    current_time_ms = msg.get("Time")

    row = {
        "run_index": RUN_INDEX,
        "request_id": msg.get("request_id"),
        "request_ts": msg.get("request_ts"),
        f"{hand_label}_hand": msg.get("Hand"),
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
        self.last_packet = None
        self.last_request_id_received = None

    def setup_server(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((HOST, self.port))
        self.server.listen(1)
        self.server.settimeout(0.5)

    def accept(self):
        while self.conn is None:
            try:
                conn, addr = self.server.accept()
                conn.settimeout(0.2)
                self.conn = conn
                self.addr = addr
                print(f"[{self.label}] Connected by {addr}")
            except socket.timeout:
                continue

    def send_json(self, obj):
        if not self.conn:
            return
        payload = json.dumps(obj) + "\n"
        self.conn.sendall(payload.encode("utf-8"))

    def recv_lines(self):
        if not self.conn:
            return []

        lines = []
        try:
            data = self.conn.recv(4096)
            if not data:
                raise ConnectionError("client disconnected")
            self.buffer += data.decode("utf-8")
            while "\n" in self.buffer:
                line, self.buffer = self.buffer.split("\n", 1)
                line = line.strip()
                if line:
                    lines.append(line)
        except socket.timeout:
            pass
        return lines

    def process_incoming(self, combined_rows):
        for line in self.recv_lines():
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                print(f"[{self.label}] Invalid JSON skipped")
                continue

            msg_type = obj.get("type")
            if msg_type is not None:
                print(f"[{self.label}] Control message received: {msg_type}")
                continue

            if self.base_time_ms is None:
                self.base_time_ms = obj.get("Time")

            self.last_packet = obj
            self.last_request_id_received = obj.get("request_id")

            req_id = obj.get("request_id")
            if req_id not in combined_rows:
                combined_rows[req_id] = {}

            flattened = flatten_glove_json(obj, self.base_time_ms, self.label)
            combined_rows[req_id].update(flattened)

            print(f"[{self.label}] Data received for request_id={self.last_request_id_received}")

    def close(self):
        if self.conn:
            try:
                self.conn.close()
            except Exception:
                pass
        if self.server:
            try:
                self.server.close()
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

    t1 = threading.Thread(target=accept_worker, args=(left,), daemon=True)
    t2 = threading.Thread(target=accept_worker, args=(right,), daemon=True)
    t1.start()
    t2.start()
    t1.join()
    t2.join()

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
            "request_ts": request_ts
        }

        left.last_request_id_received = None
        right.last_request_id_received = None

        left.send_json(cmd)
        right.send_json(cmd)

        print(f"[SERVER] Sent REQUEST_DATA {request_id} to both gloves")

        deadline = time.time() + REQUEST_INTERVAL_TIMEOUT
        while time.time() < deadline:
            left.process_incoming(combined_rows)
            right.process_incoming(combined_rows)

            if left.last_request_id_received == request_id and right.last_request_id_received == request_id:
                print(f"[SERVER] Both gloves replied for request_id={request_id}")
                break

            time.sleep(0.005)
        else:
            print(f"[SERVER] Timeout waiting for both gloves on request_id={request_id}")

    if combined_rows:
        ordered_rows = [combined_rows[k] for k in sorted(combined_rows.keys())]
        df = pd.DataFrame(ordered_rows)
        df.to_csv(OUTPUT_CSV, index=False)
        print(f"Saved {len(df)} combined row(s) to {OUTPUT_CSV}")
    else:
        print("No valid JSON data received. No CSV file created.")

    left.close()
    right.close()

if __name__ == "__main__":
    main()
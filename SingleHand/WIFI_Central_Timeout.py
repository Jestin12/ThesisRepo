#!/usr/bin/env python3
import json
import os
import signal
import socket
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import pandas as pd
from pynput import keyboard

HOST = "0.0.0.0"
PORT = 5000
SOCKET_TIMEOUT_S = 0.5
REQUEST_INTERVAL_S = 0.1
RUN_DURATION_S = 5  # <<< added: run for 5 seconds

# --- CHANGE 1: Only one glove ---
GLOVE = "LeftGlove"          # Change to "RightGlove" if needed
GLOVES = (GLOVE,)            # Kept as tuple so existing tuple-checks still work

run_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
script_dir = Path(__file__).resolve().parent

pending_conns = {}
clients = {}
ready = {hand: False for hand in GLOVES}
rows_by_request = {}
request_counter = 0
request_loop_started = False
init_sent = False
waiting_for_release_after_ready = False
stop_armed = False
state_lock = threading.Lock()
shutdown_event = threading.Event()
keys_held = set()
listener_ref = None

REQUIRED_KEYS = {"3", "r", "i", "0", "space"}


def utc_now_iso_ms() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def normalize_key(key):
    try:
        if key == keyboard.Key.space:
            return "space"
        if hasattr(key, "char") and key.char is not None:
            return key.char.lower()
    except AttributeError:
        pass
    return None


def chord_is_down() -> bool:
    return REQUIRED_KEYS.issubset(keys_held)


def flatten_hand_data(packet: dict, prefix: str) -> dict:
    row = {
        f"{prefix}_glove_time_ms": packet.get("glove_time_ms", packet.get("Time")),
        f"{prefix}_read_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
    }
    data = packet.get("Data", {})
    if isinstance(data, dict):
        for finger_name, finger_data in data.items():
            finger_prefix = f"{prefix}_{str(finger_name).lower()}"
            if isinstance(finger_data, dict):
                for metric_key, value in finger_data.items():
                    row[f"{finger_prefix}_{metric_key}"] = value
            else:
                row[finger_prefix] = finger_data
    return row


def ensure_request_row(request_id: int, request_ts: str):
    if request_id not in rows_by_request:
        rows_by_request[request_id] = {
            "run_timestamp": run_timestamp,
            "request_id": request_id,
            "request_ts": request_ts,
            # --- CHANGE 2: Only track the single glove ---
            "glove_received": False,
        }
    return rows_by_request[request_id]


def send_json(conn: socket.socket, obj: dict) -> bool:
    try:
        conn.sendall((json.dumps(obj) + "\n").encode("utf-8"))
        return True
    except OSError:
        return False


def all_connected_sockets():
    seen = set()
    sockets = []
    with state_lock:
        for conn in pending_conns.values():
            if id(conn) not in seen:
                seen.add(id(conn))
                sockets.append(conn)
        for conn in clients.values():
            if id(conn) not in seen:
                seen.add(id(conn))
                sockets.append(conn)
    return sockets


def send_restart_to_all():
    sockets = all_connected_sockets()
    print(f"Sending RESTART to {len(sockets)} glove socket(s) before shutdown...")
    for conn in sockets:
        send_json(conn, {"type": "RESTART"})
    time.sleep(0.2)


def unregister_socket(conn: socket.socket):
    global request_loop_started
    hand_to_remove = None
    addr = None

    with state_lock:
        for stored_addr, stored_conn in list(pending_conns.items()):
            if stored_conn is conn:
                addr = stored_addr
                pending_conns.pop(stored_addr, None)
                break

        for hand, stored_conn in list(clients.items()):
            if stored_conn is conn:
                hand_to_remove = hand
                clients.pop(hand, None)
                ready[hand] = False
                request_loop_started = False
                break

    try:
        conn.close()
    except OSError:
        pass

    if hand_to_remove:
        print(f"[{hand_to_remove}] disconnected")
    elif addr:
        print(f"Pending client disconnected: {addr}")


def all_ready() -> bool:
    # --- CHANGE 3: all() over the single-glove ready dict still works correctly ---
    return all(ready.values())


def send_init_to_all():
    dead = []
    with state_lock:
        items = list(pending_conns.items())

    print(f"Sending INIT to {len(items)} connected socket(s)")
    for addr, conn in items:
        print(f"INIT target socket: {addr}")
        if not send_json(conn, {"type": "INIT"}):
            dead.append(conn)

    for conn in dead:
        unregister_socket(conn)

    print("INIT sent to connected glove")


def send_request_to_all():
    global request_counter
    with state_lock:
        request_counter += 1
        request_id = request_counter
        request_ts = utc_now_iso_ms()
        ensure_request_row(request_id, request_ts)
        items = list(clients.items())

    msg = {
        "type": "REQUEST_DATA",
        "request_id": request_id,
        "request_ts": request_ts,
    }

    dead = []
    for hand, conn in items:
        if not send_json(conn, msg):
            dead.append(conn)
        else:
            print(f"REQUEST_DATA sent to {hand}, request_id={request_id}")

    for conn in dead:
        unregister_socket(conn)


def save_combined_csv():
    if not rows_by_request:
        print("No packets received; nothing to save.")
        return

    ordered_rows = [rows_by_request[k] for k in sorted(rows_by_request.keys())]
    df = pd.DataFrame(ordered_rows)
    csv_path = script_dir / f"glove_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    df.to_csv(csv_path, index=False)
    print(f"Saved CSV to: {csv_path}")


def start_request_loop_once():
    global request_loop_started, stop_armed
    with state_lock:
        if request_loop_started:
            return
        request_loop_started = True
        stop_armed = True
    print("Starting periodic request loop.")
    print("Hold 3 + R + I + 0 + Space again to stop the program.")
    threading.Thread(target=request_loop, daemon=True).start()


def trigger_ctrl_c_behavior(reason: str):
    print(reason)
    os.kill(os.getpid(), signal.SIGINT)


def on_press(key):
    global init_sent, stop_armed
    name = normalize_key(key)
    if name is None:
        return

    keys_held.add(name)

    if not init_sent and chord_is_down():
        print("Key chord detected. Sending INIT.")
        init_sent = True
        send_init_to_all()
        return

    if init_sent and stop_armed and chord_is_down() and not waiting_for_release_after_ready:
        stop_armed = False
        trigger_ctrl_c_behavior("Stop chord detected. Triggering Ctrl+C behavior...")


def on_release(key):
    global waiting_for_release_after_ready
    name = normalize_key(key)
    if name is not None and name in keys_held:
        keys_held.discard(name)

    if waiting_for_release_after_ready and not chord_is_down():
        waiting_for_release_after_ready = False
        print("Key chord released after glove READY message.")
        start_request_loop_once()


def handle_packet(packet: dict, conn: socket.socket, addr):
    global waiting_for_release_after_ready
    msg_type = packet.get("type")

    if msg_type == "READY":
        hand = packet.get("hand")
        if hand not in GLOVES:
            print(f"Ignoring READY from unknown hand: {packet}")
            return

        with state_lock:
            clients[hand] = conn
            ready[hand] = True
            everyone_ready = all_ready()

        print(f"[{hand}] READY from {addr}")

        # --- CHANGE 4: Prompt for single glove, no "both gloves" wording ---
        if everyone_ready:
            if chord_is_down():
                waiting_for_release_after_ready = True
                print("Glove is READY. Release 3 + R + I + 0 + Space to begin data requests...")
            else:
                start_request_loop_once()
        return

    hand = packet.get("Hand")
    if hand not in GLOVES:
        print(f"Ignoring packet with unknown Hand field: {packet}")
        return

    request_id = packet.get("request_id")
    request_ts = packet.get("request_ts")
    if request_id is None or request_ts is None:
        print(f"Ignoring data packet without request_id/request_ts from {hand}")
        return

    # --- CHANGE 5: Derive prefix from the configured single glove ---
    prefix = "left" if GLOVE == "LeftGlove" else "right"
    with state_lock:
        clients[hand] = conn
        row = ensure_request_row(request_id, request_ts)
        row.update(flatten_hand_data(packet, prefix))
        row["glove_received"] = True

    print(f"[{hand}] stored response for request_id={request_id}")


def client_reader(conn: socket.socket, addr):
    conn.settimeout(SOCKET_TIMEOUT_S)
    buffer = ""

    try:
        while not shutdown_event.is_set():
            try:
                data = conn.recv(4096)
            except socket.timeout:
                continue

            if not data:
                break

            buffer += data.decode(errors="ignore")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    packet = json.loads(line)
                except json.JSONDecodeError as e:
                    print(f"JSON decode failed from {addr}: {e}")
                    continue

                print(f"Packet from {addr}: {packet}")
                handle_packet(packet, conn, addr)
    except OSError as e:
        print(f"Socket error from {addr}: {e}")
    finally:
        unregister_socket(conn)


def accept_loop(server_socket: socket.socket):
    server_socket.settimeout(SOCKET_TIMEOUT_S)
    while not shutdown_event.is_set():
        try:
            conn, addr = server_socket.accept()
        except socket.timeout:
            continue
        except OSError:
            break

        print(f"Incoming TCP connection from {addr}")
        with state_lock:
            pending_conns[addr] = conn

        threading.Thread(target=client_reader, args=(conn, addr), daemon=True).start()


def request_loop():
    while not shutdown_event.is_set():
        with state_lock:
            ready_now = all_ready()
        if not ready_now:
            time.sleep(0.1)
            continue
        send_request_to_all()
        time.sleep(REQUEST_INTERVAL_S)


# <<< added: 5-second shutdown timer
def arm_shutdown_timer():
    def _timer():
        time.sleep(RUN_DURATION_S)
        print(f"{RUN_DURATION_S} seconds elapsed, stopping...")
        shutdown_event.set()
    threading.Thread(target=_timer, daemon=True).start()


def main():
    global listener_ref
    print(f"Starting TCP server on {HOST}:{PORT}")
    print(f"Configured for single glove: {GLOVE}")
    print(f"Program will auto-stop after {RUN_DURATION_S} seconds.")

    arm_shutdown_timer()  # <<< start timer

    listener_ref = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener_ref.start()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((HOST, PORT))
        server_socket.listen(5)
        print("Waiting for glove connection...")
        print("Hold 3 + R + I + 0 + Space to send INIT...")

        threading.Thread(target=accept_loop, args=(server_socket,), daemon=True).start()

        while not shutdown_event.is_set():
            shutdown_event.wait(0.25)

    if listener_ref is not None:
        listener_ref.stop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C)...")
        send_restart_to_all()
    finally:
        shutdown_event.set()
        save_combined_csv()
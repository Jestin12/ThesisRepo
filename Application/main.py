import bluetooth
import json
import threading

TARGET_NAMES = ["ESP32_A", "ESP32_B"]

def find_target_addresses():
    print("Discovering devices...")
    devices = bluetooth.discover_devices(lookup_names=True)
    addrs = {}

    for addr, name in devices:
        print(f"Found {name} @ {addr}")
        if name in TARGET_NAMES:
            addrs[name] = addr

    missing = [n for n in TARGET_NAMES if n not in addrs]
    if missing:
        raise RuntimeError(f"Did not find devices: {missing}")
    return addrs

def connect_rfccomm(addr):
    services = bluetooth.find_service(address=addr)
    if not services:
        raise RuntimeError(f"No services for {addr}")

    # Pick first RFCOMM service
    svc = next(
        (s for s in services if s["protocol"] == "RFCOMM"),
        services[0],
    )
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((svc["host"], svc["port"]))
    return sock

def reader_thread(name, sock):
    buf = b""
    while True:
        data = sock.recv(1024)
        if not data:
            break
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            if not line:
                continue
            try:
                obj = json.loads(line.decode("utf-8"))
                print(f"{name} -> {obj}")
            except json.JSONDecodeError as e:
                print(f"{name} JSON error: {e} | line={line!r}")

def main():
    addrs = find_target_addresses()
    sockets = {}
    for name in TARGET_NAMES:
        print(f"Connecting to {name} @ {addrs[name]}...")
        sock = connect_rfccomm(addrs[name])
        sockets[name] = sock
        t = threading.Thread(target=reader_thread, args=(name, sock), daemon=True)
        t.start()

    print("Connections established. Press Ctrl+C to exit.")
    try:
        while True:
            pass
    except KeyboardInterrupt:
        for s in sockets.values():
            s.close()

if __name__ == "__main__":
    main()

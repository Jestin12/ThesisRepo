import json
import time
import pandas as pd
import serial


PORT = "/dev/ttyACM0"          # Change this to your serial port, e.g. COM5 on Windows or /dev/ttyUSB0 on Linux
BAUDRATE = 115200      # Must match Serial.begin(115200) on the ESP32
LISTEN_SECONDS = 5
OUTPUT_CSV = "glove_data_serial.csv"


def flatten_glove_json(msg):
    row = {
        "hand": msg.get("Hand"),
        "glove_time_ms": msg.get("glove_time_ms"),
        "time": msg.get("Time"),
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

    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        ser.reset_input_buffer()
        print(f"Listening on {PORT} at {BAUDRATE} baud for {LISTEN_SECONDS} seconds...")

        while time.time() - start_time < LISTEN_SECONDS:
            raw = ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                obj = json.loads(line)
                rows.append(flatten_glove_json(obj))
                print("Received one JSON object")

            except json.JSONDecodeError:
                print(f"Skipping invalid JSON line: {line}")

    if rows:
        df = pd.DataFrame(rows)
        df.to_csv(OUTPUT_CSV, index=False)
        print(f"Saved {len(df)} row(s) to {OUTPUT_CSV}")
    else:
        print("No valid JSON data received. No CSV file created.")


if __name__ == "__main__":
    main()
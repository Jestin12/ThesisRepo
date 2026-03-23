import serial
import csv
from datetime import datetime
import os
import sys

# --- SERIAL CONFIG ---
PORT = "COM3"       # change for your system
BAUD = 115200
# ---------------------

def get_csv_path():
    # If user gave a filename: python logger.py myfile.csv
    if len(sys.argv) > 1:
        name = sys.argv[1]
    else:
        name = "data_w_2RC.csv"

    # If they only gave a bare name (no path), save next to this script
    if not os.path.isabs(name):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        name = os.path.join(script_dir, name)

    # Ensure it ends with .csv
    if not name.lower().endswith(".csv"):
        name += ".csv"

    return name


def main():
    csv_filename = get_csv_path()
    ser = serial.Serial(PORT, BAUD, timeout=1)

    with open(csv_filename, "a", newline="") as f:
        writer = csv.writer(f)

        if f.tell() == 0:
            writer.writerow(["pc_timestamp", "raw_line"])

        print(f"Logging from {PORT} at {BAUD} to {csv_filename} (Ctrl+C to stop)")

        try:
            while True:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                ts = datetime.now().isoformat(timespec="milliseconds")
                writer.writerow([ts, line])
                f.flush()
                print(ts, line)

        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            ser.close()


if __name__ == "__main__":
    main()

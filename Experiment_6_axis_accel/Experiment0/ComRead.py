import serial
import time
import csv
from datetime import datetime
import re

# Configuration
PORT = "COM6"          # Change to your port
BAUD_RATE = 115200
TIMEOUT = 1
CSV_FILE = "stable_hand_2G_scale_5ms_115200_baud_data.csv"

# Expected field pattern: S1_X, S1_Y, ..., S2_r
FIELD_PATTERN = re.compile(r'(S[12])_([XYZpr])\s+([-+]?\d*\.\d+)|nan')

def parse_line(line):
    """
    Parse a line like:
    S1_X 1.00,S1_Y 0.02,S1_Z 0.98,S1_p 5.70, S1_r 1.15,S2_X 0.95,S2_Y -0.03,S2_Z 1.01,S2_p 3.20,S2_r -1.70
    Returns a dictionary with keys: S1_X, S1_Y, ..., S2_r
    """
    data = {}
    # Find all key-value pairs
    matches = FIELD_PATTERN.finditer(line)
    for match in matches:
        if match.group(1) and match.group(2):  # Valid sensor field
            key = f"{match.group(1)}_{match.group(2)}"
            value = match.group(3)
            try:
                data[key] = float(value) if value != 'nan' else float('nan')
            except:
                data[key] = float('nan')
    return data

def main():
    ser = None
    try:
        # Initialize serial connection
        ser = serial.Serial(port=PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)
        print(f"Connected to {PORT} at {BAUD_RATE} baud.")

        time.sleep(2)  # Allow Arduino to reset

        # CSV setup
        with open(CSV_FILE, 'a', newline='') as file:
            # Define all expected columns
            fieldnames = [
                "Timestamp",
                "S1_X", "S1_Y", "S1_Z", "S1_p", "S1_r",
                "S2_X", "S2_Y", "S2_Z", "S2_p", "S2_r"
            ]
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            
            # Write header if file is empty
            if file.tell() == 0:
                writer.writeheader()

            print("Logging data... Press Ctrl+C to stop.")

            while True:
                if ser.in_waiting > 0:
                    raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not raw_line:
                        continue

                    parsed_data = parse_line(raw_line)
                    
                    # Only log if at least one sensor provided data
                    if any(key in parsed_data for key in fieldnames[1:]):
                        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        row = {"Timestamp": timestamp}
                        row.update(parsed_data)
                        writer.writerow(row)
                        file.flush()  # Ensure immediate write

                        # Console feedback
                        s1_str = f"S1: X={parsed_data.get('S1_X','nan'):.2f}, Y={parsed_data.get('S1_Y','nan'):.2f}, Z={parsed_data.get('S1_Z','nan'):.2f}"
                        s2_str = f"S2: X={parsed_data.get('S2_X','nan'):.2f}, Y={parsed_data.get('S2_Y','nan'):.2f}, Z={parsed_data.get('S2_Z','nan'):.2f}"
                        print(f"{timestamp} | {s1_str} | {s2_str}")
                
                time.sleep(0.01)

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except FileNotFoundError:
        print(f"Could not open file: {CSV_FILE}")
    except KeyboardInterrupt:
        print("\nLogging stopped by user.")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
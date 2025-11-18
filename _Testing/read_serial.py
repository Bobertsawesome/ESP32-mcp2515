#!/usr/bin/env python3
"""Simple serial port reader for non-interactive environments"""
import serial
import sys
import time
import os
from datetime import datetime

port = '/dev/cu.usbmodem1101'
baudrate = 115200
timeout = 180  # Read for 180 seconds (multi-speed test takes ~2 minutes)

# Create logs directory if it doesn't exist
script_dir = os.path.dirname(os.path.abspath(__file__))
logs_dir = os.path.join(script_dir, 'logs')
os.makedirs(logs_dir, exist_ok=True)

# Generate timestamped filename
timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
log_filename = os.path.join(logs_dir, f'test_{timestamp}.txt')

try:
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"Connected to {port} at {baudrate} baud", file=sys.stderr)
    print(f"Logging to: {log_filename}", file=sys.stderr)
    print("=" * 80, file=sys.stderr)

    with open(log_filename, 'w') as log_file:
        # Read for 3 seconds first to capture the repeating banner
        # Then send start command
        start_time = time.time()
        banner_captured = False

        while (time.time() - start_time) < timeout:
            # After 3 seconds, send the start command
            if not banner_captured and (time.time() - start_time) > 3:
                ser.write(b'S')
                ser.flush()
                print("Sent start command to ESP32", file=sys.stderr)
                banner_captured = True

            # Continue reading as before
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                try:
                    text = data.decode('utf-8', errors='replace')
                    # Write to both stdout and log file
                    print(text, end='', flush=True)
                    log_file.write(text)
                    log_file.flush()
                except:
                    print(data, flush=True)
                    log_file.write(str(data) + '\n')
                    log_file.flush()
            else:
                time.sleep(0.1)

    print("\n" + "=" * 80, file=sys.stderr)
    print(f"Capture complete. Log saved to: {log_filename}", file=sys.stderr)
    ser.close()

except Exception as e:
    print(f"Error: {e}", file=sys.stderr)
    sys.exit(1)

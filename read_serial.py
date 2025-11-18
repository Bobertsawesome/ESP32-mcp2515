#!/usr/bin/env python3
"""Simple serial port reader for non-interactive environments"""
import serial
import sys
import time

port = '/dev/cu.usbmodem1101'
baudrate = 115200
timeout = 30  # Read for 30 seconds

try:
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"Connected to {port} at {baudrate} baud", file=sys.stderr)
    print("=" * 80, file=sys.stderr)

    start_time = time.time()
    while (time.time() - start_time) < timeout:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            try:
                text = data.decode('utf-8', errors='replace')
                print(text, end='', flush=True)
            except:
                print(data, flush=True)
        else:
            time.sleep(0.1)

    print("\n" + "=" * 80, file=sys.stderr)
    print("Capture complete", file=sys.stderr)
    ser.close()

except Exception as e:
    print(f"Error: {e}", file=sys.stderr)
    sys.exit(1)

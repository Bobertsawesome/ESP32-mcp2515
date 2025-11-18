# PlatformIO Testing Methodology

**Document Version**: 1.0
**Date**: 2025-11-18
**Purpose**: Autonomous development and testing workflow for ESP32-MCP2515 library

---

## Overview

This document describes the testing methodology for autonomous development of the ESP32-MCP2515 library using PlatformIO. The workflow is optimized for rapid iteration and debugging without manual intervention.

## Project Structure

### Library Files (Root Directory)
```
ESP32-mcp2515/
├── mcp2515.h                    # Library header
├── mcp2515.cpp                  # Library implementation
├── mcp2515_esp32_config.h       # ESP32-specific configuration
├── can.h                        # CAN frame structures
└── platformio.ini               # Build configuration
```

### Test Application (src/)
```
src/
└── main.cpp                     # Comprehensive test suite
```

### Build Configuration
The `platformio.ini` is configured for zero-friction development:

```ini
[platformio]
src_dir = src                    # Standard PlatformIO location

[common]
build_flags =
    -Wall -Wextra
    -Wno-unused-parameter
    -Wno-unused-variable
    -I.                          # Include root directory for library headers

build_src_filter =
    +<*>                         # Include all files from src/
    +<../mcp2515.cpp>           # Build library directly from root

[env:esp32-s3]
board = esp32-s3-devkitc-1
framework = arduino
platform = espressif32@6.5.0
upload_port = /dev/cu.usbmodem1101
monitor_port = /dev/cu.usbmodem1101
monitor_speed = 115200
```

**Key Features:**
- No `lib/` directory needed - library builds directly from root
- Source files stay in root directory (no copying required)
- Changes to `mcp2515.cpp` or `mcp2515.h` immediately take effect on next build

---

## Build Process

### Quick Build Commands

**Build only:**
```bash
pio run -e esp32-s3
```

**Build and upload:**
```bash
pio run -e esp32-s3 -t upload
```

**Clean build:**
```bash
pio run -e esp32-s3 -t clean
pio run -e esp32-s3
```

### Build Output Analysis

**Successful build shows:**
```
RAM:   [=         ]   5.9% (used 19204 bytes from 327680 bytes)
Flash: [=         ]   9.1% (used ~304KB bytes from 3342336 bytes)
```

**Build errors to watch for:**
- Undefined references → Missing function implementations
- Syntax errors → Code errors in mcp2515.cpp or src/main.cpp
- Missing headers → Check `-I.` flag in build_flags

---

## Serial Monitoring

### Problem: Native PlatformIO Monitor Limitations

The built-in `pio run -t monitor` doesn't work in non-interactive environments:
```bash
# THIS FAILS in background/automated contexts:
pio run -e esp32-s3 -t monitor
# Error: termios.error: (19, 'Operation not supported by device')
```

### Solution: Custom Serial Reader

Created `read_serial.py` for autonomous testing:

```python
#!/usr/bin/env python3
import serial
import sys
import time

port = '/dev/cu.usbmodem1101'    # ESP32-S3 USB port
baudrate = 115200                 # Match monitor_speed in platformio.ini
timeout = 30                      # Capture duration in seconds

try:
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"Connected to {port} at {baudrate} baud", file=sys.stderr)

    start_time = time.time()
    while (time.time() - start_time) < timeout:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            text = data.decode('utf-8', errors='replace')
            print(text, end='', flush=True)
        else:
            time.sleep(0.1)    # Prevent CPU spinning

    ser.close()
except Exception as e:
    print(f"Error: {e}", file=sys.stderr)
    sys.exit(1)
```

### Using the Serial Reader

**Prerequisites:**
```bash
# Create dedicated virtual environment
python3.14 -m venv ~/.venvs/pyserial
source ~/.venvs/pyserial/bin/activate
pip install pyserial
```

**Basic usage:**
```bash
# Make executable
chmod +x read_serial.py

# Run with venv Python
~/.venvs/pyserial/bin/python read_serial.py
```

**Capture output to file:**
```bash
~/.venvs/pyserial/bin/python read_serial.py | tee test_output.log
```

---

## Complete Test Cycle

### 1. Build and Upload Firmware

```bash
cd /path/to/ESP32-mcp2515
pio run -e esp32-s3 -t upload
```

**Expected output:**
```
Uploading .pio/build/esp32-s3/firmware.bin
esptool.py v4.5.1
Serial port /dev/cu.usbmodem1101
Connecting...
Chip is ESP32-S3 (revision v0.2)
...
Hard resetting via RTS pin...
========================= [SUCCESS] Took X.XX seconds =========================
```

### 2. Monitor Serial Output

```bash
~/.venvs/pyserial/bin/python read_serial.py
```

**Expected test output:**
```
[PASS] Test: reset() (err=0)
[PASS] Test: setBitrate() (CAN_250KBPS @ 16MHz, err=0)
[PASS] Test: setNormalMode() (err=0)
...
=== TEST SUMMARY ===
  Total tests:    85
  Passed:         XX
  Failed:         XX
  Pass rate:      XX.XX%
```

### 3. Analyze Results

**Success criteria:**
- `[PASS]` markers for passing tests
- `[FAIL]` markers identify failing tests
- Pass rate >95% for production readiness
- No `ERROR_MUTEX`, `ERROR_PSRAM`, or crash messages

**Common failure patterns:**
- `ERROR_TIMEOUT (6)` → Queue timeout, frames not arriving
- `ERROR_NOMSG (5)` → No message in buffer/queue
- `ID mismatch: expected 0xXXX, got 0x000` → Empty frames being received
- Send errors in stress test → TX buffers busy or transmission failing

---

## Troubleshooting

### Build Issues

**Problem: Library changes not reflected**
```bash
# Force clean rebuild
pio run -e esp32-s3 -t clean
pio run -e esp32-s3
```

**Problem: Undefined reference errors**
```
Solution: Check that build_src_filter includes ../mcp2515.cpp
```

**Problem: Header not found**
```
Solution: Verify -I. flag in build_flags to include root directory
```

### Upload Issues

**Problem: Serial port busy**
```bash
# Check what's holding the port
lsof | grep cu.usbmodem1101

# Kill offending processes
killall -9 python python3 python3.14 2>/dev/null

# Wait and retry
sleep 2
pio run -e esp32-s3 -t upload
```

**Problem: ESP32 not detected**
```bash
# List USB devices
ls /dev/cu.*

# Verify correct port in platformio.ini:
upload_port = /dev/cu.usbmodem1101  # Update if different
```

### Serial Monitoring Issues

**Problem: No output from read_serial.py**
```bash
# Verify pyserial installation
~/.venvs/pyserial/bin/python -c "import serial; print('OK')"

# Check port manually
screen /dev/cu.usbmodem1101 115200
# (Ctrl+A, K to exit screen)
```

**Problem: Garbled serial output**
```
Cause: Incorrect baud rate
Solution: Ensure monitor_speed=115200 matches Serial.begin(115200) in code
```

**Problem: Missing beginning of test output**
```bash
# ESP32 starts outputting before serial reader connects
# Solution: Add delay in setup() or increase timeout in read_serial.py
```

---

## Autonomous Development Workflow

### Iterative Debugging Loop

```bash
#!/bin/bash
# autonomous_test.sh - Complete test cycle

PROJECT_DIR="/path/to/ESP32-mcp2515"
PYTHON_VENV="$HOME/.venvs/pyserial/bin/python"

cd "$PROJECT_DIR"

while true; do
    echo "=== Building and uploading ==="
    pio run -e esp32-s3 -t upload || {
        echo "Build/upload failed"
        exit 1
    }

    echo "=== Monitoring serial output ==="
    $PYTHON_VENV read_serial.py | tee "logs/test_$(date +%y%m%d_%H%M%S).log"

    echo "=== Analyzing results ==="
    # Parse last log for pass/fail
    LAST_LOG=$(ls -t logs/*.log | head -1)
    PASS_RATE=$(grep "Pass rate:" "$LAST_LOG" | awk '{print $3}')

    echo "Pass rate: $PASS_RATE"

    if [[ "$PASS_RATE" > "95.00%" ]]; then
        echo "=== Tests passing! ==="
        break
    else
        echo "=== Failures detected, iteration needed ==="
        # AI agent would analyze failures and make code changes here
        break  # Remove for continuous loop
    fi
done
```

### Key Metrics to Track

1. **Pass Rate**: Should be >95% for production
2. **Stress Test Success Rate**: Should be >95% for reliable operation
3. **Memory Usage**: Watch for increases indicating leaks
4. **Queue Overflow**: "RX queue full" warnings indicate performance issues

---

## Hardware Setup

### ESP32-S3 to MCP2515 Connections

```
ESP32-S3          MCP2515
--------          -------
GPIO 10    →      CS (Chip Select)
GPIO 4     →      INT (Interrupt, optional but recommended)
GPIO 11    →      MOSI (SPI Data Out)
GPIO 13    ←      MISO (SPI Data In)
GPIO 12    →      SCK (SPI Clock)
GND        →      GND
3.3V       →      VCC (if MCP2515 is 3.3V tolerant)
```

**IMPORTANT:** Most MCP2515 modules are 5V devices. Use level shifters for 3.3V ESP32!

### Loopback Mode Testing

No physical CAN bus required for loopback testing:
- Frames are looped back internally in the MCP2515
- No need for termination resistors or second device
- INT pin still required for interrupt-driven reception

---

## File Locations

**Build artifacts:**
```
.pio/build/esp32-s3/firmware.bin     # Compiled firmware
.pio/build/esp32-s3/firmware.elf     # ELF with debug symbols
```

**Test logs:**
```
logs/test_YYMMDD_HHMMSS.log         # Serial capture logs
```

**Key source files:**
```
mcp2515.cpp                          # Library implementation
src/main.cpp                         # Test suite
platformio.ini                       # Build configuration
read_serial.py                       # Serial monitoring script
```

---

## Version History

- **v1.0** (2025-11-18): Initial testing methodology documentation

---

## Related Documentation

- `DEVELOPMENT_WORKFLOW.md` - General development guide
- `PLATFORMIO_BUILD_TESTING_GUIDE.md` - Build testing for autonomous agents
- `Documentation/ESP32_COMPREHENSIVE_AUDIT_2025-11-15.md` - ESP32-specific implementation details

---

**For autonomous AI agents**: This document provides the complete testing workflow. Execute build → upload → monitor → analyze in a loop, making code changes based on test failures until pass rate exceeds 95%.

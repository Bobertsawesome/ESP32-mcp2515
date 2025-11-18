# Development Workflow - ESP32-MCP2515 Library

## Quick Start for Autonomous Testing

This project is configured for rapid development and testing cycles with PlatformIO. The library source files are built directly from the root directory without needing to copy files.

### Prerequisites

- PlatformIO Core installed
- ESP32-S3 (or other ESP32 variant) connected via USB
- MCP2515 CAN module connected to ESP32

---

## Basic Commands

### Build Only
```bash
pio run -e esp32-s3
```

### Upload Firmware
```bash
pio run -e esp32-s3 -t upload
```

### Monitor Serial Output
```bash
pio run -e esp32-s3 -t monitor
```

### Upload + Monitor (Autonomous Testing)
```bash
pio run -e esp32-s3 -t upload -t monitor
```

### Clean Build
```bash
pio run -e esp32-s3 -t clean
```

---

## Project Structure

```
ESP32-mcp2515/
├── src/
│   └── main.cpp              # Test file (ESP32_CAN_fulltest_loopback)
├── mcp2515.h                 # Library header (built directly)
├── mcp2515.cpp               # Library implementation (built directly)
├── mcp2515_esp32_config.h    # ESP32 configuration
├── can.h                     # CAN frame structures
├── platformio.ini            # PlatformIO configuration
└── examples/                 # Example sketches
```

### How It Works

The `platformio.ini` is configured to:
- Use `src/` directory for the main application code
- Include `mcp2515.cpp` directly from the root directory via `build_src_filter`
- Add root directory to include path via `-I.` flag

This means:
- ✅ No need to copy library files to `lib/` directory
- ✅ Changes to library files are immediately reflected in builds
- ✅ Single source of truth for library code
- ✅ Fast iteration cycles

---

## Development Cycle

### 1. Make Changes
Edit library files directly:
- `mcp2515.h` - Header file
- `mcp2515.cpp` - Implementation
- `src/main.cpp` - Test application

### 2. Build and Test
```bash
# Single command for full cycle
pio run -e esp32-s3 -t upload -t monitor
```

This will:
1. Compile library + test code
2. Upload to connected ESP32-S3
3. Start serial monitor automatically
4. Display test results in real-time

### 3. Monitor Output
Press `Ctrl+C` to exit monitor when done.

### 4. Iterate
Make changes and repeat step 2.

---

## Available Test Environments

All environments use the same source code but target different hardware:

### ESP32 Variants
- `esp32dev` - ESP32 Classic (dual-core Xtensa LX6)
- `esp32-s2` - ESP32-S2 (single-core Xtensa LX7)
- `esp32-s3` - ESP32-S3 (dual-core Xtensa LX7) **← Default for testing**
- `esp32-c3` - ESP32-C3 (single-core RISC-V)

### Arduino AVR
- `uno` - Arduino Uno (ATmega328P)
- `mega2560` - Arduino Mega2560 (ATmega2560)

Example:
```bash
pio run -e esp32-c3 -t upload -t monitor
```

---

## Monitoring Options

### Basic Monitor
```bash
pio run -e esp32-s3 -t monitor
```

### Monitor with Filters
The default configuration includes:
- `log2file` - Saves output to timestamped log file
- `esp32_exception_decoder` - Decodes stack traces automatically

### Save Output to Custom Log
```bash
pio run -e esp32-s3 -t monitor | tee logs/test-$(date +%Y%m%d-%H%M%S).txt
```

---

## Troubleshooting

### Port Not Found
```bash
# List available ports
pio device list

# Specify port manually
pio run -e esp32-s3 -t upload --upload-port /dev/cu.usbserial-*
```

### Permission Denied (macOS/Linux)
```bash
# Add user to dialout group (Linux)
sudo usermod -a -G dialout $USER

# Or use sudo (not recommended)
sudo pio run -e esp32-s3 -t upload
```

### Build Errors After Changes
```bash
# Clean and rebuild
pio run -e esp32-s3 -t clean
pio run -e esp32-s3
```

### Monitor Not Showing Output
- Check baud rate (should be 115200)
- Press reset button on ESP32
- Verify USB cable supports data transfer

---

## Autonomous Agent Workflow

For AI assistants performing autonomous testing:

### Complete Test Cycle
```bash
cd /path/to/ESP32-mcp2515
pio run -e esp32-s3 -t clean     # Clean previous build
pio run -e esp32-s3              # Verify compilation
pio run -e esp32-s3 -t upload -t monitor  # Upload and monitor
```

### Capture Output Programmatically
```bash
# Run for 30 seconds and capture output
timeout 30 pio run -e esp32-s3 -t monitor > test_output.txt 2>&1
```

### Parse Test Results
Look for these patterns in output:
- `[PASS]` - Test passed
- `[FAIL]` - Test failed
- `[WARN]` - Warning
- `Success rate:` - Overall success percentage
- `Pass rate:` - Test suite pass percentage

### Expected Success Criteria (After Loopback Fix)
```
Pass rate: >95%
Stress test success rate: >95%
No ID=0x000 errors
No ERROR_NOMSG (err=6) in loopback mode
```

---

## Log Files

Test output is automatically saved to:
```
logs/device-monitor-YYMMDD-HHMMSS.txt
```

To create custom log:
```bash
pio run -e esp32-s3 -t upload -t monitor | tee logs/my-test-$(date +%Y%m%d-%H%M%S).txt
```

---

## Hardware Setup

### ESP32-S3 to MCP2515 Connections

| ESP32-S3 Pin | MCP2515 Pin | Function |
|--------------|-------------|----------|
| GPIO 12 | SCK | SPI Clock |
| GPIO 13 | MISO | SPI Data Out |
| GPIO 11 | MOSI | SPI Data In |
| GPIO 37 | CS | Chip Select |
| GPIO 36 | INT | Interrupt (optional) |
| 3.3V | VCC | Power |
| GND | GND | Ground |

### MCP2515 to CAN Bus

For loopback testing:
- No external CAN connection needed
- Loopback mode routes TX → RX internally

For actual CAN testing:
- Connect CAN_H and CAN_L to CAN bus
- Add 120Ω termination resistors at both ends
- Use Normal mode instead of Loopback mode

---

## Performance Benchmarks

After loopback mode interrupt fix:

### Expected Build Times
- Clean build: ~3-4 seconds
- Incremental build: ~1-2 seconds

### Expected Test Results
- Initialization tests: 100% pass
- Mode switching tests: 100% pass
- Transmission tests: 100% pass
- Reception tests: >95% pass (loopback mode)
- Stress test (1000 packets): >95% success rate
- Overall pass rate: >95%

---

## Editing Test Parameters

The test file `src/main.cpp` contains configurable parameters:

```cpp
// Line ~28-32
const CAN_SPEED TEST_SPEED = CAN_250KBPS;
const CAN_CLOCK TEST_CLOCK = MCP_16MHZ;
const uint32_t settle_time_ms = 5;

// Pin definitions (GPIO numbers)
const gpio_num_t MCP2515_CS_PIN = GPIO_NUM_37;
const gpio_num_t MCP2515_INT_PIN = GPIO_NUM_36;
```

Modify these values and rebuild to test different configurations.

---

## Continuous Integration

For automated testing:

```bash
#!/bin/bash
# ci-test.sh

set -e  # Exit on error

echo "Building..."
pio run -e esp32-s3 -t clean
pio run -e esp32-s3

echo "Uploading and testing..."
timeout 60 pio run -e esp32-s3 -t upload -t monitor > test_output.txt 2>&1

echo "Checking results..."
if grep -q "Pass rate.*9[5-9]%" test_output.txt || grep -q "Pass rate.*100%" test_output.txt; then
    echo "✅ Tests PASSED"
    exit 0
else
    echo "❌ Tests FAILED"
    cat test_output.txt
    exit 1
fi
```

---

**Last Updated**: 2025-11-17
**PlatformIO Version**: Core 6.1+
**Platform**: espressif32@6.5.0
**Framework**: Arduino 2.0.14

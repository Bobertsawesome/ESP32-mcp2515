# Testing Utilities for ESP32-MCP2515

This directory contains testing utilities and test output logs for the ESP32-MCP2515 library.

## Contents

- **`read_serial.py`** - Python script for capturing serial output from ESP32 tests
- **`logs/`** - Directory containing test output logs (git-ignored)

## Using read_serial.py

The `read_serial.py` script captures serial output from the ESP32 running the comprehensive test suite. It automatically detects the serial port, captures output for 30 seconds, and saves to timestamped log files.

### Prerequisites (One-Time Setup)

```bash
# Create Python virtual environment
python3 -m venv ~/.venvs/pyserial

# Activate environment
source ~/.venvs/pyserial/bin/activate

# Install pyserial
pip install pyserial
```

### Usage

**Basic capture (30 seconds):**

```bash
~/.venvs/pyserial/bin/python _Testing/read_serial.py
```

**Save to log file:**

```bash
~/.venvs/pyserial/bin/python _Testing/read_serial.py | tee _Testing/logs/test_$(date +%y%m%d_%H%M%S).txt
```

**From project root:**

```bash
cd /path/to/ESP32-mcp2515
~/.venvs/pyserial/bin/python _Testing/read_serial.py
```

### Output Format

The script automatically:
- Detects available serial ports
- Connects at 115200 baud
- Captures output for 30 seconds
- Displays real-time output
- Saves to timestamped log file
- Prints log file path at completion

### Test Workflow

**Complete testing workflow:**

1. **Build and upload firmware:**
   ```bash
   pio run -e esp32-s3 -t upload
   ```

2. **Capture test output:**
   ```bash
   ~/.venvs/pyserial/bin/python _Testing/read_serial.py
   ```

3. **Analyze results:**
   - Look for `[PASS]` and `[FAIL]` markers
   - Check pass rate (target: >95% for production)
   - Review stress test success rate
   - Examine specific test failures

### Log Files

Test logs are saved in `_Testing/logs/` with the format:
- `test_YYMMDD_HHMMSS.txt` - Timestamped test logs

The logs directory is git-ignored by default but preserved in the repository structure.

## Testing Documentation

For complete testing methodology and workflow details, see:
- `Documentation/PLATFORMIO_TESTING_METHODOLOGY.md` - Complete testing workflow
- `Documentation/PLATFORMIO_BUILD_TESTING_GUIDE.md` - Build system guide
- `CLAUDE.md` - Development guide with testing section

## Test Metrics

**Target Metrics for Production:**
- **Pass Rate**: >95% (91/93 tests passing as of v2.1.1)
- **Stress Test**: >95% success rate (100.00% as of v2.1.1)
- **Memory Usage**: Monitor for leaks or excessive consumption
- **Queue Overflow**: No "RX queue full" warnings under normal load

## Autonomous Testing

The testing setup is designed for autonomous agent-driven development:

1. Make code changes
2. Build and upload firmware
3. Capture serial output with `read_serial.py`
4. Parse test results (pass/fail rates, error patterns)
5. If pass rate <95%, analyze failures and iterate

This enables systematic testing without manual intervention.

---

**Last Updated**: 2025-11-18
**For Library Version**: 2.1.1-ESP32

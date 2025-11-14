# Test Results Archive

This directory contains test results from the ESP32 MCP2515 CAN Library comprehensive test suite (v1.0) that were used to identify issues and drive the v2.0 enhancements.

## Test Files

- `CANTestLoopback-50KBPS.txt` - Test results at 50 kbps
- `CANTestLoopback-125KBPS.txt` - Test results at 125 kbps
- `CANTestLoopback-250KBPS.txt` - Test results at 250 kbps
- `CANTestLoopback-500KBPS.txt` - Test results at 500 kbps
- `CANTestLoopback-1000KBPS.txt` - Test results at 1000 kbps (1 Mbps)

## Key Findings

Analysis of these test results revealed several critical issues:

### 1. Inverse Error Relationship
Counter-intuitively, **lower CAN speeds had HIGHER error rates**:
- 50 kbps: 81.68% TX errors
- 125 kbps: 81.68% TX errors
- 250 kbps: 59.83% TX errors
- 500 kbps: 1.48% TX errors
- 1000 kbps: 0.00% TX errors

**Root Cause**: TX buffer saturation. The ESP32 can send frames faster than low-speed CAN buses can transmit them, causing buffer overflow.

**Fix**: Implemented adaptive stress testing in v2.0 that scales transmission rate to 80% of theoretical CAN bus capacity.

### 2. Data Integrity Failures
All speed tests showed: `Data integrity: FAIL`

**Root Cause**: Insufficient delay between transmission and reception in loopback mode.

**Fix**: Added 100ms delay after sending test frames to allow MCP2515 loopback routing to complete.

### 3. Mode Switching Failures
Sleep Mode and One-Shot Mode tests consistently failed.

**Root Cause**: These modes require transition through configuration mode first, and no pending transmissions.

**Fix**: Added proper mode transition sequence:
1. Enter config mode
2. Clear interrupts
3. Wait for any pending TX to complete
4. Then switch to target mode

### 4. Incorrect Test Logic
"Read with No Message Available" test had inverted pass/fail logic.

**Root Cause**: Test expected `ERROR_NOMSG` but marked it as FAIL when received correctly.

**Fix**: Corrected test to pass when `ERROR_NOMSG` is returned.

## Test Configuration

All tests were run with:
- **Hardware**: ESP32 + MCP2515 CAN module
- **Test Mode**: Loopback (internal self-test)
- **MCP2515 Clock**: 16 MHz
- **Framework**: Arduino IDE / ESP32 Arduino Core

## Impact on v2.0

These findings directly led to the comprehensive enhancements in test suite v2.0:
- Multi-speed testing with comparison analysis
- Adaptive stress testing
- Enhanced error reporting with data comparison
- TX buffer status monitoring
- Speed-specific error thresholds
- Production deployment recommendations

## Running New Tests

To run the enhanced v2.0 test suite with multi-speed testing:

```cpp
// In ESP32_CAN_comprehensive_test.ino
#define ENABLE_MULTI_SPEED_TEST   true
```

This will automatically test across all major CAN speeds and generate a comprehensive comparison report.

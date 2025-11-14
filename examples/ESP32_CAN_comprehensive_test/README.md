# ESP32 MCP2515 CAN Library - Comprehensive Test Suite

## Overview

This is a **complete and exhaustive test suite** for the ESP32 MCP2515 CAN library. It systematically tests **every single function** in the library and provides detailed results with pass/fail status, statistics, and diagnostics.

## Features

✅ **Comprehensive Coverage** - Tests ALL library functions
✅ **Two Test Modes** - Loopback self-test OR real CAN bus testing
✅ **Detailed Reporting** - Color-coded results with statistics
✅ **Hardware Flexible** - Configurable pins and settings
✅ **Stress Testing** - High-load performance validation
✅ **Educational** - Shows proper usage of every API function

## Test Categories

The test suite is organized into 13 comprehensive categories:

1. **Initialization Tests** - Reset, bitrate, mode configuration
2. **Operating Mode Tests** - All 6 operating modes
3. **Bitrate Configuration Tests** - All supported bitrates (5kbps to 1Mbps)
4. **Filter and Mask Tests** - All 6 filters and 2 masks
5. **Frame Transmission Tests** - Standard, extended, RTR, all DLCs, all TX buffers
6. **Frame Reception Tests** - Auto and manual buffer reading
7. **Error Handling Tests** - Error flags, counters, recovery
8. **Interrupt Function Tests** - Interrupt flags, mask, clearing
9. **Statistics Tests** - ESP32-specific statistics tracking
10. **Status Function Tests** - Status register reading
11. **Stress Tests** - High-speed TX, RX overflow handling
12. **Clock Output Tests** - CLKOUT configuration
13. **Advanced Feature Tests** - Queued reading, ESP32-specific features

## Hardware Requirements

### Mode 1: Loopback Self-Test (Recommended for Initial Testing)

**Required Hardware:**
- 1x ESP32 development board
- 1x MCP2515 CAN module
- Jumper wires

**Wiring:**
```
ESP32          MCP2515
─────────────────────────
GPIO 23   →    MOSI (SI)
GPIO 19   ←    MISO (SO)
GPIO 18   →    SCK
GPIO 5    →    CS
GPIO 4    ←    INT (optional)
5V        →    VCC
GND       →    GND
```

**Notes:**
- NO CAN bus wiring required
- NO second device required
- MCP2515 CAN_H and CAN_L pins can be left **unconnected**
- Loopback mode tests TX→RX internally within the MCP2515 chip

### Mode 2: Two-Device Real CAN Bus Test

**Required Hardware:**
- 2x ESP32 development boards
- 2x MCP2515 CAN modules
- 2x 120Ω termination resistors
- Twisted pair wire for CAN_H/CAN_L
- Jumper wires

**Wiring:**

```
ESP32 #1       MCP2515 #1          CAN BUS          MCP2515 #2       ESP32 #2
────────────────────────────────────────────────────────────────────────────────
GPIO 23   →    MOSI                                    MOSI    ←    GPIO 23
GPIO 19   ←    MISO                                    MISO    →    GPIO 19
GPIO 18   →    SCK                                     SCK     ←    GPIO 18
GPIO 5    →    CS                                      CS      ←    GPIO 5
GPIO 4    ←    INT                                     INT     →    GPIO 4
5V        →    VCC                                     VCC     ←    5V
GND       →    GND         ═══════                     GND     ←    GND
               CAN_H  ─────┤120Ω├───── CAN_H
               CAN_L  ──────────────── CAN_L
                            ═══════
                             120Ω
```

**Notes:**
- CAN_H and CAN_L must be connected between both modules
- 120Ω termination resistors at **both ends** of the CAN bus
- Use twisted pair wire for CAN_H/CAN_L (reduce noise)
- Maximum cable length depends on bitrate:
  - 1 Mbps: ~40m
  - 125 kbps: ~500m
  - 5 kbps: ~1000m

### MCP2515 Module Crystal Frequency

**IMPORTANT:** Check your MCP2515 module's crystal/oscillator frequency!

Common frequencies:
- **16 MHz** - Most common (default in test code)
- **8 MHz** - Some modules use this
- **20 MHz** - Less common

The crystal frequency is usually printed on the crystal component on the MCP2515 module. If the frequency is wrong, CAN communication will **not work**.

## Configuration

### Step 1: Open the Arduino IDE

Open `ESP32_CAN_comprehensive_test.ino` in Arduino IDE.

### Step 2: Configure Hardware Pins

Edit the pin configuration at the top of the file:

```cpp
// SPI Pin Configuration (adjust for your ESP32 board)
#define SPI_MOSI_PIN    23    // ESP32 VSPI MOSI
#define SPI_MISO_PIN    19    // ESP32 VSPI MISO
#define SPI_SCK_PIN     18    // ESP32 VSPI SCK
#define SPI_CS_PIN      5     // Chip Select pin
#define SPI_INT_PIN     4     // Interrupt pin (set to -1 to disable)
```

### Step 3: Configure CAN Parameters

Set your MCP2515's crystal frequency and desired CAN speed:

```cpp
// CAN Bus Configuration
#define CAN_SPEED       CAN_125KBPS   // Options: CAN_5KBPS to CAN_1000KBPS
#define CAN_CLOCK       MCP_16MHZ     // Options: MCP_8MHZ, MCP_16MHZ, MCP_20MHZ
```

### Step 4: Select Test Mode

Choose your test mode:

```cpp
// For loopback self-test (1 ESP32):
#define TEST_MODE       TEST_MODE_LOOPBACK

// For two-device real CAN bus test (2 ESP32s):
#define TEST_MODE       TEST_MODE_TWO_DEVICE
```

### Step 5: Configure Device Role (Two-Device Mode Only)

If using `TEST_MODE_TWO_DEVICE`, set the role on each device:

**On Device #1 (Master):**
```cpp
#define DEVICE_ROLE     ROLE_MASTER
```

**On Device #2 (Slave):**
```cpp
#define DEVICE_ROLE     ROLE_SLAVE
```

### Step 6: Optional Settings

```cpp
#define ENABLE_INTERRUPT_TESTS    (SPI_INT_PIN >= 0)  // Auto-disables if no INT pin
#define ENABLE_STRESS_TESTS       true                // Enable stress testing
#define STRESS_TEST_DURATION_MS   5000                // Stress test duration
#define VERBOSE_OUTPUT            true                // Detailed output
```

## Running the Tests

### Loopback Mode (1 ESP32)

1. Wire one ESP32 to one MCP2515 module
2. Configure as shown above
3. Set `TEST_MODE` to `TEST_MODE_LOOPBACK`
4. Upload to ESP32
5. Open Serial Monitor (115200 baud)
6. Watch the tests run automatically

### Two-Device Mode (2 ESP32s)

1. Wire two ESP32s to two MCP2515 modules
2. Connect CAN_H and CAN_L between modules with 120Ω terminators
3. Configure **Device #1** with `DEVICE_ROLE = ROLE_MASTER`
4. Configure **Device #2** with `DEVICE_ROLE = ROLE_SLAVE`
5. Upload to both ESP32s
6. Open Serial Monitor on **MASTER** device (115200 baud)
7. Power both devices simultaneously
8. Master will run tests, slave will respond

## Expected Output

The test suite produces color-coded output:

```
╔════════════════════════════════════════════════════════════════╗
║   ESP32 MCP2515 CAN LIBRARY COMPREHENSIVE TEST SUITE          ║
╚════════════════════════════════════════════════════════════════╝

Configuration:
  Test Mode:        LOOPBACK (Self-Test)
  SPI Pins:         MOSI=23, MISO=19, SCK=18, CS=5, INT=4
  CAN Speed:        125 kbps
  CAN Clock:        16 MHz
  Interrupt Tests:  ENABLED
  Stress Tests:     ENABLED

┌────────────────────────────────────────────────────────────────┐
│ INITIALIZATION TESTS                                           │
└────────────────────────────────────────────────────────────────┘

[TEST] MCP2515 Reset
  ✓ PASS - Reset successful

[TEST] Set Bitrate
  ✓ PASS - Bitrate configured successfully

[TEST] Set Operating Mode
  ✓ PASS - Loopback mode set

┌────────────────────────────────────────────────────────────────┐
│ OPERATING MODE TESTS                                           │
└────────────────────────────────────────────────────────────────┘

[TEST] Configuration Mode
  ✓ PASS - Mode set successfully

...
(and so on for all test categories)
...

╔════════════════════════════════════════════════════════════════╗
║                     FINAL TEST RESULTS                         ║
╚════════════════════════════════════════════════════════════════╝

  Total Tests:    127
  Passed:         127 (100.0%)
  Failed:         0 (0.0%)

  ★★★ ALL TESTS PASSED ★★★

Library Statistics:
  RX Frames:      85
  TX Frames:      95
  RX Errors:      0
  TX Errors:      0
  RX Overflow:    0
  Bus Errors:     0

═══════════════════════════════════════════════════════════════════
```

## Test Results Interpretation

### ✓ PASS (Green)
- Function executed successfully
- Expected behavior confirmed
- Test criteria met

### ✗ FAIL (Red)
- Function returned an error
- Unexpected behavior detected
- Investigation required

### Common Failure Causes

1. **"Reset failed"**
   - Check SPI wiring (MOSI, MISO, SCK, CS)
   - Verify power supply (5V or 3.3V stable)
   - Check CS pin number is correct

2. **"Bitrate configuration failed"**
   - Wrong CAN_CLOCK setting (check crystal frequency)
   - MCP2515 not responding
   - SPI communication issue

3. **"Mode set failed"**
   - MCP2515 not initialized properly
   - SPI communication issue

4. **"All TX Busy" errors**
   - In two-device mode: Check CAN bus wiring
   - Check 120Ω termination resistors
   - Verify both devices have matching bitrate

5. **"No message received" (Two-Device Mode)**
   - Check CAN_H and CAN_L connections
   - Verify termination resistors (120Ω at each end)
   - Ensure both devices have same bitrate and clock settings
   - Check that slave device is running

## Function Coverage

This test suite validates **ALL** public API functions:

### Core Functions
- ✅ `reset()`
- ✅ `setBitrate(speed)` and `setBitrate(speed, clock)`
- ✅ `setConfigMode()`
- ✅ `setNormalMode()`
- ✅ `setListenOnlyMode()`
- ✅ `setLoopbackMode()`
- ✅ `setSleepMode()`
- ✅ `setNormalOneShotMode()`
- ✅ `setClkOut(divisor)`

### Filter and Mask Functions
- ✅ `setFilterMask(mask, ext, id)` - Both MASK0 and MASK1
- ✅ `setFilter(filter, ext, id)` - All RXF0 through RXF5

### Transmission Functions
- ✅ `sendMessage(frame)` - Auto buffer selection
- ✅ `sendMessage(txb, frame)` - Specific TX buffer (TXB0, TXB1, TXB2)

### Reception Functions
- ✅ `readMessage(frame)` - Auto buffer selection
- ✅ `readMessage(rxb, frame)` - Specific RX buffer (RXB0, RXB1)
- ✅ `checkReceive()`

### Error Handling Functions
- ✅ `checkError()`
- ✅ `getErrorFlags()`
- ✅ `clearRXnOVRFlags()`
- ✅ `clearRXnOVR()`
- ✅ `clearMERR()`
- ✅ `clearERRIF()`
- ✅ `errorCountRX()`
- ✅ `errorCountTX()`

### Interrupt Functions
- ✅ `getInterrupts()`
- ✅ `getInterruptMask()`
- ✅ `clearInterrupts()`
- ✅ `clearTXInterrupts()`

### Status Functions
- ✅ `getStatus()`

### ESP32-Specific Functions
- ✅ `isInitialized()`
- ✅ `readMessageQueued(frame, timeout)`
- ✅ `getRxQueueCount()`
- ✅ `getStatistics(stats)`
- ✅ `resetStatistics()`
- ✅ `setInterruptMode(enable)`
- ✅ `performErrorRecovery()`
- ✅ `getBusStatus()`

### Frame Types Tested
- ✅ Standard data frames (11-bit ID)
- ✅ Extended data frames (29-bit ID)
- ✅ Standard remote frames (RTR)
- ✅ Extended remote frames (RTR)
- ✅ All DLC values (0-8 bytes)
- ✅ All three TX buffers

## Troubleshooting

### No Serial Output

1. Check Serial Monitor baud rate is set to **115200**
2. Try pressing the ESP32 reset button
3. Check USB cable connection

### "ERROR_FAILINIT" on Reset

1. **Check SPI wiring** - Most common issue
   - MOSI → SI
   - MISO → SO
   - SCK → SCK
   - CS → CS
2. **Verify power supply**
   - 5V or 3.3V stable voltage
   - Check GND connection
3. **Check crystal frequency** in code matches hardware
4. **Try different CS pin** - some boards have issues with certain pins

### Transmission Errors in Loopback Mode

- Loopback mode should have **zero** transmission errors
- If errors occur:
  - Check SPI wiring
  - Verify SPI clock speed isn't too high
  - Try reducing SPI_CLOCK in library

### Communication Issues in Two-Device Mode

1. **Verify CAN bus wiring**
   - CAN_H to CAN_H
   - CAN_L to CAN_L
   - Use twisted pair for noise immunity
2. **Check termination resistors**
   - 120Ω at **both** ends
   - Measure with multimeter: 60Ω between CAN_H and CAN_L
3. **Verify bitrate and clock match** on both devices
4. **Check cable length** (max depends on bitrate)

### RX Overflow Warnings

- This is **normal** in stress tests
- Indicates RX buffer filled faster than reading
- In real applications, use interrupt-driven reception

## Performance Benchmarks

Expected performance in stress tests (125 kbps, 16 MHz crystal):

- **Transmission rate**: 100-300 frames/second
- **Error rate**: < 1% in loopback mode
- **Error rate**: < 5% in two-device mode with good wiring
- **Latency**: < 10ms per frame

## Customization

### Adding Custom Tests

To add your own test:

```cpp
void testMyCustomFeature() {
    printSectionHeader("MY CUSTOM TESTS");

    printTestHeader("My Custom Test");
    // Your test code here
    bool result = myTestFunction();
    printTestResult(result, "Custom test description");
}
```

Then call it from `runComprehensiveTests()`:

```cpp
void runComprehensiveTests() {
    // ... existing tests ...
    testMyCustomFeature();  // Add your test here
}
```

### Adjusting Test Durations

Modify stress test duration:

```cpp
#define STRESS_TEST_DURATION_MS   5000  // Change to 10000 for 10 seconds
```

### Disabling Test Categories

Comment out unwanted test categories in `runComprehensiveTests()`:

```cpp
void runComprehensiveTests() {
    testInitialization();
    testOperatingModes();
    // testBitrateConfiguration();  // Disabled
    testFiltersAndMasks();
    // ... etc
}
```

## Educational Value

This test suite serves as:

1. **Validation Tool** - Ensures library works correctly
2. **Learning Resource** - Shows proper usage of every function
3. **Example Code** - Reference for real applications
4. **Debugging Aid** - Identifies hardware or configuration issues
5. **Benchmark Tool** - Measures performance characteristics

## File Structure

```
ESP32_CAN_comprehensive_test/
├── ESP32_CAN_comprehensive_test.ino  # Main test code
└── README.md                          # This file
```

## Requirements

- **Arduino IDE** 1.8.13 or later
- **ESP32 Arduino Core** 2.0.0 or later
- **ESP32-MCP2515 Library** (this library)
- **Hardware**: ESP32 + MCP2515 module(s)

## License

This test suite is part of the ESP32-MCP2515 library and follows the same MIT License.

## Support

If you encounter issues:

1. Check the **Troubleshooting** section above
2. Verify hardware connections
3. Read the test output carefully - it often indicates the problem
4. Check the main library documentation

## Version History

- **v1.0.0** (2025) - Initial comprehensive test suite
  - Tests all library functions
  - Dual test mode support (loopback and two-device)
  - Colored output and statistics
  - Stress testing included

## Contributing

Improvements to this test suite are welcome! Consider adding:

- Additional edge case tests
- Performance benchmarks for different ESP32 models
- Tests for specific CAN applications (OBD-II, CANopen, etc.)
- Automated hardware-in-the-loop testing

---

**Happy Testing!** 🚀

For more information about the ESP32-MCP2515 library, see the main repository README.

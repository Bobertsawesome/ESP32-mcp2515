- Commit all changes, make sure to do so when logical categories of changes have been made. Push to github default branch after doing so.
- All created documentation files MUST go into @Documentation/ folder unless README.md or github information meant for the repo

# CLAUDE.md - AI Assistant Guide for ESP32-mcp2515

## Project Overview

**Project Name**: ESP32-MCP2515
**Version**: 2.1.1-ESP32
**License**: MIT License
**Repository**: https://github.com/Bobertsawesome/ESP32-mcp2515
**Original Repository**: https://github.com/autowp/arduino-mcp2515
**Maintainer**: Bobertsawesome, ESP32-MCP2515 Contributors
**Original Author**: autowp <autowp@gmail.com>

## Documentation Index

**Core Documentation:**

- `CLAUDE.md` (this file) - Project overview and development guide
- `Documentation/MCP2515.md` - Complete MCP2515 datasheet reference
- `Documentation/PLATFORMIO_BUILD_TESTING_GUIDE.md` - Build automation guide for agent development
- `Documentation/PLATFORMIO_TESTING_METHODOLOGY.md` - Complete testing workflow (build, upload, monitor)

### Description

This is a production-hardened Arduino library for interfacing with the MCP2515 CAN (Controller Area Network) controller chip via SPI. The library provides CAN-BUS capability to both ESP32 and Arduino AVR boards, implementing CAN V2.0B protocol at speeds up to 1 Mb/s. It's commonly used for automotive diagnostics (OBD-II), industrial automation, and embedded systems communication.

### Platform Support

**Verified Compatible Platforms (Build-Tested):**

- **ESP32 Classic** (Dual-core Xtensa LX6 @ 240MHz)
- **ESP32-S2** (Single-core Xtensa LX7 @ 240MHz)
- **ESP32-S3** (Dual-core Xtensa LX7 @ 240MHz)
- **ESP32-C3** (Single-core RISC-V @ 160MHz)
- **Arduino Uno** (ATmega328P @ 16MHz)
- **Arduino Mega2560** (ATmega2560 @ 16MHz)

The library intelligently adapts to each platform, providing ESP32-specific features (FreeRTOS integration, interrupt-driven reception, statistics) when available, while maintaining core CAN functionality on all platforms.

### Key Features

- CAN V2.0B protocol support at up to 1 Mb/s
- SPI interface up to 10 MHz with optimized SPI instructions (10-15% performance boost)
- Standard (11-bit) and extended (29-bit) CAN frames
- Remote frames and data frames
- Two receive buffers with prioritized message storage
- Configurable filters and masks for selective message reception
- Multiple operating modes (Normal, Listen-Only, Loopback, One-Shot)
- Transmit priority control (0-3 priority levels per TX buffer)
- Abort transmission capability (individual buffers or all pending)
- Filter hit reporting for debugging and message routing

---

## Repository Structure

```
ESP32-mcp2515/
├── mcp2515.h                  # Main MCP2515 driver class header
├── mcp2515.cpp                # Main MCP2515 driver implementation
├── mcp2515_esp32_config.h     # ESP32-specific configuration
├── can.h                      # CAN frame structures (Linux SocketCAN compatible)
├── library.properties         # Arduino library metadata
├── keywords.txt               # Arduino IDE syntax highlighting keywords
├── platformio.ini             # PlatformIO multi-platform build configuration
├── read_serial.py             # Serial monitoring script for autonomous testing
├── README.md                  # User documentation
├── CLAUDE.md                  # AI assistant development guide (this file)
├── LICENSE.md                 # MIT License
├── BUILD_VERIFICATION_REPORT.md # Multi-platform build results
├── EMBEDDED_SYSTEMS_AUDIT.md  # Embedded systems audit
├── src/                       # PlatformIO test application
│   └── main.cpp              # Comprehensive test suite
├── examples/                  # Example Arduino sketches
│   ├── CAN_read/             # Basic CAN frame reception example
│   ├── CAN_write/            # Basic CAN frame transmission example
│   ├── CAN_SpeedTest/        # Performance testing example
│   ├── ESP32_CAN_comprehensive_test/ # ESP32-specific comprehensive test
│   ├── wiring.png            # MCP2515 shield wiring diagram
│   └── wiring-diy.png        # DIY MCP2515 wiring diagram
├── Documentation/             # Technical documentation
│   ├── MCP2515.md            # MCP2515 datasheet reference
│   ├── DATASHEET_COMPLIANCE_ANALYSIS.md
│   ├── ESP32_COMPREHENSIVE_AUDIT_2025-11-15.md
│   ├── PRODUCTION_CRITICAL_FIXES_2025-11-15.md
│   ├── PLATFORMIO_BUILD_TESTING_GUIDE.md
│   └── PLATFORMIO_TESTING_METHODOLOGY.md
└── logs/                      # Test output logs (gitignored)
    └── test_YYMMDD_HHMMSS.log
```

### Core Files

#### `can.h`

- Defines Linux SocketCAN-compatible structures and constants
- Main structure: `struct can_frame` with `can_id`, `can_dlc`, and `data[8]`
- CAN ID flags: `CAN_EFF_FLAG`, `CAN_RTR_FLAG`, `CAN_ERR_FLAG`
- Frame format masks: `CAN_SFF_MASK` (11-bit), `CAN_EFF_MASK` (29-bit)

#### `mcp2515.h`

- Main driver class definition
- Bitrate configuration tables for 8 MHz, 16 MHz, and 20 MHz oscillators
- Enumerations for speeds, clocks, modes, errors, registers, and instructions
- Public API for initialization, configuration, send/receive operations
- Private SPI communication methods and register manipulation

#### `mcp2515.cpp`

- Complete implementation of the MCP2515 driver
- SPI communication protocol implementation
- Register read/write operations
- Mode switching, bitrate configuration
- Frame transmission and reception logic
- Filter and mask configuration

#### `src/main.cpp`

- Comprehensive test suite for library validation
- Tests initialization, transmission, reception, modes, filters, and stress testing
- Outputs pass/fail results with detailed diagnostics
- Used for autonomous testing and development

---

## Architecture and Design Patterns

### Class Design

**Main Class**: `MCP2515`

**Constructors (Platform-Specific):**

**Arduino AVR Constructor:**

```cpp
MCP2515(const uint8_t _CS, const uint32_t _SPI_CLOCK = 10000000, SPIClass * _SPI = nullptr)
```

- `_CS`: SPI chip select pin number
- `_SPI_CLOCK`: SPI clock speed (default 10 MHz)
- `_SPI`: Optional custom SPI interface (for multi-SPI boards)

**ESP32 Simplified Constructor:**

```cpp
MCP2515(gpio_num_t cs_pin, gpio_num_t int_pin = GPIO_NUM_NC)
```

- `cs_pin`: Chip select GPIO pin
- `int_pin`: Interrupt GPIO pin (optional, GPIO_NUM_NC to disable)

**ESP32 Advanced Constructor:**

```cpp
MCP2515(const mcp2515_esp32_config_t* config)
```

- `config`: Full ESP32 configuration structure with FreeRTOS settings

### Operating Modes

The MCP2515 supports several operating modes:

1. **Normal Mode** (`setNormalMode()`): Full CAN operation with acknowledgments
2. **Normal One-Shot Mode** (`setNormalOneShotMode()`): No automatic retransmission
3. **Listen-Only Mode** (`setListenOnlyMode()`): Receive-only, no acknowledgments sent
4. **Loopback Mode** (`setLoopbackMode()`): Internal loopback for testing
5. **Sleep Mode** (`setSleepMode()`): Low-power mode
6. **Configuration Mode** (`setConfigMode()`): Required for changing settings

### CAN Frame Structure

The library uses Linux SocketCAN-compatible frame structure:

```cpp
struct can_frame {
    canid_t can_id;  // 32-bit: CAN ID (11/29 bits) + flags (3 bits MSB)
    __u8 can_dlc;    // Data Length Code (0-8 bytes)
    __u8 data[8];    // Payload data (aligned to 8 bytes)
};
```

**CAN ID Layout** (32 bits):

- Bit 0-28: CAN identifier (11 or 29 bits depending on frame type)
- Bit 29: Error frame flag (`CAN_ERR_FLAG`)
- Bit 30: Remote Transmission Request (`CAN_RTR_FLAG`)
- Bit 31: Extended Frame Format flag (`CAN_EFF_FLAG`)

### Error Handling

All major operations return `MCP2515::ERROR` enum:

- `ERROR_OK` (0): Success
- `ERROR_FAIL` (1): General failure
- `ERROR_ALLTXBUSY` (2): All transmit buffers busy
- `ERROR_FAILINIT` (3): Initialization failed
- `ERROR_FAILTX` (4): Transmission failed
- `ERROR_NOMSG` (5): No message available
- `ERROR_TIMEOUT` (6): Timeout error (ESP32)
- `ERROR_MUTEX` (7): Mutex acquisition failed (ESP32)
- `ERROR_PSRAM` (8): PSRAM+DMA conflict detected (ESP32)

### Hardware Abstraction

The library abstracts SPI communication through:

- `startSPI()`: Begin SPI transaction with chip select
- `endSPI()`: End SPI transaction
- `readRegister()`: Read single MCP2515 register
- `setRegister()`: Write single register
- `modifyRegister()`: Bit-modify register (read-modify-write)

---

## Key Conventions and Standards

### Coding Style

1. **Naming Conventions**:

   - Classes: PascalCase (e.g., `MCP2515`)
   - Enums: UPPER_CASE (e.g., `CAN_SPEED`, `CANINTF`)
   - Constants: UPPER_CASE with underscores (e.g., `CAN_MAX_DLC`)
   - Private members: camelCase (e.g., `readRegister`)
   - Public methods: camelCase (e.g., `setBitrate`, `sendMessage`)

2. **Register Definitions**:

   - Configuration registers use prefix `MCP_` (e.g., `MCP_CNF1`, `MCP_CANCTRL`)
   - Bitrate configs use pattern: `MCP_<CLOCK>_<SPEED>_CFG<N>` (e.g., `MCP_16MHz_125kBPS_CFG1`)

3. **Hardware Constants**:
   - Default SPI clock: 10 MHz
   - Supported oscillator frequencies: 8 MHz, 16 MHz (default), 20 MHz
   - Number of TX buffers: 3 (`TXB0`, `TXB1`, `TXB2`)
   - Number of RX buffers: 2 (`RXB0`, `RXB1`)
   - Number of filters: 6 (`RXF0` through `RXF5`)
   - Number of masks: 2 (`MASK0`, `MASK1`)

### Arduino Library Standards

The library follows Arduino Library Specification 1.5:

- `library.properties`: Contains metadata (name, version, author, etc.)
- `keywords.txt`: Defines syntax highlighting for Arduino IDE
- `examples/`: Contains `.ino` sketch files in subdirectories
- Main include: `#include <mcp2515.h>` (lowercase)

---

## Testing and Development

### Build System

The project uses **PlatformIO** for development and testing with zero-friction configuration:

**Build configuration** (`platformio.ini`):

- Library source files build directly from root directory (no copying required)
- Changes to `mcp2515.cpp`/`mcp2515.h` take effect immediately on next build
- Test application in `src/main.cpp`
- Supports ESP32 (Classic, S2, S3, C3) and Arduino (Uno, Mega) platforms

### Quick Start Commands

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

### Serial Monitoring

**Problem**: Native PlatformIO monitor (`pio run -t monitor`) doesn't work in non-interactive environments.

**Solution**: Use the custom `read_serial.py` script:

```bash
# Prerequisites (one-time setup)
python3 -m venv ~/.venvs/pyserial
source ~/.venvs/pyserial/bin/activate
pip install pyserial

# Monitor serial output (30 second capture)
~/.venvs/pyserial/bin/python read_serial.py

# Save to log file
~/.venvs/pyserial/bin/python read_serial.py | tee logs/test_$(date +%y%m%d_%H%M%S).log
```

### Complete Test Cycle

**1. Build and Upload:**

```bash
cd /path/to/ESP32-mcp2515
pio run -e esp32-s3 -t upload
```

**2. Monitor Output:**

```bash
~/.venvs/pyserial/bin/python read_serial.py
```

**3. Analyze Results:**

- Look for `[PASS]` and `[FAIL]` markers
- Check pass rate (should be >95% for production)
- Review stress test success rate
- Identify specific failing tests for debugging

### Autonomous Testing Workflow

For AI agents performing iterative development:

1. Make code changes to `mcp2515.cpp`/`mcp2515.h`
2. Build and upload firmware
3. Capture serial output with `read_serial.py`
4. Parse test results (pass/fail rates, error patterns)
5. If pass rate <95%, analyze failures and repeat

**Key Metrics:**

- **Pass Rate**: >95% required for production
- **Stress Test Success**: >95% required for reliable operation
- **Memory Usage**: Monitor for leaks or excessive consumption
- **Queue Overflow**: "RX queue full" warnings indicate performance issues

**Detailed documentation**: See `Documentation/PLATFORMIO_TESTING_METHODOLOGY.md`

---

## Common Development Tasks

### 1. Adding New Bitrate Configurations

When adding support for a new bitrate or clock frequency:

**Location**: `mcp2515.h` (bitrate configuration section)

**Pattern**:

```cpp
#define MCP_<CLOCK>MHz_<SPEED>kBPS_CFG1 (0xXX)
#define MCP_<CLOCK>MHz_<SPEED>kBPS_CFG2 (0xXX)
#define MCP_<CLOCK>MHz_<SPEED>kBPS_CFG3 (0xXX)
```

**Implementation**: Update `setBitrate()` function in `mcp2515.cpp`

**Important**: These values are calculated based on MCP2515 datasheet timing parameters (BRP, PRSEG, PHSEG1, PHSEG2, SJW). Verify with oscilloscope or CAN analyzer.

### 2. Adding New Operating Modes

**Existing modes** (see `CANCTRL_REQOP_MODE` enum in `mcp2515.h`):

- `CANCTRL_REQOP_NORMAL`
- `CANCTRL_REQOP_LOOPBACK`
- `CANCTRL_REQOP_LISTENONLY`
- `CANCTRL_REQOP_CONFIG`
- `CANCTRL_REQOP_OSM` (One-Shot Mode)

**To add new mode**:

1. Add enum value in `CANCTRL_REQOP_MODE`
2. Create public method: `ERROR set<Mode>Mode()`
3. Implement using `setMode(CANCTRL_REQOP_<MODE>)`

### 3. Extending Filter/Mask Functionality

**Current implementation**:

- `setFilterMask(MASK num, bool ext, uint32_t ulData)`
- `setFilter(RXF num, bool ext, uint32_t ulData)`

**Key considerations**:

- Filters are applied during initialization in `reset()`
- Default: Accept all frames (masks set to 0)
- `ext` parameter: `false` for standard (11-bit), `true` for extended (29-bit)
- Filter matching uses bitwise AND with mask

### 4. Hardware Testing

**Manual Two-Device Testing**:

- Use two boards with MCP2515 modules
- One as sender (CAN_write example), one as receiver (CAN_read example)
- Connect CAN_H, CAN_L, and GND between modules
- Add 120Ω termination resistors at both ends of CAN bus

**Loopback Testing**:

- Single device setup (no physical CAN bus required)
- MCP2515 loops frames back internally
- Still requires INT pin for interrupt-driven reception
- Use `setLoopbackMode()` for testing

---

## Important Implementation Details

### SPI Communication Protocol

The MCP2515 uses SPI Mode 0 (CPOL=0, CPHA=0):

- Data sampled on rising edge, shifted on falling edge
- MSB first
- Maximum SPI clock: 10 MHz

**Key SPI Instructions**:

- `INSTRUCTION_RESET` (0xC0): Software reset
- `INSTRUCTION_READ` (0x03): Read register
- `INSTRUCTION_WRITE` (0x02): Write register
- `INSTRUCTION_BITMOD` (0x05): Bit modify
- `INSTRUCTION_READ_STATUS` (0xA0): Quick status read
- `INSTRUCTION_RTS_TXn`: Request to send from buffer n

### Timing Considerations

1. **Reset delay**: 10ms after reset command
2. **Mode switching**: Waits for mode change confirmation (polling CANSTAT register)
3. **Message transmission**: Non-blocking; check status with `getInterrupts()`

### Memory Layout

**TX Buffers** (`N_TXBUFFERS = 3`):

- Each has control register, ID registers, DLC, and 8 data bytes
- Can be loaded and triggered independently

**RX Buffers** (`N_RXBUFFERS = 2`):

- RXB0 has higher priority than RXB1
- RXB0 can rollover to RXB1 if full (`RXB0CTRL_BUKT` bit)

### Interrupt Handling

The MCP2515 supports interrupt-driven reception:

**Interrupt flags** (see CANINTF register definitions in `mcp2515.h`):

- `CANINTF_RX0IF`: Message in RXB0
- `CANINTF_RX1IF`: Message in RXB1
- `CANINTF_TX0IF/1IF/2IF`: Transmission complete
- `CANINTF_ERRIF`: Error interrupt
- `CANINTF_WAKIF`: Wake-up interrupt
- `CANINTF_MERRF`: Message error interrupt

**Methods**:

- `getInterrupts()`: Read interrupt flags
- `clearInterrupts()`: Clear all interrupt flags
- `clearTXInterrupts()`: Clear TX interrupt flags only

---

## Known Hardware Limitations and Edge Cases

### Loopback Mode Behavior

**Critical Edge Cases** (Fixed in v2.1.1):

1. **ABAT (Abort All) Bit Stuck-On in Loopback Mode**
   - **Issue**: When `abortAllTransmissions()` is called in loopback mode, the MCP2515 hardware may not auto-clear the ABAT bit
   - **Root Cause**: No actual bus arbitration occurs in loopback mode, so the hardware doesn't complete the normal abort sequence
   - **Symptom**: All subsequent transmissions fail immediately (100% failure rate)
   - **Fix**: Library now manually clears ABAT if hardware doesn't auto-clear within 10ms (mcp2515.cpp:1201-1208)
   - **Impact**: Stress test success rate improved from 10.70% to 100.00%

2. **ISR Task Frame Consumption in Polling Mode**
   - **Issue**: Prior to v2.1.1, the ISR task would continue consuming frames from hardware even when interrupts were "disabled"
   - **Root Cause**: ISR task wakes up on timeout (10ms) and processed interrupts regardless of `use_interrupts` flag
   - **Symptom**: Frames disappear from hardware, `checkReceive()` returns false despite RXnIF flags set, polling mode fails
   - **Fix**: `processInterrupts()` now checks `use_interrupts` flag before consuming frames (mcp2515.cpp:1663)
   - **Impact**: Polling mode diagnostic now works correctly

3. **Queue vs Hardware Reception Order**
   - **Issue**: On ESP32 with interrupt mode, checking hardware before queue causes missed frames
   - **Root Cause**: ISR task consumes hardware buffers and places frames in FreeRTOS queue
   - **Symptom**: `checkReceive()` returns false even when frames are in queue
   - **Fix**: `checkReceive()` now checks queue first, then hardware (mcp2515.cpp:1379)
   - **Impact**: Correct frame detection in interrupt mode

**Hardware Limitations** (Cannot be fixed by library):

1. **Filters Apply in Loopback Mode**
   - Acceptance filters and masks are still evaluated even in loopback mode
   - **Workaround**: Set masks to 0x000 to accept all frames during loopback testing
   - **Example**: `mcp2515.setFilterMask(MCP2515::MASK0, false, 0x00000000)`

2. **Loopback Mode Timing Requirements**
   - MCP2515 requires 5-10ms settle time after entering loopback mode
   - **Workaround**: Add `delay(10)` after `setLoopbackMode()`
   - **Reason**: Hardware needs time to reconfigure internal bus connections

### ESP32-Specific Considerations

1. **PSRAM + DMA Incompatibility**
   - ESP32 DMA cannot access PSRAM memory
   - Library detects this configuration and returns `ERROR_PSRAM`
   - **Solution**: Either disable PSRAM in sdkconfig OR disable SPI DMA in `mcp2515_esp32_config.h`

2. **ISR Task Core Affinity**
   - ISR task is pinned to Core 1 by default for deterministic performance
   - Core 0 runs WiFi/BLE stack (higher priority system tasks)
   - Core 1 is recommended for CAN processing
   - **Configure**: `MCP2515_ISR_TASK_CORE` in `mcp2515_esp32_config.h`

3. **Mutex Timeout**
   - Reduced from 100ms to 10ms in v2.1.0 to catch deadlocks faster
   - Worst-case SPI transaction is ~1ms, so 10ms is generous
   - Faster timeout prevents RX queue overflow during mutex contention
   - **Configure**: `MCP2515_MUTEX_TIMEOUT` in `mcp2515_esp32_config.h`

**Testing Recommendations**:

- Use loopback mode for automated testing (no physical CAN bus required)
- Always configure filters to accept all frames in loopback mode
- Allow settle time after mode switches
- Test both polling mode and interrupt mode
- Monitor queue depth with `getRxQueueCount()` on ESP32
- Check statistics with `getStatistics()` to detect silent failures

**See Also**: `Documentation/API_REFERENCE.md` section "Known Issues and Edge Cases" for complete API-level documentation.

---

## AI Assistant Guidelines

### When Modifying Code

1. **Preserve compatibility**: This library is used in production systems. Breaking changes require major version bump.

2. **Hardware-specific code**: Be extremely careful with register values and timing. Incorrect values can prevent communication.

3. **SPI thread safety**: The library is not thread-safe. Document if adding multi-threading support.

4. **Memory constraints**: Arduino Uno has only 2KB RAM. Avoid large buffers or dynamic allocation.

5. **Test on hardware**: Simulation is insufficient for CAN/SPI communication. Always recommend hardware testing.

### Common Questions

**Q: Why use SocketCAN structure?**
A: Compatibility with Linux CAN tools and easy porting between platforms.

**Q: Can I use multiple MCP2515 chips?**
A: Yes, use different CS pins and create separate MCP2515 objects.

**Q: Why 120Ω termination?**
A: CAN bus impedance is 120Ω. Terminators prevent signal reflections.

**Q: What's the difference between Normal and One-Shot mode?**
A: One-Shot mode doesn't retry failed transmissions (useful for time-critical data).

### Code Review Checklist

When reviewing changes:

- [ ] Verify register addresses against MCP2515 datasheet
- [ ] Check for proper error handling (return `ERROR` enum)
- [ ] Ensure SPI transactions are wrapped in `startSPI()`/`endSPI()`
- [ ] Validate timing calculations for bitrate configurations
- [ ] Confirm examples compile for Arduino Uno, ESP32, and other targets
- [ ] Check that `library.properties` version is updated if needed
- [ ] Verify keywords.txt updated for new public constants/methods
- [ ] Test with actual hardware if possible

### Documentation Standards

- Keep README.md as primary user documentation
- Use C++ style comments for implementation details
- Document register bit manipulation with references to datasheet sections
- Include wiring diagrams for hardware changes
- Update CHANGELOG.md for user-facing changes (create if doesn't exist)

---

## Debugging Tips

### Common Issues

1. **"ERROR_FAILINIT"**:

   - Check SPI wiring (MISO, MOSI, SCK, CS)
   - Verify oscillator frequency matches code (8/16/20 MHz)
   - Check power supply (MCP2515 requires stable 5V or 3.3V)

2. **"ERROR_ALLTXBUSY"**:

   - No acknowledgment from other CAN node
   - Check CAN_H, CAN_L connections
   - Verify termination resistors (120Ω at both ends)
   - Confirm matching bitrate on all nodes

3. **Messages not received**:

   - Check filters/masks configuration
   - Verify bitrate matches sender
   - Ensure transceiver (MCP2551/2562/TJA1055) is powered

4. **Intermittent communication**:
   - Cable length too long (max ~1000m at 50kbps, ~40m at 1Mbps)
   - EMI interference (keep cables away from motors, high-current lines)
   - Improper grounding

### Debug Methods

**Check Hardware Connection:**

- Use `reset()` and check for `ERROR_OK` return value
- Verify SPI communication is working

**Monitor Error Flags:**

- Use `getErrorFlags()` to read error conditions
- Check for `EFLG_RX0OVR`, `EFLG_RX1OVR` (buffer overflow)
- Check for `EFLG_TXBO` (bus-off condition)

**Check Error Counters:**

- Use `errorCountRX()` to read receive error counter
- Use `errorCountTX()` to read transmit error counter
- Rising counters indicate communication issues

**ESP32-Specific Diagnostics:**

- Use `getStatistics()` to retrieve comprehensive stats
- Monitor RX/TX frame counts, errors, overflows, and bus-off events

---

## External References

### Datasheets

- [MCP2515 Datasheet](https://www.microchip.com/wwwproducts/en/MCP2515) - CAN controller
- [MCP2551 Datasheet](https://www.microchip.com/wwwproducts/en/MCP2551) - CAN transceiver (legacy)
- [MCP2562 Datasheet](https://www.microchip.com/wwwproducts/en/MCP2562) - Modern CAN transceiver
- [TJA1055 Datasheet](https://www.nxp.com/docs/en/data-sheet/TJA1055.pdf) - Automotive CAN transceiver

### Standards

- [ISO 11898-1](https://www.iso.org/standard/63648.html) - CAN protocol specification
- [SocketCAN Documentation](https://www.kernel.org/doc/Documentation/networking/can.txt) - Linux CAN framework

### Related Projects

- [can-usb](https://github.com/autowp/can-usb) - CanHacker/lawicel protocol implementation
- [Seeed CAN-BUS Shield Wiki](http://www.seeedstudio.com/wiki/CAN-BUS_Shield) - Hardware reference

---

## Version History (Recent)

- **2.1.1-ESP32** (2025-11-18): Critical bug fixes for loopback mode and interrupt handling
  - **Fixed ABAT Stuck-On in Loopback Mode**: `abortAllTransmissions()` now manually clears ABAT bit if hardware doesn't auto-clear within 10ms. In loopback mode, the MCP2515 hardware may not auto-clear ABAT because no actual bus arbitration occurs. This caused 100% transmission failure after calling `abortAllTransmissions()`.
  - **Fixed ISR Task Frame Consumption in Polling Mode**: `processInterrupts()` now checks `use_interrupts` flag before consuming frames. Prior to this fix, the ISR task would continue processing on timeout even when interrupts were "disabled" via `setInterruptMode(false)`, causing race conditions where frames disappeared from hardware buffers.
  - **Fixed Queue-First Reception Check**: `checkReceive()` now checks FreeRTOS queue before hardware registers when in interrupt mode on ESP32. This prevents missed frames because the ISR task consumes hardware buffers and places frames in the queue.
  - **Impact**: Loopback stress test success rate improved from 10.70% to 100.00%. Overall test pass rate improved from 74.71% to 97.85% (91/93 tests passing).
  - **Files Modified**: `mcp2515.cpp` (3 critical fixes), `mcp2515.h` (added diagnostic method)
  - See `Documentation/API_REFERENCE.md` sections: "Known Issues and Edge Cases" and "Version History - Critical Fixes" for detailed documentation
- **2.1.0-ESP32** (2025-11-15): BREAKING CHANGES - Production hardening and multi-platform support
  - **BREAKING**: Changed 4 functions from void→ERROR return type for explicit failure detection
  - **BREAKING**: Added ERROR_MUTEX (7) and ERROR_PSRAM (8) error codes
  - Added IRAM_ATTR to 14 functions for flash-safe ISR execution
  - Pinned ISR task to Core 1 for predictable performance
  - Added comprehensive PSRAM safety checks (prevents DMA+PSRAM crashes)
  - Verified multi-platform build support:
    - ESP32 Classic, ESP32-S2, ESP32-S3, ESP32-C3
    - Arduino Uno, Arduino Mega2560
  - Platform-specific conditional compilation
  - Updated ~30 caller sites with proper error checking
  - Comprehensive embedded systems audit completed
- **2.0.0-ESP32** (2025-11-15): Performance and feature enhancements
  - Added SPI optimization using READ RX BUFFER instruction (10-15% faster reception)
  - Added SPI optimization using LOAD TX BUFFER instruction (5-10% faster transmission)
  - Added `setTransmitPriority()` for message priority control (0-3 priority levels)
  - Added `abortTransmission()` to abort specific TX buffer
  - Added `abortAllTransmissions()` to abort all pending messages
  - Added `getFilterHit()` to report which acceptance filter matched received message
- **1.3.1**: One-Shot Mode refactoring, added `CANCTRL_REQOP_OSM`
- **1.3.0**: Can.h alignment fix for compatibility
- Custom SPI peripheral support added
- 95kbps @ 16MHz support added
- SPI clock speed customization

---

## Contributing

This is an active open-source project welcoming contributions:

1. Fork the repository
2. Create feature branch
3. Test changes on real hardware
4. Submit pull request with clear description
5. Update documentation as needed

**Maintainer Response Time**: Usually within 1-2 weeks for PRs

---

## Quick Reference: Essential Functions

### Initialization

```cpp
MCP2515 mcp2515(CS_PIN);
mcp2515.reset();
mcp2515.setBitrate(CAN_125KBPS, MCP_16MHZ);
mcp2515.setNormalMode();
```

### Sending

```cpp
struct can_frame frame;
frame.can_id = 0x123;
frame.can_dlc = 8;
frame.data[0] = 0xAA;
// ... set remaining data
mcp2515.sendMessage(&frame);
```

### Receiving (Polling)

```cpp
struct can_frame frame;
if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    // Process frame
}
```

### Receiving (Interrupt)

```cpp
void irqHandler() { /* set flag */ }
attachInterrupt(digitalPinToInterrupt(INT_PIN), irqHandler, FALLING);

if (interrupt_flag) {
    uint8_t irq = mcp2515.getInterrupts();
    if (irq & MCP2515::CANINTF_RX0IF) {
        mcp2515.readMessage(MCP2515::RXB0, &frame);
    }
}
```

### Transmit Priority Control

```cpp
// Set message priority for a specific TX buffer (0-3, where 3 = highest)
// Must be called before sending message on that buffer
mcp2515.setTransmitPriority(MCP2515::TXB0, 3);  // Highest priority
mcp2515.setTransmitPriority(MCP2515::TXB1, 1);  // Low priority
mcp2515.setTransmitPriority(MCP2515::TXB2, 2);  // Medium priority

// When multiple buffers are pending, highest priority is transmitted first
// If priorities are equal, buffer with higher number (TXB2 > TXB1 > TXB0) wins
```

### Abort Transmission

```cpp
// Abort transmission from specific buffer
mcp2515.abortTransmission(MCP2515::TXB0);

// Abort all pending transmissions
mcp2515.abortAllTransmissions();
```

### Filter Hit Reporting

```cpp
// Determine which acceptance filter matched the received message
struct can_frame frame;
if (mcp2515.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) {
    uint8_t filter = mcp2515.getFilterHit(MCP2515::RXB0);
    // filter = 0-5 for RXF0-RXF5
    // Useful for routing messages or debugging filter configuration
}
```

### Performance Optimizations

The library automatically uses optimized SPI instructions for better performance:

- **READ RX BUFFER** instruction (0x90/0x94): Saves 1 byte per RX operation, 10-15% faster reception
- **LOAD TX BUFFER** instruction (0x40/0x42/0x44): Saves 1 byte per TX operation, 5-10% faster transmission

These optimizations are transparent to the user and work with existing code.

---

**Last Updated**: 2025-11-18
**Document Version**: 2.2
**For Library Version**: 2.1.1-ESP32
**Platforms Verified**: ESP32 (Classic/S2/S3/C3), Arduino (Uno/Mega2560)

**Key Additions in v2.2:**

- Version 2.1.1 critical bug fixes documented
- Loopback mode edge cases and hardware limitations
- Detailed API reference updates for all affected functions
- Complete edge case documentation in API_REFERENCE.md

**Key Additions in v2.1:**

- Testing and Development section with PlatformIO workflow
- Serial monitoring with `read_serial.py` script
- Autonomous testing workflow for AI agents
- Removed specific line number references for maintainability

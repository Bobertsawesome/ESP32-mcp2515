# Arduino IDE Compatibility Guide

## Overview

This ESP32-MCP2515 library is fully compatible with the Arduino IDE and follows the Arduino Library Specification 1.5. The library has been properly configured with all required metadata files for seamless integration.

---

## Library Metadata

### Library Name
**ESP32-MCP2515** (renamed from `autowp-mcp2515`)

### Version
**2.0.0** - Major version bump to reflect significant ESP32 enhancements

### Authors
- **Original Author**: autowp <autowp@gmail.com>
- **ESP32 Refactor**: Bobertsawesome and ESP32-MCP2515 Contributors

### Architecture
**esp32** - Specifically targets ESP32 platform while maintaining backward compatibility

---

## Arduino IDE Installation

### Method 1: Manual Installation

1. Download or clone this repository
2. Locate your Arduino libraries folder:
   - **Windows**: `Documents/Arduino/libraries/`
   - **macOS**: `~/Documents/Arduino/libraries/`
   - **Linux**: `~/Arduino/libraries/`
3. Copy the entire `ESP32-mcp2515` folder into the libraries directory
4. Restart Arduino IDE
5. The library will appear under **Sketch → Include Library → ESP32-MCP2515**

### Method 2: ZIP Import

1. Download the repository as a ZIP file
2. In Arduino IDE: **Sketch → Include Library → Add .ZIP Library...**
3. Select the downloaded ZIP file
4. Restart Arduino IDE

### Method 3: Git Clone (Advanced)

```bash
cd ~/Arduino/libraries/
git clone https://github.com/Bobertsawesome/ESP32-mcp2515.git
```

---

## Verifying Installation

After installation, verify the library is properly loaded:

1. Open Arduino IDE
2. Go to **File → Examples**
3. Scroll down to **ESP32-MCP2515** section
4. You should see the following examples:
   - ESP32_CAN_read
   - ESP32_CAN_write
   - ESP32_CAN_advanced
   - ESP32_CAN_comprehensive_test
   - CAN_read (legacy)
   - CAN_write (legacy)
   - CAN_SpeedTest (legacy)

---

## Board Setup

### Required Board Package

Install the **ESP32 Arduino Core** board package:

1. Open **File → Preferences**
2. Add this URL to "Additional Boards Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Open **Tools → Board → Boards Manager**
4. Search for "esp32"
5. Install **esp32 by Espressif Systems** (version 2.0.0 or later recommended)

### Selecting Your Board

1. Go to **Tools → Board → ESP32 Arduino**
2. Select your ESP32 variant:
   - ESP32 Dev Module
   - ESP32-S2 Dev Module
   - ESP32-C3 Dev Module
   - DOIT ESP32 DEVKIT V1
   - NodeMCU-32S
   - etc.

---

## Using the Library in Your Sketch

### Basic Include

```cpp
#include <SPI.h>
#include <mcp2515.h>
```

### Syntax Highlighting

The library provides full syntax highlighting in the Arduino IDE for:

#### Classes and Data Types (Blue)
- `MCP2515`
- `can_frame`
- `mcp2515_esp32_pins_t`
- `mcp2515_esp32_config_t`
- `mcp2515_statistics_t`

#### Methods (Orange/Brown)
- All public methods like `reset()`, `sendMessage()`, `readMessage()`, etc.

#### Constants (Teal/Cyan)
- Speed constants: `CAN_125KBPS`, `CAN_500KBPS`, etc.
- Clock constants: `MCP_8MHZ`, `MCP_16MHZ`, `MCP_20MHZ`
- Error codes: `ERROR_OK`, `ERROR_FAIL`, etc.
- Buffer names: `TXB0`, `RXB0`, etc.
- Flags: `CANINTF_RX0IF`, `EFLG_TXBO`, etc.

---

## Library Structure

The library follows Arduino Library Specification 1.5:

```
ESP32-mcp2515/
├── library.properties          # Library metadata
├── keywords.txt                # Syntax highlighting definitions
├── LICENSE.md                  # MIT License
├── README.md                   # Main documentation
├── README_ESP32.md             # ESP32-specific documentation
├── CLAUDE.md                   # Developer guide
├── mcp2515.h                   # Main header file
├── mcp2515.cpp                 # Main implementation
├── can.h                       # CAN frame structures
├── mcp2515_esp32_config.h      # ESP32 configuration
├── CMakeLists.txt              # ESP-IDF support
├── Kconfig                     # ESP-IDF configuration menu
└── examples/                   # Example sketches
    ├── ESP32_CAN_read/
    ├── ESP32_CAN_write/
    ├── ESP32_CAN_advanced/
    ├── ESP32_CAN_comprehensive_test/
    ├── CAN_read/               # Legacy compatibility
    ├── CAN_write/              # Legacy compatibility
    └── CAN_SpeedTest/          # Legacy compatibility
```

---

## Compilation Settings

### Recommended Settings

For optimal performance with this library:

**Tools → Arduino IDE Settings:**
- **Upload Speed**: 921600 (for faster uploads)
- **CPU Frequency**: 240MHz (default)
- **Flash Frequency**: 80MHz (default)
- **Flash Mode**: QIO (default)
- **Flash Size**: 4MB (or your board's actual size)
- **Partition Scheme**: Default 4MB with spiffs
- **Core Debug Level**: None (or "Info" for debugging)

### Compiler Warnings

The library compiles cleanly with zero warnings. If you see warnings:
1. Ensure you're using ESP32 Arduino Core 2.0.0 or later
2. Check that all dependencies are installed
3. Verify board selection matches your hardware

---

## Dependencies

### Required Libraries (Auto-installed with ESP32 Core)
- **SPI.h** - Built-in SPI library
- **FreeRTOS** - Built into ESP32 core (for RTOS features)
- **driver/spi_master.h** - ESP32 native SPI driver (ESP-IDF mode)
- **driver/gpio.h** - ESP32 GPIO driver (ESP-IDF mode)

### No External Dependencies Required
This library has **zero external dependencies** beyond the ESP32 Arduino Core.

---

## Examples Overview

### ESP32-Specific Examples (Recommended)

1. **ESP32_CAN_read**
   - Simple CAN frame reception with interrupts
   - Shows queued reading and statistics
   - Best starting point for ESP32

2. **ESP32_CAN_write**
   - Simple CAN frame transmission
   - Demonstrates error handling
   - Thread-safe operation example

3. **ESP32_CAN_advanced**
   - Full-featured example with filters, masks
   - Multiple operating modes
   - Error recovery and diagnostics

4. **ESP32_CAN_comprehensive_test**
   - Complete library test suite
   - Tests all 38+ functions
   - Hardware validation tool
   - Loopback and two-device modes

### Legacy Examples (Arduino Compatibility)

5. **CAN_read** - Basic Arduino-style reception
6. **CAN_write** - Basic Arduino-style transmission
7. **CAN_SpeedTest** - Performance benchmarking

---

## Troubleshooting

### Library Not Showing in Arduino IDE

**Solution:**
1. Check library is in correct folder: `Arduino/libraries/ESP32-mcp2515/`
2. Restart Arduino IDE
3. Verify folder structure matches above
4. Check that `library.properties` exists in library root

### Compilation Errors

**"mcp2515.h: No such file or directory"**
- Reinstall library
- Check include statement: `#include <mcp2515.h>` (not `#include "mcp2515.h"`)

**"'MCP2515' does not name a type"**
- Add `#include <mcp2515.h>` to top of sketch
- Verify ESP32 board is selected

**Undefined reference errors**
- Select correct ESP32 board in Tools → Board
- Clean build: Delete build cache and recompile

### Upload Issues

**"Failed to connect to ESP32"**
1. Check USB cable (must be data cable, not charge-only)
2. Hold BOOT button during upload
3. Try different upload speed (115200)
4. Check serial port selection

---

## Advanced Configuration

### Custom SPI Pins

You can use non-default SPI pins:

```cpp
SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
MCP2515 mcp2515(CS_PIN);
```

### Using Multiple MCP2515 Modules

Use different CS pins for each module:

```cpp
MCP2515 can1(5);  // CS on GPIO 5
MCP2515 can2(15); // CS on GPIO 15
```

### ESP-IDF Framework

This library also works with ESP-IDF (not just Arduino):
- See `CMakeLists.txt` for component configuration
- See `Kconfig` for menuconfig integration
- Native ESP-IDF mode uses optimized SPI driver

---

## Performance Tips

### For Maximum Throughput
1. Use interrupt mode (provide INT pin)
2. Set CPU to 240MHz
3. Use DMA (enabled by default)
4. Increase RX queue size in config

### For Minimum Power
1. Use polling instead of interrupts
2. Lower CPU frequency when idle
3. Use sleep mode when not communicating
4. Enable ESP32 power management

---

## Support

### Documentation
- **README.md** - General library documentation
- **README_ESP32.md** - ESP32-specific features
- **CLAUDE.md** - Developer and AI assistant guide
- **Example sketches** - Inline comments and usage

### Issues and Questions
- GitHub Issues: https://github.com/Bobertsawesome/ESP32-mcp2515/issues
- Check existing examples first
- Include code, wiring, and error messages

### Contributing
Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Test thoroughly with examples
4. Submit pull request with description

---

## Version History

### v2.0.0 (2025)
- **Major refactor for ESP32**
- FreeRTOS integration with thread-safe operations
- Native ESP-IDF SPI driver support
- Interrupt-driven reception with queues
- Statistics tracking and error recovery
- Comprehensive test suite
- Updated Arduino IDE metadata
- Enhanced examples

### v1.3.1 (Previous - autowp)
- Original Arduino library
- Basic MCP2515 support
- Standard Arduino API

---

## License

**MIT License** - See LICENSE.md for full text

Copyright:
- © 2013 Seeed Technology Inc.
- © 2016 Dmitry (autowp)
- © 2025 ESP32-MCP2515 Contributors

---

## Credits

### Original Library
- **autowp** - Original arduino-mcp2515 library
- **Seeed Technology** - Initial CAN-BUS shield library

### ESP32 Refactor
- **ESP32-MCP2515 Contributors** - FreeRTOS integration, bug fixes, enhancements
- **Community** - Testing, bug reports, feature requests

---

**Installation complete! You're ready to use ESP32-MCP2515 in your projects.**

For quick start, try: **File → Examples → ESP32-MCP2515 → ESP32_CAN_read**

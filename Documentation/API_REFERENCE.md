# ESP32-MCP2515 Library API Reference

**Version**: 2.1.0-ESP32
**Date**: 2025-11-16
**License**: MIT

---

## Table of Contents

1. [Library Overview](#library-overview)
2. [Quick Start](#quick-start)
3. [CAN Frame Structure](#can-frame-structure)
4. [Class Reference: MCP2515](#class-reference-mcp2515)
5. [Configuration Structures (ESP32)](#configuration-structures-esp32)
6. [Enumerations](#enumerations)
7. [Error Codes](#error-codes)
8. [Bitrate Configuration Constants](#bitrate-configuration-constants)
9. [Platform-Specific Behavior](#platform-specific-behavior)
10. [Memory and Performance](#memory-and-performance)
11. [ISR Safety and Thread Safety](#isr-safety-and-thread-safety)
12. [Example Code Patterns](#example-code-patterns)
13. [Troubleshooting Guide](#troubleshooting-guide)

---

## Library Overview

### Purpose

The ESP32-MCP2515 library provides full-featured CAN (Controller Area Network) communication for embedded systems through the MCP2515 SPI-to-CAN bridge chip. It implements CAN V2.0B protocol with support for standard (11-bit) and extended (29-bit) identifiers at speeds up to 1 Mbps.

### Key Features

- **CAN V2.0B Protocol**: Full support for standard and extended frames
- **Multi-Platform**: ESP32 (all variants), Arduino AVR (Uno, Mega2560)
- **Optimized SPI**: Uses MCP2515 fast instructions (READ RX BUFFER, LOAD TX BUFFER)
- **ESP32 FreeRTOS**: Interrupt-driven reception, thread-safe operations, statistics tracking
- **Flexible Filtering**: 6 acceptance filters, 2 masks for selective message reception
- **Multiple Operating Modes**: Normal, Listen-Only, Loopback, One-Shot, Sleep
- **Transmit Priority Control**: 4-level priority system per TX buffer
- **Error Recovery**: Automatic bus-off recovery, comprehensive error reporting

### Hardware Requirements

**Required Components**:
- MCP2515 CAN controller IC
- MCP2551/MCP2562/TJA1055 CAN transceiver
- CAN-compatible oscillator (8 MHz, 16 MHz, or 20 MHz crystal)
- 120Ω termination resistors (both ends of CAN bus)

**Pin Connections**:
```
MCU          MCP2515
────────────────────
MISO    →    SO
MOSI    →    SI
SCK     →    SCK
CS      →    CS
INT     →    INT (optional, for interrupts)
3.3V/5V →    VCC
GND     →    GND

MCP2515      CAN Transceiver
───────────────────────────────
CANL    →    CANL
CANH    →    CANH
```

**Power Requirements**:
- MCP2515: 5V or 3.3V (check datasheet for your module)
- Current: ~5mA (standby), ~10mA (active)
- CAN transceiver: Per IC datasheet

**Timing Requirements**:
- SPI clock: Up to 10 MHz
- CAN bitrate: 5 kbps to 1 Mbps
- Oscillator accuracy: ±0.5% for reliable communication

---

## Quick Start

### Arduino AVR (Uno, Mega2560)

```cpp
#include <SPI.h>
#include <mcp2515.h>

// Basic constructor (Arduino AVR)
MCP2515 mcp2515(10);  // CS pin = 10

void setup() {
  Serial.begin(115200);

  // Initialize MCP2515
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();

  Serial.println("CAN initialized");
}

void loop() {
  struct can_frame frame;

  // Receive
  if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    Serial.print("ID: 0x");
    Serial.println(frame.can_id, HEX);
  }

  // Send
  frame.can_id = 0x123;
  frame.can_dlc = 8;
  for (int i = 0; i < 8; i++) {
    frame.data[i] = i;
  }
  mcp2515.sendMessage(&frame);

  delay(100);
}
```

### ESP32 (Arduino Framework)

```cpp
#include <SPI.h>
#include <mcp2515.h>

// ESP32 simplified constructor with interrupts
MCP2515 mcp2515(GPIO_NUM_5, GPIO_NUM_4);  // CS=GPIO5, INT=GPIO4

void setup() {
  Serial.begin(115200);

  // Initialize MCP2515
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();

  Serial.println("ESP32 CAN initialized with interrupts");
}

void loop() {
  struct can_frame frame;

  // Non-blocking queued read (interrupt mode)
  if (mcp2515.readMessageQueued(&frame, 0) == MCP2515::ERROR_OK) {
    Serial.printf("Received ID: 0x%03X, DLC: %d\n",
                  frame.can_id, frame.can_dlc);
  }

  delay(10);
}
```

### ESP32 (Native ESP-IDF)

```cpp
#include "mcp2515.h"

// Full configuration structure
mcp2515_esp32_config_t config = {
  .spi_host = SPI2_HOST,
  .spi_clock_speed = 10000000,  // 10 MHz
  .pins = {
    .miso = GPIO_NUM_19,
    .mosi = GPIO_NUM_23,
    .sclk = GPIO_NUM_18,
    .cs = GPIO_NUM_5,
    .irq = GPIO_NUM_4
  },
  .use_interrupts = true,
  .use_mutex = true,
  .rx_queue_size = 32,
  .isr_task_priority = 5,
  .isr_task_stack_size = 4096
};

MCP2515 mcp2515(&config);

void app_main() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();

  while(1) {
    struct can_frame frame;
    if (mcp2515.readMessageQueued(&frame, pdMS_TO_TICKS(100))
        == MCP2515::ERROR_OK) {
      // Process frame
    }
  }
}
```

---

## CAN Frame Structure

### struct can_frame

**Definition** (`can.h`):
```cpp
struct can_frame {
    canid_t can_id;   // 32-bit: ID (11/29 bits) + flags (3 bits MSB)
    __u8    can_dlc;  // Data Length Code (0-8 bytes)
    __u8    data[8];  // Payload data (8-byte aligned)
};
```

**Type Definitions**:
```cpp
typedef uint32_t canid_t;
typedef uint8_t  __u8;
```

### can_id Field Layout (32 bits)

```
Bit 31   Bit 30   Bit 29   Bits 28-0
─────────────────────────────────────
EFF      RTR      ERR      CAN ID
```

- **Bit 31 (EFF Flag)**: Extended Frame Format
  - `1` = Extended 29-bit identifier
  - `0` = Standard 11-bit identifier

- **Bit 30 (RTR Flag)**: Remote Transmission Request
  - `1` = Remote frame (request data)
  - `0` = Data frame

- **Bit 29 (ERR Flag)**: Error frame
  - `1` = Error message frame
  - `0` = Normal data/remote frame

- **Bits 28-0**: CAN identifier
  - Standard: Bits 10-0 used, bits 28-11 ignored
  - Extended: Bits 28-0 used

### CAN ID Constants

**Frame Format Flags**:
```cpp
#define CAN_EFF_FLAG  0x80000000UL  // Extended frame
#define CAN_RTR_FLAG  0x40000000UL  // Remote frame
#define CAN_ERR_FLAG  0x20000000UL  // Error frame
```

**ID Masks**:
```cpp
#define CAN_SFF_MASK  0x000007FFUL  // Standard 11-bit mask
#define CAN_EFF_MASK  0x1FFFFFFFUL  // Extended 29-bit mask
#define CAN_ERR_MASK  0x1FFFFFFFUL  // Error mask (no flags)
```

**Data Length**:
```cpp
#define CAN_MAX_DLC   8  // Maximum Data Length Code
#define CAN_MAX_DLEN  8  // Maximum data bytes
```

**Identifier Bit Counts**:
```cpp
#define CAN_SFF_ID_BITS  11  // Standard frame ID bits
#define CAN_EFF_ID_BITS  29  // Extended frame ID bits
```

### Frame Construction Examples

**Standard Data Frame (11-bit ID)**:
```cpp
struct can_frame frame;
frame.can_id = 0x123;              // Standard ID
frame.can_dlc = 8;                 // 8 data bytes
frame.data[0] = 0xAA;
frame.data[1] = 0xBB;
// ... fill remaining bytes
```

**Extended Data Frame (29-bit ID)**:
```cpp
struct can_frame frame;
frame.can_id = 0x12345678 | CAN_EFF_FLAG;  // Extended ID
frame.can_dlc = 4;
memcpy(frame.data, "TEST", 4);
```

**Remote Frame (Request Data)**:
```cpp
struct can_frame frame;
frame.can_id = 0x456 | CAN_RTR_FLAG;  // Remote request
frame.can_dlc = 8;                    // Expected data length
// data[] field ignored for RTR frames
```

**Extended Remote Frame**:
```cpp
struct can_frame frame;
frame.can_id = 0x1FFFFFFF | CAN_EFF_FLAG | CAN_RTR_FLAG;
frame.can_dlc = 8;
```

### Frame Validation

**Valid DLC Range**: 0-8 bytes
- Values > 8 will cause `sendMessage()` to return `ERROR_FAILTX`
- Remote frames can have DLC > 0 even with no data

**Valid ID Ranges**:
- Standard: 0x000 to 0x7FF (11 bits)
- Extended: 0x00000000 to 0x1FFFFFFF (29 bits)

**Memory Alignment**:
- `data[]` field is 8-byte aligned for efficient copying
- Use `memcpy()` for bulk data transfer

---

## Class Reference: MCP2515

### Constructors

#### Arduino AVR Constructor

```cpp
MCP2515(const uint8_t _CS,
        const uint32_t _SPI_CLOCK = 10000000,
        SPIClass * _SPI = nullptr)
```

**Purpose**: Create MCP2515 instance for Arduino AVR platforms (Uno, Mega2560).

**Parameters**:
- `_CS` (uint8_t): SPI chip select pin number
  - Valid range: Any digital pin (typically 10 for Uno, 53 for Mega)
  - Pin is automatically configured as OUTPUT

- `_SPI_CLOCK` (uint32_t): SPI clock frequency in Hz
  - Default: 10,000,000 (10 MHz)
  - Valid range: 100,000 to 10,000,000 Hz
  - Recommended: 10 MHz for reliable operation
  - Maximum: Limited by MCP2515 datasheet (10 MHz)

- `_SPI` (SPIClass*): Pointer to SPI interface object
  - Default: `nullptr` (uses default `SPI` object)
  - Use for boards with multiple SPI buses
  - If `nullptr`, calls `SPI.begin()` automatically

**Memory Usage**:
- Instance size: ~140 bytes (Arduino AVR)
- No dynamic allocation

**Example**:
```cpp
// Basic usage (default SPI, 10 MHz)
MCP2515 can(10);

// Custom SPI clock (5 MHz)
MCP2515 can(10, 5000000);

// Arduino Mega with second SPI bus
SPIClass mySPI(&PORTB, 2, 1, 0);
MCP2515 can(53, 10000000, &mySPI);
```

**Thread Safety**: Not thread-safe (Arduino AVR is single-threaded)

**ISR Safety**: Not ISR-safe (constructor should not be called from ISR)

---

#### ESP32 Simplified Constructor

```cpp
MCP2515(gpio_num_t cs_pin, gpio_num_t int_pin = GPIO_NUM_NC)
```

**Purpose**: Create MCP2515 instance for ESP32 with default settings and optional interrupts.

**Parameters**:
- `cs_pin` (gpio_num_t): Chip select GPIO pin
  - Valid range: Any valid GPIO (check ESP32 variant pinout)
  - Example: `GPIO_NUM_5`, `GPIO_NUM_10`

- `int_pin` (gpio_num_t): Interrupt GPIO pin (optional)
  - Default: `GPIO_NUM_NC` (interrupts disabled)
  - Use any GPIO capable of interrupts
  - Enables interrupt-driven reception when specified

**Default Configuration**:
- SPI host: `MCP2515_SPI_HOST` (variant-specific, typically SPI2 or SPI3)
- SPI clock: 10 MHz
- SPI pins: Variant-specific defaults (e.g., ESP32 Classic uses GPIO 23/19/18)
- Mutex: Enabled (thread-safe)
- RX queue size: 32 frames
- ISR task priority: `configMAX_PRIORITIES - 2`
- ISR task stack: 4096 bytes
- ISR task core: Core 1 (deterministic performance)

**Memory Usage**:
- Instance size: ~180 bytes
- RX queue: 32 * sizeof(can_frame) = 32 * 16 = 512 bytes
- ISR task stack: 4096 bytes
- **Total**: ~4.8 KB

**Blocking Behavior**:
- Constructor blocks for ~1-2ms (SPI initialization)
- Non-blocking after construction

**Failure Modes**:
- Returns without error if SPI init fails (check with `reset()` return value)
- Logs error to ESP_LOGE if ISR/queue creation fails

**Example**:
```cpp
// Polling mode (no interrupts)
MCP2515 can(GPIO_NUM_5);

// Interrupt mode
MCP2515 can(GPIO_NUM_5, GPIO_NUM_4);

// Custom GPIO pins
MCP2515 can(GPIO_NUM_10, GPIO_NUM_9);
```

**Thread Safety**: Thread-safe (uses FreeRTOS recursive mutex)

**ISR Safety**: Not ISR-safe (constructor should not be called from ISR)

---

#### ESP32 Advanced Constructor

```cpp
MCP2515(const mcp2515_esp32_config_t* config)
```

**Purpose**: Create MCP2515 instance with full control over all ESP32-specific features.

**Parameters**:
- `config` (mcp2515_esp32_config_t*): Pointer to configuration structure
  - Must remain valid during constructor execution
  - Can be stack-allocated or static
  - See [Configuration Structures](#configuration-structures-esp32) for details

**Memory Usage**:
- Instance size: ~180 bytes
- RX queue: `config->rx_queue_size * sizeof(can_frame)` bytes
- ISR task stack: `config->isr_task_stack_size` bytes

**Failure Modes**:
- Returns without error if initialization fails
- Check `isInitialized()` after construction
- Logs detailed errors via ESP_LOGE

**Example**:
```cpp
mcp2515_esp32_config_t config = {
  .spi_host = SPI2_HOST,
  .spi_clock_speed = 10000000,
  .pins = {
    .miso = GPIO_NUM_19,
    .mosi = GPIO_NUM_23,
    .sclk = GPIO_NUM_18,
    .cs = GPIO_NUM_5,
    .irq = GPIO_NUM_4
  },
  .use_interrupts = true,
  .use_mutex = true,
  .rx_queue_size = 64,            // Larger queue
  .isr_task_priority = 10,        // Higher priority
  .isr_task_stack_size = 8192     // Larger stack
};

MCP2515 can(&config);

if (!can.isInitialized()) {
  // Handle initialization failure
}
```

**Thread Safety**: Thread-safe if `config->use_mutex = true`

**ISR Safety**: Not ISR-safe

---

#### Destructor

```cpp
~MCP2515()
```

**Purpose**: Clean up resources and stop ISR task.

**Behavior**:
- ESP32: Signals ISR task to stop, waits up to 100ms for graceful exit
- ESP32: Removes GPIO interrupt handler
- ESP32: Deletes mutex, semaphore, and RX queue
- ESP32: Removes SPI device (native ESP-IDF only)
- All platforms: Automatic cleanup on object destruction

**Timing**:
- Typical: 10-20ms (ISR task exits quickly)
- Maximum: 110ms (100ms wait + 10ms forced cleanup)

**Thread Safety**: Not thread-safe during destruction (ensure no other threads access object)

**ISR Safety**: Not ISR-safe

**Example**:
```cpp
void task(void* param) {
  MCP2515* can = new MCP2515(GPIO_NUM_5, GPIO_NUM_4);

  // Use CAN...

  delete can;  // Destructor cleans up
}
```

---

### Initialization and Configuration

#### reset()

```cpp
ERROR reset(void)
```

**Purpose**: Software reset of MCP2515 and initialize to default state.

**Return Value**:
- `ERROR_OK` (0): Reset successful
- `ERROR_FAIL` (1): Reset failed or timeout waiting for mode change
- `ERROR_MUTEX` (7): Failed to acquire mutex (ESP32 only)

**Side Effects**:
- Sends RESET instruction via SPI
- Clears all TX/RX buffers
- Resets all registers to power-on defaults
- Configures RX buffers for rollover (RXB0 → RXB1)
- Sets default filters (accept all frames)
- Sets default masks (no filtering)
- Enables RX interrupts (RX0IF, RX1IF, ERRIF, MERRF)

**Timing**:
- Blocks for 10ms (hardware reset delay)
- Additional 5-15ms for register configuration
- **Total**: 15-25ms typical

**Calling Constraints**:
- **MUST** be called before `setBitrate()` or mode setting
- Safe to call multiple times
- Call from: `setup()` or main task
- **NOT** from ISR (blocking operation)

**Hardware State After Reset**:
- Mode: Configuration mode
- Filters: Accept all (masks = 0)
- TX buffers: Empty, ready
- RX buffers: Empty
- Error counters: Reset to 0

**Example**:
```cpp
void setup() {
  MCP2515 can(10);

  if (can.reset() != MCP2515::ERROR_OK) {
    Serial.println("MCP2515 reset failed!");
    Serial.println("Check: SPI wiring, power, oscillator");
    while(1);  // Halt
  }

  // Continue with initialization...
}
```

**Thread Safety**: Thread-safe on ESP32 (uses mutex)

**ISR Safety**: NOT ISR-safe (blocking I/O)

---

#### setBitrate() - Single Parameter

```cpp
ERROR setBitrate(const CAN_SPEED canSpeed)
```

**Purpose**: Configure CAN bus bitrate with default 16 MHz oscillator.

**Parameters**:
- `canSpeed` (CAN_SPEED): Desired CAN bus speed
  - See [CAN_SPEED enum](#can_speed) for valid values
  - Range: `CAN_5KBPS` to `CAN_1000KBPS`

**Return Value**:
- `ERROR_OK` (0): Bitrate configured successfully
- `ERROR_FAIL` (1): Invalid speed or configuration failed
- `ERROR_MUTEX` (7): Mutex acquisition failed (ESP32)

**Assumptions**:
- MCP2515 oscillator frequency: **16 MHz** (most common)
- If using 8 MHz or 20 MHz, use two-parameter version

**Example**:
```cpp
// Assumes 16 MHz oscillator
can.setBitrate(CAN_500KBPS);  // 500 kbps
can.setBitrate(CAN_125KBPS);  // 125 kbps (common default)
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

#### setBitrate() - Two Parameters

```cpp
ERROR setBitrate(const CAN_SPEED canSpeed, const CAN_CLOCK canClock)
```

**Purpose**: Configure CAN bus bitrate with explicit oscillator frequency.

**Parameters**:
- `canSpeed` (CAN_SPEED): Desired CAN bus speed
  - See [CAN_SPEED enum](#can_speed) for all values

- `canClock` (CAN_CLOCK): MCP2515 oscillator frequency
  - `MCP_8MHZ`: 8 MHz crystal
  - `MCP_16MHZ`: 16 MHz crystal (most common)
  - `MCP_20MHZ`: 20 MHz crystal

**Return Value**:
- `ERROR_OK` (0): Success
- `ERROR_FAIL` (1): Unsupported speed/clock combination or config failed
- `ERROR_MUTEX` (7): Mutex failure (ESP32)

**Supported Combinations**:

| Clock | Supported Speeds |
|-------|-----------------|
| 8 MHz | 5k, 10k, 20k, 31.25k, 33k, 40k, 50k, 80k, 100k, 125k, 200k, 250k, 500k, 1000k |
| 16 MHz | 5k, 10k, 20k, 33k, 40k, 50k, 80k, 83.3k, 95k, 100k, 125k, 200k, 250k, 500k, 1000k |
| 20 MHz | 33k, 40k, 50k, 80k, 83.3k, 100k, 125k, 200k, 250k, 500k, 1000k |

**Note**: Not all combinations are supported. Check datasheet for timing calculations.

**Side Effects**:
- Temporarily switches to Configuration mode
- Updates CNF1, CNF2, CNF3 registers
- Returns to original mode after configuration

**Timing**:
- Typical: 1-2ms (mode switches)
- Maximum: 5ms

**Example**:
```cpp
// 8 MHz oscillator, 125 kbps
can.setBitrate(CAN_125KBPS, MCP_8MHZ);

// 20 MHz oscillator, 1 Mbps
can.setBitrate(CAN_1000KBPS, MCP_20MHZ);

// 16 MHz oscillator (explicit)
can.setBitrate(CAN_500KBPS, MCP_16MHZ);
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

**Common Mistakes**:
- Using wrong oscillator frequency (causes communication failure)
- Setting bitrate without calling `reset()` first
- Mismatched bitrates between CAN nodes

---

#### setNormalMode()

```cpp
ERROR setNormalMode()
```

**Purpose**: Set MCP2515 to Normal operating mode for full CAN operation.

**Return Value**:
- `ERROR_OK` (0): Mode change successful
- `ERROR_FAIL` (1): Timeout waiting for mode change
- `ERROR_MUTEX` (7): Mutex failure (ESP32)

**Mode Behavior**:
- **Transmit**: Sends messages with automatic retransmission
- **Receive**: Receives all messages passing filters
- **Acknowledge**: Sends ACK for valid frames
- **Error Detection**: Active error detection and recovery

**Timing**:
- Typical: <1ms
- Maximum: 10ms (timeout)

**Prerequisites**:
- `reset()` must have been called
- `setBitrate()` must have been called
- Filters/masks configured (optional)

**Example**:
```cpp
void setup() {
  can.reset();
  can.setBitrate(CAN_500KBPS);

  if (can.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("Failed to set normal mode!");
  }
}
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe (blocking)

---

#### setNormalOneShotMode()

```cpp
ERROR setNormalOneShotMode()
```

**Purpose**: Set Normal mode with One-Shot transmission (no automatic retransmission).

**Return Value**: Same as `setNormalMode()`

**Mode Behavior**:
- Same as Normal mode, except:
- **Transmit**: No automatic retransmission on failure
- **Use Case**: Time-critical data where old data is useless

**Example**:
```cpp
// Real-time sensor data (don't retry old values)
can.setNormalOneShotMode();
can.sendMessage(&sensorFrame);  // Sends once, no retries
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

#### setListenOnlyMode()

```cpp
ERROR setListenOnlyMode()
```

**Purpose**: Set Listen-Only mode for passive monitoring (no ACKs sent).

**Return Value**: Same as `setNormalMode()`

**Mode Behavior**:
- **Transmit**: Disabled (cannot send)
- **Receive**: Receives all messages (ignores filters)
- **Acknowledge**: Does NOT send ACK
- **Error Detection**: Passive (does not affect bus)
- **Use Cases**: Bus monitoring, sniffing, diagnostics

**Example**:
```cpp
// CAN bus analyzer/sniffer
can.setListenOnlyMode();

void loop() {
  struct can_frame frame;
  if (can.readMessage(&frame) == MCP2515::ERROR_OK) {
    logFrame(&frame);  // Log all traffic
  }
}
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

#### setLoopbackMode()

```cpp
ERROR setLoopbackMode()
```

**Purpose**: Set Loopback mode for internal testing (no CAN bus interaction).

**Return Value**: Same as `setNormalMode()`

**Mode Behavior**:
- **Transmit**: Messages looped back internally (no actual CAN bus transmission)
- **Receive**: Only receives own transmitted messages
- **Acknowledgment**: ACK bit is ignored (self-acknowledged)
- **Bus**: Completely isolated from CAN bus (TXCAN pin remains recessive)
- **Arbitration**: No bus arbitration occurs (internal loopback)
- **Use Cases**: Self-test, library validation, no CAN transceiver needed

**Known Hardware Limitations**:
- **ABAT Auto-Clear Issue**: In loopback mode, the MCP2515 may not auto-clear the ABAT (Abort All) bit when `abortAllTransmissions()` is called. This is because no actual bus arbitration occurs. The library compensates by manually clearing ABAT if it doesn't auto-clear within 10ms.
- **Filters Still Apply**: Acceptance filters and masks are still evaluated even in loopback mode. Set masks to 0x000 to accept all frames during testing.

**Example**:
```cpp
// Self-test without CAN transceiver
can.setLoopbackMode();

// Configure to accept all frames (recommended for loopback testing)
can.setFilterMask(MCP2515::MASK0, false, 0x000);
can.setFilterMask(MCP2515::MASK1, false, 0x000);

struct can_frame txFrame, rxFrame;
txFrame.can_id = 0x123;
txFrame.can_dlc = 8;
memset(txFrame.data, 0xAA, 8);

can.sendMessage(&txFrame);
delay(10);  // Allow time for internal loopback

if (can.readMessage(&rxFrame) == MCP2515::ERROR_OK) {
  if (rxFrame.can_id == 0x123 && rxFrame.can_dlc == 8) {
    Serial.println("Loopback test PASSED");
  }
}
```

**Best Practices**:
- Allow 5-10ms settle time after `setLoopbackMode()` before testing
- Disable or configure filters to accept all frames
- Use with interrupts enabled for best results on ESP32
- Avoid calling `abortAllTransmissions()` repeatedly in loopback mode

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

#### setSleepMode()

```cpp
ERROR setSleepMode()
```

**Purpose**: Set Sleep mode for low-power operation.

**Return Value**: Same as `setNormalMode()`

**Mode Behavior**:
- **Transmit**: Disabled
- **Receive**: Disabled
- **Power**: Reduced power consumption (~5µA)
- **Wake-up**: Wakes on CAN activity or SPI command
- **Use Cases**: Battery-powered nodes, periodic wake-up

**Example**:
```cpp
// Low-power periodic sensing
void loop() {
  can.setNormalMode();
  struct can_frame frame;
  frame.can_id = 0x100;
  frame.can_dlc = 4;
  memcpy(frame.data, &sensorValue, 4);
  can.sendMessage(&frame);

  can.setSleepMode();
  delay(60000);  // Sleep 1 minute
}
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

#### setConfigMode()

```cpp
ERROR setConfigMode()
```

**Purpose**: Set Configuration mode for changing settings (bitrate, filters, masks).

**Return Value**: Same as `setNormalMode()`

**Mode Behavior**:
- **Transmit**: Disabled
- **Receive**: Disabled
- **Configuration**: Allows changing protected registers
- **Auto Use**: Called internally by `setBitrate()`, `setFilter()`, `setFilterMask()`

**Manual Use**:
- Rarely needed (library handles mode switching)
- Use when making multiple configuration changes

**Example**:
```cpp
can.setConfigMode();
can.setFilter(MCP2515::RXF0, false, 0x123);
can.setFilter(MCP2515::RXF1, false, 0x456);
can.setFilterMask(MCP2515::MASK0, false, 0x7FF);
can.setNormalMode();  // Return to normal
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

#### setClkOut()

```cpp
ERROR setClkOut(const CAN_CLKOUT divisor)
```

**Purpose**: Configure CLKOUT pin behavior (oscillator output).

**Parameters**:
- `divisor` (CAN_CLKOUT): Clock output divisor
  - `CLKOUT_DISABLE` (-1): Disable clock output, enable SOF signal
  - `CLKOUT_DIV1` (0): Oscillator frequency / 1
  - `CLKOUT_DIV2` (1): Oscillator frequency / 2
  - `CLKOUT_DIV4` (2): Oscillator frequency / 4
  - `CLKOUT_DIV8` (3): Oscillator frequency / 8

**Return Value**:
- `ERROR_OK` (0): Success
- `ERROR_FAIL` (1): Configuration failed
- `ERROR_MUTEX` (7): Mutex failure (ESP32)

**Use Cases**:
- Provide clock to other ICs
- Debug timing issues
- Verify oscillator frequency

**Example**:
```cpp
// Output 8 MHz (from 16 MHz oscillator)
can.setClkOut(CLKOUT_DIV2);

// Disable CLKOUT
can.setClkOut(CLKOUT_DISABLE);
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

### Filter and Mask Configuration

#### setFilterMask()

```cpp
ERROR setFilterMask(const MASK num, const bool ext, const uint32_t ulData)
```

**Purpose**: Configure acceptance mask for selective message filtering.

**Parameters**:
- `num` (MASK): Mask number
  - `MASK0`: Applies to RXB0 (filters RXF0, RXF1)
  - `MASK1`: Applies to RXB1 (filters RXF2-RXF5)

- `ext` (bool): Frame format
  - `false`: Standard 11-bit identifier
  - `true`: Extended 29-bit identifier

- `ulData` (uint32_t): Mask value
  - Bit = 1: Must match filter
  - Bit = 0: Don't care (accept any)
  - Standard: Use bits 10-0
  - Extended: Use bits 28-0

**Return Value**:
- `ERROR_OK` (0): Mask configured
- `ERROR_FAIL` (1): Invalid mask number or config failed
- `ERROR_MUTEX` (7): Mutex failure (ESP32)

**Side Effects**:
- Temporarily enters Configuration mode
- Restores previous mode after setting

**Mask Logic**:
```
Received ID:  0x123
Filter:       0x120
Mask:         0x7F0
Result: (Received & Mask) == (Filter & Mask)
        (0x123 & 0x7F0) == (0x120 & 0x7F0)
        0x120 == 0x120  → MATCH (frame accepted)
```

**Example**:
```cpp
// Accept IDs 0x100-0x10F (standard frames)
can.setFilter(MCP2515::RXF0, false, 0x100);
can.setFilterMask(MCP2515::MASK0, false, 0x7F0);
// Mask 0x7F0 = 0b11111110000 (upper 7 bits must match)

// Accept all standard frames (no filtering)
can.setFilterMask(MCP2515::MASK0, false, 0x000);

// Accept all extended frames (no filtering)
can.setFilterMask(MCP2515::MASK1, true, 0x00000000);

// Accept specific extended ID exactly
can.setFilter(MCP2515::RXF2, true, 0x12345678);
can.setFilterMask(MCP2515::MASK1, true, 0x1FFFFFFF);
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

#### setFilter()

```cpp
ERROR setFilter(const RXF num, const bool ext, const uint32_t ulData)
```

**Purpose**: Configure acceptance filter for selective message reception.

**Parameters**:
- `num` (RXF): Filter number
  - `RXF0`, `RXF1`: Apply to RXB0 (use MASK0)
  - `RXF2`, `RXF3`, `RXF4`, `RXF5`: Apply to RXB1 (use MASK1)

- `ext` (bool): Frame format (same as `setFilterMask()`)

- `ulData` (uint32_t): Filter ID value
  - Compared against received ID using mask

**Return Value**: Same as `setFilterMask()`

**Filter Matching**:
- Frame accepted if: `(frame_id & mask) == (filter & mask)`
- Multiple filters: Frame accepted if ANY filter matches
- RXB0: Checks RXF0, RXF1
- RXB1: Checks RXF2, RXF3, RXF4, RXF5

**Example**:
```cpp
// Accept two specific standard IDs (0x100 and 0x200)
can.setFilter(MCP2515::RXF0, false, 0x100);
can.setFilter(MCP2515::RXF1, false, 0x200);
can.setFilterMask(MCP2515::MASK0, false, 0x7FF);  // Exact match

// Accept range of extended IDs (0x1000-0x1FFF)
can.setFilter(MCP2515::RXF2, true, 0x1000);
can.setFilterMask(MCP2515::MASK1, true, 0x1FFFF000);

// Mixed standard/extended (advanced)
can.setFilter(MCP2515::RXF0, false, 0x123);  // Standard
can.setFilter(MCP2515::RXF1, true, 0x12345);  // Extended
// Note: Both use MASK0, so mask must be compatible
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

### Transmission Methods

#### sendMessage() - Specific Buffer

```cpp
ERROR sendMessage(const TXBn txbn, const struct can_frame *frame)
```

**Purpose**: Send CAN frame using specific transmit buffer.

**Parameters**:
- `txbn` (TXBn): Transmit buffer number
  - `TXB0`: First buffer (default priority 3)
  - `TXB1`: Second buffer (default priority 3)
  - `TXB2`: Third buffer (default priority 3)

- `frame` (can_frame*): Pointer to CAN frame structure
  - Must not be `nullptr`
  - `frame->can_dlc` must be 0-8
  - Frame content is copied (safe to modify after call)

**Return Value**:
- `ERROR_OK` (0): Frame queued for transmission
- `ERROR_FAILTX` (4): Transmission failed
  - Buffer already in use (TXREQ bit set)
  - Invalid DLC (> 8)
  - Null frame pointer
  - Hardware error (ABTF, MLOA, TXERR flags)
- `ERROR_MUTEX` (7): Mutex failure (ESP32)

**Transmission Timing**:
- Function returns immediately (non-blocking)
- Actual transmission happens asynchronously
- Check completion with `getInterrupts()` (TXnIF flags)

**Error Flags After Send**:
- `TXB_ABTF`: Message was aborted
- `TXB_MLOA`: Message lost arbitration
- `TXB_TXERR`: Transmission error

**Example**:
```cpp
// Send high-priority message on TXB0
can.setTransmitPriority(MCP2515::TXB0, 3);  // Highest

struct can_frame frame;
frame.can_id = 0x123;
frame.can_dlc = 8;
memset(frame.data, 0xAA, 8);

MCP2515::ERROR result = can.sendMessage(MCP2515::TXB0, &frame);
if (result != MCP2515::ERROR_OK) {
  Serial.println("Send failed!");
}

// Wait for transmission complete
while (!(can.getInterrupts() & MCP2515::CANINTF_TX0IF)) {
  delay(1);
}
can.clearTXInterrupts();
```

**Thread Safety**: Thread-safe on ESP32 (mutex protected)

**ISR Safety**: NOT ISR-safe (uses SPI, mutex)

**Performance**:
- SPI transfer: ~50-100µs (10 MHz SPI)
- Actual CAN transmission: Depends on bitrate and arbitration

---

#### sendMessage() - Auto Buffer Selection

```cpp
ERROR sendMessage(const struct can_frame *frame)
```

**Purpose**: Send CAN frame using first available transmit buffer.

**Parameters**:
- `frame` (can_frame*): Pointer to CAN frame (same as above)

**Return Value**:
- `ERROR_OK` (0): Frame queued on available buffer
- `ERROR_ALLTXBUSY` (2): All 3 TX buffers are busy
- `ERROR_FAILTX` (4): Transmission setup failed
- `ERROR_MUTEX` (7): Mutex failure (ESP32)

**Buffer Selection Logic**:
1. Check TXB0: If free, use it
2. Check TXB1: If free, use it
3. Check TXB2: If free, use it
4. If all busy, return `ERROR_ALLTXBUSY`

**Priority Handling**:
- If multiple buffers pending, hardware sends highest priority first
- If equal priority, highest buffer number wins (TXB2 > TXB1 > TXB0)

**Example**:
```cpp
struct can_frame frame;
frame.can_id = 0x456;
frame.can_dlc = 4;
memcpy(frame.data, "TEST", 4);

MCP2515::ERROR result = can.sendMessage(&frame);
switch (result) {
  case MCP2515::ERROR_OK:
    Serial.println("Message sent");
    break;
  case MCP2515::ERROR_ALLTXBUSY:
    Serial.println("All TX buffers busy");
    break;
  case MCP2515::ERROR_FAILTX:
    Serial.println("Transmission error");
    break;
}
```

**Thread Safety**: Thread-safe on ESP32 (entire operation is mutex-protected)

**ISR Safety**: NOT ISR-safe

**When to Use**:
- Most applications (simple, automatic)
- When priority doesn't matter
- When buffer selection doesn't matter

**When to Use Specific Buffer**:
- Need explicit priority control
- Need to track which buffer was used
- Implementing round-robin or custom scheduling

---

#### setTransmitPriority()

```cpp
ERROR setTransmitPriority(const TXBn txbn, const uint8_t priority)
```

**Purpose**: Set transmission priority for a specific TX buffer.

**Parameters**:
- `txbn` (TXBn): Buffer number (`TXB0`, `TXB1`, `TXB2`)

- `priority` (uint8_t): Priority level
  - Valid range: 0-3
  - `0`: Lowest priority
  - `1`: Low priority
  - `2`: High priority
  - `3`: Highest priority (default)

**Return Value**:
- `ERROR_OK` (0): Priority set
- `ERROR_FAIL` (1): Invalid priority (> 3) or buffer is transmitting
- `ERROR_MUTEX` (7): Mutex failure (ESP32)

**Priority Arbitration**:
- Higher priority messages sent first
- If equal priority: TXB2 > TXB1 > TXB0
- CAN bus arbitration is separate (based on CAN ID)

**Timing Constraints**:
- Must call BEFORE `sendMessage()` on that buffer
- Cannot change priority while transmission pending (TXREQ = 1)

**Example**:
```cpp
// High-priority emergency message
can.setTransmitPriority(MCP2515::TXB0, 3);
emergencyFrame.can_id = 0x001;
can.sendMessage(MCP2515::TXB0, &emergencyFrame);

// Low-priority status message
can.setTransmitPriority(MCP2515::TXB1, 0);
statusFrame.can_id = 0x7FF;
can.sendMessage(MCP2515::TXB1, &statusFrame);

// Even if status was queued first, emergency sends first
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

#### abortTransmission()

```cpp
ERROR abortTransmission(const TXBn txbn)
```

**Purpose**: Abort pending transmission on specific buffer.

**Parameters**:
- `txbn` (TXBn): Buffer to abort (`TXB0`, `TXB1`, `TXB2`)

**Return Value**:
- `ERROR_OK` (0): Transmission aborted
- `ERROR_FAIL` (1): Abort failed
- `ERROR_MUTEX` (7): Mutex failure (ESP32)

**Behavior**:
- Clears TXREQ bit to abort
- Clears ABTF flag if set
- Safe to call even if not transmitting

**Use Cases**:
- Cancel outdated data
- Error recovery
- Re-prioritization

**Example**:
```cpp
// Send sensor data
can.sendMessage(MCP2515::TXB0, &oldData);

// New data available - cancel old transmission
can.abortTransmission(MCP2515::TXB0);
can.sendMessage(MCP2515::TXB0, &newData);
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

#### abortAllTransmissions()

```cpp
ERROR abortAllTransmissions(void)
```

**Purpose**: Abort all pending transmissions on all 3 buffers.

**Return Value**: Same as `abortTransmission()`

**Behavior**:
- Sets ABAT bit in CANCTRL register
- Hardware aborts all pending TX
- Waits up to 10ms for ABAT to auto-clear
- **If ABAT doesn't clear** (loopback mode edge case), manually clears it
- Clears all TXREQ bits to free buffers
- ABTF flags auto-clear when ABAT is cleared

**Loopback Mode Edge Case**:
In loopback mode, the MCP2515 hardware may not auto-clear the ABAT bit because no actual bus arbitration occurs. The library detects this condition and manually clears ABAT to prevent all subsequent transmissions from being instantly aborted. This was a critical bug fixed in version 2.1.1.

**Example**:
```cpp
// Emergency stop - cancel all pending messages
can.abortAllTransmissions();

// Safe to send new messages immediately after
can.sendMessage(&newFrame);  // Will work correctly
```

**Critical Notes**:
- Prior to v2.1.1, ABAT could remain stuck ON in loopback mode
- This caused 100% transmission failure after calling this function
- Current implementation verifies ABAT cleared and manually clears if needed

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

### Reception Methods

#### readMessage() - Specific Buffer

```cpp
ERROR readMessage(const RXBn rxbn, struct can_frame *frame)
```

**Purpose**: Read CAN frame from specific receive buffer.

**Parameters**:
- `rxbn` (RXBn): Receive buffer number
  - `RXB0`: High-priority buffer (receives first)
  - `RXB1`: Low-priority buffer (rollover from RXB0)

- `frame` (can_frame*): Pointer to frame structure to fill
  - Must not be `nullptr`
  - Contents overwritten on success

**Return Value**:
- `ERROR_OK` (0): Frame read successfully
- `ERROR_FAIL` (1): Read failed or null pointer
- `ERROR_MUTEX` (7): Mutex failure (ESP32)

**Side Effects**:
- Clears RXnIF interrupt flag
- Frees buffer for next message

**Timing**:
- SPI read: ~40-80µs (10 MHz SPI)
- Uses optimized READ RX BUFFER instruction (faster than generic READ)

**Example**:
```cpp
// Check for message in RXB0
uint8_t irq = can.getInterrupts();
if (irq & MCP2515::CANINTF_RX0IF) {
  struct can_frame frame;
  if (can.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) {
    Serial.printf("RXB0: ID=0x%03X, DLC=%d\n",
                  frame.can_id & CAN_SFF_MASK, frame.can_dlc);
  }
}
```

**Thread Safety**: Thread-safe on ESP32 (uses mutex, marked IRAM_ATTR)

**ISR Safety**: NOT ISR-safe (uses mutex)

**IRAM Placement**: Function is in IRAM (flash-safe during cache operations)

---

#### readMessage() - Auto Buffer Selection

```cpp
ERROR readMessage(struct can_frame *frame)
```

**Purpose**: Read CAN frame from any available receive buffer.

**Parameters**:
- `frame` (can_frame*): Pointer to frame structure

**Return Value**:
- `ERROR_OK` (0): Frame read
- `ERROR_NOMSG` (5): No message available
- `ERROR_FAIL` (1): Read failed
- `ERROR_MUTEX` (7): Mutex failure (ESP32)

**Buffer Priority**:
1. Check RXB0 first (higher priority)
2. If empty, check RXB1
3. If both empty, return `ERROR_NOMSG`

**Example**:
```cpp
void loop() {
  struct can_frame frame;

  // Polling mode
  if (can.readMessage(&frame) == MCP2515::ERROR_OK) {
    processFrame(&frame);
  }

  delay(10);
}
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

**IRAM Placement**: Yes

---

#### readMessageQueued() (ESP32 Only)

```cpp
ERROR readMessageQueued(struct can_frame *frame, uint32_t timeout_ms = 0)
```

**Purpose**: Read CAN frame from FreeRTOS queue (interrupt mode).

**Parameters**:
- `frame` (can_frame*): Pointer to frame structure

- `timeout_ms` (uint32_t): Timeout in milliseconds
  - `0`: Non-blocking (return immediately)
  - `1-N`: Block up to N milliseconds
  - `portMAX_DELAY`: Block forever

**Return Value**:
- `ERROR_OK` (0): Frame read from queue
- `ERROR_NOMSG` (5): No message (timeout_ms = 0)
- `ERROR_TIMEOUT` (6): Timeout expired
- `ERROR_FAIL` (1): Null pointer

**Prerequisites**:
- Interrupts must be enabled (constructor with `int_pin`)
- If interrupts disabled, falls back to polling `readMessage()`

**Queue Behavior**:
- FIFO order (first received, first read)
- ISR task fills queue on interrupt
- Application reads from queue

**Example**:
```cpp
// Non-blocking check
struct can_frame frame;
if (can.readMessageQueued(&frame, 0) == MCP2515::ERROR_OK) {
  // Frame available
}

// Blocking with timeout (100ms)
MCP2515::ERROR result = can.readMessageQueued(&frame, 100);
if (result == MCP2515::ERROR_OK) {
  // Frame received
} else if (result == MCP2515::ERROR_TIMEOUT) {
  // No frame within 100ms
}

// Block forever (wait for frame)
can.readMessageQueued(&frame, portMAX_DELAY);
```

**Thread Safety**: Thread-safe (uses FreeRTOS queue)

**ISR Safety**: NOT ISR-safe (application should not call from ISR)

**When to Use**:
- Interrupt-driven reception (most efficient)
- Avoids polling overhead
- Task blocks until data available

---

#### getFilterHit()

```cpp
uint8_t getFilterHit(const RXBn rxbn)
```

**Purpose**: Determine which acceptance filter matched received frame.

**Parameters**:
- `rxbn` (RXBn): Buffer to check (`RXB0` or `RXB1`)

**Return Value**:
- RXB0: 0-3
  - `0`, `1`: Matched filter RXF0 or RXF1
  - `2`, `3`: Rollover from RXB1 (filters RXF2-RXF5)

- RXB1: 0-7 (typically 3-5)
  - `0-2`: Reserved
  - `3`: Matched RXF3
  - `4`: Matched RXF4
  - `5`: Matched RXF5
  - `6`, `7`: Reserved

**Use Cases**:
- Route messages based on filter
- Debug filter configuration
- Statistics tracking

**Example**:
```cpp
struct can_frame frame;
if (can.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) {
  uint8_t filter = can.getFilterHit(MCP2515::RXB0);

  switch (filter) {
    case 0:  // Matched RXF0
      handleSensorData(&frame);
      break;
    case 1:  // Matched RXF1
      handleCommandData(&frame);
      break;
  }
}
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: Conditional (read-only register access)

---

#### checkReceive()

```cpp
bool checkReceive(void)
```

**Purpose**: Quick check if any receive buffer has data (non-blocking).

**Return Value**:
- `true`: At least one buffer/queue has message
- `false`: No messages waiting

**Platform-Specific Behavior**:

**ESP32 with Interrupts Enabled**:
1. **First** checks FreeRTOS RX queue
2. If queue has frames, returns `true`
3. If queue empty, falls through to hardware check
4. Reads hardware status register
5. Returns `true` if RX0IF or RX1IF set

**ESP32 with Interrupts Disabled** or **Arduino AVR**:
- Reads hardware status register only
- Returns `true` if RX0IF or RX1IF set

**Critical Implementation Detail**:
On ESP32 with interrupts enabled, this function checks the queue **first** before checking hardware. This is essential because the ISR task consumes frames from hardware and places them in the queue. Checking hardware first would cause missed frames. This behavior was fixed in v2.1.1 to prevent race conditions.

**Example**:
```cpp
// Polling loop (works in all modes)
if (can.checkReceive()) {
  struct can_frame frame;
  can.readMessage(&frame);  // Or readMessageQueued() on ESP32
}

// Interrupt mode (ESP32) - queue-aware
if (can.checkReceive()) {
  // Queue has frames
  struct can_frame frame;
  can.readMessageQueued(&frame, 0);
}
```

**Performance**:
- Queue check: ~1-2µs (ESP32)
- Status register read: ~5-10µs (SPI transaction)

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: Conditional (fast operation)

---

#### getRxQueueCount() (ESP32 Only)

```cpp
uint32_t getRxQueueCount(void)
```

**Purpose**: Get number of frames waiting in RX queue.

**Return Value**:
- `0`: Queue empty
- `1-N`: Number of frames in queue
- `0`: If interrupts not enabled

**Example**:
```cpp
uint32_t count = can.getRxQueueCount();
Serial.printf("%d frames in queue\n", count);

if (count > 30) {
  Serial.println("WARNING: Queue nearly full!");
}
```

**Thread Safety**: Thread-safe (FreeRTOS queue API)

**ISR Safety**: NOT ISR-safe

---

### Status and Diagnostics

#### getStatus()

```cpp
uint8_t getStatus(void)
```

**Purpose**: Read MCP2515 quick status register.

**Return Value**: 8-bit status register
- Bit 0 (0x01): `STAT_RX0IF` - Message in RXB0
- Bit 1 (0x02): `STAT_RX1IF` - Message in RXB1
- Bits 2-7: TX buffer status, interrupt flags

**Timing**: Fast (single SPI transaction, ~5µs)

**Example**:
```cpp
uint8_t status = can.getStatus();
if (status & MCP2515::STAT_RX0IF) {
  Serial.println("RX0 has message");
}
if (status & MCP2515::STAT_RX1IF) {
  Serial.println("RX1 has message");
}
```

**Thread Safety**: Thread-safe on ESP32 (marked IRAM_ATTR)

**ISR Safety**: Conditional (if mutex-protected path is avoided)

---

#### getInterrupts()

```cpp
uint8_t getInterrupts(void)
```

**Purpose**: Read interrupt flags register (CANINTF).

**Return Value**: 8-bit interrupt flags
- Bit 0 (0x01): `CANINTF_RX0IF` - RXB0 full
- Bit 1 (0x02): `CANINTF_RX1IF` - RXB1 full
- Bit 2 (0x04): `CANINTF_TX0IF` - TXB0 empty
- Bit 3 (0x08): `CANINTF_TX1IF` - TXB1 empty
- Bit 4 (0x10): `CANINTF_TX2IF` - TXB2 empty
- Bit 5 (0x20): `CANINTF_ERRIF` - Error interrupt
- Bit 6 (0x40): `CANINTF_WAKIF` - Wake-up interrupt
- Bit 7 (0x80): `CANINTF_MERRF` - Message error

**Example**:
```cpp
uint8_t irq = can.getInterrupts();

if (irq & MCP2515::CANINTF_RX0IF) {
  // RX0 interrupt
}
if (irq & MCP2515::CANINTF_ERRIF) {
  Serial.println("CAN error!");
  uint8_t errors = can.getErrorFlags();
  Serial.printf("Error flags: 0x%02X\n", errors);
}
if (irq & (MCP2515::CANINTF_TX0IF |
           MCP2515::CANINTF_TX1IF |
           MCP2515::CANINTF_TX2IF)) {
  Serial.println("TX complete");
  can.clearTXInterrupts();
}
```

**Thread Safety**: Thread-safe on ESP32 (marked IRAM_ATTR)

**ISR Safety**: Conditional (fast register read)

---

#### getInterruptMask()

```cpp
uint8_t getInterruptMask(void)
```

**Purpose**: Read interrupt enable mask (CANINTE).

**Return Value**: 8-bit mask (same flags as `getInterrupts()`)
- Bit = 1: Interrupt enabled
- Bit = 0: Interrupt disabled

**Example**:
```cpp
uint8_t mask = can.getInterruptMask();
Serial.printf("Enabled interrupts: 0x%02X\n", mask);
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: Conditional

---

#### getErrorFlags()

```cpp
uint8_t getErrorFlags(void)
```

**Purpose**: Read error flags register (EFLG).

**Return Value**: 8-bit error flags
- Bit 0 (0x01): `EFLG_EWARN` - Error warning (TEC/REC ≥ 96)
- Bit 1 (0x02): `EFLG_RXWAR` - RX error warning
- Bit 2 (0x04): `EFLG_TXWAR` - TX error warning
- Bit 3 (0x08): `EFLG_RXEP` - RX error passive (REC ≥ 128)
- Bit 4 (0x10): `EFLG_TXEP` - TX error passive (TEC ≥ 128)
- Bit 5 (0x20): `EFLG_TXBO` - Bus-off (TEC ≥ 256)
- Bit 6 (0x40): `EFLG_RX0OVR` - RXB0 overflow
- Bit 7 (0x80): `EFLG_RX1OVR` - RXB1 overflow

**Example**:
```cpp
uint8_t eflg = can.getErrorFlags();

if (eflg & MCP2515::EFLG_TXBO) {
  Serial.println("CRITICAL: Bus-off!");
  can.performErrorRecovery();  // ESP32 only
}
if (eflg & MCP2515::EFLG_RX0OVR) {
  Serial.println("RX0 overflow - messages lost");
  can.clearRXnOVR();
}
```

**Thread Safety**: Thread-safe on ESP32 (marked IRAM_ATTR)

**ISR Safety**: Conditional

---

#### checkError()

```cpp
bool checkError(void)
```

**Purpose**: Quick check if any error condition exists.

**Return Value**:
- `true`: Error detected
- `false`: No errors

**Implementation**: Reads `getErrorFlags()` and masks error bits

**Example**:
```cpp
if (can.checkError()) {
  Serial.println("CAN error detected");
  uint8_t eflg = can.getErrorFlags();
  // Analyze specific errors
}
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: Conditional

---

#### errorCountRX()

```cpp
uint8_t errorCountRX(void)
```

**Purpose**: Read receive error counter (REC).

**Return Value**:
- `0`: No RX errors
- `1-127`: Error active
- `128-255`: Error passive
- If > 96: Error warning threshold

**Example**:
```cpp
uint8_t rxErrors = can.errorCountRX();
uint8_t txErrors = can.errorCountTX();

Serial.printf("Errors - RX: %d, TX: %d\n", rxErrors, txErrors);

if (rxErrors > 96 || txErrors > 96) {
  Serial.println("WARNING: High error rate");
}
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: Conditional

---

#### errorCountTX()

```cpp
uint8_t errorCountTX(void)
```

**Purpose**: Read transmit error counter (TEC).

**Return Value**: Same interpretation as `errorCountRX()`

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: Conditional

---

### Interrupt Management

#### clearInterrupts()

```cpp
void clearInterrupts(void)
```

**Purpose**: Clear all interrupt flags (CANINTF = 0).

**Side Effects**: Clears ALL interrupts (RX, TX, error)

**Example**:
```cpp
can.clearInterrupts();  // Reset all flags
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

#### clearTXInterrupts()

```cpp
void clearTXInterrupts(void)
```

**Purpose**: Clear only TX complete interrupt flags.

**Flags Cleared**:
- `CANINTF_TX0IF`
- `CANINTF_TX1IF`
- `CANINTF_TX2IF`

**Example**:
```cpp
// After transmission complete
if (can.getInterrupts() & MCP2515::CANINTF_TX0IF) {
  can.clearTXInterrupts();
}
```

**Thread Safety**: Thread-safe on ESP32 (marked IRAM_ATTR)

**ISR Safety**: Conditional

---

#### clearRXnOVR()

```cpp
void clearRXnOVR(void)
```

**Purpose**: Clear RX buffer overflow flags and error interrupt.

**Example**:
```cpp
if (can.getErrorFlags() & MCP2515::EFLG_RX0OVR) {
  Serial.println("RX overflow!");
  can.clearRXnOVR();
}
```

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

#### clearRXnOVRFlags()

```cpp
void clearRXnOVRFlags(void)
```

**Purpose**: Clear only overflow flags (not interrupt).

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

#### clearMERR()

```cpp
void clearMERR()
```

**Purpose**: Clear message error interrupt flag.

**Thread Safety**: Thread-safe on ESP32

**ISR Safety**: NOT ISR-safe

---

#### clearERRIF()

```cpp
void clearERRIF()
```

**Purpose**: Clear error interrupt flag.

**Thread Safety**: Thread-safe on ESP32 (marked IRAM_ATTR)

**ISR Safety**: Conditional

---

### ESP32-Specific Methods

#### getStatistics() (ESP32 Only)

```cpp
void getStatistics(mcp2515_statistics_t* stats)
```

**Purpose**: Get CAN frame statistics (interrupt mode).

**Parameters**:
- `stats` (mcp2515_statistics_t*): Pointer to statistics structure

**Statistics Structure**:
```cpp
struct mcp2515_statistics_t {
  uint32_t rx_frames;       // Total RX frames
  uint32_t tx_frames;       // Total TX frames
  uint32_t rx_errors;       // RX errors
  uint32_t tx_errors;       // TX errors
  uint32_t rx_overflow;     // RX buffer overflows
  uint32_t tx_timeouts;     // TX timeouts
  uint32_t bus_errors;      // Bus errors
  uint32_t bus_off_count;   // Bus-off events
};
```

**Example**:
```cpp
mcp2515_statistics_t stats;
can.getStatistics(&stats);

Serial.printf("RX: %u, TX: %u\n", stats.rx_frames, stats.tx_frames);
Serial.printf("Errors - RX: %u, TX: %u\n", stats.rx_errors, stats.tx_errors);
Serial.printf("Overflows: %u\n", stats.rx_overflow);

// Calculate success rate
float rx_success_rate = 100.0 * stats.rx_frames /
                        (stats.rx_frames + stats.rx_errors);
Serial.printf("RX success: %.2f%%\n", rx_success_rate);
```

**Thread Safety**: Thread-safe (uses spinlock)

**ISR Safety**: NOT ISR-safe (use from tasks only)

---

#### resetStatistics() (ESP32 Only)

```cpp
void resetStatistics(void)
```

**Purpose**: Reset all statistics counters to zero.

**Example**:
```cpp
can.resetStatistics();
Serial.println("Statistics reset");
```

**Thread Safety**: Thread-safe

**ISR Safety**: NOT ISR-safe

---

#### isInitialized() (ESP32 Only)

```cpp
bool isInitialized(void)
```

**Purpose**: Check if library initialization succeeded.

**Return Value**:
- `true`: Initialization successful
- `false`: Initialization failed

**Example**:
```cpp
MCP2515 can(GPIO_NUM_5, GPIO_NUM_4);

if (!can.isInitialized()) {
  Serial.println("MCP2515 init failed!");
  Serial.println("Check: SPI pins, power, oscillator");
  return;
}
```

**Thread Safety**: Thread-safe (read-only)

**ISR Safety**: ISR-safe

---

#### setInterruptMode() (ESP32 Only)

```cpp
ERROR setInterruptMode(bool enable)
```

**Purpose**: Enable or disable interrupt-driven reception at runtime.

**Parameters**:
- `enable` (bool):
  - `true`: Enable interrupts and ISR processing
  - `false`: Disable interrupts (pure polling mode)

**Return Value**: `ERROR_OK` always

**Prerequisites**: Interrupt pin must be configured in constructor

**Behavior**:
- **When `enable = true`**:
  - Enables GPIO interrupt on INT pin
  - ISR task processes interrupts and queues frames
  - Application reads from queue via `readMessageQueued()`
  - Hardware buffers managed by ISR task

- **When `enable = false`**:
  - Disables GPIO interrupt
  - **ISR task stops processing** (does not consume frames)
  - Application must poll with `readMessage()` or `checkReceive()`
  - Hardware buffers managed by application

**Critical Implementation Detail**:
The ISR task checks the `use_interrupts` flag before processing. This prevents the ISR task from consuming frames when interrupts are disabled, allowing true polling mode operation. Prior to v2.1.1, the ISR task would continue processing on timeout even when interrupts were "disabled", causing race conditions.

**Example**:
```cpp
// Temporarily disable interrupts for diagnostics
can.setInterruptMode(false);

// Now safe to poll hardware directly
struct can_frame frame;
if (can.readMessage(&frame) == MCP2515::ERROR_OK) {
  // Frame read directly from hardware
}

// Re-enable interrupt mode
can.setInterruptMode(true);

// Now read from queue
can.readMessageQueued(&frame, 100);
```

**Use Cases**:
- Disable for polling-based diagnostics
- Disable for time-critical direct hardware access
- Enable for normal operation (most efficient)

**Thread Safety**: Thread-safe

**ISR Safety**: NOT ISR-safe

---

#### performErrorRecovery() (ESP32 Only)

```cpp
ERROR performErrorRecovery(void)
```

**Purpose**: Perform automatic bus-off recovery.

**Return Value**:
- `ERROR_OK` (0): Recovery successful
- `ERROR_FAIL` (1): Recovery failed

**Behavior**:
- Logs error counts
- Clears error flags
- If bus-off (TXBO): Resets controller and restores mode
- Automatic if `MCP2515_AUTO_BUS_OFF_RECOVERY` enabled

**Example**:
```cpp
if (can.getErrorFlags() & MCP2515::EFLG_TXBO) {
  Serial.println("Bus-off detected!");
  if (can.performErrorRecovery() == MCP2515::ERROR_OK) {
    Serial.println("Recovery successful");
  } else {
    Serial.println("Recovery failed - check hardware");
  }
}
```

**Thread Safety**: Thread-safe

**ISR Safety**: NOT ISR-safe (blocking)

---

#### getBusStatus() (ESP32 Only)

```cpp
uint8_t getBusStatus(void)
```

**Purpose**: Get detailed bus status (CANSTAT register).

**Return Value**: CANSTAT register value
- Bits 7-5: Operating mode
- Bits 3-1: Interrupt code

**Example**:
```cpp
uint8_t status = can.getBusStatus();
uint8_t mode = (status >> 5) & 0x07;

switch (mode) {
  case 0: Serial.println("Mode: Normal"); break;
  case 1: Serial.println("Mode: Sleep"); break;
  case 2: Serial.println("Mode: Loopback"); break;
  case 3: Serial.println("Mode: Listen-Only"); break;
  case 4: Serial.println("Mode: Configuration"); break;
}
```

**Thread Safety**: Thread-safe

**ISR Safety**: Conditional

---

## Configuration Structures (ESP32)

### mcp2515_esp32_pins_t

```cpp
struct mcp2515_esp32_pins_t {
  gpio_num_t miso;  // SPI MISO pin
  gpio_num_t mosi;  // SPI MOSI pin
  gpio_num_t sclk;  // SPI SCLK pin
  gpio_num_t cs;    // SPI Chip Select
  gpio_num_t irq;   // Interrupt pin (GPIO_NUM_NC to disable)
};
```

**Default Pins by ESP32 Variant**:

| Variant | MISO | MOSI | SCLK | CS | INT |
|---------|------|------|------|----|----|
| ESP32 Classic | GPIO 19 | GPIO 23 | GPIO 18 | GPIO 5 | GPIO 4 |
| ESP32-S2 | GPIO 37 | GPIO 35 | GPIO 36 | GPIO 34 | GPIO 33 |
| ESP32-S3 | GPIO 13 | GPIO 11 | GPIO 12 | GPIO 10 | GPIO 9 |
| ESP32-C3 | GPIO 5 | GPIO 6 | GPIO 4 | GPIO 7 | GPIO 8 |
| ESP32-C6 | GPIO 20 | GPIO 19 | GPIO 18 | GPIO 22 | GPIO 21 |

---

### mcp2515_esp32_config_t

```cpp
struct mcp2515_esp32_config_t {
  spi_host_device_t spi_host;      // SPI2_HOST or SPI3_HOST
  uint32_t spi_clock_speed;        // Hz (typically 10,000,000)
  mcp2515_esp32_pins_t pins;       // GPIO configuration
  bool use_interrupts;             // Enable interrupt mode
  bool use_mutex;                  // Enable thread safety
  uint8_t rx_queue_size;           // RX queue depth (frames)
  uint8_t isr_task_priority;       // FreeRTOS task priority
  uint16_t isr_task_stack_size;    // Stack size (bytes)
};
```

**Field Details**:

**spi_host**:
- `SPI2_HOST` (1): HSPI peripheral
- `SPI3_HOST` (2): VSPI peripheral (ESP32 Classic)
- Use `MCP2515_SPI_HOST` for default

**spi_clock_speed**:
- Recommended: 10,000,000 (10 MHz)
- Maximum: 10,000,000 (MCP2515 limit)
- Minimum: 100,000 (100 kHz, slow but reliable)

**use_interrupts**:
- `true`: Interrupt-driven RX (efficient, recommended)
- `false`: Polling mode (simple, lower performance)

**use_mutex**:
- `true`: Thread-safe (required for multi-task access)
- `false`: Not thread-safe (single-task only)

**rx_queue_size**:
- Default: 32 frames
- Range: 1-255
- Each frame: 16 bytes
- Total memory: `rx_queue_size * 16` bytes

**isr_task_priority**:
- Default: `configMAX_PRIORITIES - 2` (typically 23 on ESP32)
- Range: 1 to `configMAX_PRIORITIES - 1` (0 = idle)
- Higher = more responsive, may starve lower-priority tasks

**isr_task_stack_size**:
- Default: 4096 bytes
- Minimum: 2048 bytes (tight, may overflow)
- Recommended: 4096 bytes
- Maximum: 8192 bytes (if using logging/debug)

**Example Configuration**:
```cpp
mcp2515_esp32_config_t config = {
  .spi_host = SPI3_HOST,
  .spi_clock_speed = 10000000,
  .pins = {
    .miso = GPIO_NUM_19,
    .mosi = GPIO_NUM_23,
    .sclk = GPIO_NUM_18,
    .cs = GPIO_NUM_5,
    .irq = GPIO_NUM_4
  },
  .use_interrupts = true,
  .use_mutex = true,
  .rx_queue_size = 64,
  .isr_task_priority = 20,
  .isr_task_stack_size = 4096
};
```

---

### mcp2515_statistics_t

```cpp
struct mcp2515_statistics_t {
  uint32_t rx_frames;       // Frames received
  uint32_t tx_frames;       // Frames transmitted
  uint32_t rx_errors;       // RX errors
  uint32_t tx_errors;       // TX errors
  uint32_t rx_overflow;     // RX buffer/queue overflows
  uint32_t tx_timeouts;     // TX timeouts (reserved)
  uint32_t bus_errors;      // Bus errors (ERRIF)
  uint32_t bus_off_count;   // Bus-off events (TXBO)
};
```

**Counter Behavior**:
- All counters are cumulative (reset with `resetStatistics()`)
- Updated atomically (spinlock-protected)
- Safe to read from any task

---

## Enumerations

### CAN_SPEED

```cpp
enum CAN_SPEED {
  CAN_5KBPS,
  CAN_10KBPS,
  CAN_20KBPS,
  CAN_31K25BPS,  // 31.25 kbps
  CAN_33KBPS,    // 33.333 kbps
  CAN_40KBPS,
  CAN_50KBPS,
  CAN_80KBPS,
  CAN_83K3BPS,   // 83.333 kbps
  CAN_95KBPS,    // 95 kbps (16 MHz only)
  CAN_100KBPS,
  CAN_125KBPS,   // Common default
  CAN_200KBPS,
  CAN_250KBPS,   // Common automotive
  CAN_500KBPS,   // Common industrial
  CAN_1000KBPS   // 1 Mbps (max)
};
```

**Common Bitrates**:
- **125 kbps**: Default for many applications
- **250 kbps**: Automotive (low speed CAN)
- **500 kbps**: Automotive (high speed CAN), industrial
- **1000 kbps**: High-speed industrial, short distances

**Cable Length Limits** (approximate):
- 1000 kbps: 40 meters
- 500 kbps: 100 meters
- 250 kbps: 200 meters
- 125 kbps: 500 meters
- 50 kbps: 1000 meters

---

### CAN_CLOCK

```cpp
enum CAN_CLOCK {
  MCP_20MHZ,
  MCP_16MHZ,
  MCP_8MHZ
};
```

**Oscillator Selection**:
- Check MCP2515 module datasheet
- Common modules use 8 MHz or 16 MHz
- Accuracy required: ±0.5% for reliable communication

---

### CAN_CLKOUT

```cpp
enum CAN_CLKOUT {
  CLKOUT_DISABLE = -1,
  CLKOUT_DIV1 = 0,
  CLKOUT_DIV2 = 1,
  CLKOUT_DIV4 = 2,
  CLKOUT_DIV8 = 3
};
```

**CLKOUT Pin Output**:
- Oscillator frequency divided by 1, 2, 4, or 8
- Disable replaces CLKOUT with SOF (Start of Frame) signal

---

### MASK

```cpp
enum MASK {
  MASK0,  // Applies to RXB0 (filters RXF0, RXF1)
  MASK1   // Applies to RXB1 (filters RXF2-RXF5)
};
```

---

### RXF

```cpp
enum RXF {
  RXF0 = 0,  // Filter 0 (RXB0, MASK0)
  RXF1 = 1,  // Filter 1 (RXB0, MASK0)
  RXF2 = 2,  // Filter 2 (RXB1, MASK1)
  RXF3 = 3,  // Filter 3 (RXB1, MASK1)
  RXF4 = 4,  // Filter 4 (RXB1, MASK1)
  RXF5 = 5   // Filter 5 (RXB1, MASK1)
};
```

---

### RXBn

```cpp
enum RXBn {
  RXB0 = 0,  // High-priority receive buffer
  RXB1 = 1   // Low-priority receive buffer
};
```

---

### TXBn

```cpp
enum TXBn {
  TXB0 = 0,  // Transmit buffer 0
  TXB1 = 1,  // Transmit buffer 1
  TXB2 = 2   // Transmit buffer 2
};
```

---

### CANINTF (Interrupt Flags)

```cpp
enum CANINTF : uint8_t {
  CANINTF_RX0IF = 0x01,  // RX buffer 0 full
  CANINTF_RX1IF = 0x02,  // RX buffer 1 full
  CANINTF_TX0IF = 0x04,  // TX buffer 0 empty
  CANINTF_TX1IF = 0x08,  // TX buffer 1 empty
  CANINTF_TX2IF = 0x10,  // TX buffer 2 empty
  CANINTF_ERRIF = 0x20,  // Error interrupt
  CANINTF_WAKIF = 0x40,  // Wake-up interrupt
  CANINTF_MERRF = 0x80   // Message error
};
```

---

### EFLG (Error Flags)

```cpp
enum EFLG : uint8_t {
  EFLG_EWARN   = 0x01,  // Error warning (TEC/REC ≥ 96)
  EFLG_RXWAR   = 0x02,  // RX error warning
  EFLG_TXWAR   = 0x04,  // TX error warning
  EFLG_RXEP    = 0x08,  // RX error passive (REC ≥ 128)
  EFLG_TXEP    = 0x10,  // TX error passive (TEC ≥ 128)
  EFLG_TXBO    = 0x20,  // Bus-off (TEC ≥ 256)
  EFLG_RX0OVR  = 0x40,  // RX buffer 0 overflow
  EFLG_RX1OVR  = 0x80   // RX buffer 1 overflow
};
```

---

## Error Codes

### ERROR Enum

```cpp
enum ERROR {
  ERROR_OK        = 0,  // Success
  ERROR_FAIL      = 1,  // General failure
  ERROR_ALLTXBUSY = 2,  // All TX buffers busy
  ERROR_FAILINIT  = 3,  // Initialization failed
  ERROR_FAILTX    = 4,  // Transmission failed
  ERROR_NOMSG     = 5,  // No message available
  ERROR_TIMEOUT   = 6,  // Timeout (ESP32)
  ERROR_MUTEX     = 7,  // Mutex acquisition failed (ESP32)
  ERROR_PSRAM     = 8   // PSRAM+DMA conflict (ESP32)
};
```

### Error Code Meanings

**ERROR_OK (0)**:
- Operation completed successfully
- Safe to proceed

**ERROR_FAIL (1)**:
- Generic failure
- Causes: Hardware not responding, invalid parameters, mode change timeout
- Check: Wiring, power, oscillator, SPI clock

**ERROR_ALLTXBUSY (2)**:
- All 3 TX buffers have pending transmissions
- Solutions:
  - Wait for transmission to complete
  - Abort pending messages with `abortAllTransmissions()`
  - Increase bitrate (reduce transmission time)
  - Check for bus-off condition

**ERROR_FAILINIT (3)**:
- MCP2515 initialization failed
- Causes: SPI communication failure, hardware fault
- Check: CS pin, SPI wiring, power supply, oscillator

**ERROR_FAILTX (4)**:
- Transmission setup or execution failed
- Causes: Invalid DLC, null pointer, hardware error, bus-off
- Check: Frame validity, error flags, bus connection

**ERROR_NOMSG (5)**:
- No CAN message available to read
- Normal condition when polling
- Not an error, just no data

**ERROR_TIMEOUT (6)** (ESP32 only):
- Operation timed out
- Causes: Queue read timeout, no frame within timeout period
- Adjust timeout parameter if needed

**ERROR_MUTEX (7)** (ESP32 only):
- Failed to acquire mutex within timeout
- Causes: Deadlock, task priority issue, timeout too short
- Default timeout: 10ms
- Check: Task priorities, mutex usage patterns

**ERROR_PSRAM (8)** (ESP32 only):
- PSRAM and SPI DMA both enabled (incompatible)
- Fix: Disable PSRAM OR set `MCP2515_SPI_DMA_CHAN=SPI_DMA_DISABLED`
- DMA cannot access PSRAM memory (will crash)

---

## Bitrate Configuration Constants

### 8 MHz Oscillator

```cpp
MCP_8MHz_5kBPS_CFG1/2/3
MCP_8MHz_10kBPS_CFG1/2/3
MCP_8MHz_20kBPS_CFG1/2/3
MCP_8MHz_31k25BPS_CFG1/2/3
MCP_8MHz_33k3BPS_CFG1/2/3
MCP_8MHz_40kBPS_CFG1/2/3
MCP_8MHz_50kBPS_CFG1/2/3
MCP_8MHz_80kBPS_CFG1/2/3
MCP_8MHz_100kBPS_CFG1/2/3
MCP_8MHz_125kBPS_CFG1/2/3
MCP_8MHz_200kBPS_CFG1/2/3
MCP_8MHz_250kBPS_CFG1/2/3
MCP_8MHz_500kBPS_CFG1/2/3
MCP_8MHz_1000kBPS_CFG1/2/3
```

### 16 MHz Oscillator (Most Common)

```cpp
MCP_16MHz_5kBPS_CFG1/2/3
MCP_16MHz_10kBPS_CFG1/2/3
MCP_16MHz_20kBPS_CFG1/2/3
MCP_16MHz_33k3BPS_CFG1/2/3
MCP_16MHz_40kBPS_CFG1/2/3
MCP_16MHz_50kBPS_CFG1/2/3
MCP_16MHz_80kBPS_CFG1/2/3
MCP_16MHz_83k3BPS_CFG1/2/3
MCP_16MHz_95kBPS_CFG1/2/3
MCP_16MHz_100kBPS_CFG1/2/3
MCP_16MHz_125kBPS_CFG1/2/3
MCP_16MHz_200kBPS_CFG1/2/3
MCP_16MHz_250kBPS_CFG1/2/3
MCP_16MHz_500kBPS_CFG1/2/3
MCP_16MHz_1000kBPS_CFG1/2/3
```

### 20 MHz Oscillator

```cpp
MCP_20MHz_33k3BPS_CFG1/2/3
MCP_20MHz_40kBPS_CFG1/2/3
MCP_20MHz_50kBPS_CFG1/2/3
MCP_20MHz_80kBPS_CFG1/2/3
MCP_20MHz_83k3BPS_CFG1/2/3
MCP_20MHz_100kBPS_CFG1/2/3
MCP_20MHz_125kBPS_CFG1/2/3
MCP_20MHz_200kBPS_CFG1/2/3
MCP_20MHz_250kBPS_CFG1/2/3
MCP_20MHz_500kBPS_CFG1/2/3
MCP_20MHz_1000kBPS_CFG1/2/3
```

**Note**: These constants are used internally by `setBitrate()`. Do not access directly.

---

## Platform-Specific Behavior

### Arduino AVR (Uno, Mega2560)

**Memory Constraints**:
- Arduino Uno: 2 KB RAM (very limited)
- Arduino Mega2560: 8 KB RAM (sufficient)
- Library instance: ~140 bytes
- No dynamic allocation
- No FreeRTOS features

**Performance**:
- 16 MHz CPU
- SPI: Up to 8 MHz practical (10 MHz theoretical)
- Frame reception: ~200-300µs per frame
- No interrupt-driven queue (use polling)

**Example**:
```cpp
MCP2515 can(10);  // CS = pin 10

void setup() {
  can.reset();
  can.setBitrate(CAN_125KBPS, MCP_16MHZ);
  can.setNormalMode();
}

void loop() {
  struct can_frame frame;
  if (can.readMessage(&frame) == MCP2515::ERROR_OK) {
    // Process frame
  }
}
```

---

### ESP32 (Arduino Framework)

**Features**:
- Dual-core 240 MHz (Classic/S3), single-core 160 MHz (C3)
- FreeRTOS multitasking
- Interrupt-driven reception
- Thread-safe operations
- Statistics tracking

**Memory**:
- Instance: ~180 bytes
- RX queue: 512 bytes (default)
- ISR task: 4096 bytes stack

**Example**:
```cpp
MCP2515 can(GPIO_NUM_5, GPIO_NUM_4);  // CS, INT

void setup() {
  can.reset();
  can.setBitrate(CAN_500KBPS);
  can.setNormalMode();
}

void loop() {
  struct can_frame frame;
  if (can.readMessageQueued(&frame, 100) == MCP2515::ERROR_OK) {
    // Process frame (interrupt mode)
  }
}
```

---

### ESP32 (Native ESP-IDF)

**Features**: Same as Arduino-ESP32, plus:
- Native SPI driver (no Arduino overhead)
- ESP-IDF logging (ESP_LOGI, ESP_LOGE)
- Direct FreeRTOS access

**Example**:
```cpp
mcp2515_esp32_config_t config = { /* ... */ };
MCP2515 can(&config);

extern "C" void app_main() {
  can.reset();
  can.setBitrate(CAN_500KBPS);
  can.setNormalMode();

  while(1) {
    struct can_frame frame;
    can.readMessageQueued(&frame, portMAX_DELAY);
    // Process frame
  }
}
```

---

## Memory and Performance

### Memory Usage Summary

| Platform | Instance | Queue | ISR Stack | Total |
|----------|----------|-------|-----------|-------|
| Arduino Uno | 140 B | 0 | 0 | 140 B |
| Arduino Mega | 140 B | 0 | 0 | 140 B |
| ESP32 (polling) | 180 B | 0 | 0 | 180 B |
| ESP32 (interrupt) | 180 B | 512 B | 4096 B | ~4.8 KB |

### SPI Performance

**Timing (10 MHz SPI)**:
- Single register read: ~5µs
- Register write: ~5µs
- Read CAN frame: ~50µs (optimized instruction)
- Write CAN frame: ~60µs (optimized instruction)

**Optimizations**:
- Uses READ RX BUFFER instruction (0x90/0x94) - saves 1 byte per read
- Uses LOAD TX BUFFER instruction (0x40/0x42/0x44) - saves 1 byte per write
- Performance gain: 10-15% faster reception, 5-10% faster transmission

### CAN Transmission Time

**Frame Duration** (depends on bitrate):

| Bitrate | Standard Frame (8 bytes) | Extended Frame (8 bytes) |
|---------|-------------------------|-------------------------|
| 125 kbps | ~864µs | ~960µs |
| 250 kbps | ~432µs | ~480µs |
| 500 kbps | ~216µs | ~240µs |
| 1 Mbps | ~108µs | ~120µs |

**Formula**:
```
Standard: (47 + 8*DLC) / bitrate
Extended: (67 + 8*DLC) / bitrate
```

### Interrupt Latency (ESP32)

**ISR Trigger to Task Wake**:
- GPIO ISR: <5µs (IRAM_ATTR)
- Semaphore give: ~10µs
- Task wake: <100µs (depends on scheduler)
- **Total**: <200µs typical

**Queue Processing**:
- Frame read: ~50µs (SPI)
- Queue send: ~20µs
- **Total**: ~70µs per frame

---

## ISR Safety and Thread Safety

### ISR-Safe Functions (ESP32)

**Functions marked IRAM_ATTR** (safe to call during flash operations):
```cpp
startSPI()
endSPI()
readRegister()
readRegisters()
setRegister()
setRegisters()
modifyRegister()
readMessage()  // Both overloads
getErrorFlags()
getInterrupts()
clearTXInterrupts()
clearERRIF()
getStatus()
```

**NOT ISR-Safe** (all others):
- Any function that uses mutex
- Mode switching functions
- Filter/mask configuration
- Transmission functions (use mutex)

**ISR Best Practice**:
```cpp
volatile bool dataReady = false;

void IRAM_ATTR canISR() {
  dataReady = true;  // Set flag only
}

void loop() {
  if (dataReady) {
    dataReady = false;
    struct can_frame frame;
    can.readMessage(&frame);  // Safe in loop
  }
}
```

---

### Thread-Safe Functions (ESP32)

**All public functions are thread-safe when**:
- Constructor called with `use_mutex = true` (default)
- Recursive mutex allows nested calls
- Same task can call multiple times

**Thread Safety Example**:
```cpp
// Task 1 (Core 0)
void task1(void* param) {
  while(1) {
    struct can_frame frame;
    if (can.readMessageQueued(&frame, 100) == MCP2515::ERROR_OK) {
      // Process frame
    }
  }
}

// Task 2 (Core 1)
void task2(void* param) {
  while(1) {
    struct can_frame frame;
    frame.can_id = 0x123;
    frame.can_dlc = 8;
    can.sendMessage(&frame);  // Thread-safe
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
```

---

### PSRAM + DMA Safety (ESP32)

**Critical Warning**:
- If PSRAM enabled AND SPI DMA enabled: **WILL CRASH**
- DMA cannot access PSRAM memory
- Library detects at compile-time and runtime

**Compile-Time Detection**:
```cpp
#if CONFIG_SPIRAM_USE_MALLOC && MCP2515_SPI_DMA_CHAN != SPI_DMA_DISABLED
  #warning "PSRAM and DMA conflict!"
#endif
```

**Runtime Detection**:
```cpp
ERROR initSPI(config) {
  #if CONFIG_SPIRAM_USE_MALLOC
    if (MCP2515_SPI_DMA_CHAN != SPI_DMA_DISABLED) {
      return ERROR_PSRAM;  // Fail initialization
    }
  #endif
}
```

**Fix Options**:
1. **Disable DMA**: Set `MCP2515_SPI_DMA_CHAN=SPI_DMA_DISABLED` in `mcp2515_esp32_config.h`
2. **Disable PSRAM**: Turn off PSRAM in `sdkconfig`
3. **Use DMA-capable memory**: Allocate buffers with `heap_caps_malloc(MALLOC_CAP_DMA)`

---

## Example Code Patterns

### Basic Polling (Arduino AVR)

```cpp
#include <SPI.h>
#include <mcp2515.h>

MCP2515 can(10);

void setup() {
  Serial.begin(115200);

  can.reset();
  can.setBitrate(CAN_125KBPS, MCP_16MHZ);
  can.setNormalMode();

  Serial.println("CAN Initialized");
}

void loop() {
  struct can_frame frame;

  // Receive
  if (can.readMessage(&frame) == MCP2515::ERROR_OK) {
    Serial.print("ID: 0x");
    Serial.print(frame.can_id, HEX);
    Serial.print(" DLC: ");
    Serial.print(frame.can_dlc);
    Serial.print(" Data: ");
    for (int i = 0; i < frame.can_dlc; i++) {
      Serial.print(frame.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // Send
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 1000) {
    lastSend = millis();

    frame.can_id = 0x123;
    frame.can_dlc = 8;
    for (int i = 0; i < 8; i++) {
      frame.data[i] = i;
    }

    can.sendMessage(&frame);
  }
}
```

---

### Interrupt-Driven (ESP32)

```cpp
#include <mcp2515.h>

MCP2515 can(GPIO_NUM_5, GPIO_NUM_4);  // CS, INT

void setup() {
  Serial.begin(115200);

  can.reset();
  can.setBitrate(CAN_500KBPS);
  can.setNormalMode();

  Serial.println("ESP32 CAN (interrupt mode)");
}

void loop() {
  struct can_frame frame;

  // Non-blocking queue read
  if (can.readMessageQueued(&frame, 0) == MCP2515::ERROR_OK) {
    Serial.printf("ID: 0x%03X, DLC: %d, Data: ",
                  frame.can_id & CAN_SFF_MASK, frame.can_dlc);
    for (int i = 0; i < frame.can_dlc; i++) {
      Serial.printf("%02X ", frame.data[i]);
    }
    Serial.println();
  }

  // Other tasks...
  delay(10);
}
```

---

### Filter Configuration

```cpp
void setupFilters() {
  // Accept only IDs 0x100-0x1FF (standard frames)
  can.setFilter(MCP2515::RXF0, false, 0x100);
  can.setFilterMask(MCP2515::MASK0, false, 0x700);
  // Mask 0x700 = 0b11100000000 (upper 3 bits must match)

  // Accept specific extended ID
  can.setFilter(MCP2515::RXF2, true, 0x12345678);
  can.setFilterMask(MCP2515::MASK1, true, 0x1FFFFFFF);

  can.setNormalMode();
}
```

---

### Error Handling

```cpp
void checkErrors() {
  if (can.checkError()) {
    uint8_t eflg = can.getErrorFlags();

    if (eflg & MCP2515::EFLG_TXBO) {
      Serial.println("CRITICAL: Bus-off!");
      Serial.printf("TX errors: %d\n", can.errorCountTX());
      can.performErrorRecovery();  // ESP32 only
    }

    if (eflg & MCP2515::EFLG_RX0OVR) {
      Serial.println("RX0 overflow - messages lost!");
      can.clearRXnOVR();
    }

    if (eflg & MCP2515::EFLG_EWARN) {
      Serial.printf("Error warning - RX: %d, TX: %d\n",
                    can.errorCountRX(), can.errorCountTX());
    }
  }
}
```

---

### Multi-Task ESP32

```cpp
void rxTask(void* param) {
  MCP2515* can = (MCP2515*)param;
  struct can_frame frame;

  while(1) {
    if (can->readMessageQueued(&frame, portMAX_DELAY)
        == MCP2515::ERROR_OK) {
      // Process received frame
      processRxFrame(&frame);
    }
  }
}

void txTask(void* param) {
  MCP2515* can = (MCP2515*)param;
  struct can_frame frame;

  while(1) {
    // Generate frame
    frame.can_id = 0x456;
    frame.can_dlc = 8;
    memcpy(frame.data, &sensorData, 8);

    can->sendMessage(&frame);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  can.reset();
  can.setBitrate(CAN_500KBPS);
  can.setNormalMode();

  xTaskCreatePinnedToCore(rxTask, "can_rx", 4096, &can, 5, NULL, 1);
  xTaskCreatePinnedToCore(txTask, "can_tx", 4096, &can, 4, NULL, 1);
}
```

---

## Troubleshooting Guide

### Initialization Fails (reset() returns ERROR_FAIL)

**Symptoms**:
- `reset()` returns `ERROR_FAIL` or `ERROR_FAILINIT`
- `isInitialized()` returns `false` (ESP32)

**Checks**:
1. **SPI Wiring**:
   - MISO, MOSI, SCK, CS connected correctly
   - No loose connections
   - Cable length < 30cm

2. **Power Supply**:
   - MCP2515: 5V or 3.3V (check module spec)
   - Stable power (no brown-out)
   - Decoupling capacitor (100nF ceramic near VCC)

3. **Oscillator**:
   - Crystal frequency correct (8/16/20 MHz)
   - Crystal soldered properly
   - Load capacitors present

4. **SPI Clock**:
   - Try reducing to 5 MHz or 1 MHz
   - Check for interference

5. **CS Pin**:
   - Correct pin number in constructor
   - Pin not used by other peripheral

**Debug Code**:
```cpp
MCP2515::ERROR err = can.reset();
if (err != MCP2515::ERROR_OK) {
  Serial.print("Reset failed: ");
  Serial.println(err);

  // Try manual SPI test
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0xC0);  // RESET instruction
  digitalWrite(CS_PIN, HIGH);
  delay(10);

  // Read CANCTRL register (should be 0x87 after reset)
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x03);  // READ instruction
  SPI.transfer(0x0F);  // CANCTRL address
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);

  Serial.printf("CANCTRL: 0x%02X (expected 0x87)\n", val);
}
```

---

### Communication Fails (ERROR_ALLTXBUSY)

**Symptoms**:
- `sendMessage()` returns `ERROR_ALLTXBUSY`
- No messages received on other node

**Checks**:
1. **CAN Bus Wiring**:
   - CAN_H and CAN_L connected
   - 120Ω termination resistors at BOTH ends
   - Twisted pair cable (or shielded)
   - Cable length within limit for bitrate

2. **Bitrate Mismatch**:
   - All nodes use same bitrate
   - Oscillator frequency correct

3. **Bus-Off Condition**:
   - Check `getErrorFlags()` for `EFLG_TXBO`
   - Error counters: `errorCountTX()`, `errorCountRX()`
   - Call `performErrorRecovery()` (ESP32)

4. **No Acknowledgment**:
   - At least one other node must be on bus
   - Other nodes in Normal mode (not Listen-Only)

5. **CAN Transceiver**:
   - MCP2551/2562/TJA1055 powered
   - CANL/CANH not shorted to ground
   - Transceiver not in standby/sleep mode

**Debug Code**:
```cpp
uint8_t eflg = can.getErrorFlags();
Serial.printf("Error flags: 0x%02X\n", eflg);
Serial.printf("RX errors: %d, TX errors: %d\n",
              can.errorCountRX(), can.errorCountTX());

if (eflg & MCP2515::EFLG_TXBO) {
  Serial.println("Bus-off detected!");
}

// Check if any node acknowledges
can.setLoopbackMode();  // Self-test
if (can.sendMessage(&frame) == MCP2515::ERROR_OK) {
  Serial.println("Loopback OK - MCP2515 works");
} else {
  Serial.println("Loopback FAIL - MCP2515 hardware issue");
}
```

---

### Messages Not Received

**Symptoms**:
- `readMessage()` returns `ERROR_NOMSG`
- `checkReceive()` returns `false`

**Checks**:
1. **Filters**:
   - Filters not blocking desired IDs
   - Try accepting all: `setFilterMask(MASK0, false, 0x000)`

2. **Mode**:
   - MCP2515 in Normal or Listen-Only mode
   - Not in Sleep or Configuration mode

3. **Interrupts** (ESP32):
   - INT pin connected
   - ISR task running (`getRxQueueCount()` should be > 0)

4. **Queue Overflow** (ESP32):
   - `statistics.rx_overflow` increasing
   - Increase queue size or read faster

5. **Bus Errors**:
   - Check error flags
   - Verify wiring

**Debug Code**:
```cpp
// Disable filters
can.setFilterMask(MCP2515::MASK0, false, 0x000);
can.setFilterMask(MCP2515::MASK1, true, 0x00000000);

// Check status
Serial.printf("Status: 0x%02X\n", can.getStatus());
Serial.printf("Interrupts: 0x%02X\n", can.getInterrupts());

// ESP32: Check queue
#ifdef ESP32
Serial.printf("Queue count: %d\n", can.getRxQueueCount());
mcp2515_statistics_t stats;
can.getStatistics(&stats);
Serial.printf("RX frames: %u, Overflows: %u\n",
              stats.rx_frames, stats.rx_overflow);
#endif
```

---

### Intermittent Failures

**Symptoms**:
- Communication works sometimes, fails other times
- Random errors

**Checks**:
1. **Cable Quality**:
   - Use twisted pair or shielded cable
   - Keep away from high-current wires
   - Check for loose connections

2. **Electromagnetic Interference**:
   - Move away from motors, relays, switching power supplies
   - Add ferrite beads on CAN bus

3. **Ground Loops**:
   - Ensure all nodes share common ground
   - Avoid multiple ground paths

4. **Power Supply Noise**:
   - Add larger decoupling capacitors (100µF electrolytic + 100nF ceramic)
   - Check for voltage dips during transmission

5. **Timing Issues**:
   - Reduce bitrate (try 125 kbps)
   - Check oscillator accuracy (±0.5% required)

---

### ESP32 Mutex Timeout (ERROR_MUTEX)

**Symptoms**:
- Functions return `ERROR_MUTEX`
- System hangs

**Causes**:
1. **Deadlock**: Two tasks waiting for each other
2. **Priority Inversion**: Low-priority task holds mutex
3. **Timeout Too Short**: Default 10ms may be insufficient

**Fixes**:
```cpp
// Increase timeout in mcp2515_esp32_config.h:
#define MCP2515_MUTEX_TIMEOUT pdMS_TO_TICKS(100)

// Check task priorities:
Serial.printf("Task priority: %d\n", uxTaskPriorityGet(NULL));

// Avoid nested calls from different tasks
```

---

### PSRAM Crash (ESP32)

**Symptoms**:
- System crashes during CAN operation
- Guru Meditation Error / Core Panic

**Cause**: PSRAM enabled with SPI DMA (incompatible)

**Fix**:
```cpp
// Option 1: Disable DMA in mcp2515_esp32_config.h
#define MCP2515_SPI_DMA_CHAN SPI_DMA_DISABLED

// Option 2: Disable PSRAM in sdkconfig
CONFIG_SPIRAM_USE_MALLOC=n

// Option 3: Use DMA-capable memory
struct can_frame* frame = (struct can_frame*)
  heap_caps_malloc(sizeof(struct can_frame), MALLOC_CAP_DMA);
```

---

## Appendix: Register Map

### Configuration Registers

| Register | Address | Description |
|----------|---------|-------------|
| CNF1 | 0x2A | Bit timing configuration 1 |
| CNF2 | 0x29 | Bit timing configuration 2 |
| CNF3 | 0x28 | Bit timing configuration 3 |
| CANCTRL | 0x0F | CAN control register |
| CANSTAT | 0x0E | CAN status register |

### Interrupt Registers

| Register | Address | Description |
|----------|---------|-------------|
| CANINTE | 0x2B | Interrupt enable |
| CANINTF | 0x2C | Interrupt flags |
| EFLG | 0x2D | Error flags |

### Error Counters

| Register | Address | Description |
|----------|---------|-------------|
| TEC | 0x1C | Transmit error counter |
| REC | 0x1D | Receive error counter |

### Filter Registers

| Filter | Base Address | Mask |
|--------|--------------|------|
| RXF0 | 0x00 | MASK0 |
| RXF1 | 0x04 | MASK0 |
| RXF2 | 0x08 | MASK1 |
| RXF3 | 0x10 | MASK1 |
| RXF4 | 0x14 | MASK1 |
| RXF5 | 0x18 | MASK1 |

### Mask Registers

| Mask | Base Address |
|------|--------------|
| MASK0 | 0x20 |
| MASK1 | 0x24 |

### Buffer Registers

| Buffer | Control | SIDH | Data |
|--------|---------|------|------|
| TXB0 | 0x30 | 0x31 | 0x36 |
| TXB1 | 0x40 | 0x41 | 0x46 |
| TXB2 | 0x50 | 0x51 | 0x56 |
| RXB0 | 0x60 | 0x61 | 0x66 |
| RXB1 | 0x70 | 0x71 | 0x76 |

---

## Known Issues and Edge Cases

### Loopback Mode ABAT Stuck-On Bug (Fixed in v2.1.1)

**Issue**: In loopback mode, the MCP2515 hardware may not auto-clear the ABAT (Abort All Pending Transmissions) bit after calling `abortAllTransmissions()`.

**Root Cause**: Loopback mode is a "silent" mode with no actual CAN bus arbitration. The MCP2515 hardware expects bus arbitration to complete before clearing ABAT, but in loopback mode this never occurs.

**Symptoms** (prior to v2.1.1):
- After calling `abortAllTransmissions()` in loopback mode, ALL subsequent transmissions failed
- Frames were instantly aborted with ABTF (Abort Flag) set
- `sendMessage()` returned ERROR_OK, but transmission never occurred
- Stress tests showed 10.70% success rate instead of 100%
- CANCTRL register showed ABAT bit stuck ON (0x57 instead of 0x47)

**Fix**: The library now detects when ABAT doesn't auto-clear within 10ms and manually clears it. This is safe and complies with the MCP2515 datasheet.

**Impact**: Fixed in version 2.1.1. Users on older versions should upgrade immediately if using loopback mode or calling `abortAllTransmissions()`.

**Code Location**: `mcp2515.cpp:1201-1208`

---

### ISR Task Frame Consumption in "Polling Mode" (Fixed in v2.1.1)

**Issue**: When interrupts were "disabled" via `setInterruptMode(false)`, the ISR task continued consuming frames from hardware buffers, causing race conditions.

**Root Cause**: `setInterruptMode(false)` only disabled the GPIO interrupt, but the ISR task continued running with a 100ms timeout. When the timeout expired, `processInterrupts()` was called, which consumed frames before application polling code could read them.

**Symptoms** (prior to v2.1.1):
- Polling mode (`setInterruptMode(false)`) didn't work correctly
- `checkReceive()` would return `true`, but `readMessage()` returned ERROR_NOMSG
- Frames disappeared between `checkReceive()` and `readMessage()`
- Diagnostic tests in polling mode failed with status register showing 0x00 (no RXnIF)

**Fix**: `processInterrupts()` now checks the `use_interrupts` flag before processing. When interrupts are disabled, the ISR task does not consume frames.

**Impact**: Fixed in version 2.1.1. This enables true polling mode operation for diagnostics and testing.

**Code Location**: `mcp2515.cpp:1663`

---

### Queue-First Reception Check (Implemented in v2.1.1)

**Issue**: On ESP32 with interrupts enabled, `checkReceive()` must check the FreeRTOS queue before checking hardware registers.

**Reason**: The ISR task consumes frames from hardware RX buffers and places them in a queue. If `checkReceive()` only checks hardware, it misses frames that are already in the queue.

**Implementation**: `checkReceive()` now checks the queue first when interrupts are enabled. If queue is non-empty, it returns `true` immediately. Otherwise, it falls through to hardware check.

**Impact**: This prevents missed frames and ensures correct behavior in interrupt mode.

**Code Location**: `mcp2515.cpp:1379` (checkReceive function)

---

### Loopback Mode Filter Behavior

**Issue**: Acceptance filters and masks are still evaluated in loopback mode, even though the MCP2515 datasheet states loopback is for internal testing.

**Behavior**: If filters are configured for specific IDs, loopback frames that don't match will be rejected.

**Workaround**: Set all masks to 0x000 (accept all) when using loopback mode for testing:
```cpp
can.setLoopbackMode();
can.setFilterMask(MCP2515::MASK0, false, 0x000);
can.setFilterMask(MCP2515::MASK1, false, 0x000);
```

**Status**: This is MCP2515 hardware behavior, not a library bug. Documented for user awareness.

---

### Loopback Mode Timing Requirements

**Issue**: Loopback mode requires 5-10ms settle time after mode change before reliable operation.

**Symptoms**: Immediate transmission after `setLoopbackMode()` may fail or return corrupted data.

**Workaround**: Add a 5-10ms delay after `setLoopbackMode()` before sending frames.

**Example**:
```cpp
can.setLoopbackMode();
delay(10);  // Settle time
can.sendMessage(&frame);  // Now reliable
```

**Status**: This is normal MCP2515 behavior. The library does not enforce this delay automatically to avoid blocking.

---

## Version History - Critical Fixes

### Version 2.1.1 (2025-11-18)

**Critical Bugs Fixed**:

1. **Fixed ABAT Stuck-On in Loopback Mode**
   - Manually clear ABAT if hardware doesn't auto-clear
   - Prevents 100% transmission failure after `abortAllTransmissions()`
   - **Impact**: Loopback stress test success rate: 10.70% → 100.00%

2. **Fixed ISR Task Frame Consumption in Polling Mode**
   - `processInterrupts()` now respects `use_interrupts` flag
   - Enables true polling mode operation
   - **Impact**: Polling mode diagnostic now works correctly

3. **Fixed checkReceive() Queue-First Logic**
   - Checks FreeRTOS queue before hardware (ESP32 interrupt mode)
   - Prevents missed frames
   - **Impact**: Correct frame detection in interrupt mode

**Test Results**:
- Overall pass rate: 74.71% → 97.85%
- Tests passing: 65/87 → 91/93
- Stress test (1000 frames): 10.70% → 100.00% success
- Polling mode: FAIL → PASS

**Files Modified**:
- `mcp2515.cpp` (3 critical fixes)
- `mcp2515.h` (added getTXB0CTRL() diagnostic method)

---

### Version 2.1.0-ESP32 (2025-11-15)

**Major Changes**:
- BREAKING: Changed 4 functions from void→ERROR return type
- Added IRAM_ATTR to 14 functions for flash-safe ISR execution
- Pinned ISR task to Core 1 for predictable performance
- Added PSRAM safety checks
- Verified multi-platform build support

---

### Version 2.0.0-ESP32 (2025-11-15)

**Features Added**:
- SPI optimization (READ RX BUFFER, LOAD TX BUFFER instructions)
- Transmit priority control (`setTransmitPriority()`)
- Abort functionality (`abortTransmission()`, `abortAllTransmissions()`)
- Filter hit reporting (`getFilterHit()`)

---

## Summary

This API reference provides complete documentation for the ESP32-MCP2515 library. Key points:

- **Multi-platform**: ESP32 (all variants), Arduino AVR
- **Two reception modes**: Polling (all platforms), Interrupt-driven (ESP32)
- **Thread-safe**: ESP32 with FreeRTOS mutex
- **Optimized**: Fast SPI instructions, IRAM placement
- **Production-ready**: Comprehensive error handling, statistics, recovery

For additional information:
- See `CLAUDE.md` for development guide
- See `Documentation/ESP32_COMPREHENSIVE_AUDIT_2025-11-15.md` for ESP32 details
- See examples in `examples/` directory

**License**: MIT
**Repository**: https://github.com/Bobertsawesome/ESP32-mcp2515

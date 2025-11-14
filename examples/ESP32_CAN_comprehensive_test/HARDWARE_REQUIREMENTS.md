# Hardware Requirements for Comprehensive CAN Test Suite

This document outlines the **exact hardware needed** for each test mode.

---

## 📋 Bill of Materials

### Mode 1: Loopback Self-Test (Minimum Setup)

| Component | Quantity | Notes |
|-----------|----------|-------|
| ESP32 Development Board | 1 | Any ESP32 variant (ESP32, ESP32-S2, ESP32-C3) |
| MCP2515 CAN Module | 1 | Must include MCP2515 + CAN transceiver |
| Jumper Wires | 7 | Male-to-female or male-to-male depending on boards |
| USB Cable | 1 | For programming and serial monitor |

**Total Cost:** ~$10-20 USD

### Mode 2: Two-Device Real CAN Bus Test (Full Setup)

| Component | Quantity | Notes |
|-----------|----------|-------|
| ESP32 Development Board | 2 | Matching or different variants OK |
| MCP2515 CAN Module | 2 | Both must have same crystal frequency |
| 120Ω Resistors | 2 | 1/4W, used for bus termination |
| Twisted Pair Wire | 0.5-1m | Or regular wire for short distances |
| Jumper Wires | 14 | Male-to-female or male-to-male |
| USB Cables | 2 | For programming and power |
| Breadboard (optional) | 1-2 | For organizing connections |

**Total Cost:** ~$20-40 USD

---

## 🔌 Wiring Diagrams

### Mode 1: Loopback Self-Test

```
┌─────────────────────┐
│      ESP32          │
│                     │
│  GPIO 23 (MOSI) ────┼─────► SI (MOSI)  ┌────────────────┐
│  GPIO 19 (MISO) ◄───┼───── SO (MISO)   │   MCP2515      │
│  GPIO 18 (SCK)  ────┼─────► SCK         │   CAN Module   │
│  GPIO 5  (CS)   ────┼─────► CS          │                │
│  GPIO 4  (INT)  ◄───┼───── INT          │   ┌──────┐     │
│                     │                   │   │Crystal│    │
│  5V ────────────────┼─────► VCC         │   │16 MHz│    │
│  GND ───────────────┼─────► GND         │   └──────┘     │
│                     │                   │                │
└─────────────────────┘                   │  CAN_H  (NC)   │
                                          │  CAN_L  (NC)   │
     [USB for Serial Monitor]            └────────────────┘

     NC = Not Connected (leave floating)
```

**Key Points:**
- ✅ CAN_H and CAN_L are **NOT CONNECTED** to anything
- ✅ No termination resistors needed
- ✅ Works with just 1 device
- ✅ Tests library functionality in isolation

---

### Mode 2: Two-Device Real CAN Bus Test

```
┌───────────────┐                                        ┌───────────────┐
│    ESP32 #1   │                                        │    ESP32 #2   │
│   (MASTER)    │                                        │    (SLAVE)    │
│               │                                        │               │
│ GPIO 23 ──────┼──► SI     ┌────────────┐      ┌────────────┐  ◄── GPIO 23 │
│ GPIO 19 ◄─────┼─── SO     │  MCP2515   │      │  MCP2515   │  ──► GPIO 19 │
│ GPIO 18 ──────┼──► SCK    │   Module   │      │   Module   │  ◄── GPIO 18 │
│ GPIO 5  ──────┼──► CS     │     #1     │      │     #2     │  ◄── GPIO 5  │
│ GPIO 4  ◄─────┼─── INT    │            │      │            │  ──► GPIO 4  │
│               │           │            │      │            │              │
│ 5V ───────────┼──► VCC    │  CAN_H ────┼──┐   │  CAN_H ────┼──┐           │
│ GND ──────────┼──► GND    │  CAN_L ────┼──┼───┼── CAN_L    │  │           │
│               │           └────────────┘  │   └────────────┘  │           │
└───────────────┘                           │                   │           │
                                           │                   │
                                          ╔═══════╗          ╔═══════╗
                                          ║ 120Ω  ║          ║ 120Ω  ║
                                          ╚═══════╝          ╚═══════╝
                                        Termination        Termination
                                        Resistor #1        Resistor #2

                         CAN Bus (Twisted Pair Recommended)
```

**Key Points:**
- ✅ CAN_H connects to CAN_H
- ✅ CAN_L connects to CAN_L
- ✅ Two 120Ω resistors at **both ends** of bus
- ✅ Total bus impedance = 60Ω (120Ω || 120Ω)
- ✅ Use twisted pair wire for distances > 10cm

**Resistor Connection Detail:**
```
       CAN_H ─────┬─────────────┬───── CAN_H
                  │             │
                 ┌┴┐           ┌┴┐
                 │ │ 120Ω      │ │ 120Ω
                 └┬┘           └┬┘
                  │             │
       CAN_L ─────┴─────────────┴───── CAN_L
```

---

## 🔧 MCP2515 Module Variants

### Common MCP2515 Modules

Most MCP2515 modules include:
- MCP2515 CAN Controller chip
- SPI interface pins
- CAN transceiver (TJA1050, SN65HVD230, or similar)
- Voltage regulator (5V or 3.3V)
- Crystal oscillator (8/16/20 MHz)

### Identifying Crystal Frequency

Look for the crystal component on your MCP2515 module:

```
    ┌─────────────┐
    │   MCP2515   │
    │             │
    │   ┌─────┐   │
    │   │XTAL │   │  ← Crystal oscillator
    │   │16.0 │   │     (frequency printed on it)
    │   └─────┘   │
    │             │
    └─────────────┘
```

**Common markings:**
- "16.000" or "16M" = 16 MHz (most common)
- "8.000" or "8M" = 8 MHz
- "20.000" or "20M" = 20 MHz

**⚠️ CRITICAL:** The `CAN_CLOCK` setting in code **MUST** match your crystal!

---

## ⚡ Power Requirements

### ESP32 Power
- **Voltage:** 3.3V (internal) or 5V (via USB/VIN)
- **Current:** ~200-500mA during operation
- **Source:** USB cable, battery, or power supply

### MCP2515 Module Power
- **Voltage:** 5V (most modules) or 3.3V (some modules)
- **Current:** ~100mA typical, ~200mA max
- **Source:** ESP32 5V pin or external supply

### Power Connection Options

**Option 1: USB Power (Testing/Development)**
```
USB Cable → ESP32 USB port → 5V pin → MCP2515 VCC
```
✅ Simple, no external supply needed
⚠️ Limited current (~500mA from USB)

**Option 2: External Power (Production)**
```
5V Power Supply → ESP32 VIN → 5V pin → MCP2515 VCC
                            ↓
                           GND → MCP2515 GND
```
✅ More current available
✅ More stable power

---

## 📏 Cable Length Guidelines

Maximum cable length depends on CAN bitrate:

| CAN Speed | Max Cable Length | Use Case |
|-----------|------------------|----------|
| 1 Mbps | ~40 meters | High-speed industrial |
| 500 kbps | ~100 meters | Automotive (CAN FD) |
| 250 kbps | ~250 meters | Automation systems |
| 125 kbps | ~500 meters | Building automation |
| 50 kbps | ~1000 meters | Long-distance sensing |
| 5 kbps | ~1000+ meters | Extended networks |

**For this test:**
- Loopback mode: N/A (no cables)
- Two-device mode: Keep cables **< 1 meter** for initial testing

---

## 🛒 Where to Buy

### Development Boards
- **ESP32 DevKit**: Amazon, AliExpress, Adafruit, SparkFun
- **Price**: $5-15 USD

### MCP2515 Modules
- **MCP2515 + TJA1050**: Amazon, AliExpress, eBay
- **MCP2515 + SN65HVD230**: Better for 3.3V systems
- **Price**: $3-8 USD per module

### Recommended MCP2515 Modules
1. **Standard 5V Module with TJA1050**
   - Most compatible
   - Works with 5V and 3.3V ESP32s
   - Usually has 16 MHz crystal

2. **3.3V Module with SN65HVD230**
   - Better for pure 3.3V systems
   - Lower power consumption
   - May have 8 MHz crystal

### Other Components
- **120Ω Resistors**: Any electronics supplier
- **Twisted Pair Wire**: Ethernet cable (use 1 pair)
- **Jumper Wires**: Amazon, electronics stores

---

## 🔍 Module Compatibility Check

Before purchasing, verify:

1. ✅ **Crystal frequency is specified** (8/16/20 MHz)
2. ✅ **Includes CAN transceiver** (not just MCP2515 chip)
3. ✅ **Voltage regulator included** (check input voltage)
4. ✅ **Pins are labeled** (SI/SO/SCK/CS/INT/VCC/GND/CANH/CANL)
5. ✅ **Reviews mention ESP32 compatibility**

---

## ⚙️ Pin Configuration for Different ESP32 Boards

### Standard ESP32 DevKit (30-pin)

```
Default VSPI Pins:
MOSI → GPIO 23
MISO → GPIO 19
SCK  → GPIO 18
CS   → GPIO 5  (or any free GPIO)
INT  → GPIO 4  (or any free GPIO)
```

### ESP32-S2

```
Default SPI Pins:
MOSI → GPIO 35
MISO → GPIO 37
SCK  → GPIO 36
CS   → GPIO 34 (or any free GPIO)
INT  → GPIO 33 (or any free GPIO)
```

### ESP32-C3

```
Default SPI Pins:
MOSI → GPIO 6
MISO → GPIO 5
SCK  → GPIO 4
CS   → GPIO 7  (or any free GPIO)
INT  → GPIO 8  (or any free GPIO)
```

**Note:** Adjust pin numbers in the test code configuration section if using non-standard pins.

---

## 📊 Testing Checklist

### Pre-Test Hardware Verification

Before running tests, verify:

- [ ] All wiring connections are secure
- [ ] Power LED on MCP2515 module is lit
- [ ] Crystal frequency matches code configuration
- [ ] (Two-device mode) 120Ω resistors are connected
- [ ] (Two-device mode) CAN_H and CAN_L are not swapped
- [ ] ESP32 is recognized by Arduino IDE
- [ ] Serial Monitor baud rate is set to 115200

### Post-Connection Multimeter Checks

**Single Device (Loopback):**
- [ ] Measure 5V between VCC and GND on MCP2515
- [ ] Measure continuity on each SPI connection

**Two Devices (Real Bus):**
- [ ] Measure ~60Ω between CAN_H and CAN_L (with resistors)
- [ ] Measure 5V on both MCP2515 modules
- [ ] Verify no short between CAN_H and GND
- [ ] Verify no short between CAN_L and GND

---

## 🎯 Quick Start Recommendations

### For Beginners
**Start with Mode 1: Loopback Self-Test**
- Requires only 1 ESP32 and 1 MCP2515
- No CAN bus wiring complications
- Validates library and hardware independently
- Build confidence before moving to real CAN bus

### For Experienced Users
**Use Mode 2: Two-Device Test**
- Validates real-world CAN communication
- Tests bus loading and termination
- Identifies wiring and signal integrity issues
- Closer to actual deployment scenario

---

## 📞 Support

If hardware issues persist after checking this guide:

1. **Try loopback mode first** - isolates software from bus issues
2. **Check all connections with multimeter** - measure voltages and continuity
3. **Verify crystal frequency** - most common configuration mistake
4. **Test with different ESP32 pins** - some pins have special functions
5. **Try different MCP2515 module** - hardware defects are rare but possible

---

**Ready to test?** See the main [README.md](README.md) for software configuration and running instructions.

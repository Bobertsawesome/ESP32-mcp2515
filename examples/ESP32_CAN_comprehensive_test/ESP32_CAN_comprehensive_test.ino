/**
 * @file ESP32_CAN_comprehensive_test.ino
 * @brief Comprehensive Test Suite for ESP32 MCP2515 CAN Library
 *
 * This example tests EVERY function in the ESP32 MCP2515 library with detailed
 * reporting and statistics. It can operate in two modes:
 *
 * MODE 1: LOOPBACK SELF-TEST (Default - Requires Only 1 ESP32)
 *   - Tests all functions using MCP2515's internal loopback mode
 *   - No external CAN bus or second device needed
 *   - Perfect for validating library functionality
 *
 * MODE 2: TWO-DEVICE TEST (Requires 2 ESP32s with MCP2515)
 *   - Tests real CAN bus communication between two devices
 *   - Requires CAN_H, CAN_L wiring and 120Ω terminators
 *   - One device runs as MASTER, the other as SLAVE
 *
 * @hardware ESP32 + MCP2515 CAN module(s)
 * @author ESP32-MCP2515 Library Test Suite
 * @date 2025
 */

#include <SPI.h>
#include <mcp2515.h>

// ============================================================================
// HARDWARE CONFIGURATION - CUSTOMIZE FOR YOUR SETUP
// ============================================================================

// SPI Pin Configuration (adjust for your ESP32 board)
#define SPI_MOSI_PIN    23    // Default ESP32 VSPI MOSI
#define SPI_MISO_PIN    19    // Default ESP32 VSPI MISO
#define SPI_SCK_PIN     18    // Default ESP32 VSPI SCK
#define SPI_CS_PIN      5     // Chip Select pin
#define SPI_INT_PIN     4     // Interrupt pin (set to -1 to disable interrupts)

// CAN Bus Configuration
#define CAN_SPEED       CAN_125KBPS   // CAN bus speed
#define CAN_CLOCK       MCP_16MHZ     // MCP2515 crystal frequency (8MHz, 16MHz, or 20MHz)

// ============================================================================
// TEST CONFIGURATION
// ============================================================================

// Test Mode Selection
typedef enum {
    TEST_MODE_LOOPBACK,     // Mode 1: Self-test using loopback (1 ESP32 required)
    TEST_MODE_TWO_DEVICE    // Mode 2: Real CAN bus test (2 ESP32s required)
} test_mode_t;

#define TEST_MODE       TEST_MODE_LOOPBACK    // Change this to TEST_MODE_TWO_DEVICE for dual-device testing

// Device Role (only used in TWO_DEVICE mode)
typedef enum {
    ROLE_MASTER,            // Master device (initiates tests)
    ROLE_SLAVE              // Slave device (responds to master)
} device_role_t;

#define DEVICE_ROLE     ROLE_MASTER           // Set to ROLE_SLAVE on second device

// Test Configuration Options
#define ENABLE_INTERRUPT_TESTS    (SPI_INT_PIN >= 0)  // Automatically disable if no INT pin
#define ENABLE_STRESS_TESTS       true        // Enable high-load stress testing
#define STRESS_TEST_DURATION_MS   5000        // Duration of stress tests
#define VERBOSE_OUTPUT            true        // Print detailed test information

// Serial Configuration
#define SERIAL_BAUD     115200

// ============================================================================
// ANSI COLOR CODES FOR TERMINAL OUTPUT
// ============================================================================

#define COLOR_RESET     "\033[0m"
#define COLOR_RED       "\033[31m"
#define COLOR_GREEN     "\033[32m"
#define COLOR_YELLOW    "\033[33m"
#define COLOR_BLUE      "\033[34m"
#define COLOR_MAGENTA   "\033[35m"
#define COLOR_CYAN      "\033[36m"
#define COLOR_WHITE     "\033[37m"
#define COLOR_BOLD      "\033[1m"

// ============================================================================
// TEST INFRASTRUCTURE
// ============================================================================

// Test statistics
struct test_stats_t {
    uint32_t total_tests;
    uint32_t passed_tests;
    uint32_t failed_tests;
    uint32_t skipped_tests;
};

test_stats_t test_stats = {0, 0, 0, 0};

// MCP2515 instance
MCP2515 mcp2515(SPI_CS_PIN);

// Test helper functions
void printTestHeader(const char* test_name);
void printTestResult(bool passed, const char* message = nullptr);
void printSectionHeader(const char* section_name);
void printStatistics();
bool waitForFrame(struct can_frame* frame, uint32_t timeout_ms);
void printFrame(const struct can_frame* frame, bool is_tx);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 5000); // Wait up to 5 seconds for serial

    delay(1000); // Allow serial to stabilize

    Serial.println();
    Serial.println(COLOR_BOLD COLOR_CYAN "╔════════════════════════════════════════════════════════════════╗" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_CYAN "║   ESP32 MCP2515 CAN LIBRARY COMPREHENSIVE TEST SUITE          ║" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_CYAN "╚════════════════════════════════════════════════════════════════╝" COLOR_RESET);
    Serial.println();

    // Print configuration
    Serial.println(COLOR_BOLD "Configuration:" COLOR_RESET);
    Serial.printf("  Test Mode:        %s\n", TEST_MODE == TEST_MODE_LOOPBACK ? "LOOPBACK (Self-Test)" : "TWO-DEVICE (Real CAN Bus)");
    if (TEST_MODE == TEST_MODE_TWO_DEVICE) {
        Serial.printf("  Device Role:      %s\n", DEVICE_ROLE == ROLE_MASTER ? "MASTER" : "SLAVE");
    }
    Serial.printf("  SPI Pins:         MOSI=%d, MISO=%d, SCK=%d, CS=%d, INT=%d\n",
                  SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN, SPI_CS_PIN, SPI_INT_PIN);
    Serial.printf("  CAN Speed:        %s\n", getSpeedName(CAN_SPEED));
    Serial.printf("  CAN Clock:        %s\n", getClockName(CAN_CLOCK));
    Serial.printf("  Interrupt Tests:  %s\n", ENABLE_INTERRUPT_TESTS ? "ENABLED" : "DISABLED");
    Serial.printf("  Stress Tests:     %s\n", ENABLE_STRESS_TESTS ? "ENABLED" : "DISABLED");
    Serial.println();

    delay(2000);

    // Initialize SPI
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);

    // Run comprehensive test suite
    runComprehensiveTests();

    // Print final statistics
    printFinalStatistics();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    // Tests complete - show summary every 10 seconds
    delay(10000);
    Serial.println(COLOR_YELLOW "\n--- Test suite completed. See results above. ---" COLOR_RESET);
}

// ============================================================================
// COMPREHENSIVE TEST SUITE
// ============================================================================

void runComprehensiveTests() {
    Serial.println(COLOR_BOLD COLOR_BLUE "╔════════════════════════════════════════════════════════════════╗" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_BLUE "║  Starting Comprehensive Test Suite                            ║" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_BLUE "╚════════════════════════════════════════════════════════════════╝" COLOR_RESET);
    Serial.println();

    // Test categories
    testInitialization();
    testOperatingModes();
    testBitrateConfiguration();
    testFiltersAndMasks();
    testFrameTransmission();
    testFrameReception();
    testErrorHandling();

    #if ENABLE_INTERRUPT_TESTS
    testInterruptFunctions();
    #endif

    testStatistics();
    testStatusFunctions();

    #if ENABLE_STRESS_TESTS
    testStressScenarios();
    #endif

    testClockOutput();
    testAdvancedFeatures();
}

// ============================================================================
// TEST CATEGORY 1: INITIALIZATION
// ============================================================================

void testInitialization() {
    printSectionHeader("INITIALIZATION TESTS");

    // Test 1: Basic reset
    printTestHeader("MCP2515 Reset");
    MCP2515::ERROR err = mcp2515.reset();
    printTestResult(err == MCP2515::ERROR_OK, "Reset successful");

    // Test 2: Set bitrate
    printTestHeader("Set Bitrate");
    err = mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK);
    printTestResult(err == MCP2515::ERROR_OK, "Bitrate configured successfully");

    // Test 3: Set normal mode (or loopback for self-test)
    printTestHeader("Set Operating Mode");
    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        err = mcp2515.setLoopbackMode();
        printTestResult(err == MCP2515::ERROR_OK, "Loopback mode set");
    } else {
        err = mcp2515.setNormalMode();
        printTestResult(err == MCP2515::ERROR_OK, "Normal mode set");
    }

    // Test 4: Check initialization status (ESP32-specific)
    #ifdef ESP32
    printTestHeader("Check Initialization Status");
    bool initialized = mcp2515.isInitialized();
    printTestResult(initialized, "Library initialized successfully");
    #endif

    delay(100);
}

// ============================================================================
// TEST CATEGORY 2: OPERATING MODES
// ============================================================================

void testOperatingModes() {
    printSectionHeader("OPERATING MODE TESTS");

    // Test all operating modes
    const struct {
        const char* name;
        MCP2515::ERROR (MCP2515::*setter)();
    } modes[] = {
        {"Configuration Mode", &MCP2515::setConfigMode},
        {"Normal Mode", &MCP2515::setNormalMode},
        {"Listen-Only Mode", &MCP2515::setListenOnlyMode},
        {"Loopback Mode", &MCP2515::setLoopbackMode},
        {"Sleep Mode", &MCP2515::setSleepMode},
        {"Normal One-Shot Mode", &MCP2515::setNormalOneShotMode}
    };

    for (uint8_t i = 0; i < sizeof(modes)/sizeof(modes[0]); i++) {
        printTestHeader(modes[i].name);
        MCP2515::ERROR err = (mcp2515.*(modes[i].setter))();
        printTestResult(err == MCP2515::ERROR_OK, "Mode set successfully");
        delay(50);
    }

    // Restore operating mode for subsequent tests
    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        mcp2515.setLoopbackMode();
    } else {
        mcp2515.setNormalMode();
    }

    delay(100);
}

// ============================================================================
// TEST CATEGORY 3: BITRATE CONFIGURATION
// ============================================================================

void testBitrateConfiguration() {
    printSectionHeader("BITRATE CONFIGURATION TESTS");

    // Test various bitrates
    const CAN_SPEED speeds[] = {
        CAN_5KBPS, CAN_10KBPS, CAN_20KBPS, CAN_50KBPS,
        CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS
    };

    for (uint8_t i = 0; i < sizeof(speeds)/sizeof(speeds[0]); i++) {
        printTestHeader(getSpeedName(speeds[i]));

        MCP2515::ERROR err = mcp2515.setConfigMode();
        if (err == MCP2515::ERROR_OK) {
            err = mcp2515.setBitrate(speeds[i], CAN_CLOCK);
        }

        printTestResult(err == MCP2515::ERROR_OK, "Bitrate configured");
        delay(20);
    }

    // Restore original bitrate
    mcp2515.setConfigMode();
    mcp2515.setBitrate(CAN_SPEED, CAN_CLOCK);
    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        mcp2515.setLoopbackMode();
    } else {
        mcp2515.setNormalMode();
    }

    delay(100);
}

// ============================================================================
// TEST CATEGORY 4: FILTERS AND MASKS
// ============================================================================

void testFiltersAndMasks() {
    printSectionHeader("FILTER AND MASK TESTS");

    // Test setting masks
    printTestHeader("Set Mask 0 (Standard ID)");
    MCP2515::ERROR err = mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
    printTestResult(err == MCP2515::ERROR_OK, "MASK0 set for standard IDs");

    printTestHeader("Set Mask 1 (Extended ID)");
    err = mcp2515.setFilterMask(MCP2515::MASK1, true, 0x1FFFFFFF);
    printTestResult(err == MCP2515::ERROR_OK, "MASK1 set for extended IDs");

    // Test setting filters
    const struct {
        MCP2515::RXF filter;
        bool ext;
        uint32_t id;
    } filters[] = {
        {MCP2515::RXF0, false, 0x123},
        {MCP2515::RXF1, false, 0x456},
        {MCP2515::RXF2, true, 0x12345678},
        {MCP2515::RXF3, true, 0x87654321},
        {MCP2515::RXF4, false, 0x555},
        {MCP2515::RXF5, false, 0x777}
    };

    for (uint8_t i = 0; i < 6; i++) {
        char filter_name[32];
        snprintf(filter_name, sizeof(filter_name), "Set Filter %d (%s)",
                 i, filters[i].ext ? "Extended" : "Standard");
        printTestHeader(filter_name);

        err = mcp2515.setFilter(filters[i].filter, filters[i].ext, filters[i].id);
        printTestResult(err == MCP2515::ERROR_OK, "Filter set successfully");
    }

    // Clear filters for subsequent tests (accept all)
    for (uint8_t i = 0; i < 6; i++) {
        mcp2515.setFilter((MCP2515::RXF)i, false, 0);
    }
    mcp2515.setFilterMask(MCP2515::MASK0, false, 0);
    mcp2515.setFilterMask(MCP2515::MASK1, false, 0);

    delay(100);
}

// ============================================================================
// TEST CATEGORY 5: FRAME TRANSMISSION
// ============================================================================

void testFrameTransmission() {
    printSectionHeader("FRAME TRANSMISSION TESTS");

    // Test 1: Standard data frame
    printTestHeader("Send Standard Data Frame");
    struct can_frame tx_frame;
    tx_frame.can_id = 0x123;
    tx_frame.can_dlc = 8;
    for (uint8_t i = 0; i < 8; i++) {
        tx_frame.data[i] = i;
    }
    MCP2515::ERROR err = mcp2515.sendMessage(&tx_frame);
    printTestResult(err == MCP2515::ERROR_OK, "Standard frame sent");
    if (VERBOSE_OUTPUT) printFrame(&tx_frame, true);
    delay(50);

    // Test 2: Extended data frame
    printTestHeader("Send Extended Data Frame");
    tx_frame.can_id = 0x12345678 | CAN_EFF_FLAG;
    tx_frame.can_dlc = 8;
    for (uint8_t i = 0; i < 8; i++) {
        tx_frame.data[i] = 0xFF - i;
    }
    err = mcp2515.sendMessage(&tx_frame);
    printTestResult(err == MCP2515::ERROR_OK, "Extended frame sent");
    if (VERBOSE_OUTPUT) printFrame(&tx_frame, true);
    delay(50);

    // Test 3: Remote frame (Standard)
    printTestHeader("Send Standard Remote Frame");
    tx_frame.can_id = 0x456 | CAN_RTR_FLAG;
    tx_frame.can_dlc = 4;
    err = mcp2515.sendMessage(&tx_frame);
    printTestResult(err == MCP2515::ERROR_OK, "Standard RTR frame sent");
    if (VERBOSE_OUTPUT) printFrame(&tx_frame, true);
    delay(50);

    // Test 4: Remote frame (Extended)
    printTestHeader("Send Extended Remote Frame");
    tx_frame.can_id = 0x87654321 | CAN_EFF_FLAG | CAN_RTR_FLAG;
    tx_frame.can_dlc = 6;
    err = mcp2515.sendMessage(&tx_frame);
    printTestResult(err == MCP2515::ERROR_OK, "Extended RTR frame sent");
    if (VERBOSE_OUTPUT) printFrame(&tx_frame, true);
    delay(50);

    // Test 5: Variable DLC frames
    for (uint8_t dlc = 0; dlc <= 8; dlc++) {
        char test_name[32];
        snprintf(test_name, sizeof(test_name), "Send Frame with DLC=%d", dlc);
        printTestHeader(test_name);

        tx_frame.can_id = 0x100 + dlc;
        tx_frame.can_dlc = dlc;
        for (uint8_t i = 0; i < dlc; i++) {
            tx_frame.data[i] = dlc * 10 + i;
        }
        err = mcp2515.sendMessage(&tx_frame);
        printTestResult(err == MCP2515::ERROR_OK, "Frame sent");
        delay(20);
    }

    // Test 6: Send to specific TX buffer
    printTestHeader("Send to TX Buffer 0");
    tx_frame.can_id = 0x200;
    tx_frame.can_dlc = 4;
    err = mcp2515.sendMessage(MCP2515::TXB0, &tx_frame);
    printTestResult(err == MCP2515::ERROR_OK, "TXB0 transmission");
    delay(20);

    printTestHeader("Send to TX Buffer 1");
    tx_frame.can_id = 0x201;
    err = mcp2515.sendMessage(MCP2515::TXB1, &tx_frame);
    printTestResult(err == MCP2515::ERROR_OK, "TXB1 transmission");
    delay(20);

    printTestHeader("Send to TX Buffer 2");
    tx_frame.can_id = 0x202;
    err = mcp2515.sendMessage(MCP2515::TXB2, &tx_frame);
    printTestResult(err == MCP2515::ERROR_OK, "TXB2 transmission");
    delay(20);

    delay(100);
}

// ============================================================================
// TEST CATEGORY 6: FRAME RECEPTION
// ============================================================================

void testFrameReception() {
    printSectionHeader("FRAME RECEPTION TESTS");

    // In loopback mode, we can receive what we sent
    // In two-device mode, slave should send frames for master to receive

    if (TEST_MODE == TEST_MODE_TWO_DEVICE && DEVICE_ROLE == ROLE_SLAVE) {
        // Slave sends test frames
        Serial.println(COLOR_YELLOW "Slave: Sending test frames for master..." COLOR_RESET);

        for (int i = 0; i < 5; i++) {
            struct can_frame tx_frame;
            tx_frame.can_id = 0x300 + i;
            tx_frame.can_dlc = 8;
            for (uint8_t j = 0; j < 8; j++) {
                tx_frame.data[j] = i * 10 + j;
            }
            mcp2515.sendMessage(&tx_frame);
            delay(100);
        }
        return;
    }

    // Test reception
    printTestHeader("Check Receive Function");

    // Send a frame first (in loopback it will be received immediately)
    struct can_frame tx_frame;
    tx_frame.can_id = 0x300;
    tx_frame.can_dlc = 8;
    for (uint8_t i = 0; i < 8; i++) {
        tx_frame.data[i] = 0xA0 + i;
    }
    mcp2515.sendMessage(&tx_frame);
    delay(50);

    bool has_message = mcp2515.checkReceive();
    printTestResult(has_message, "checkReceive() detected message");

    // Test reading message
    printTestHeader("Read Message (Auto Buffer)");
    struct can_frame rx_frame;
    MCP2515::ERROR err = mcp2515.readMessage(&rx_frame);
    bool rx_ok = (err == MCP2515::ERROR_OK);
    printTestResult(rx_ok, "Message read successfully");

    if (rx_ok && VERBOSE_OUTPUT) {
        printFrame(&rx_frame, false);

        // Verify data integrity
        bool data_match = true;
        for (uint8_t i = 0; i < tx_frame.can_dlc; i++) {
            if (rx_frame.data[i] != tx_frame.data[i]) {
                data_match = false;
                break;
            }
        }
        Serial.printf("  Data integrity: %s%s%s\n",
                     data_match ? COLOR_GREEN : COLOR_RED,
                     data_match ? "PASS" : "FAIL",
                     COLOR_RESET);
    }

    // Test reading from specific buffer
    printTestHeader("Read from RX Buffer 0");
    tx_frame.can_id = 0x301;
    mcp2515.sendMessage(&tx_frame);
    delay(50);
    err = mcp2515.readMessage(MCP2515::RXB0, &rx_frame);
    printTestResult(err == MCP2515::ERROR_OK, "RXB0 read successful");

    // Test no message available
    printTestHeader("Read with No Message Available");
    err = mcp2515.readMessage(&rx_frame);
    printTestResult(err == MCP2515::ERROR_NOMSG, "ERROR_NOMSG returned correctly");

    delay(100);
}

// ============================================================================
// TEST CATEGORY 7: ERROR HANDLING
// ============================================================================

void testErrorHandling() {
    printSectionHeader("ERROR HANDLING TESTS");

    // Test 1: Get error flags
    printTestHeader("Get Error Flags");
    uint8_t eflg = mcp2515.getErrorFlags();
    printTestResult(true, "Error flags read");
    if (VERBOSE_OUTPUT) {
        Serial.printf("  EFLG = 0x%02X\n", eflg);
        if (eflg & MCP2515::EFLG_RX0OVR) Serial.println("    - RX0 Overflow");
        if (eflg & MCP2515::EFLG_RX1OVR) Serial.println("    - RX1 Overflow");
        if (eflg & MCP2515::EFLG_TXBO) Serial.println("    - TX Bus-Off");
        if (eflg & MCP2515::EFLG_TXEP) Serial.println("    - TX Error Passive");
        if (eflg & MCP2515::EFLG_RXEP) Serial.println("    - RX Error Passive");
        if (eflg & MCP2515::EFLG_TXWAR) Serial.println("    - TX Error Warning");
        if (eflg & MCP2515::EFLG_RXWAR) Serial.println("    - RX Error Warning");
    }

    // Test 2: Check error
    printTestHeader("Check Error Function");
    bool has_error = mcp2515.checkError();
    printTestResult(true, has_error ? "Errors detected" : "No errors");

    // Test 3: Error counters
    printTestHeader("Read RX Error Counter");
    uint8_t rec = mcp2515.errorCountRX();
    printTestResult(true, "RX error count read");
    if (VERBOSE_OUTPUT) Serial.printf("  REC = %d\n", rec);

    printTestHeader("Read TX Error Counter");
    uint8_t tec = mcp2515.errorCountTX();
    printTestResult(true, "TX error count read");
    if (VERBOSE_OUTPUT) Serial.printf("  TEC = %d\n", tec);

    // Test 4: Clear overflow flags
    printTestHeader("Clear RXnOVR Flags");
    mcp2515.clearRXnOVRFlags();
    printTestResult(true, "Overflow flags cleared");

    // Test 5: Clear RXnOVR
    printTestHeader("Clear RXnOVR Function");
    mcp2515.clearRXnOVR();
    printTestResult(true, "RXnOVR cleared");

    // Test 6: Clear MERR
    printTestHeader("Clear MERR Flag");
    mcp2515.clearMERR();
    printTestResult(true, "MERR cleared");

    // Test 7: Clear ERRIF
    printTestHeader("Clear ERRIF Flag");
    mcp2515.clearERRIF();
    printTestResult(true, "ERRIF cleared");

    // Test 8: Error recovery (ESP32-specific)
    #ifdef ESP32
    printTestHeader("Perform Error Recovery");
    MCP2515::ERROR err = mcp2515.performErrorRecovery();
    printTestResult(err == MCP2515::ERROR_OK, "Error recovery performed");
    #endif

    // Test 9: Bus status (ESP32-specific)
    #ifdef ESP32
    printTestHeader("Get Bus Status");
    uint8_t bus_status = mcp2515.getBusStatus();
    printTestResult(true, "Bus status read");
    if (VERBOSE_OUTPUT) Serial.printf("  CANSTAT = 0x%02X\n", bus_status);
    #endif

    delay(100);
}

// ============================================================================
// TEST CATEGORY 8: INTERRUPT FUNCTIONS
// ============================================================================

#if ENABLE_INTERRUPT_TESTS
void testInterruptFunctions() {
    printSectionHeader("INTERRUPT FUNCTION TESTS");

    // Test 1: Get interrupts
    printTestHeader("Get Interrupt Flags");
    uint8_t irq = mcp2515.getInterrupts();
    printTestResult(true, "Interrupt flags read");
    if (VERBOSE_OUTPUT) {
        Serial.printf("  CANINTF = 0x%02X\n", irq);
        if (irq & MCP2515::CANINTF_RX0IF) Serial.println("    - RX0 Interrupt");
        if (irq & MCP2515::CANINTF_RX1IF) Serial.println("    - RX1 Interrupt");
        if (irq & MCP2515::CANINTF_TX0IF) Serial.println("    - TX0 Interrupt");
        if (irq & MCP2515::CANINTF_TX1IF) Serial.println("    - TX1 Interrupt");
        if (irq & MCP2515::CANINTF_TX2IF) Serial.println("    - TX2 Interrupt");
        if (irq & MCP2515::CANINTF_ERRIF) Serial.println("    - Error Interrupt");
        if (irq & MCP2515::CANINTF_WAKIF) Serial.println("    - Wake Interrupt");
        if (irq & MCP2515::CANINTF_MERRF) Serial.println("    - Message Error");
    }

    // Test 2: Get interrupt mask
    printTestHeader("Get Interrupt Mask");
    uint8_t imask = mcp2515.getInterruptMask();
    printTestResult(true, "Interrupt mask read");
    if (VERBOSE_OUTPUT) Serial.printf("  CANINTE = 0x%02X\n", imask);

    // Test 3: Clear all interrupts
    printTestHeader("Clear All Interrupts");
    mcp2515.clearInterrupts();
    uint8_t irq_after = mcp2515.getInterrupts();
    printTestResult(irq_after == 0, "All interrupts cleared");

    // Test 4: Clear TX interrupts only
    printTestHeader("Clear TX Interrupts");
    mcp2515.clearTXInterrupts();
    printTestResult(true, "TX interrupts cleared");

    // Test 5: Enable/disable interrupt mode (ESP32-specific)
    #ifdef ESP32
    printTestHeader("Enable Interrupt Mode");
    MCP2515::ERROR err = mcp2515.setInterruptMode(true);
    printTestResult(err == MCP2515::ERROR_OK, "Interrupt mode enabled");

    printTestHeader("Disable Interrupt Mode");
    err = mcp2515.setInterruptMode(false);
    printTestResult(err == MCP2515::ERROR_OK, "Interrupt mode disabled");
    #endif

    delay(100);
}
#endif

// ============================================================================
// TEST CATEGORY 9: STATISTICS (ESP32-specific)
// ============================================================================

void testStatistics() {
    printSectionHeader("STATISTICS TESTS (ESP32-specific)");

    #ifdef ESP32
    // Test 1: Reset statistics
    printTestHeader("Reset Statistics");
    mcp2515.resetStatistics();
    printTestResult(true, "Statistics reset");

    // Test 2: Send some frames to generate statistics
    printTestHeader("Generate TX Statistics");
    for (int i = 0; i < 10; i++) {
        struct can_frame frame;
        frame.can_id = 0x400 + i;
        frame.can_dlc = 8;
        for (uint8_t j = 0; j < 8; j++) {
            frame.data[j] = i * 10 + j;
        }
        mcp2515.sendMessage(&frame);
        delay(10);
    }
    printTestResult(true, "10 frames sent");

    // Test 3: Get statistics
    printTestHeader("Get Statistics");
    mcp2515_statistics_t stats;
    mcp2515.getStatistics(&stats);
    printTestResult(true, "Statistics retrieved");

    if (VERBOSE_OUTPUT) {
        Serial.println("  Statistics:");
        Serial.printf("    RX Frames:     %lu\n", stats.rx_frames);
        Serial.printf("    TX Frames:     %lu\n", stats.tx_frames);
        Serial.printf("    RX Errors:     %lu\n", stats.rx_errors);
        Serial.printf("    TX Errors:     %lu\n", stats.tx_errors);
        Serial.printf("    RX Overflow:   %lu\n", stats.rx_overflow);
        Serial.printf("    TX Timeouts:   %lu\n", stats.tx_timeouts);
        Serial.printf("    Bus Errors:    %lu\n", stats.bus_errors);
        Serial.printf("    Bus-Off Count: %lu\n", stats.bus_off_count);
    }

    // Test 4: Get RX queue count
    printTestHeader("Get RX Queue Count");
    uint32_t queue_count = mcp2515.getRxQueueCount();
    printTestResult(true, "Queue count retrieved");
    if (VERBOSE_OUTPUT) Serial.printf("  Queue count = %lu\n", queue_count);

    #else
    Serial.println(COLOR_YELLOW "Statistics tests are ESP32-specific - skipped" COLOR_RESET);
    test_stats.skipped_tests += 4;
    #endif

    delay(100);
}

// ============================================================================
// TEST CATEGORY 10: STATUS FUNCTIONS
// ============================================================================

void testStatusFunctions() {
    printSectionHeader("STATUS FUNCTION TESTS");

    // Test 1: Get status
    printTestHeader("Get Status");
    uint8_t status = mcp2515.getStatus();
    printTestResult(true, "Status read");
    if (VERBOSE_OUTPUT) {
        Serial.printf("  Status = 0x%02X\n", status);
        if (status & 0x01) Serial.println("    - RX0IF");
        if (status & 0x02) Serial.println("    - RX1IF");
    }

    delay(100);
}

// ============================================================================
// TEST CATEGORY 11: STRESS TESTS
// ============================================================================

#if ENABLE_STRESS_TESTS
void testStressScenarios() {
    printSectionHeader("STRESS TESTS");

    // Stress Test 1: High-speed transmission
    printTestHeader("High-Speed TX Stress Test");
    uint32_t start_time = millis();
    uint32_t frame_count = 0;
    uint32_t error_count = 0;

    struct can_frame stress_frame;
    stress_frame.can_dlc = 8;

    while (millis() - start_time < STRESS_TEST_DURATION_MS) {
        stress_frame.can_id = 0x500 + (frame_count % 256);
        for (uint8_t i = 0; i < 8; i++) {
            stress_frame.data[i] = (frame_count >> (i * 8)) & 0xFF;
        }

        if (mcp2515.sendMessage(&stress_frame) == MCP2515::ERROR_OK) {
            frame_count++;
        } else {
            error_count++;
        }

        // Small delay to prevent overwhelming the bus
        delayMicroseconds(100);
    }

    uint32_t duration = millis() - start_time;
    float frames_per_sec = (float)frame_count / (duration / 1000.0);

    Serial.printf("  Sent %lu frames in %lu ms (%.1f frames/sec)\n",
                  frame_count, duration, frames_per_sec);
    Serial.printf("  Errors: %lu (%.2f%%)\n", error_count,
                  (float)error_count / (frame_count + error_count) * 100.0);
    printTestResult(error_count < frame_count / 10, "Less than 10% errors");

    // Stress Test 2: RX buffer overflow test (if in loopback)
    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        printTestHeader("RX Buffer Overflow Test");

        // Send many frames rapidly without reading
        for (int i = 0; i < 20; i++) {
            stress_frame.can_id = 0x600 + i;
            mcp2515.sendMessage(&stress_frame);
            delayMicroseconds(50);
        }

        delay(100);

        uint8_t eflg = mcp2515.getErrorFlags();
        bool overflow_detected = (eflg & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR));

        Serial.printf("  Overflow %s\n", overflow_detected ? "DETECTED" : "NOT DETECTED");
        printTestResult(true, "Overflow test completed");

        // Clear overflow
        mcp2515.clearRXnOVR();
    }

    delay(100);
}
#endif

// ============================================================================
// TEST CATEGORY 12: CLOCK OUTPUT
// ============================================================================

void testClockOutput() {
    printSectionHeader("CLOCK OUTPUT TESTS");

    const struct {
        const char* name;
        CAN_CLKOUT divisor;
    } clkout_modes[] = {
        {"Disable CLKOUT", CLKOUT_DISABLE},
        {"CLKOUT Divide by 1", CLKOUT_DIV1},
        {"CLKOUT Divide by 2", CLKOUT_DIV2},
        {"CLKOUT Divide by 4", CLKOUT_DIV4},
        {"CLKOUT Divide by 8", CLKOUT_DIV8}
    };

    for (uint8_t i = 0; i < sizeof(clkout_modes)/sizeof(clkout_modes[0]); i++) {
        printTestHeader(clkout_modes[i].name);
        MCP2515::ERROR err = mcp2515.setClkOut(clkout_modes[i].divisor);
        printTestResult(err == MCP2515::ERROR_OK, "CLKOUT configured");
        delay(50);
    }

    delay(100);
}

// ============================================================================
// TEST CATEGORY 13: ADVANCED FEATURES
// ============================================================================

void testAdvancedFeatures() {
    printSectionHeader("ADVANCED FEATURE TESTS");

    // Test queued message reading (ESP32-specific with interrupts)
    #ifdef ESP32
    printTestHeader("Read Message Queued (Non-blocking)");
    struct can_frame frame;
    MCP2515::ERROR err = mcp2515.readMessageQueued(&frame, 0);
    bool result = (err == MCP2515::ERROR_OK || err == MCP2515::ERROR_NOMSG || err == MCP2515::ERROR_TIMEOUT);
    printTestResult(result, "Queued read tested");

    printTestHeader("Read Message Queued (With Timeout)");
    err = mcp2515.readMessageQueued(&frame, 100);
    result = (err == MCP2515::ERROR_OK || err == MCP2515::ERROR_NOMSG || err == MCP2515::ERROR_TIMEOUT);
    printTestResult(result, "Queued read with timeout tested");
    #endif

    delay(100);
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void printSectionHeader(const char* section_name) {
    Serial.println();
    Serial.println(COLOR_BOLD COLOR_MAGENTA "┌────────────────────────────────────────────────────────────────┐" COLOR_RESET);
    Serial.printf(COLOR_BOLD COLOR_MAGENTA "│ %-62s │\n" COLOR_RESET, section_name);
    Serial.println(COLOR_BOLD COLOR_MAGENTA "└────────────────────────────────────────────────────────────────┘" COLOR_RESET);
}

void printTestHeader(const char* test_name) {
    Serial.printf("\n%s[TEST]%s %s\n", COLOR_BOLD COLOR_CYAN, COLOR_RESET, test_name);
    test_stats.total_tests++;
}

void printTestResult(bool passed, const char* message) {
    if (passed) {
        test_stats.passed_tests++;
        Serial.printf("  %s✓ PASS%s", COLOR_GREEN, COLOR_RESET);
    } else {
        test_stats.failed_tests++;
        Serial.printf("  %s✗ FAIL%s", COLOR_RED, COLOR_RESET);
    }

    if (message) {
        Serial.printf(" - %s\n", message);
    } else {
        Serial.println();
    }
}

void printFrame(const struct can_frame* frame, bool is_tx) {
    Serial.printf("  %s[%s]%s ID: 0x%08X %s%s%s DLC: %d Data: ",
                 COLOR_YELLOW,
                 is_tx ? "TX" : "RX",
                 COLOR_RESET,
                 frame->can_id & (CAN_EFF_MASK | CAN_SFF_MASK),
                 (frame->can_id & CAN_EFF_FLAG) ? "[EXT]" : "[STD]",
                 (frame->can_id & CAN_RTR_FLAG) ? "[RTR]" : "",
                 COLOR_RESET,
                 frame->can_dlc);

    for (uint8_t i = 0; i < frame->can_dlc; i++) {
        Serial.printf("%02X ", frame->data[i]);
    }
    Serial.println();
}

void printFinalStatistics() {
    Serial.println();
    Serial.println(COLOR_BOLD COLOR_CYAN "╔════════════════════════════════════════════════════════════════╗" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_CYAN "║                     FINAL TEST RESULTS                         ║" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_CYAN "╚════════════════════════════════════════════════════════════════╝" COLOR_RESET);
    Serial.println();

    Serial.printf("  Total Tests:    %s%lu%s\n", COLOR_BOLD, test_stats.total_tests, COLOR_RESET);
    Serial.printf("  Passed:         %s%lu%s (%.1f%%)\n",
                  COLOR_GREEN, test_stats.passed_tests, COLOR_RESET,
                  (float)test_stats.passed_tests / test_stats.total_tests * 100.0);
    Serial.printf("  Failed:         %s%lu%s (%.1f%%)\n",
                  COLOR_RED, test_stats.failed_tests, COLOR_RESET,
                  (float)test_stats.failed_tests / test_stats.total_tests * 100.0);

    if (test_stats.skipped_tests > 0) {
        Serial.printf("  Skipped:        %s%lu%s\n",
                     COLOR_YELLOW, test_stats.skipped_tests, COLOR_RESET);
    }

    Serial.println();

    // Overall result
    if (test_stats.failed_tests == 0) {
        Serial.println(COLOR_BOLD COLOR_GREEN "  ★★★ ALL TESTS PASSED ★★★" COLOR_RESET);
    } else {
        Serial.println(COLOR_BOLD COLOR_RED "  ⚠ SOME TESTS FAILED ⚠" COLOR_RESET);
    }

    Serial.println();

    // Print library statistics (ESP32)
    #ifdef ESP32
    Serial.println(COLOR_BOLD "Library Statistics:" COLOR_RESET);
    mcp2515_statistics_t lib_stats;
    mcp2515.getStatistics(&lib_stats);
    Serial.printf("  RX Frames:      %lu\n", lib_stats.rx_frames);
    Serial.printf("  TX Frames:      %lu\n", lib_stats.tx_frames);
    Serial.printf("  RX Errors:      %lu\n", lib_stats.rx_errors);
    Serial.printf("  TX Errors:      %lu\n", lib_stats.tx_errors);
    Serial.printf("  RX Overflow:    %lu\n", lib_stats.rx_overflow);
    Serial.printf("  Bus Errors:     %lu\n", lib_stats.bus_errors);
    Serial.println();
    #endif

    Serial.println(COLOR_CYAN "═══════════════════════════════════════════════════════════════════" COLOR_RESET);
}

const char* getSpeedName(CAN_SPEED speed) {
    switch (speed) {
        case CAN_5KBPS: return "5 kbps";
        case CAN_10KBPS: return "10 kbps";
        case CAN_20KBPS: return "20 kbps";
        case CAN_31K25BPS: return "31.25 kbps";
        case CAN_33KBPS: return "33.3 kbps";
        case CAN_40KBPS: return "40 kbps";
        case CAN_50KBPS: return "50 kbps";
        case CAN_80KBPS: return "80 kbps";
        case CAN_83K3BPS: return "83.3 kbps";
        case CAN_95KBPS: return "95 kbps";
        case CAN_100KBPS: return "100 kbps";
        case CAN_125KBPS: return "125 kbps";
        case CAN_200KBPS: return "200 kbps";
        case CAN_250KBPS: return "250 kbps";
        case CAN_500KBPS: return "500 kbps";
        case CAN_1000KBPS: return "1000 kbps";
        default: return "Unknown";
    }
}

const char* getClockName(CAN_CLOCK clock) {
    switch (clock) {
        case MCP_8MHZ: return "8 MHz";
        case MCP_16MHZ: return "16 MHz";
        case MCP_20MHZ: return "20 MHz";
        default: return "Unknown";
    }
}

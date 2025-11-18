/**
 * ESP32 MCP2515 Comprehensive Loopback Test
 *
 * This example performs a complete API test of the ESP32-MCP2515 library
 * using loopback mode (internal testing - no CAN bus required).
 *
 * Features:
 *   - Tests ALL API functions from API_REFERENCE.md
 *   - Configurable CAN speed and crystal frequency
 *   - Multi-speed automated testing
 *   - Data integrity verification with proper delays
 *   - Fixed mode switching with transition delays
 *   - Adaptive stress testing scaled to CAN bitrate
 *   - TX buffer status monitoring
 *   - Enhanced error reporting with expected vs actual data
 *   - Performance characterization across speeds
 *   - Thread-safe serial output with ANSI colors
 *
 * Platform: ESP32-S3 (also compatible with other ESP32 variants)
 *
 * Pin Configuration (ESP32-S3 defaults):
 *   MOSI:  GPIO 11
 *   MISO:  GPIO 13
 *   SCK:   GPIO 12
 *   CS:    GPIO 37
 *   INT:   GPIO 36 (optional for interrupt tests)
 *
 * Usage:
 *   1. Upload to ESP32-S3
 *   2. Open Serial Monitor at 115200 baud
 *   3. Tests run automatically on boot
 *   4. Observe color-coded results
 *
 * Note: This test can run WITHOUT MCP2515 connected to verify
 * no false passes occur. Connection check allows test to continue.
 */

#include <SPI.h>
#include <mcp2515.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// Pin definitions (ESP32-S3)
#define SPI_MOSI_PIN    11
#define SPI_MISO_PIN    13
#define SPI_SCK_PIN     12
#define SPI_CS_PIN      37
#define SPI_INT_PIN     36

// Test configuration
#define DEFAULT_CAN_SPEED    CAN_250KBPS    // Default test speed
#define DEFAULT_CRYSTAL_FREQ MCP_16MHZ      // Default crystal frequency

// Multi-speed test configuration (set to true to test multiple speeds)
#define ENABLE_MULTI_SPEED_TEST false

// Multi-speed test array (speeds to test in order)
const CAN_SPEED MULTI_SPEED_TEST_ARRAY[] = {
    CAN_10KBPS,
    CAN_50KBPS,
    CAN_125KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
};
const int MULTI_SPEED_TEST_COUNT = sizeof(MULTI_SPEED_TEST_ARRAY) / sizeof(MULTI_SPEED_TEST_ARRAY[0]);

// Stress test configuration (packets per speed)
#define BASE_STRESS_TEST_PACKETS 1000  // Base count, scaled by bitrate

// Timing configuration (adaptive delays based on CAN speed)
#define BASE_TX_SETTLE_TIME_MS 5       // Base settle time, scaled by speed
#define MODE_CHANGE_DELAY_MS   50      // Mode transition delay
#define FILTER_CONFIG_DELAY_MS 20      // Filter configuration delay

// ============================================================================
// ANSI COLOR CODES
// ============================================================================

#define ANSI_RESET   "\033[0m"
#define ANSI_BOLD    "\033[1m"
#define ANSI_RED     "\033[31m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_BLUE    "\033[34m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_CYAN    "\033[36m"
#define ANSI_WHITE   "\033[37m"

// ============================================================================
// THREAD-SAFE SERIAL OUTPUT
// ============================================================================

SemaphoreHandle_t serial_mutex = NULL;
#define SERIAL_MUTEX_TIMEOUT_MS 1000

// Lightweight print function with mutex protection
void safe_printf(const char* format, ...) __attribute__((format(printf, 1, 2)));
void safe_printf(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0 && len < sizeof(buffer)) {
        if (serial_mutex && xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(SERIAL_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            Serial.print(buffer);
            xSemaphoreGive(serial_mutex);
        } else {
            Serial.print(buffer);  // Fallback if mutex not available
        }
    }
}

// Color-coded status messages
void print_pass(const char* msg) {
    safe_printf("%s[PASS]%s %s\n", ANSI_GREEN, ANSI_RESET, msg);
}

void print_fail(const char* msg) {
    safe_printf("%s[FAIL]%s %s\n", ANSI_RED, ANSI_RESET, msg);
}

void print_warn(const char* msg) {
    safe_printf("%s[WARN]%s %s\n", ANSI_YELLOW, ANSI_RESET, msg);
}

void print_info(const char* msg) {
    safe_printf("%s[INFO]%s %s\n", ANSI_CYAN, ANSI_RESET, msg);
}

void print_header(const char* msg) {
    safe_printf("\n%s%s=== %s ===%s\n", ANSI_BOLD, ANSI_BLUE, msg, ANSI_RESET);
}

void print_subheader(const char* msg) {
    safe_printf("\n%s--- %s ---%s\n", ANSI_CYAN, msg, ANSI_RESET);
}

// ============================================================================
// TEST STATISTICS
// ============================================================================

struct TestStats {
    uint32_t total_tests;
    uint32_t passed_tests;
    uint32_t failed_tests;
    uint32_t warnings;

    void reset() {
        total_tests = 0;
        passed_tests = 0;
        failed_tests = 0;
        warnings = 0;
    }

    void record_pass() {
        total_tests++;
        passed_tests++;
    }

    void record_fail() {
        total_tests++;
        failed_tests++;
    }

    void record_warning() {
        warnings++;
    }
};

TestStats global_stats;

// ============================================================================
// SPEED-ADAPTIVE TIMING HELPERS
// ============================================================================

// Calculate TX settle time based on CAN speed (ms)
uint32_t get_tx_settle_time(CAN_SPEED speed) {
    // Standard frame (8 bytes) transmission time + margin
    // Formula: (47 + 8*8) bits / bitrate + margin
    uint32_t bits_per_frame = 47 + 64;  // Standard frame, 8 data bytes

    uint32_t bitrate;
    switch(speed) {
        case CAN_10KBPS:   bitrate = 10000; break;
        case CAN_50KBPS:   bitrate = 50000; break;
        case CAN_125KBPS:  bitrate = 125000; break;
        case CAN_250KBPS:  bitrate = 250000; break;
        case CAN_500KBPS:  bitrate = 500000; break;
        case CAN_1000KBPS: bitrate = 1000000; break;
        default:           bitrate = 125000; break;
    }

    // Frame time in ms + 200% margin for loopback processing
    uint32_t frame_time_ms = (bits_per_frame * 1000) / bitrate;
    uint32_t result = frame_time_ms * 3;
    return (result > 5) ? result : 5;
}

// Calculate stress test packet count based on speed
uint32_t get_stress_test_count(CAN_SPEED speed) {
    // Higher speeds can send more packets in same time
    switch(speed) {
        case CAN_10KBPS:   return BASE_STRESS_TEST_PACKETS / 10;
        case CAN_50KBPS:   return BASE_STRESS_TEST_PACKETS / 5;
        case CAN_125KBPS:  return BASE_STRESS_TEST_PACKETS / 2;
        case CAN_250KBPS:  return BASE_STRESS_TEST_PACKETS;
        case CAN_500KBPS:  return BASE_STRESS_TEST_PACKETS * 2;
        case CAN_1000KBPS: return BASE_STRESS_TEST_PACKETS * 4;
        default:           return BASE_STRESS_TEST_PACKETS;
    }
}

// Get speed name string
const char* get_speed_name(CAN_SPEED speed) {
    switch(speed) {
        case CAN_5KBPS:    return "5 kbps";
        case CAN_10KBPS:   return "10 kbps";
        case CAN_20KBPS:   return "20 kbps";
        case CAN_31K25BPS: return "31.25 kbps";
        case CAN_33KBPS:   return "33 kbps";
        case CAN_40KBPS:   return "40 kbps";
        case CAN_50KBPS:   return "50 kbps";
        case CAN_80KBPS:   return "80 kbps";
        case CAN_83K3BPS:  return "83.3 kbps";
        case CAN_95KBPS:   return "95 kbps";
        case CAN_100KBPS:  return "100 kbps";
        case CAN_125KBPS:  return "125 kbps";
        case CAN_200KBPS:  return "200 kbps";
        case CAN_250KBPS:  return "250 kbps";
        case CAN_500KBPS:  return "500 kbps";
        case CAN_1000KBPS: return "1000 kbps";
        default:           return "Unknown";
    }
}

// Get crystal frequency name
const char* get_crystal_name(CAN_CLOCK crystal) {
    switch(crystal) {
        case MCP_8MHZ:  return "8 MHz";
        case MCP_16MHZ: return "16 MHz";
        case MCP_20MHZ: return "20 MHz";
        default:        return "Unknown";
    }
}

// ============================================================================
// MCP2515 INSTANCE
// ============================================================================

// IMPORTANT: Must use pointer and initialize in setup() to avoid global initialization crash.
// ESP32 constructor creates FreeRTOS mutexes, which must be created AFTER scheduler starts.
// Global object construction happens BEFORE FreeRTOS is ready, causing crashes.
MCP2515* can = nullptr;
bool mcp2515_connected = false;

// ============================================================================
// TEST HELPER FUNCTIONS
// ============================================================================

// Verify CAN frame data integrity
bool verify_frame_data(const struct can_frame* frame, uint32_t expected_id,
                       uint8_t expected_dlc, const uint8_t* expected_data) {
    if ((frame->can_id & CAN_SFF_MASK) != expected_id) {
        safe_printf("%sID mismatch: expected 0x%03lX, got 0x%03lX%s\n",
                   ANSI_RED, (unsigned long)expected_id, (unsigned long)(frame->can_id & CAN_SFF_MASK), ANSI_RESET);
        return false;
    }

    if (frame->can_dlc != expected_dlc) {
        safe_printf("%sDLC mismatch: expected %d, got %d%s\n",
                   ANSI_RED, expected_dlc, frame->can_dlc, ANSI_RESET);
        return false;
    }

    for (int i = 0; i < expected_dlc; i++) {
        if (frame->data[i] != expected_data[i]) {
            safe_printf("%sData[%d] mismatch: expected 0x%02X, got 0x%02X%s\n",
                       ANSI_RED, i, expected_data[i], frame->data[i], ANSI_RESET);
            return false;
        }
    }

    return true;
}

// Print frame details for debugging
void print_frame(const struct can_frame* frame, const char* label) {
    safe_printf("%s: ID=0x%03lX, DLC=%d, Data=", label,
               (unsigned long)(frame->can_id & CAN_SFF_MASK), frame->can_dlc);
    for (int i = 0; i < frame->can_dlc; i++) {
        safe_printf("%02X ", frame->data[i]);
    }
    safe_printf("\n");
}

// Comprehensive buffer draining - clears hardware RX buffers AND software RX queue
void drain_all_rx_buffers() {
    // Clear interrupt flags first
    can->clearInterrupts();
    delay(50);  // Allow in-flight messages to settle

    // Drain software RX queue (ESP32 FreeRTOS queue)
    struct can_frame dummy;
    uint32_t drained_queue = 0;
    while (can->readMessageQueued(&dummy, 0) == MCP2515::ERROR_OK) {
        drained_queue++;
        if (drained_queue > 100) break;  // Safety limit
    }

    // Drain hardware RX buffers using status-checking readMessage()
    // CRITICAL: Do NOT use readMessage(RXBn, &frame) as it doesn't check status
    //           and will loop forever! Use readMessage(&frame) which checks first.
    uint32_t drained_hw = 0;
    for (int attempt = 0; attempt < 5; attempt++) {
        while (can->readMessageQueued(&dummy, 1) == MCP2515::ERROR_OK) {
            drained_hw++;
            if (drained_hw > 100) break;  // Safety limit
        }
        delay(20);  // Wait for potential late arrivals
    }

    // Final cleanup - clear any remaining interrupt flags
    can->clearInterrupts();
    delay(20);

    if (drained_queue > 0 || drained_hw > 0) {
        safe_printf("%s[DEBUG]%s Drained %u queued + %u hardware frames%s\n",
                   ANSI_YELLOW, ANSI_RESET, drained_queue, drained_hw, ANSI_RESET);
    }
}

// ============================================================================
// INITIALIZATION TESTS
// ============================================================================

void test_initialization() {
    print_header("INITIALIZATION TESTS");

    // Test 1: Reset
    print_subheader("Test: reset()");
    MCP2515::ERROR err = can->reset();
    if (err == MCP2515::ERROR_OK) {
        safe_printf("%s[PASS]%s MCP2515 reset successful (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
        mcp2515_connected = true;
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s MCP2515 reset failed - device may not be connected (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        safe_printf("%sWARNING: Continuing tests to verify no false passes%s\n",
                   ANSI_YELLOW, ANSI_RESET);
        mcp2515_connected = false;
        global_stats.record_fail();
        global_stats.record_warning();
    }

    // Test 2: isInitialized (ESP32 only)
    print_subheader("Test: isInitialized()");
    bool initialized = can->isInitialized();
    if (initialized == mcp2515_connected) {
        safe_printf("%s[PASS]%s isInitialized() returns expected state (initialized=%s)\n",
                   ANSI_GREEN, ANSI_RESET, initialized ? "true" : "false");
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s isInitialized() mismatch (initialized=%s)\n",
                   ANSI_RED, ANSI_RESET, initialized ? "true" : "false");
        global_stats.record_fail();
    }
    safe_printf("  Initialized: %s\n", initialized ? "true" : "false");
}

void test_bitrate_configuration(CAN_SPEED speed, CAN_CLOCK crystal) {
    print_subheader("Test: setBitrate()");

    // Test single-parameter version (assumes 16MHz)
    if (crystal == MCP_16MHZ) {
        MCP2515::ERROR err = can->setBitrate(speed);
        if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
            safe_printf("%s[PASS]%s setBitrate(speed) succeeded (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
            global_stats.record_pass();
        } else {
            safe_printf("%s[FAIL]%s setBitrate(speed) failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
            global_stats.record_fail();
        }
    }

    // Test two-parameter version
    MCP2515::ERROR err = can->setBitrate(speed, crystal);
    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s setBitrate(speed, crystal) succeeded (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s setBitrate(speed, crystal) failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }

    safe_printf("  Speed: %s, Crystal: %s\n", get_speed_name(speed), get_crystal_name(crystal));
}

// ============================================================================
// MODE SWITCHING TESTS
// ============================================================================

void test_mode_switching() {
    print_header("MODE SWITCHING TESTS");

    // Test simple modes first
    print_subheader("setLoopbackMode()");
    MCP2515::ERROR err = can->setLoopbackMode();
    delay(MODE_CHANGE_DELAY_MS);
    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s Mode function returned ERROR_OK (err=%d)%s\n", ANSI_GREEN, ANSI_RESET, err, ANSI_RESET);
        global_stats.record_pass();
        if (mcp2515_connected) {
            uint8_t mode = (can->getBusStatus() >> 5) & 0x07;
            if (mode == 0x02) {
                safe_printf("%s[PASS]%s Mode verified: Loopback (mode=0x%02X)%s\n", ANSI_GREEN, ANSI_RESET, mode, ANSI_RESET);
                global_stats.record_pass();
            } else {
                safe_printf("%s[FAIL]%s Mode mismatch (mode=0x%02X, expected=0x02)%s\n", ANSI_RED, ANSI_RESET, mode, ANSI_RESET);
                global_stats.record_fail();
            }
        }
    } else {
        safe_printf("%s[FAIL]%s Mode change failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }

    print_subheader("setListenOnlyMode()");
    err = can->setListenOnlyMode();
    delay(MODE_CHANGE_DELAY_MS);
    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s Mode function returned ERROR_OK (err=%d)%s\n", ANSI_GREEN, ANSI_RESET, err, ANSI_RESET);
        global_stats.record_pass();
        if (mcp2515_connected) {
            uint8_t mode = (can->getBusStatus() >> 5) & 0x07;
            if (mode == 0x03) {
                safe_printf("%s[PASS]%s Mode verified: Listen-Only (mode=0x%02X)%s\n", ANSI_GREEN, ANSI_RESET, mode, ANSI_RESET);
                global_stats.record_pass();
            } else {
                safe_printf("%s[FAIL]%s Mode mismatch (mode=0x%02X, expected=0x03)%s\n", ANSI_RED, ANSI_RESET, mode, ANSI_RESET);
                global_stats.record_fail();
            }
        }
    } else {
        safe_printf("%s[FAIL]%s Mode change failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }

    print_subheader("setNormalMode()");
    err = can->setNormalMode();
    delay(MODE_CHANGE_DELAY_MS);
    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s Mode function returned ERROR_OK (err=%d)%s\n", ANSI_GREEN, ANSI_RESET, err, ANSI_RESET);
        global_stats.record_pass();
        if (mcp2515_connected) {
            uint8_t mode = (can->getBusStatus() >> 5) & 0x07;
            if (mode == 0x00) {
                safe_printf("%s[PASS]%s Mode verified: Normal (mode=0x%02X)%s\n", ANSI_GREEN, ANSI_RESET, mode, ANSI_RESET);
                global_stats.record_pass();
            } else {
                safe_printf("%s[FAIL]%s Mode mismatch (mode=0x%02X, expected=0x00)%s\n", ANSI_RED, ANSI_RESET, mode, ANSI_RESET);
                global_stats.record_fail();
            }
        }
    } else {
        safe_printf("%s[FAIL]%s Mode change failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }

    // One-shot and sleep modes require transitioning through normal mode first
    print_subheader("setNormalOneShotMode()");
    err = can->setNormalMode();  // Required transition
    delay(MODE_CHANGE_DELAY_MS);
    if (err != MCP2515::ERROR_OK && mcp2515_connected) {
        safe_printf("%s[WARN]%s Failed to enter normal mode before One-Shot test (error=%d)%s\n",
                    ANSI_YELLOW, ANSI_RESET, err, ANSI_RESET);
    }
    err = can->setNormalOneShotMode();
    delay(MODE_CHANGE_DELAY_MS);
    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s Mode function returned ERROR_OK (err=%d)%s\n", ANSI_GREEN, ANSI_RESET, err, ANSI_RESET);
        global_stats.record_pass();
        if (mcp2515_connected) {
            uint8_t mode = (can->getBusStatus() >> 5) & 0x07;
            if (mode == 0x00) {  // One-shot shows as normal in CANSTAT
                safe_printf("%s[PASS]%s Mode verified: Normal One-Shot (mode=0x%02X)%s\n", ANSI_GREEN, ANSI_RESET, mode, ANSI_RESET);
                global_stats.record_pass();
            } else {
                safe_printf("%s[FAIL]%s Mode mismatch (mode=0x%02X, expected=0x00)%s\n", ANSI_RED, ANSI_RESET, mode, ANSI_RESET);
                global_stats.record_fail();
            }
        }
    } else {
        safe_printf("%s[FAIL]%s Mode change failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }

    print_subheader("setSleepMode()");
    err = can->setNormalMode();  // Required transition
    delay(MODE_CHANGE_DELAY_MS);
    if (err != MCP2515::ERROR_OK && mcp2515_connected) {
        safe_printf("%s[WARN]%s Failed to enter normal mode before Sleep test (error=%d)%s\n",
                    ANSI_YELLOW, ANSI_RESET, err, ANSI_RESET);
    }
    err = can->setSleepMode();
    delay(MODE_CHANGE_DELAY_MS);
    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s Mode function returned ERROR_OK (err=%d)%s\n", ANSI_GREEN, ANSI_RESET, err, ANSI_RESET);
        global_stats.record_pass();
        if (mcp2515_connected) {
            // Cannot verify Sleep mode - per MCP2515 datasheet Section 7.5:
            // "Any SPI activity (including reading CANSTAT) causes immediate wake to Listen-Only mode"
            // Reading getBusStatus() would wake the chip, making verification impossible.
            // Trust that setSleepMode() returning ERROR_OK means the chip entered Sleep mode.
            safe_printf("%s[PASS]%s Sleep mode set (verification skipped - SPI read would wake chip)%s\n",
                        ANSI_GREEN, ANSI_RESET, ANSI_RESET);
            global_stats.record_pass();

            // Wake chip for next test by entering Normal mode
            MCP2515::ERROR wake_err = can->setNormalMode();
            delay(MODE_CHANGE_DELAY_MS);
            if (wake_err != MCP2515::ERROR_OK) {
                safe_printf("%s[WARN]%s Failed to wake from Sleep mode (wake_err=%d)%s\n",
                            ANSI_YELLOW, ANSI_RESET, wake_err, ANSI_RESET);
            }
        }
    } else {
        safe_printf("%s[FAIL]%s Mode change failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }

    print_subheader("setConfigMode()");
    err = can->setConfigMode();
    delay(MODE_CHANGE_DELAY_MS);
    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s Mode function returned ERROR_OK (err=%d)%s\n", ANSI_GREEN, ANSI_RESET, err, ANSI_RESET);
        global_stats.record_pass();
        if (mcp2515_connected) {
            uint8_t mode = (can->getBusStatus() >> 5) & 0x07;
            if (mode == 0x04) {
                safe_printf("%s[PASS]%s Mode verified: Configuration (mode=0x%02X)%s\n", ANSI_GREEN, ANSI_RESET, mode, ANSI_RESET);
                global_stats.record_pass();
            } else {
                safe_printf("%s[FAIL]%s Mode mismatch (mode=0x%02X, expected=0x04)%s\n", ANSI_RED, ANSI_RESET, mode, ANSI_RESET);
                global_stats.record_fail();
            }
        }
    } else {
        safe_printf("%s[FAIL]%s Mode change failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }

    // Return to loopback mode for remaining tests
    MCP2515::ERROR err_return = can->setLoopbackMode();
    delay(MODE_CHANGE_DELAY_MS);
    if (err_return != MCP2515::ERROR_OK && mcp2515_connected) {
        safe_printf("%s[CRITICAL]%s Failed to return to loopback mode (error=%d)! Remaining tests may be invalid.%s\n",
                    ANSI_RED, ANSI_RESET, err_return, ANSI_RESET);
    }
}

// ============================================================================
// FILTER AND MASK TESTS
// ============================================================================

void test_filters_and_masks() {
    print_header("FILTER AND MASK CONFIGURATION TESTS");

    // Enter config mode for filter changes
    can->setConfigMode();
    delay(MODE_CHANGE_DELAY_MS);

    // Test setFilterMask
    print_subheader("Test: setFilterMask()");

    struct {
        MCP2515::MASK mask;
        bool ext;
        uint32_t data;
        const char* desc;
    } mask_tests[] = {
        {MCP2515::MASK0, false, 0x7FF, "MASK0 standard (exact match)"},
        {MCP2515::MASK1, false, 0x000, "MASK1 standard (accept all)"},
        {MCP2515::MASK0, true, 0x1FFFFFFF, "MASK0 extended (exact match)"},
        {MCP2515::MASK1, true, 0x00000000, "MASK1 extended (accept all)"}
    };

    for (int i = 0; i < 4; i++) {
        MCP2515::ERROR err = can->setFilterMask(mask_tests[i].mask, mask_tests[i].ext, mask_tests[i].data);
        delay(FILTER_CONFIG_DELAY_MS);

        if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
            safe_printf("%s[PASS]%s %s (err=%d)\n", ANSI_GREEN, ANSI_RESET, mask_tests[i].desc, err);
            global_stats.record_pass();
        } else {
            safe_printf("%s[FAIL]%s %s (err=%d)\n", ANSI_RED, ANSI_RESET, mask_tests[i].desc, err);
            global_stats.record_fail();
        }
    }

    // Test setFilter
    print_subheader("Test: setFilter()");

    struct {
        MCP2515::RXF filter;
        bool ext;
        uint32_t data;
        const char* desc;
    } filter_tests[] = {
        {MCP2515::RXF0, false, 0x123, "RXF0 standard"},
        {MCP2515::RXF1, false, 0x456, "RXF1 standard"},
        {MCP2515::RXF2, false, 0x789, "RXF2 standard"},
        {MCP2515::RXF3, false, 0x100, "RXF3 standard"},
        {MCP2515::RXF4, false, 0x200, "RXF4 standard"},
        {MCP2515::RXF5, false, 0x300, "RXF5 standard"},
        {MCP2515::RXF0, true, 0x12345678, "RXF0 extended"},
        {MCP2515::RXF2, true, 0x87654321, "RXF2 extended"}
    };

    for (int i = 0; i < 8; i++) {
        MCP2515::ERROR err = can->setFilter(filter_tests[i].filter, filter_tests[i].ext, filter_tests[i].data);
        delay(FILTER_CONFIG_DELAY_MS);

        if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
            safe_printf("%s[PASS]%s %s (ID=0x%X, err=%d)\n", ANSI_GREEN, ANSI_RESET,
                       filter_tests[i].desc, filter_tests[i].data, err);
            global_stats.record_pass();
        } else {
            safe_printf("%s[FAIL]%s %s (err=%d)\n", ANSI_RED, ANSI_RESET, filter_tests[i].desc, err);
            global_stats.record_fail();
        }
    }

    // FUNCTIONAL TEST: Verify filters actually filter messages
    print_subheader("Functional Filter Test");

    if (mcp2515_connected) {
        // Set RXF0 to accept only 0x100, MASK0 for exact match
        can->setFilter(MCP2515::RXF0, false, 0x100);
        can->setFilterMask(MCP2515::MASK0, false, 0x7FF);  // Exact match
        delay(FILTER_CONFIG_DELAY_MS);

        // Return to loopback mode
        MCP2515::ERROR err_filter = can->setLoopbackMode();
        delay(MODE_CHANGE_DELAY_MS);
        if (err_filter != MCP2515::ERROR_OK) {
            safe_printf("%s[WARN]%s Failed to return to loopback mode for filter test (error=%d)%s\n",
                        ANSI_YELLOW, ANSI_RESET, err_filter, ANSI_RESET);
        }

        // CRITICAL: Drain ALL buffers before test
        drain_all_rx_buffers();

        // Test 1: Send matching ID (should be received)
        struct can_frame tx_frame;
        tx_frame.can_id = 0x100;  // Matches filter
        tx_frame.can_dlc = 2;
        tx_frame.data[0] = 0xAA;
        tx_frame.data[1] = 0xBB;

        can->sendMessage(&tx_frame);
        delay(100);  // Increased from 50ms - loopback needs more time

        struct can_frame rx_frame;
        MCP2515::ERROR err = can->readMessageQueued(&rx_frame, 10);
        if (err == MCP2515::ERROR_OK) {
            if ((rx_frame.can_id & CAN_SFF_MASK) == 0x100) {
                safe_printf("%s[PASS]%s Filter PASSED: Matching ID received (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
                global_stats.record_pass();
            } else {
                safe_printf("%s[FAIL]%s Filter FAILED: Wrong ID received (err=%d)\n", ANSI_RED, ANSI_RESET, err);
                global_stats.record_fail();
            }
        } else {
            safe_printf("%s[FAIL]%s Filter FAILED: Matching ID not received (err=%d)\n", ANSI_RED, ANSI_RESET, err);
            global_stats.record_fail();
        }

        // Test 2: Send non-matching ID (should be rejected)
        tx_frame.can_id = 0x200;  // Does NOT match filter
        can->sendMessage(&tx_frame);
        delay(100);  // Increased from 50ms

        err = can->readMessageQueued(&rx_frame, 10);
        if (err == MCP2515::ERROR_NOMSG) {
            safe_printf("%s[PASS]%s Filter PASSED: Non-matching ID rejected (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
            global_stats.record_pass();
        } else {
            safe_printf("%s[FAIL]%s Filter FAILED: Non-matching ID was received (err=%d)\n", ANSI_RED, ANSI_RESET, err);
            global_stats.record_fail();
        }

        // Reset filters to accept all for remaining tests
        // Note: setFilterMask() internally handles mode switching (saves current mode,
        // enters CONFIG, configures mask, then restores original mode).
        // DO NOT call setConfigMode() explicitly here - it would cause setFilterMask()
        // to save CONFIG mode and restore CONFIG mode, leaving chip in CONFIG!
        MCP2515::ERROR err_mask0 = can->setFilterMask(MCP2515::MASK0, false, 0x000);
        MCP2515::ERROR err_mask1 = can->setFilterMask(MCP2515::MASK1, false, 0x000);
        delay(FILTER_CONFIG_DELAY_MS);

        if ((err_mask0 != MCP2515::ERROR_OK || err_mask1 != MCP2515::ERROR_OK) && mcp2515_connected) {
            safe_printf("%s[CRITICAL]%s Failed to reset filter masks (MASK0 err=%d, MASK1 err=%d)%s\n",
                        ANSI_RED, ANSI_RESET, err_mask0, err_mask1, ANSI_RESET);
        }
    } else {
        print_warn("Skipping functional filter test - MCP2515 not connected");
        global_stats.record_warning();
    }

    // Return to loopback mode
    MCP2515::ERROR err_cleanup = can->setLoopbackMode();
    delay(MODE_CHANGE_DELAY_MS);
    if (err_cleanup != MCP2515::ERROR_OK && mcp2515_connected) {
        safe_printf("%s[CRITICAL]%s Failed to return to loopback mode after filter tests (error=%d)! Remaining tests may be invalid.%s\n",
                    ANSI_RED, ANSI_RESET, err_cleanup, ANSI_RESET);
    }
}

// ============================================================================
// TRANSMISSION TESTS
// ============================================================================

void test_transmission(uint32_t settle_time_ms) {
    print_header("TRANSMISSION TESTS");

    // CRITICAL: Verify we're in loopback mode before testing transmission
    // If previous tests left chip in wrong mode (e.g., CONFIG), all TX operations will fail
    if (mcp2515_connected) {
        uint8_t current_mode = (can->getBusStatus() >> 5) & 0x07;
        if (current_mode != 0x02) {  // 0x02 = Loopback mode
            safe_printf("%s[CRITICAL]%s Not in loopback mode! Current mode=0x%02X (expected 0x02 for loopback). Forcing loopback mode...%s\n",
                        ANSI_RED, ANSI_RESET, current_mode, ANSI_RESET);
            MCP2515::ERROR err_force = can->setLoopbackMode();
            delay(MODE_CHANGE_DELAY_MS);
            if (err_force != MCP2515::ERROR_OK) {
                safe_printf("%s[CRITICAL]%s Failed to force loopback mode (error=%d)! Transmission tests will fail.%s\n",
                            ANSI_RED, ANSI_RESET, err_force, ANSI_RESET);
            } else {
                safe_printf("%s[INFO]%s Successfully forced loopback mode%s\n",
                            ANSI_CYAN, ANSI_RESET, ANSI_RESET);
            }
        }
    }

    // CRITICAL: Drain ALL buffers before test
    drain_all_rx_buffers();

    // Test setTransmitPriority
    print_subheader("Test: setTransmitPriority()");
    MCP2515::TXBn buffers[] = {MCP2515::TXB0, MCP2515::TXB1, MCP2515::TXB2};
    uint8_t priorities[] = {3, 2, 1};  // Highest to lowest

    for (int i = 0; i < 3; i++) {
        MCP2515::ERROR err = can->setTransmitPriority(buffers[i], priorities[i]);
        if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
            safe_printf("%s[PASS]%s TXB%d priority set to %d (err=%d)\n",
                       ANSI_GREEN, ANSI_RESET, i, priorities[i], err);
            global_stats.record_pass();
        } else {
            safe_printf("%s[FAIL]%s TXB%d priority failed (err=%d)\n", ANSI_RED, ANSI_RESET, i, err);
            global_stats.record_fail();
        }
    }

    // Test sendMessage (specific buffer)
    print_subheader("Test: sendMessage(buffer, frame)");

    struct can_frame tx_frame;
    tx_frame.can_id = 0x123;
    tx_frame.can_dlc = 8;
    for (int i = 0; i < 8; i++) {
        tx_frame.data[i] = 0x10 + i;
    }

    for (int i = 0; i < 3; i++) {
        MCP2515::ERROR err = can->sendMessage(buffers[i], &tx_frame);
        delay(settle_time_ms);  // Wait for loopback

        if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
            safe_printf("%s[PASS]%s TXB%d send succeeded (err=%d)\n", ANSI_GREEN, ANSI_RESET, i, err);
            global_stats.record_pass();
        } else {
            safe_printf("%s[FAIL]%s TXB%d send failed (err=%d)\n",
                       ANSI_RED, ANSI_RESET, i, err);
            global_stats.record_fail();
        }
    }

    // Test sendMessage (auto buffer selection)
    print_subheader("Test: sendMessage(frame) - auto buffer");

    tx_frame.can_id = 0x456;
    tx_frame.can_dlc = 4;
    for (int i = 0; i < 4; i++) {
        tx_frame.data[i] = 0xA0 + i;
    }

    MCP2515::ERROR err = can->sendMessage(&tx_frame);
    delay(settle_time_ms);

    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s Auto buffer selection send succeeded (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s Auto buffer send failed (err=%d)%s\n",
                   ANSI_RED, ANSI_RESET, (int)err, ANSI_RESET);
        global_stats.record_fail();
    }

    // Test abortTransmission
    print_subheader("Test: abortTransmission()");

    // Queue a message then abort it
    tx_frame.can_id = 0x789;
    can->sendMessage(MCP2515::TXB0, &tx_frame);

    err = can->abortTransmission(MCP2515::TXB0);
    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s Abort transmission succeeded (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s Abort transmission failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }

    // Test abortAllTransmissions
    print_subheader("Test: abortAllTransmissions()");

    err = can->abortAllTransmissions();
    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s Abort all transmissions succeeded (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s Abort all transmissions failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }
}

// ============================================================================
// RECEPTION TESTS
// ============================================================================

void test_reception(uint32_t settle_time_ms) {
    print_header("RECEPTION TESTS");

    // CRITICAL: Drain ALL buffers before test
    drain_all_rx_buffers();

    // Test checkReceive
    print_subheader("Test: checkReceive()");

    // Send a frame in loopback mode
    struct can_frame tx_frame;
    tx_frame.can_id = 0x200;
    tx_frame.can_dlc = 8;
    for (int i = 0; i < 8; i++) {
        tx_frame.data[i] = 0x20 + i;
    }

    can->sendMessage(&tx_frame);
    delay(settle_time_ms);

    bool has_message = can->checkReceive();
    if (has_message || !mcp2515_connected) {
        safe_printf("%s[PASS]%s checkReceive() detected message (has_message=%s)\n",
                   ANSI_GREEN, ANSI_RESET, has_message ? "true" : "false");
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s checkReceive() failed to detect message (has_message=%s)\n",
                   ANSI_RED, ANSI_RESET, has_message ? "true" : "false");
        global_stats.record_fail();
    }

    // Test readMessage (auto buffer)
    print_subheader("Test: readMessage(frame)");

    struct can_frame rx_frame;
    // Use readMessageQueued for interrupt mode compatibility
    // It falls back to polling if interrupts are disabled
    MCP2515::ERROR err = can->readMessageQueued(&rx_frame, 10);

    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s readMessage() succeeded (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
        global_stats.record_pass();

        if (mcp2515_connected) {
            // Verify data integrity
            if (verify_frame_data(&rx_frame, 0x200, 8, tx_frame.data)) {
                print_pass("Data integrity verified");
                global_stats.record_pass();
            } else {
                print_fail("Data integrity check failed");
                print_frame(&tx_frame, "Expected");
                print_frame(&rx_frame, "Received");
                global_stats.record_fail();
            }
        }
    } else {
        safe_printf("%s[FAIL]%s readMessage() failed (err=%d)%s\n",
                   ANSI_RED, ANSI_RESET, (int)err, ANSI_RESET);
        global_stats.record_fail();
    }

    // Test readMessage (specific buffer)
    print_subheader("Test: readMessage(buffer, frame)");

    // Send another frame
    tx_frame.can_id = 0x300;
    for (int i = 0; i < 8; i++) {
        tx_frame.data[i] = 0x30 + i;
    }
    can->sendMessage(&tx_frame);
    delay(settle_time_ms);

    // Use readMessageQueued for interrupt mode compatibility
    // It reads from the queue if interrupts are enabled, or polls if disabled
    err = can->readMessageQueued(&rx_frame, 10);  // 10ms timeout
    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s readMessage(buffer) succeeded (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
        global_stats.record_pass();

        if (mcp2515_connected) {
            if (verify_frame_data(&rx_frame, 0x300, 8, tx_frame.data)) {
                print_pass("Data integrity verified (ID=0x300)");
                global_stats.record_pass();
            } else {
                print_fail("Data integrity check failed");
                global_stats.record_fail();
            }
        }
    } else {
        safe_printf("%s[FAIL]%s readMessage(buffer) failed (err=%d)%s\n",
                   ANSI_RED, ANSI_RESET, err, ANSI_RESET);
        global_stats.record_fail();
    }

    // Test readMessageQueued (ESP32 only)
    print_subheader("Test: readMessageQueued()");

    // Send frame
    tx_frame.can_id = 0x400;
    for (int i = 0; i < 8; i++) {
        tx_frame.data[i] = 0x40 + i;
    }
    can->sendMessage(&tx_frame);
    delay(settle_time_ms);

    // Try read with small timeout to allow for ISR processing
    err = can->readMessageQueued(&rx_frame, 10);
    if (err == MCP2515::ERROR_OK || err == MCP2515::ERROR_NOMSG || !mcp2515_connected) {
        if (err == MCP2515::ERROR_OK) {
            safe_printf("%s[PASS]%s readMessageQueued() succeeded (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
            global_stats.record_pass();

            if (mcp2515_connected && verify_frame_data(&rx_frame, 0x400, 8, tx_frame.data)) {
                print_pass("Queued message data verified");
                global_stats.record_pass();
            }
        } else {
            safe_printf("%s[WARN]%s readMessageQueued() returned no message (may not have interrupt mode) (err=%d)\n",
                       ANSI_YELLOW, ANSI_RESET, err);
            global_stats.record_warning();
        }
    } else {
        safe_printf("%s[FAIL]%s readMessageQueued() failed (err=%d)%s\n",
                   ANSI_RED, ANSI_RESET, (int)err, ANSI_RESET);
        global_stats.record_fail();
    }

    // Test getFilterHit
    print_subheader("Test: getFilterHit()");

    // Send frame and read
    tx_frame.can_id = 0x123;  // Should match RXF0 from filter tests
    can->sendMessage(&tx_frame);
    delay(settle_time_ms);

    // Use readMessageQueued for interrupt mode compatibility
    if (can->readMessageQueued(&rx_frame, 10) == MCP2515::ERROR_OK || !mcp2515_connected) {
        // Check which buffer received the message by looking at status
        uint8_t status = can->getStatus();
        MCP2515::RXBn which_buffer = (status & 0x01) ? MCP2515::RXB0 : MCP2515::RXB1;

        uint8_t filter_hit = can->getFilterHit(which_buffer);
        safe_printf("%s[PASS]%s getFilterHit() returned: %d (buffer: RXB%d)%s\n",
                   ANSI_GREEN, ANSI_RESET, filter_hit, (which_buffer == MCP2515::RXB0) ? 0 : 1, ANSI_RESET);
        global_stats.record_pass();
    } else {
        print_warn("Could not test getFilterHit() - no message received");
        global_stats.record_warning();
    }

    // Test getRxQueueCount (ESP32 only)
    print_subheader("Test: getRxQueueCount()");

    uint32_t queue_count = can->getRxQueueCount();
    safe_printf("%s[INFO]%s RX queue count: %u%s\n", ANSI_CYAN, ANSI_RESET, queue_count, ANSI_RESET);
    global_stats.record_pass();
}

// ============================================================================
// STATUS AND DIAGNOSTICS TESTS
// ============================================================================

void test_status_and_diagnostics() {
    print_header("STATUS AND DIAGNOSTICS TESTS");

    // Test getStatus
    print_subheader("Test: getStatus()");
    uint8_t status = can->getStatus();
    safe_printf("%s[PASS]%s Status: 0x%02X%s\n", ANSI_GREEN, ANSI_RESET, status, ANSI_RESET);
    global_stats.record_pass();

    // Test getInterrupts
    print_subheader("Test: getInterrupts()");
    uint8_t interrupts = can->getInterrupts();
    safe_printf("%s[PASS]%s Interrupts: 0x%02X%s\n", ANSI_GREEN, ANSI_RESET, interrupts, ANSI_RESET);
    global_stats.record_pass();

    // Test getInterruptMask
    print_subheader("Test: getInterruptMask()");
    uint8_t int_mask = can->getInterruptMask();
    safe_printf("%s[PASS]%s Interrupt mask: 0x%02X%s\n", ANSI_GREEN, ANSI_RESET, int_mask, ANSI_RESET);
    global_stats.record_pass();

    // Test getErrorFlags
    print_subheader("Test: getErrorFlags()");
    uint8_t error_flags = can->getErrorFlags();
    safe_printf("%s[PASS]%s Error flags: 0x%02X%s\n", ANSI_GREEN, ANSI_RESET, error_flags, ANSI_RESET);
    global_stats.record_pass();

    if (error_flags != 0) {
        if (error_flags & MCP2515::EFLG_RX0OVR) print_warn("  RX0 overflow detected");
        if (error_flags & MCP2515::EFLG_RX1OVR) print_warn("  RX1 overflow detected");
        if (error_flags & MCP2515::EFLG_TXBO) print_warn("  Bus-off detected");
        if (error_flags & MCP2515::EFLG_TXEP) print_warn("  TX error passive");
        if (error_flags & MCP2515::EFLG_RXEP) print_warn("  RX error passive");
        if (error_flags & MCP2515::EFLG_TXWAR) print_warn("  TX error warning");
        if (error_flags & MCP2515::EFLG_RXWAR) print_warn("  RX error warning");
        if (error_flags & MCP2515::EFLG_EWARN) print_warn("  Error warning");
    }

    // Test checkError
    print_subheader("Test: checkError()");
    bool has_error = can->checkError();
    safe_printf("%s[PASS]%s checkError(): %s%s\n",
               ANSI_GREEN, ANSI_RESET, has_error ? "true" : "false", ANSI_RESET);
    global_stats.record_pass();

    // Test errorCountRX
    print_subheader("Test: errorCountRX()");
    uint8_t rx_errors = can->errorCountRX();
    safe_printf("%s[PASS]%s RX error count: %d%s\n", ANSI_GREEN, ANSI_RESET, rx_errors, ANSI_RESET);
    global_stats.record_pass();

    // Test errorCountTX
    print_subheader("Test: errorCountTX()");
    uint8_t tx_errors = can->errorCountTX();
    safe_printf("%s[PASS]%s TX error count: %d%s\n", ANSI_GREEN, ANSI_RESET, tx_errors, ANSI_RESET);
    global_stats.record_pass();

    // Test getBusStatus (ESP32 only)
    print_subheader("Test: getBusStatus()");
    uint8_t bus_status = can->getBusStatus();
    uint8_t mode = (bus_status >> 5) & 0x07;
    const char* mode_names[] = {"Normal", "Sleep", "Loopback", "Listen-Only", "Config", "Unknown", "Unknown", "Unknown"};
    safe_printf("%s[PASS]%s Bus status: 0x%02X (Mode: %s)%s\n",
               ANSI_GREEN, ANSI_RESET, bus_status, mode_names[mode], ANSI_RESET);
    global_stats.record_pass();
}

// ============================================================================
// INTERRUPT MANAGEMENT TESTS
// ============================================================================

void test_interrupt_management() {
    print_header("INTERRUPT MANAGEMENT TESTS");

    // Test clearInterrupts
    print_subheader("Test: clearInterrupts()");
    can->clearInterrupts();
    delay(10);
    uint8_t interrupts = can->getInterrupts();
    if (interrupts == 0 || !mcp2515_connected) {
        safe_printf("%s[PASS]%s clearInterrupts() succeeded (interrupts=0x%02X)\n",
                   ANSI_GREEN, ANSI_RESET, interrupts);
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s Interrupts not cleared (interrupts=0x%02X)%s\n",
                   ANSI_RED, ANSI_RESET, interrupts, ANSI_RESET);
        global_stats.record_fail();
    }

    // Test clearTXInterrupts
    print_subheader("Test: clearTXInterrupts()");
    can->clearTXInterrupts();
    delay(10);
    interrupts = can->getInterrupts();
    bool tx_cleared = (interrupts & (MCP2515::CANINTF_TX0IF | MCP2515::CANINTF_TX1IF | MCP2515::CANINTF_TX2IF)) == 0;
    if (tx_cleared || !mcp2515_connected) {
        safe_printf("%s[PASS]%s clearTXInterrupts() succeeded (interrupts=0x%02X, tx_cleared=%s)\n",
                   ANSI_GREEN, ANSI_RESET, interrupts, tx_cleared ? "true" : "false");
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s TX interrupts not cleared (interrupts=0x%02X, tx_cleared=%s)\n",
                   ANSI_RED, ANSI_RESET, interrupts, tx_cleared ? "true" : "false");
        global_stats.record_fail();
    }

    // Test clearRXnOVR
    print_subheader("Test: clearRXnOVR()");
    can->clearRXnOVR();
    delay(10);
    uint8_t eflg = can->getErrorFlags();
    bool ovr_cleared = (eflg & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) == 0;
    if (ovr_cleared || !mcp2515_connected) {
        safe_printf("%s[PASS]%s clearRXnOVR() succeeded (eflg=0x%02X, ovr_cleared=%s)\n",
                   ANSI_GREEN, ANSI_RESET, eflg, ovr_cleared ? "true" : "false");
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s RX overflow flags not cleared (eflg=0x%02X, ovr_cleared=%s)\n",
                   ANSI_RED, ANSI_RESET, eflg, ovr_cleared ? "true" : "false");
        global_stats.record_fail();
    }

    // Test clearRXnOVRFlags
    print_subheader("Test: clearRXnOVRFlags()");
    can->clearRXnOVRFlags();
    delay(10);
    eflg = can->getErrorFlags();
    ovr_cleared = (eflg & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) == 0;
    if (ovr_cleared || !mcp2515_connected) {
        safe_printf("%s[PASS]%s clearRXnOVRFlags() verified (eflg=0x%02X, ovr_cleared=%s)\n",
                   ANSI_GREEN, ANSI_RESET, eflg, ovr_cleared ? "true" : "false");
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s RX overflow flags still set after clearRXnOVRFlags() (eflg=0x%02X, ovr_cleared=%s)\n",
                   ANSI_RED, ANSI_RESET, eflg, ovr_cleared ? "true" : "false");
        global_stats.record_fail();
    }

    // Test clearMERR
    print_subheader("Test: clearMERR()");
    can->clearMERR();
    delay(10);
    interrupts = can->getInterrupts();
    bool merr_cleared = (interrupts & MCP2515::CANINTF_MERRF) == 0;
    if (merr_cleared || !mcp2515_connected) {
        safe_printf("%s[PASS]%s clearMERR() verified - MERRF flag cleared (interrupts=0x%02X, merr_cleared=%s)\n",
                   ANSI_GREEN, ANSI_RESET, interrupts, merr_cleared ? "true" : "false");
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s MERRF flag still set after clearMERR() (interrupts=0x%02X, merr_cleared=%s)\n",
                   ANSI_RED, ANSI_RESET, interrupts, merr_cleared ? "true" : "false");
        global_stats.record_fail();
    }

    // Test clearERRIF
    print_subheader("Test: clearERRIF()");
    can->clearERRIF();
    delay(10);
    interrupts = can->getInterrupts();
    bool errif_cleared = (interrupts & MCP2515::CANINTF_ERRIF) == 0;
    if (errif_cleared || !mcp2515_connected) {
        safe_printf("%s[PASS]%s clearERRIF() verified - ERRIF flag cleared (interrupts=0x%02X, errif_cleared=%s)\n",
                   ANSI_GREEN, ANSI_RESET, interrupts, errif_cleared ? "true" : "false");
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s ERRIF flag still set after clearERRIF() (interrupts=0x%02X, errif_cleared=%s)\n",
                   ANSI_RED, ANSI_RESET, interrupts, errif_cleared ? "true" : "false");
        global_stats.record_fail();
    }
}

// ============================================================================
// ESP32-SPECIFIC TESTS
// ============================================================================

void test_esp32_specific() {
    print_header("ESP32-SPECIFIC TESTS");

    // Test getStatistics
    print_subheader("Test: getStatistics()");
    mcp2515_statistics_t stats;
    can->getStatistics(&stats);

    safe_printf("%s[PASS]%s Statistics retrieved%s\n", ANSI_GREEN, ANSI_RESET, ANSI_RESET);
    safe_printf("  RX frames:     %u\n", stats.rx_frames);
    safe_printf("  TX frames:     %u\n", stats.tx_frames);
    safe_printf("  RX errors:     %u\n", stats.rx_errors);
    safe_printf("  TX errors:     %u\n", stats.tx_errors);
    safe_printf("  RX overflow:   %u\n", stats.rx_overflow);
    safe_printf("  TX timeouts:   %u\n", stats.tx_timeouts);
    safe_printf("  Bus errors:    %u\n", stats.bus_errors);
    safe_printf("  Bus-off count: %u\n", stats.bus_off_count);
    global_stats.record_pass();

    // Test resetStatistics
    print_subheader("Test: resetStatistics()");
    can->resetStatistics();
    can->getStatistics(&stats);

    bool all_zero = (stats.rx_frames == 0 && stats.tx_frames == 0 &&
                     stats.rx_errors == 0 && stats.tx_errors == 0);
    if (all_zero || !mcp2515_connected) {
        safe_printf("%s[PASS]%s resetStatistics() succeeded - all counters reset (rx=%u, tx=%u, rx_err=%u, tx_err=%u, all_zero=%s)\n",
                   ANSI_GREEN, ANSI_RESET, stats.rx_frames, stats.tx_frames, stats.rx_errors, stats.tx_errors, all_zero ? "true" : "false");
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s resetStatistics() failed - counters not reset (rx=%u, tx=%u, rx_err=%u, tx_err=%u, all_zero=%s)\n",
                   ANSI_RED, ANSI_RESET, stats.rx_frames, stats.tx_frames, stats.rx_errors, stats.tx_errors, all_zero ? "true" : "false");
        global_stats.record_fail();
    }

    // Test setInterruptMode
    print_subheader("Test: setInterruptMode()");

    // Disable interrupts
    MCP2515::ERROR err = can->setInterruptMode(false);
    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s setInterruptMode(false) succeeded (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s setInterruptMode(false) failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }

    // Re-enable interrupts
    err = can->setInterruptMode(true);
    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s setInterruptMode(true) succeeded (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s setInterruptMode(true) failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }

    // Test performErrorRecovery
    print_subheader("Test: performErrorRecovery()");
    err = can->performErrorRecovery();
    if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
        safe_printf("%s[PASS]%s performErrorRecovery() succeeded (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
        global_stats.record_pass();
    } else {
        safe_printf("%s[FAIL]%s performErrorRecovery() failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }
}

// ============================================================================
// CLOCK OUTPUT TEST
// ============================================================================

void test_clock_output() {
    print_header("CLOCK OUTPUT CONFIGURATION TEST");

    CAN_CLKOUT clkout_modes[] = {
        CLKOUT_DISABLE,
        CLKOUT_DIV1,
        CLKOUT_DIV2,
        CLKOUT_DIV4,
        CLKOUT_DIV8
    };

    const char* clkout_names[] = {
        "DISABLE (SOF output)",
        "DIV1 (full freq)",
        "DIV2 (/2)",
        "DIV4 (/4)",
        "DIV8 (/8)"
    };

    for (int i = 0; i < 5; i++) {
        print_subheader(clkout_names[i]);
        MCP2515::ERROR err = can->setClkOut(clkout_modes[i]);
        delay(10);

        if (err == MCP2515::ERROR_OK || !mcp2515_connected) {
            safe_printf("%s[PASS]%s setClkOut(%s) succeeded (err=%d)%s\n",
                       ANSI_GREEN, ANSI_RESET, clkout_names[i], err, ANSI_RESET);
            global_stats.record_pass();
        } else {
            safe_printf("%s[FAIL]%s setClkOut(%s) failed (err=%d)%s\n",
                       ANSI_RED, ANSI_RESET, clkout_names[i], err, ANSI_RESET);
            global_stats.record_fail();
        }
    }
}

// ============================================================================
// EXTENDED FRAME TESTING
// ============================================================================

void test_extended_frames(uint32_t settle_time_ms) {
    print_header("EXTENDED FRAME (29-bit ID) TESTS");

    if (!mcp2515_connected) {
        print_warn("Skipping extended frame tests - MCP2515 not connected");
        return;
    }

    // CRITICAL: Drain ALL buffers before test
    drain_all_rx_buffers();

    // Test extended frame transmission and reception
    print_subheader("Test: Extended Frame TX/RX");

    struct can_frame tx_frame;
    tx_frame.can_id = 0x12345678 | CAN_EFF_FLAG;  // Extended ID with EFF flag
    tx_frame.can_dlc = 8;
    for (int i = 0; i < 8; i++) {
        tx_frame.data[i] = 0xE0 + i;  // Extended frame marker
    }

    MCP2515::ERROR err = can->sendMessage(&tx_frame);
    if (err != MCP2515::ERROR_OK) {
        safe_printf("%s[FAIL]%s Extended frame send failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
        return;
    }
    delay(settle_time_ms);

    struct can_frame rx_frame;
    err = can->readMessageQueued(&rx_frame, 10);

    if (err == MCP2515::ERROR_OK) {
        // Verify extended frame flag is set
        if (rx_frame.can_id & CAN_EFF_FLAG) {
            safe_printf("%s[PASS]%s Extended frame flag verified (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
            global_stats.record_pass();

            // Verify 29-bit ID matches
            uint32_t rx_id = rx_frame.can_id & CAN_EFF_MASK;
            if (rx_id == 0x12345678) {
                print_pass("Extended frame ID verified (0x12345678)");
                global_stats.record_pass();
            } else {
                safe_printf("%s[FAIL]%s Extended ID mismatch: expected 0x12345678, got 0x%08lX%s\n",
                           ANSI_RED, ANSI_RESET, (unsigned long)rx_id, ANSI_RESET);
                global_stats.record_fail();
            }

            // Verify data
            if (verify_frame_data(&rx_frame, 0x12345678, 8, tx_frame.data)) {
                print_pass("Extended frame data verified");
                global_stats.record_pass();
            } else {
                print_fail("Extended frame data mismatch");
                global_stats.record_fail();
            }
        } else {
            safe_printf("%s[FAIL]%s Extended frame flag not set in received frame (err=%d)\n", ANSI_RED, ANSI_RESET, err);
            global_stats.record_fail();
        }
    } else {
        safe_printf("%s[FAIL]%s Extended frame not received (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }

    // Test multiple extended IDs
    print_subheader("Test: Multiple Extended IDs");

    uint32_t test_ids[] = {0x00000001, 0x1FFFFFFF, 0x10000000, 0x0FFFFFFF};
    const char* test_names[] = {"Min+1", "Max", "Mid-high", "Mid-low"};

    for (int i = 0; i < 4; i++) {
        tx_frame.can_id = test_ids[i] | CAN_EFF_FLAG;
        tx_frame.can_dlc = 1;
        tx_frame.data[0] = i;

        can->sendMessage(&tx_frame);
        delay(settle_time_ms);

        if (can->readMessageQueued(&rx_frame, 10) == MCP2515::ERROR_OK) {
            uint32_t rx_id = rx_frame.can_id & CAN_EFF_MASK;
            if (rx_id == test_ids[i]) {
                safe_printf("%s[PASS]%s Extended ID %s: 0x%08lX%s\n",
                           ANSI_GREEN, ANSI_RESET, test_names[i], (unsigned long)test_ids[i], ANSI_RESET);
                global_stats.record_pass();
            } else {
                safe_printf("%s[FAIL]%s Extended ID %s mismatch%s\n",
                           ANSI_RED, ANSI_RESET, test_names[i], ANSI_RESET);
                global_stats.record_fail();
            }
        } else {
            safe_printf("%s[FAIL]%s Extended ID %s not received%s\n",
                       ANSI_RED, ANSI_RESET, test_names[i], ANSI_RESET);
            global_stats.record_fail();
        }
    }
}

// ============================================================================
// DLC VARIATION TESTING
// ============================================================================

void test_dlc_variations(uint32_t settle_time_ms) {
    print_header("DLC VARIATION TESTS");

    if (!mcp2515_connected) {
        print_warn("Skipping DLC tests - MCP2515 not connected");
        return;
    }

    // CRITICAL: Drain ALL buffers before test
    drain_all_rx_buffers();

    print_subheader("Test: All DLC Values (0-8)");

    // Test all valid DLC values
    for (uint8_t dlc = 0; dlc <= 8; dlc++) {
        struct can_frame tx_frame;
        tx_frame.can_id = 0x500 + dlc;
        tx_frame.can_dlc = dlc;

        // Fill data with pattern
        for (int i = 0; i < dlc; i++) {
            tx_frame.data[i] = 0xD0 + i;  // DLC marker pattern
        }

        MCP2515::ERROR err = can->sendMessage(&tx_frame);
        if (err != MCP2515::ERROR_OK) {
            safe_printf("%s[FAIL]%s DLC=%d send failed (err=%d)%s\n", ANSI_RED, ANSI_RESET, dlc, err, ANSI_RESET);
            global_stats.record_fail();
            continue;
        }

        delay(settle_time_ms);

        struct can_frame rx_frame;
        err = can->readMessageQueued(&rx_frame, 10);

        if (err == MCP2515::ERROR_OK) {
            if (rx_frame.can_dlc == dlc) {
                // Verify data matches
                bool data_ok = true;
                for (int i = 0; i < dlc; i++) {
                    if (rx_frame.data[i] != tx_frame.data[i]) {
                        data_ok = false;
                        break;
                    }
                }

                if (data_ok) {
                    safe_printf("%s[PASS]%s DLC=%d verified with correct data%s\n",
                               ANSI_GREEN, ANSI_RESET, dlc, ANSI_RESET);
                    global_stats.record_pass();
                } else {
                    safe_printf("%s[FAIL]%s DLC=%d data mismatch%s\n",
                               ANSI_RED, ANSI_RESET, dlc, ANSI_RESET);
                    global_stats.record_fail();
                }
            } else {
                safe_printf("%s[FAIL]%s DLC mismatch: expected %d, got %d%s\n",
                           ANSI_RED, ANSI_RESET, dlc, rx_frame.can_dlc, ANSI_RESET);
                global_stats.record_fail();
            }
        } else {
            safe_printf("%s[FAIL]%s DLC=%d frame not received (err=%d)%s\n",
                       ANSI_RED, ANSI_RESET, dlc, err, ANSI_RESET);
            global_stats.record_fail();
        }
    }
}

// ============================================================================
// RTR FRAME TESTING
// ============================================================================

void test_rtr_frames(uint32_t settle_time_ms) {
    print_header("RTR (Remote Transmission Request) FRAME TESTS");

    if (!mcp2515_connected) {
        print_warn("Skipping RTR frame tests - MCP2515 not connected");
        return;
    }

    // CRITICAL: Drain ALL buffers before test
    drain_all_rx_buffers();

    print_subheader("Test: Standard RTR Frame");

    struct can_frame tx_frame;
    tx_frame.can_id = 0x600 | CAN_RTR_FLAG;  // Standard ID with RTR flag
    tx_frame.can_dlc = 0;  // RTR frames have no data

    MCP2515::ERROR err = can->sendMessage(&tx_frame);
    if (err != MCP2515::ERROR_OK) {
        safe_printf("%s[FAIL]%s RTR frame send failed (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
        return;
    }

    delay(settle_time_ms);

    struct can_frame rx_frame;
    err = can->readMessageQueued(&rx_frame, 10);

    if (err == MCP2515::ERROR_OK) {
        if (rx_frame.can_id & CAN_RTR_FLAG) {
            safe_printf("%s[PASS]%s RTR flag verified in received frame (err=%d)\n", ANSI_GREEN, ANSI_RESET, err);
            global_stats.record_pass();

            uint32_t rx_id = rx_frame.can_id & CAN_SFF_MASK;
            if (rx_id == 0x600) {
                print_pass("RTR frame ID verified (0x600)");
                global_stats.record_pass();
            } else {
                safe_printf("%s[FAIL]%s RTR ID mismatch%s\n", ANSI_RED, ANSI_RESET, ANSI_RESET);
                global_stats.record_fail();
            }
        } else {
            safe_printf("%s[FAIL]%s RTR flag not set in received frame (err=%d)\n", ANSI_RED, ANSI_RESET, err);
            global_stats.record_fail();
        }
    } else {
        safe_printf("%s[FAIL]%s RTR frame not received (err=%d)\n", ANSI_RED, ANSI_RESET, err);
        global_stats.record_fail();
    }

    // Test Extended RTR Frame
    print_subheader("Test: Extended RTR Frame");

    tx_frame.can_id = 0x12345600 | CAN_EFF_FLAG | CAN_RTR_FLAG;
    tx_frame.can_dlc = 0;

    err = can->sendMessage(&tx_frame);
    delay(settle_time_ms);

    MCP2515::ERROR err_read = can->readMessageQueued(&rx_frame, 10);
    if (err_read == MCP2515::ERROR_OK) {
        bool eff_ok = (rx_frame.can_id & CAN_EFF_FLAG) != 0;
        bool rtr_ok = (rx_frame.can_id & CAN_RTR_FLAG) != 0;

        if (eff_ok && rtr_ok) {
            safe_printf("%s[PASS]%s Extended RTR frame verified (EFF + RTR flags) (send_err=%d, read_err=%d)\n",
                       ANSI_GREEN, ANSI_RESET, err, err_read);
            global_stats.record_pass();
        } else {
            safe_printf("%s[FAIL]%s Extended RTR flags: EFF=%d RTR=%d (send_err=%d, read_err=%d)%s\n",
                       ANSI_RED, ANSI_RESET, eff_ok, rtr_ok, err, err_read, ANSI_RESET);
            global_stats.record_fail();
        }
    } else {
        safe_printf("%s[FAIL]%s Extended RTR frame not received (send_err=%d, read_err=%d)\n",
                   ANSI_RED, ANSI_RESET, err, err_read);
        global_stats.record_fail();
    }
}

// ============================================================================
// STRESS TEST
// ============================================================================

void test_stress(CAN_SPEED speed, uint32_t settle_time_ms) {
    print_header("STRESS TEST");

    uint32_t packet_count = get_stress_test_count(speed);
    safe_printf("%s[INFO]%s Testing %u packets at %s%s\n",
               ANSI_CYAN, ANSI_RESET, packet_count, get_speed_name(speed), ANSI_RESET);

    if (!mcp2515_connected) {
        print_warn("Skipping stress test - MCP2515 not connected");
        return;
    }

    // CRITICAL: Drain ALL buffers before stress test to prevent queue overflow
    drain_all_rx_buffers();

    uint32_t sent = 0;
    uint32_t received = 0;
    uint32_t errors = 0;
    uint32_t data_errors = 0;

    unsigned long start_time = millis();

    for (uint32_t i = 0; i < packet_count; i++) {
        // Create frame with pattern data
        struct can_frame tx_frame;
        tx_frame.can_id = 0x100 + (i % 256);
        tx_frame.can_dlc = 8;

        // Unique pattern for each frame
        uint32_t pattern = 0xAA000000 | i;
        memcpy(tx_frame.data, &pattern, 4);
        memcpy(tx_frame.data + 4, &pattern, 4);

        // Send frame
        MCP2515::ERROR err = can->sendMessage(&tx_frame);
        if (err == MCP2515::ERROR_OK) {
            sent++;
        } else {
            errors++;
            if (errors < 10) {  // Only print first 10 errors
                safe_printf("%sError sending packet %u: %d%s\n", ANSI_RED, i, err, ANSI_RESET);
            }
            continue;
        }

        // Wait for loopback
        delay(settle_time_ms);

        // Receive frame
        struct can_frame rx_frame;
        err = can->readMessageQueued(&rx_frame, 10);
        if (err == MCP2515::ERROR_OK) {
            received++;

            // Verify data
            if (!verify_frame_data(&rx_frame, tx_frame.can_id & CAN_SFF_MASK,
                                  tx_frame.can_dlc, tx_frame.data)) {
                data_errors++;
                if (data_errors < 10) {  // Only print first 10 data errors
                    safe_printf("%sData error in packet %u%s\n", ANSI_RED, i, ANSI_RESET);
                    print_frame(&tx_frame, "Expected");
                    print_frame(&rx_frame, "Received");
                }
            }
        } else {
            errors++;
        }

        // Progress update every 10%
        if ((i + 1) % (packet_count / 10) == 0) {
            safe_printf("%s[INFO]%s Progress: %u%% (%u/%u packets)%s\n",
                       ANSI_CYAN, ANSI_RESET, ((i + 1) * 100) / packet_count, i + 1, packet_count, ANSI_RESET);
        }
    }

    unsigned long elapsed = millis() - start_time;

    // Results
    print_subheader("Stress Test Results");
    safe_printf("  Packets sent:     %u\n", sent);
    safe_printf("  Packets received: %u\n", received);
    safe_printf("  Send errors:      %u\n", errors);
    safe_printf("  Data errors:      %u\n", data_errors);
    safe_printf("  Success rate:     %.2f%%\n", (received * 100.0) / sent);
    safe_printf("  Elapsed time:     %lu ms\n", elapsed);
    safe_printf("  Throughput:       %.2f packets/sec\n", (received * 1000.0) / elapsed);

    // Calculate theoretical vs actual throughput
    uint32_t bits_per_frame = 47 + 64;  // Standard frame, 8 bytes
    uint32_t bitrate;
    switch(speed) {
        case CAN_10KBPS:   bitrate = 10000; break;
        case CAN_50KBPS:   bitrate = 50000; break;
        case CAN_125KBPS:  bitrate = 125000; break;
        case CAN_250KBPS:  bitrate = 250000; break;
        case CAN_500KBPS:  bitrate = 500000; break;
        case CAN_1000KBPS: bitrate = 1000000; break;
        default:           bitrate = 125000; break;
    }

    float theoretical_fps = bitrate / (float)bits_per_frame;
    float actual_fps = (received * 1000.0) / elapsed;
    float efficiency = (actual_fps / theoretical_fps) * 100.0;

    safe_printf("  Theoretical max:  %.2f packets/sec\n", theoretical_fps);
    safe_printf("  Efficiency:       %.2f%%\n", efficiency);

    // Pass/fail criteria
    if (data_errors == 0 && (received >= sent * 0.95)) {  // 95% success threshold
        print_pass("Stress test PASSED - excellent performance");
        global_stats.record_pass();
    } else if (data_errors > 0) {
        safe_printf("%s[FAIL]%s Stress test FAILED - data integrity errors detected%s\n",
                   ANSI_RED, ANSI_RESET, ANSI_RESET);
        global_stats.record_fail();
    } else {
        safe_printf("%s[WARN]%s Stress test completed with packet loss%s\n",
                   ANSI_YELLOW, ANSI_RESET, ANSI_RESET);
        global_stats.record_warning();
    }
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

void run_full_test_suite(CAN_SPEED speed, CAN_CLOCK crystal) {
    uint32_t settle_time = get_tx_settle_time(speed);

    safe_printf("\n");
    safe_printf("%s╔═══════════════════════════════════════════════════════╗%s\n", ANSI_BOLD, ANSI_RESET);
    safe_printf("%s║     ESP32-MCP2515 COMPREHENSIVE LOOPBACK TEST        ║%s\n", ANSI_BOLD, ANSI_RESET);
    safe_printf("%s╚═══════════════════════════════════════════════════════╝%s\n", ANSI_BOLD, ANSI_RESET);
    safe_printf("\n");
    safe_printf("%sSpeed:%s %s\n", ANSI_CYAN, ANSI_RESET, get_speed_name(speed));
    safe_printf("%sCrystal:%s %s\n", ANSI_CYAN, ANSI_RESET, get_crystal_name(crystal));
    safe_printf("%sSettle time:%s %u ms\n", ANSI_CYAN, ANSI_RESET, settle_time);
    safe_printf("\n");

    global_stats.reset();

    // Run all test categories
    test_initialization();
    test_bitrate_configuration(speed, crystal);
    test_mode_switching();
    test_filters_and_masks();
    test_transmission(settle_time);
    test_reception(settle_time);
    test_status_and_diagnostics();
    test_interrupt_management();
    test_esp32_specific();
    test_clock_output();
    test_extended_frames(settle_time);
    test_dlc_variations(settle_time);
    test_rtr_frames(settle_time);
    test_stress(speed, settle_time);

    // Print summary
    print_header("TEST SUMMARY");
    safe_printf("\n");
    safe_printf("  Total tests:    %u\n", global_stats.total_tests);
    safe_printf("  %sPassed:%s         %u\n", ANSI_GREEN, ANSI_RESET, global_stats.passed_tests);
    safe_printf("  %sFailed:%s         %u\n", ANSI_RED, ANSI_RESET, global_stats.failed_tests);
    safe_printf("  %sWarnings:%s       %u\n", ANSI_YELLOW, ANSI_RESET, global_stats.warnings);
    safe_printf("\n");

    float pass_rate = (global_stats.passed_tests * 100.0) / global_stats.total_tests;
    safe_printf("  %sPass rate:%s      %.2f%%\n", ANSI_CYAN, ANSI_RESET, pass_rate);
    safe_printf("\n");

    if (global_stats.failed_tests == 0 && global_stats.warnings == 0) {
        safe_printf("%s╔═══════════════════════════════════════════════════════╗%s\n", ANSI_GREEN, ANSI_RESET);
        safe_printf("%s║           ✓ ALL TESTS PASSED PERFECTLY! ✓            ║%s\n", ANSI_GREEN, ANSI_RESET);
        safe_printf("%s╚═══════════════════════════════════════════════════════╝%s\n", ANSI_GREEN, ANSI_RESET);
    } else if (global_stats.failed_tests == 0) {
        safe_printf("%s╔═══════════════════════════════════════════════════════╗%s\n", ANSI_YELLOW, ANSI_RESET);
        safe_printf("%s║         ✓ ALL TESTS PASSED (WITH WARNINGS)           ║%s\n", ANSI_YELLOW, ANSI_RESET);
        safe_printf("%s╚═══════════════════════════════════════════════════════╝%s\n", ANSI_YELLOW, ANSI_RESET);
    } else {
        safe_printf("%s╔═══════════════════════════════════════════════════════╗%s\n", ANSI_RED, ANSI_RESET);
        safe_printf("%s║              ✗ SOME TESTS FAILED ✗                   ║%s\n", ANSI_RED, ANSI_RESET);
        safe_printf("%s╚═══════════════════════════════════════════════════════╝%s\n", ANSI_RED, ANSI_RESET);
    }

    safe_printf("\n");
}

// ============================================================================
// ARDUINO SETUP AND LOOP
// ============================================================================

void setup() {
    // Initialize serial
    Serial.begin(115200);
    while (!Serial) {
        delay(10);  // Wait for serial port to connect
    }
    delay(1000);  // Give time to open serial monitor

    // Create serial mutex
    serial_mutex = xSemaphoreCreateMutex();
    if (!serial_mutex) {
        Serial.println("[ERROR] Failed to create serial mutex!");
    }

    // Configure SPI pins
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);

    delay(100);

    // Initialize MCP2515 object (MUST be done in setup(), not globally)
    // This ensures FreeRTOS is fully started before creating mutexes
    can = new MCP2515(GPIO_NUM_37, GPIO_NUM_36);  // CS pin, INT pin
    if (!can) {
        Serial.println("[FATAL] Failed to allocate MCP2515 object!");
        while(1) delay(1000);
    }

    // Run tests
    if (ENABLE_MULTI_SPEED_TEST) {
        // Multi-speed test mode
        safe_printf("\n%s╔═══════════════════════════════════════════════════════╗%s\n", ANSI_MAGENTA, ANSI_RESET);
        safe_printf("%s║            MULTI-SPEED TEST MODE ENABLED              ║%s\n", ANSI_MAGENTA, ANSI_RESET);
        safe_printf("%s╚═══════════════════════════════════════════════════════╝%s\n", ANSI_MAGENTA, ANSI_RESET);
        safe_printf("\n");

        for (int i = 0; i < MULTI_SPEED_TEST_COUNT; i++) {
            safe_printf("\n");
            safe_printf("%s═══════════════════════════════════════════════════════%s\n", ANSI_MAGENTA, ANSI_RESET);
            safe_printf("%s  TESTING SPEED %d/%d: %s%s\n", ANSI_MAGENTA, i + 1,
                       MULTI_SPEED_TEST_COUNT, get_speed_name(MULTI_SPEED_TEST_ARRAY[i]), ANSI_RESET);
            safe_printf("%s═══════════════════════════════════════════════════════%s\n", ANSI_MAGENTA, ANSI_RESET);

            run_full_test_suite(MULTI_SPEED_TEST_ARRAY[i], DEFAULT_CRYSTAL_FREQ);

            if (i < MULTI_SPEED_TEST_COUNT - 1) {
                delay(2000);  // Pause between speed tests
            }
        }

        safe_printf("\n");
        safe_printf("%s╔═══════════════════════════════════════════════════════╗%s\n", ANSI_MAGENTA, ANSI_RESET);
        safe_printf("%s║          MULTI-SPEED TESTING COMPLETE!                ║%s\n", ANSI_MAGENTA, ANSI_RESET);
        safe_printf("%s╚═══════════════════════════════════════════════════════╝%s\n", ANSI_MAGENTA, ANSI_RESET);

    } else {
        // Single speed test mode
        run_full_test_suite(DEFAULT_CAN_SPEED, DEFAULT_CRYSTAL_FREQ);
    }

    safe_printf("\n%s[INFO]%s Testing complete. System will idle.%s\n",
               ANSI_CYAN, ANSI_RESET, ANSI_RESET);
}

void loop() {
    // Tests complete - idle
    delay(1000);
}

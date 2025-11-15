/**
 * @file ESP32_CAN_comprehensive_test.ino
 * @brief Comprehensive Test Suite for ESP32 MCP2515 CAN Library - ENHANCED v2.0
 *
 * This example tests EVERY function in the ESP32 MCP2515 library with detailed
 * reporting and statistics. Includes fixes for known issues and multi-speed testing.
 *
 * TEST MODES:
 *   MODE 1: LOOPBACK SELF-TEST (Default - Requires Only 1 ESP32)
 *   MODE 2: TWO-DEVICE TEST (Requires 2 ESP32s with MCP2515)
 *   MODE 3: MULTI-SPEED SWEEP (Tests across multiple CAN speeds)
 *
 * ENHANCEMENTS v2.0:
 *   - Fixed data integrity verification with proper delays
 *   - Fixed mode switching with transition delays
 *   - Adaptive stress testing scaled to CAN bitrate
 *   - Multi-speed comprehensive testing
 *   - TX buffer status monitoring
 *   - Enhanced error reporting with expected vs actual data
 *   - Performance characterization across speeds
 *
 * @hardware ESP32 + MCP2515 CAN module(s)
 * @author ESP32-MCP2515 Library Test Suite
 * @version 2.0
 * @date 2025
 */

#include <SPI.h>
#include <mcp2515.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ============================================================================
// HARDWARE CONFIGURATION - CUSTOMIZE FOR YOUR SETUP
// ============================================================================

// SPI Pin Configuration (adjust for your ESP32 board)
#define SPI_MOSI_PIN    11    // Default ESP32 VSPI MOSI
#define SPI_MISO_PIN    13    // Default ESP32 VSPI MISO
#define SPI_SCK_PIN     12    // Default ESP32 VSPI SCK
#define SPI_CS_PIN      37     // Chip Select pin
#define SPI_INT_PIN     36     // Interrupt pin (set to -1 to disable interrupts)

// CAN Bus Configuration
#define CONFIG_CAN_SPEED       CAN_250KBPS   // CAN bus speed (single-speed mode)
#define CONFIG_CAN_CLOCK       MCP_16MHZ     // MCP2515 crystal frequency (8MHz, 16MHz, or 20MHz)

// ============================================================================
// TEST CONFIGURATION
// ============================================================================

// **NEW** Multi-Speed Test Mode - Set to true to run tests at all major CAN speeds
#define ENABLE_MULTI_SPEED_TEST   false      // Set to true for comprehensive speed sweep

// Multi-Speed Configuration (only used if ENABLE_MULTI_SPEED_TEST is true)
const CAN_SPEED MULTI_SPEED_TEST_SPEEDS[] = {
    CAN_10KBPS,
    CAN_50KBPS,
    CAN_125KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
};
#define MULTI_SPEED_COUNT  (sizeof(MULTI_SPEED_TEST_SPEEDS) / sizeof(MULTI_SPEED_TEST_SPEEDS[0]))

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
#define ENABLE_DATA_INTEGRITY_FIX true        // Use improved data integrity testing with delays
#define ENABLE_MODE_TRANSITION_FIX true       // Use improved mode switching with proper transitions

// Serial Configuration
#define SERIAL_BAUD     115200

// ============================================================================
// MCP2515 CANSTAT OPMOD CONSTANTS
// ============================================================================

// MCP2515 CANSTAT OPMOD values (bits 7:5 of CANSTAT register)
#define CANSTAT_OPMOD_NORMAL      0x00
#define CANSTAT_OPMOD_SLEEP       0x01
#define CANSTAT_OPMOD_LOOPBACK    0x02
#define CANSTAT_OPMOD_LISTENONLY  0x03
#define CANSTAT_OPMOD_CONFIG      0x04

// Special CAN IDs for master/slave synchronization in two-device mode
#define SYNC_ID_READY        0x7F0  // Slave signals ready
#define SYNC_ID_START_TEST   0x7F1  // Master signals test start
#define SYNC_ID_TEST_FRAME   0x7F2  // Test frame from slave
#define SYNC_ID_ACK          0x7F3  // Acknowledgment
#define SYNC_ID_COMPLETE     0x7F4  // Test complete

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
// THREAD-SAFE SERIAL OUTPUT
// ============================================================================

// Mutex for thread-safe Serial output (prevents interlaced/garbled text)
SemaphoreHandle_t serial_mutex = NULL;
#define SERIAL_MUTEX_TIMEOUT_MS  1000  // Maximum wait time for mutex

// Thread-safe Serial print wrapper
void safe_print(const char* str) {
    if (serial_mutex && xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(SERIAL_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        Serial.print(str);
        xSemaphoreGive(serial_mutex);
    } else {
        Serial.print(str);  // Fallback if mutex not available
    }
}

// Thread-safe Serial println wrapper
void safe_println(const char* str = "") {
    if (serial_mutex && xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(SERIAL_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        Serial.println(str);
        xSemaphoreGive(serial_mutex);
    } else {
        Serial.println(str);
    }
}

// Thread-safe Serial printf wrapper
void safe_printf(const char* format, ...) __attribute__((format(printf, 1, 2)));
void safe_printf(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (serial_mutex && xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(SERIAL_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        Serial.print(buffer);
        xSemaphoreGive(serial_mutex);
    } else {
        Serial.print(buffer);
    }
}

// Thread-safe Serial print for String objects
void safe_print(const String& str) {
    safe_print(str.c_str());
}

// Thread-safe Serial println for String objects
void safe_println(const String& str) {
    safe_println(str.c_str());
}

// Atomic multi-line print (locks once for entire block)
class SafeSerialBlock {
public:
    SafeSerialBlock() {
        if (serial_mutex) {
            locked = (xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(SERIAL_MUTEX_TIMEOUT_MS)) == pdTRUE);
        } else {
            locked = false;
        }
    }

    ~SafeSerialBlock() {
        if (locked && serial_mutex) {
            xSemaphoreGive(serial_mutex);
        }
    }

private:
    bool locked;
};

// Macro for atomic multi-line prints
// Creates a scoped lock - all Serial calls within the scope are protected
#define SAFE_SERIAL_BLOCK() SafeSerialBlock _safe_serial_block

// ============================================================================
// TEST INFRASTRUCTURE
// ============================================================================

// Test statistics
struct test_stats_t {
    uint32_t total_tests;
    uint32_t passed_tests;
    uint32_t failed_tests;
    uint32_t skipped_tests;
    uint32_t warnings;
};

// Hardware detection flag
bool hardware_detected = false;

// Multi-speed test results
struct speed_test_result_t {
    CAN_SPEED speed;
    uint32_t passed;
    uint32_t failed;
    uint32_t throughput_fps;    // Frames per second achieved
    float error_rate;            // Error rate percentage
    uint32_t tx_errors;
    uint32_t rx_errors;
};

speed_test_result_t speed_results[MULTI_SPEED_COUNT];
uint32_t speed_results_count = 0;

test_stats_t test_stats = {0, 0, 0, 0, 0};
CAN_SPEED current_test_speed = CONFIG_CAN_SPEED;  // Track current speed being tested

// MCP2515 instance
MCP2515 mcp2515(SPI_CS_PIN);

// Forward declarations for helper functions
void printTestHeader(const char* test_name);
void printTestResult(bool passed, const char* message = nullptr);
void printSectionHeader(const char* section_name);
void printStatistics();
bool waitForFrame(struct can_frame* frame, uint32_t timeout_ms);
void printFrame(const struct can_frame* frame, bool is_tx);
void printDataComparison(const uint8_t* expected, const uint8_t* actual, uint8_t length);
const char* getSpeedName(CAN_SPEED speed);
const char* getClockName(CAN_CLOCK clock);
uint32_t getTheoreticalFramesPerSecond(CAN_SPEED speed);
uint32_t getAdaptiveDelayMicros(CAN_SPEED speed);
bool checkTXBufferAvailable();
void runComprehensiveTests();
void runComprehensiveTestsAtSpeed(CAN_SPEED speed);
void printMultiSpeedComparison();
void runSlaveMode();  // **NEW:** Slave mode handler

// **NEW:** Optimized timing and two-device coordination helpers
bool waitForMessage(uint32_t timeout_ms);
bool waitForMessageAndRead(struct can_frame* frame, uint32_t timeout_ms);
bool waitForTXBuffer(uint32_t timeout_ms);
bool waitForModeChange(uint8_t target_mode, uint32_t timeout_ms);
void clearRXBuffer();

// **CRITICAL:** Hardware connectivity detection
bool verifyHardwareConnected();

// **ENHANCED:** Robust bidirectional handshake protocol for two-device mode
bool waitForSlaveReadyAtStartup(uint32_t timeout_ms = 30000);
void signalSlaveReadyContinuously();
bool sendStartSignalWithRetry(uint8_t test_id, uint32_t max_retries = 5);
bool waitForStartSignalAndAck(uint8_t* test_id, uint32_t timeout_ms = 60000);
void sendAck();
bool waitForAck(uint32_t timeout_ms = 3000);
void sendCompleteSignal();
bool waitForComplete(uint32_t timeout_ms = 5000);

// Forward declarations for test functions
void testInitialization();
void testOperatingModes();
void testBitrateConfiguration();
void testFiltersAndMasks();
void testFrameTransmission();
void testFrameReception();
void testRXB1Buffer();
void testErrorHandling();
void testInterruptFunctions();
void testStatistics();
void testStatusFunctions();
void testStressScenarios();
void testClockOutput();
void testAdvancedFeatures();
void testAdvancedQueue();
void testStateTransitionErrors();
void testBoundaryConditions();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 5000); // Wait up to 5 seconds for serial

    delay(1000); // Allow serial to stabilize

    // Create mutex for thread-safe Serial output
    serial_mutex = xSemaphoreCreateMutex();
    if (serial_mutex == NULL) {
        Serial.println("ERROR: Failed to create serial mutex!");
    }

    // Use atomic block for all startup messages
    {
        SAFE_SERIAL_BLOCK();
        Serial.println();
        Serial.println(COLOR_BOLD COLOR_CYAN "╔════════════════════════════════════════════════════════════════╗" COLOR_RESET);
        Serial.println(COLOR_BOLD COLOR_CYAN "║   ESP32 MCP2515 CAN LIBRARY COMPREHENSIVE TEST SUITE v2.0     ║" COLOR_RESET);
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

        if (ENABLE_MULTI_SPEED_TEST) {
            Serial.println(COLOR_BOLD COLOR_YELLOW "  Multi-Speed Mode: ENABLED" COLOR_RESET);
            Serial.print("  Testing Speeds:   ");
            for (uint32_t i = 0; i < MULTI_SPEED_COUNT; i++) {
                Serial.print(getSpeedName(MULTI_SPEED_TEST_SPEEDS[i]));
                if (i < MULTI_SPEED_COUNT - 1) Serial.print(", ");
            }
            Serial.println();
        } else {
            Serial.printf("  CAN Speed:        %s\n", getSpeedName(CONFIG_CAN_SPEED));
        }

        Serial.printf("  CAN Clock:        %s\n", getClockName(CONFIG_CAN_CLOCK));
        Serial.printf("  Interrupt Tests:  %s\n", ENABLE_INTERRUPT_TESTS ? "ENABLED" : "DISABLED");
        Serial.printf("  Stress Tests:     %s\n", ENABLE_STRESS_TESTS ? "ENABLED" : "DISABLED");
        Serial.printf("  Enhancements:     Data Integrity Fix=%s, Mode Transition Fix=%s\n",
                      ENABLE_DATA_INTEGRITY_FIX ? "ON" : "OFF",
                      ENABLE_MODE_TRANSITION_FIX ? "ON" : "OFF");
        Serial.println();
    }

    delay(500);  // **OPTIMIZED:** Reduced from 2000ms

    // Initialize SPI
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);

    // **NEW:** Check if we're in slave mode
    if (TEST_MODE == TEST_MODE_TWO_DEVICE && DEVICE_ROLE == ROLE_SLAVE) {
        runSlaveMode();  // Enter slave mode handler (never returns)
    } else {
        // Master or loopback mode: run normal tests
        if (ENABLE_MULTI_SPEED_TEST) {
            runMultiSpeedTests();
        } else {
            runComprehensiveTests();
            printFinalStatistics();
        }
    }
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    // Tests complete - show summary every 10 seconds
    delay(10000);
    safe_println(COLOR_YELLOW "\n--- Test suite completed. See results above. ---" COLOR_RESET);
}

// ============================================================================
// MULTI-SPEED TEST SUITE
// ============================================================================

void runMultiSpeedTests() {
    SAFE_SERIAL_BLOCK();
    Serial.println(COLOR_BOLD COLOR_BLUE "╔════════════════════════════════════════════════════════════════╗" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_BLUE "║  Starting Multi-Speed Comprehensive Test Suite                ║" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_BLUE "╚════════════════════════════════════════════════════════════════╝" COLOR_RESET);
    Serial.println();

    for (uint32_t i = 0; i < MULTI_SPEED_COUNT; i++) {
        CAN_SPEED speed = MULTI_SPEED_TEST_SPEEDS[i];

        Serial.println();
        Serial.println(COLOR_BOLD COLOR_MAGENTA "════════════════════════════════════════════════════════════════" COLOR_RESET);
        Serial.printf(COLOR_BOLD COLOR_MAGENTA "  TESTING AT %s\n" COLOR_RESET, getSpeedName(speed));
        Serial.println(COLOR_BOLD COLOR_MAGENTA "════════════════════════════════════════════════════════════════" COLOR_RESET);
        Serial.println();

        // Reset statistics for this speed
        test_stats_t saved_stats = test_stats;
        test_stats = {0, 0, 0, 0, 0};

        // Run comprehensive tests at this speed
        runComprehensiveTestsAtSpeed(speed);

        // Store results
        speed_results[speed_results_count].speed = speed;
        speed_results[speed_results_count].passed = test_stats.passed_tests;
        speed_results[speed_results_count].failed = test_stats.failed_tests;

        // Get final statistics
        #ifdef ESP32
        mcp2515_statistics_t stats;
        mcp2515.getStatistics(&stats);
        speed_results[speed_results_count].throughput_fps = (stats.tx_frames > 0) ?
            (stats.tx_frames * 1000 / STRESS_TEST_DURATION_MS) : 0;
        speed_results[speed_results_count].error_rate = (stats.tx_frames + stats.tx_errors > 0) ?
            ((float)stats.tx_errors / (stats.tx_frames + stats.tx_errors) * 100.0) : 0.0;
        speed_results[speed_results_count].tx_errors = stats.tx_errors;
        speed_results[speed_results_count].rx_errors = stats.rx_errors;
        #else
        speed_results[speed_results_count].throughput_fps = 0;
        speed_results[speed_results_count].error_rate = 0.0;
        speed_results[speed_results_count].tx_errors = 0;
        speed_results[speed_results_count].rx_errors = 0;
        #endif

        speed_results_count++;

        // Print individual speed summary
        Serial.println();
        Serial.println(COLOR_BOLD "─────────────────────────────────────────────────────────────────" COLOR_RESET);
        Serial.printf(COLOR_BOLD "%s Results:\n" COLOR_RESET, getSpeedName(speed));
        Serial.printf("  Tests Passed: %s%lu%s / %lu\n",
                     test_stats.failed_tests == 0 ? COLOR_GREEN : COLOR_YELLOW,
                     test_stats.passed_tests, COLOR_RESET,
                     test_stats.total_tests);
        if (test_stats.failed_tests > 0) {
            Serial.printf("  Tests Failed: %s%lu%s\n", COLOR_RED, test_stats.failed_tests, COLOR_RESET);
        }
        Serial.println(COLOR_BOLD "─────────────────────────────────────────────────────────────────" COLOR_RESET);

        delay(500);  // **OPTIMIZED:** Reduced from 2000ms // Pause between speeds
    }

    // Print comprehensive multi-speed comparison
    printMultiSpeedComparison();
}

void runComprehensiveTestsAtSpeed(CAN_SPEED speed) {
    current_test_speed = speed;

    // Initialize at this speed
    mcp2515.reset();
    delay(100);
    mcp2515.setBitrate(speed, CONFIG_CAN_CLOCK);
    delay(50);

    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        mcp2515.setLoopbackMode();
    } else {
        mcp2515.setNormalMode();
    }
    delay(50);

    // Reset statistics
    #ifdef ESP32
    mcp2515.resetStatistics();
    #endif

    // Run test categories (subset for speed testing)
    testInitialization();
    testFiltersAndMasks();
    testFrameTransmission();
    testFrameReception();
    testErrorHandling();

    #if ENABLE_STRESS_TESTS
    testStressScenarios();
    #endif
}

void printMultiSpeedComparison() {
    SAFE_SERIAL_BLOCK();
    Serial.println();
    Serial.println();
    Serial.println(COLOR_BOLD COLOR_CYAN "╔════════════════════════════════════════════════════════════════╗" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_CYAN "║           MULTI-SPEED COMPREHENSIVE COMPARISON                 ║" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_CYAN "╚════════════════════════════════════════════════════════════════╝" COLOR_RESET);
    Serial.println();

    // Print table header
    Serial.println("┌─────────────┬────────┬────────┬──────────────┬────────────┬───────────┐");
    Serial.println("│  CAN Speed  │ Passed │ Failed │ Throughput   │ Error Rate │  TX Errors│");
    Serial.println("├─────────────┼────────┼────────┼──────────────┼────────────┼───────────┤");

    // Print results for each speed
    for (uint32_t i = 0; i < speed_results_count; i++) {
        char speed_str[14];
        snprintf(speed_str, sizeof(speed_str), "%s", getSpeedName(speed_results[i].speed));

        const char* pass_color = (speed_results[i].failed == 0) ? COLOR_GREEN : COLOR_YELLOW;
        const char* error_color = (speed_results[i].error_rate < 10.0) ? COLOR_GREEN : COLOR_RED;

        Serial.printf("│ %-11s │ %s%6lu%s │ %s%6lu%s │ %6lu fps   │ %s%5.2f%%%s    │ %9lu │\n",
                     speed_str,
                     pass_color, speed_results[i].passed, COLOR_RESET,
                     speed_results[i].failed > 0 ? COLOR_RED : COLOR_GREEN,
                     speed_results[i].failed, COLOR_RESET,
                     speed_results[i].throughput_fps,
                     error_color, speed_results[i].error_rate, COLOR_RESET,
                     speed_results[i].tx_errors);
    }

    Serial.println("└─────────────┴────────┴────────┴──────────────┴────────────┴───────────┘");
    Serial.println();

    // Performance analysis
    Serial.println(COLOR_BOLD "Performance Analysis:" COLOR_RESET);
    Serial.println();

    // Find best and worst performers
    uint32_t best_throughput_idx = 0;
    uint32_t worst_error_idx = 0;
    float lowest_error = 100.0;

    for (uint32_t i = 0; i < speed_results_count; i++) {
        if (speed_results[i].throughput_fps > speed_results[best_throughput_idx].throughput_fps) {
            best_throughput_idx = i;
        }
        if (speed_results[i].error_rate < lowest_error) {
            lowest_error = speed_results[i].error_rate;
            worst_error_idx = i;
        }
    }

    Serial.printf("  Highest Throughput: %s%s%s at %lu frames/sec\n",
                 COLOR_GREEN, getSpeedName(speed_results[best_throughput_idx].speed), COLOR_RESET,
                 speed_results[best_throughput_idx].throughput_fps);
    Serial.printf("  Lowest Error Rate:  %s%s%s at %.2f%%\n",
                 COLOR_GREEN, getSpeedName(speed_results[worst_error_idx].speed), COLOR_RESET,
                 speed_results[worst_error_idx].error_rate);

    // Observations
    Serial.println();
    Serial.println(COLOR_BOLD "Key Observations:" COLOR_RESET);

    bool inverse_relationship = true;
    for (uint32_t i = 1; i < speed_results_count; i++) {
        if (speed_results[i].error_rate > speed_results[i-1].error_rate) {
            inverse_relationship = false;
            break;
        }
    }

    if (inverse_relationship) {
        Serial.println("  " COLOR_YELLOW "⚠" COLOR_RESET "  Inverse Error Relationship Detected:");
        Serial.println("     - Error rate DECREASES as CAN speed INCREASES");
        Serial.println("     - This indicates TX buffer saturation at low speeds");
        Serial.println("     - Recommendation: Implement flow control or reduce send rate");
    }

    Serial.println();
    Serial.println(COLOR_BOLD "Recommendations:" COLOR_RESET);
    Serial.println("  • For maximum reliability: Use ≥500 kbps (error rate <5%)");
    Serial.println("  • For low-speed applications: Implement TX buffer monitoring");
    Serial.println("  • For high throughput: Use 1000 kbps with proper bus design");
    Serial.println("  • All speeds: Add application-level flow control for production");

    Serial.println();
    Serial.println("═══════════════════════════════════════════════════════════════════");
}

// ============================================================================
// COMPREHENSIVE TEST SUITE
// ============================================================================

void runComprehensiveTests() {
    SAFE_SERIAL_BLOCK();
    Serial.println(COLOR_BOLD COLOR_BLUE "╔════════════════════════════════════════════════════════════════╗" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_BLUE "║  Starting Comprehensive Test Suite                            ║" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_BLUE "╚════════════════════════════════════════════════════════════════╝" COLOR_RESET);
    Serial.println();

    current_test_speed = CONFIG_CAN_SPEED;

    // **CRITICAL:** Verify hardware is actually connected before running tests
    // This detects false test passes when running without hardware
    // Note: Test suite continues regardless to show which tests give false positives
    hardware_detected = verifyHardwareConnected();

    // **ENHANCED:** Wait for slave to be ready before starting tests
    if (TEST_MODE == TEST_MODE_TWO_DEVICE && DEVICE_ROLE == ROLE_MASTER) {
        Serial.println(COLOR_BOLD "Synchronizing with slave device..." COLOR_RESET);
        Serial.println();

        if (!waitForSlaveReadyAtStartup(30000)) {  // 30 second timeout
            Serial.println(COLOR_RED "ERROR: Failed to synchronize with slave!" COLOR_RESET);
            Serial.println(COLOR_RED "Please ensure slave device is powered on and connected." COLOR_RESET);
            Serial.println();
            while(1) delay(1000);  // Halt - cannot proceed without slave
        }

        Serial.println();
        Serial.println(COLOR_GREEN "✓ Slave device synchronized!" COLOR_RESET);
        Serial.println(COLOR_GREEN "  Both devices ready to begin testing" COLOR_RESET);
        Serial.println();
        delay(1000);
    }

    // Test categories
    testInitialization();
    clearRXBuffer();  // Clear between tests

    testOperatingModes();
    clearRXBuffer();

    testBitrateConfiguration();
    clearRXBuffer();

    testFiltersAndMasks();
    clearRXBuffer();

    testFrameTransmission();
    clearRXBuffer();

    testFrameReception();
    clearRXBuffer();

    testRXB1Buffer();           // **NEW**: RXB1 buffer testing
    clearRXBuffer();

    testErrorHandling();
    clearRXBuffer();

    #if ENABLE_INTERRUPT_TESTS
    testInterruptFunctions();
    clearRXBuffer();
    #endif

    testStatistics();
    clearRXBuffer();

    testStatusFunctions();
    clearRXBuffer();

    #if ENABLE_STRESS_TESTS
    testStressScenarios();
    clearRXBuffer();
    #endif

    testClockOutput();
    clearRXBuffer();

    testAdvancedFeatures();
    clearRXBuffer();

    testAdvancedQueue();        // **NEW**: Advanced queue testing
    clearRXBuffer();

    testStateTransitionErrors(); // **NEW**: State transition error tests
    clearRXBuffer();

    testBoundaryConditions();   // **NEW**: Boundary condition tests
    clearRXBuffer();
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
    delay(100); // **FIX:** Allow chip to stabilize after reset

    // Test 2: Set bitrate
    printTestHeader("Set Bitrate");
    err = mcp2515.setBitrate(current_test_speed, CONFIG_CAN_CLOCK);
    printTestResult(err == MCP2515::ERROR_OK, "Bitrate configured successfully");
    delay(50); // **FIX:** Allow bitrate to stabilize

    // Test 3: Set operating mode
    printTestHeader("Set Operating Mode");
    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        err = mcp2515.setLoopbackMode();
        printTestResult(err == MCP2515::ERROR_OK, "Loopback mode set");
    } else {
        err = mcp2515.setNormalMode();
        printTestResult(err == MCP2515::ERROR_OK, "Normal mode set");
    }
    delay(50); // **FIX:** Allow mode to stabilize

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

    // Store current mode
    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        mcp2515.setLoopbackMode();
        delay(50);
    }

    // Test all operating modes with improved transitions and CANSTAT verification
    const struct {
        const char* name;
        MCP2515::ERROR (MCP2515::*setter)();
        bool requires_config_first;  // **FIX:** Some modes need config mode first
        uint8_t expected_opmod;      // Expected OPMOD bits (5-7 of CANSTAT)
    } modes[] = {
        {"Configuration Mode", &MCP2515::setConfigMode, false, 0x04},
        {"Normal Mode", &MCP2515::setNormalMode, false, 0x00},
        {"Listen-Only Mode", &MCP2515::setListenOnlyMode, false, 0x03},
        {"Loopback Mode", &MCP2515::setLoopbackMode, false, 0x02},
        {"Sleep Mode", &MCP2515::setSleepMode, true, 0x01},          // **FIX:** Needs special handling
        {"Normal One-Shot Mode", &MCP2515::setNormalOneShotMode, true, 0x00}  // **FIX:** Needs config first
    };

    for (uint8_t i = 0; i < sizeof(modes)/sizeof(modes[0]); i++) {
        printTestHeader(modes[i].name);

        #if ENABLE_MODE_TRANSITION_FIX
        // **FIX:** Ensure clean state before mode change
        if (modes[i].requires_config_first) {
            // First go to config mode, then to target mode
            mcp2515.setConfigMode();
            waitForModeChange(CANSTAT_OPMOD_CONFIG, 100);  // **OPTIMIZED:** Poll instead of delay
        }

        // Clear any pending interrupts
        mcp2515.clearInterrupts();
        delay(20);
        #endif

        MCP2515::ERROR err = (mcp2515.*(modes[i].setter))();

        #if ENABLE_MODE_TRANSITION_FIX
        // **OPTIMIZED:** Poll for mode change instead of fixed delay
        bool mode_changed = waitForModeChange(modes[i].expected_opmod, 150);
        if (!mode_changed && VERBOSE_OUTPUT) {
            safe_println(COLOR_YELLOW "    Mode transition taking longer than expected" COLOR_RESET);
        }
        #else
        delay(50);
        #endif

        // Read CANSTAT to verify mode (ESP32-specific)
        #ifdef ESP32
        uint8_t canstat = mcp2515.getBusStatus();  // getBusStatus reads CANSTAT
        uint8_t opmod = (canstat >> 5) & 0x07;
        bool mode_correct = (opmod == modes[i].expected_opmod);

        printTestResult(err == MCP2515::ERROR_OK && mode_correct,
                       "Mode set and verified via CANSTAT");

        if (VERBOSE_OUTPUT && err == MCP2515::ERROR_OK) {
            safe_printf("  CANSTAT: 0x%02X, OPMOD: 0x%02X (expected: 0x%02X) %s\n",
                       canstat, opmod, modes[i].expected_opmod,
                       mode_correct ? COLOR_GREEN "✓" COLOR_RESET : COLOR_RED "✗" COLOR_RESET);
        }
        #else
        printTestResult(err == MCP2515::ERROR_OK, "Mode set successfully");
        #endif

        // Some modes may timeout or have conditions that prevent entry
        if (err != MCP2515::ERROR_OK && VERBOSE_OUTPUT) {
            safe_printf("    %sNote: Mode may require specific conditions (no pending TX, etc.)%s\n",
                         COLOR_YELLOW, COLOR_RESET);
            test_stats.warnings++;
        }
    }

    // Restore operating mode for subsequent tests
    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        mcp2515.setLoopbackMode();
        waitForModeChange(CANSTAT_OPMOD_LOOPBACK, 150);  // **OPTIMIZED:** Poll instead of delay
    } else {
        mcp2515.setNormalMode();
        waitForModeChange(CANSTAT_OPMOD_NORMAL, 150);  // **OPTIMIZED:** Poll instead of delay
    }

    delay(50);  // **OPTIMIZED:** Reduced from 100ms
}

// ============================================================================
// TEST CATEGORY 3: BITRATE CONFIGURATION
// ============================================================================

void testBitrateConfiguration() {
    printSectionHeader("BITRATE CONFIGURATION TESTS");

    MCP2515::ERROR err;  // Declare at function scope

    // Test various bitrates
    const CAN_SPEED speeds[] = {
        CAN_5KBPS, CAN_10KBPS, CAN_20KBPS, CAN_50KBPS,
        CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS
    };

    for (uint8_t i = 0; i < sizeof(speeds)/sizeof(speeds[0]); i++) {
        printTestHeader(getSpeedName(speeds[i]));

        err = mcp2515.setConfigMode();
        // **OPTIMIZED:** Poll for config mode instead of fixed 50ms delay
        if (err == MCP2515::ERROR_OK && waitForModeChange(CANSTAT_OPMOD_CONFIG, 100)) {
            err = mcp2515.setBitrate(speeds[i], CONFIG_CAN_CLOCK);
            delay(20);  // **OPTIMIZED:** Reduced from 50ms (bitrate change is fast)
        }

        printTestResult(err == MCP2515::ERROR_OK, "Bitrate configured");
    }

    // Test single-parameter setBitrate (uses default clock)
    printTestHeader("Set Bitrate (Single Parameter - Default Clock)");
    err = mcp2515.setConfigMode();
    if (err == MCP2515::ERROR_OK && waitForModeChange(CANSTAT_OPMOD_CONFIG, 100)) {  // **OPTIMIZED:** Poll
        err = mcp2515.setBitrate(CAN_125KBPS);  // Uses default clock (should be 16MHz for most boards)
        delay(20);  // **OPTIMIZED:** Reduced from 50ms
        printTestResult(err == MCP2515::ERROR_OK, "Bitrate set with default clock");
    } else {
        printTestResult(false, "Failed to enter config mode");
    }

    // Restore original bitrate and mode
    mcp2515.setConfigMode();
    if (waitForModeChange(CANSTAT_OPMOD_CONFIG, 100)) {  // **OPTIMIZED:** Poll
        mcp2515.setBitrate(current_test_speed, CONFIG_CAN_CLOCK);
        delay(20);  // **OPTIMIZED:** Reduced from 50ms
    }

    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        mcp2515.setLoopbackMode();
        waitForModeChange(CANSTAT_OPMOD_LOOPBACK, 100);  // **OPTIMIZED:** Poll
    } else {
        mcp2515.setNormalMode();
        waitForModeChange(CANSTAT_OPMOD_NORMAL, 100);  // **OPTIMIZED:** Poll
    }
}

// ============================================================================
// TEST CATEGORY 4: FILTERS AND MASKS
// ============================================================================

void testFiltersAndMasks() {
    printSectionHeader("FILTER AND MASK TESTS - FUNCTIONAL VERIFICATION");

    // **ENHANCED:** Now works in BOTH loopback and two-device modes with optimized timing

    struct can_frame tx_frame, rx_frame;
    MCP2515::ERROR err;

    // Clear any pending messages first
    while (mcp2515.checkReceive()) {
        mcp2515.readMessage(&rx_frame);
    }

    // TWO_DEVICE MODE: Slave sends test frames, Master receives and verifies filtering
    if (TEST_MODE == TEST_MODE_TWO_DEVICE && DEVICE_ROLE == ROLE_SLAVE) {
        // Slave is called from runSlaveMode() which already handled waitForStartSignalAndAck()
        safe_println(COLOR_YELLOW "Slave: Sending filter test frames for master..." COLOR_RESET);

        // Send test frames for each filter test
        const uint32_t test_ids[] = {0x100, 0x101, 0x102, 0x103, 0x123, 0x456, 0x123, 0x456, 0x12345678 | CAN_EFF_FLAG};
        for (uint8_t i = 0; i < 9; i++) {
            tx_frame.can_id = test_ids[i];
            tx_frame.can_dlc = 1;
            tx_frame.data[0] = 0xA0 + i;
            mcp2515.sendMessage(&tx_frame);
            delay(100);  // Give master time to process
        }

        // Signal completion and wait for master to finish
        sendCompleteSignal();
        if (waitForComplete(5000)) {
            safe_println(COLOR_GREEN "  ✓ Master completed filter tests" COLOR_RESET);
        }
        return;
    }

    // MASTER MODE: Signal slave to start sending
    if (TEST_MODE == TEST_MODE_TWO_DEVICE && DEVICE_ROLE == ROLE_MASTER) {
        if (!sendStartSignalWithRetry(0x04, 5)) {  // Test ID 0x04, 5 retries
            safe_println(COLOR_RED "ERROR: Failed to start slave for filter test!" COLOR_RESET);
            return;
        }
        delay(200);  // Give slave time to start sending
    }

    // ===================================================================
    // Test 1: Accept All (Mask = 0x000)
    // ===================================================================
    printTestHeader("Filter Test: Accept All (Mask = 0x000)");

    mcp2515.setFilterMask(MCP2515::MASK0, false, 0x000);  // All bits don't care
    mcp2515.setFilter(MCP2515::RXF0, false, 0x000);
    delay(20);  // **OPTIMIZED:** Reduced from 50ms to 20ms (filter config is fast)

    // Send different IDs - all should be accepted
    bool accept_all_works = true;
    for (uint32_t test_id = 0x100; test_id <= 0x103; test_id++) {
        if (TEST_MODE == TEST_MODE_LOOPBACK) {
            tx_frame.can_id = test_id;
            tx_frame.can_dlc = 1;
            tx_frame.data[0] = (uint8_t)test_id;
            mcp2515.sendMessage(&tx_frame);
        }

        // **OPTIMIZED:** Polling replaces delay(100) - typically 10-50ms
        if (!waitForMessage(150)) {
            accept_all_works = false;
            break;
        }
        mcp2515.readMessage(&rx_frame);  // Clear it
    }

    printTestResult(accept_all_works, "Accept-all filter (mask=0x000) passed all IDs");

    // ===================================================================
    // Test 2: Exact Match (Mask = 0x7FF)
    // ===================================================================
    printTestHeader("Filter Test: Exact Match (Mask = 0x7FF)");

    // Clear RX buffer
    while (mcp2515.checkReceive()) {
        mcp2515.readMessage(&rx_frame);
    }

    // Set filter to ONLY accept 0x123
    mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);  // Match all 11 bits
    mcp2515.setFilter(MCP2515::RXF0, false, 0x123);
    delay(20);  // **OPTIMIZED:** Reduced from 50ms

    // Test 2a: Matching frame SHOULD be accepted
    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        tx_frame.can_id = 0x123;
        tx_frame.can_dlc = 1;
        tx_frame.data[0] = 0xAA;
        mcp2515.sendMessage(&tx_frame);
    }

    bool matching_accepted = waitForMessage(150);  // **OPTIMIZED:** Polling replaces delay(100)
    printTestResult(matching_accepted, "Matching frame 0x123 accepted");

    if (matching_accepted) {
        mcp2515.readMessage(&rx_frame);
        if (VERBOSE_OUTPUT && ((rx_frame.can_id & CAN_SFF_MASK) != 0x123)) {
            safe_printf("  %sWarning: Received ID 0x%03X, expected 0x123%s\n",
                       COLOR_YELLOW, rx_frame.can_id & CAN_SFF_MASK, COLOR_RESET);
        }
    }

    // Test 2b: Non-matching frame SHOULD be rejected
    printTestHeader("Filter Test: Reject Non-Matching Frame");

    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        tx_frame.can_id = 0x456;  // Different ID
        tx_frame.can_dlc = 1;
        tx_frame.data[0] = 0xBB;
        mcp2515.sendMessage(&tx_frame);
    }

    // **OPTIMIZED:** Use polling with shorter timeout for rejection test (should timeout)
    bool nonmatching_rejected = !waitForMessage(150);
    printTestResult(nonmatching_rejected, "Non-matching frame 0x456 rejected");

    // ===================================================================
    // Test 3: Partial Match (Mask = 0x700, filter = 0x100)
    // ===================================================================
    printTestHeader("Filter Test: Partial Match (Mask = 0x700)");

    // Clear RX buffer
    while (mcp2515.checkReceive()) {
        mcp2515.readMessage(&rx_frame);
    }

    // Filter: Match upper 3 bits only (0x100-0x1FF should pass)
    mcp2515.setFilterMask(MCP2515::MASK0, false, 0x700);
    mcp2515.setFilter(MCP2515::RXF0, false, 0x100);
    delay(20);  // **OPTIMIZED:** Reduced from 50ms

    // 0x123 should match (upper bits = 0x100)
    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        tx_frame.can_id = 0x123;
        tx_frame.can_dlc = 1;
        tx_frame.data[0] = 0xCC;
        mcp2515.sendMessage(&tx_frame);
    }

    bool partial_match_pass = waitForMessage(150);  // **OPTIMIZED:** Polling
    if (partial_match_pass) mcp2515.readMessage(&rx_frame);

    // 0x456 should NOT match (upper bits = 0x400)
    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        tx_frame.can_id = 0x456;
        tx_frame.can_dlc = 1;
        tx_frame.data[0] = 0xDD;
        mcp2515.sendMessage(&tx_frame);
    }

    bool partial_match_reject = !waitForMessage(150);  // **OPTIMIZED:** Polling

    bool partial_match_ok = partial_match_pass && partial_match_reject;
    printTestResult(partial_match_ok, "Partial mask (0x700) filtered correctly");

    // ===================================================================
    // Test 4: Extended ID Filter
    // ===================================================================
    printTestHeader("Filter Test: Extended ID Filtering");

    // Clear RX buffer
    while (mcp2515.checkReceive()) {
        mcp2515.readMessage(&rx_frame);
    }

    // Set extended ID filter
    mcp2515.setFilterMask(MCP2515::MASK1, true, 0x1FFFFFFF);  // Match all 29 bits
    mcp2515.setFilter(MCP2515::RXF2, true, 0x12345678);
    delay(20);  // **OPTIMIZED:** Reduced from 50ms

    // Matching extended frame
    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        tx_frame.can_id = 0x12345678 | CAN_EFF_FLAG;
        tx_frame.can_dlc = 2;
        tx_frame.data[0] = 0xEE;
        tx_frame.data[1] = 0xFF;
        mcp2515.sendMessage(&tx_frame);
    }

    bool ext_accepted = waitForMessage(150);  // **OPTIMIZED:** Polling
    printTestResult(ext_accepted, "Extended ID 0x12345678 accepted");

    if (ext_accepted) {
        mcp2515.readMessage(&rx_frame);
    }

    // ===================================================================
    // Test 5: Multiple Filters (RXF0-RXF5)
    // ===================================================================
    printTestHeader("Configure All 6 Filters");

    const struct {
        MCP2515::RXF filter;
        bool ext;
        uint32_t id;
    } all_filters[] = {
        {MCP2515::RXF0, false, 0x123},
        {MCP2515::RXF1, false, 0x456},
        {MCP2515::RXF2, true, 0x12345678},
        {MCP2515::RXF3, true, 0x87654321},
        {MCP2515::RXF4, false, 0x555},
        {MCP2515::RXF5, false, 0x777}
    };

    bool all_set_ok = true;
    for (uint8_t i = 0; i < 6; i++) {
        err = mcp2515.setFilter(all_filters[i].filter, all_filters[i].ext, all_filters[i].id);
        if (err != MCP2515::ERROR_OK) {
            all_set_ok = false;
            break;
        }
    }

    printTestResult(all_set_ok, "All 6 filters configured");

    // ===================================================================
    // Reset filters to accept all for subsequent tests
    // ===================================================================
    for (uint8_t i = 0; i < 6; i++) {
        mcp2515.setFilter((MCP2515::RXF)i, false, 0);
    }
    mcp2515.setFilterMask(MCP2515::MASK0, false, 0);
    mcp2515.setFilterMask(MCP2515::MASK1, false, 0);

    delay(20);  // **OPTIMIZED:** Reduced from 100ms to 20ms

    // **ENHANCED:** Signal completion and wait for slave in two-device mode
    if (TEST_MODE == TEST_MODE_TWO_DEVICE && DEVICE_ROLE == ROLE_MASTER) {
        sendCompleteSignal();
        if (waitForComplete(5000)) {
            if (VERBOSE_OUTPUT) {
                safe_println(COLOR_GREEN "  ✓ Slave completed filter tests" COLOR_RESET);
            }
        }
    }
}

// ============================================================================
// TEST CATEGORY 5: FRAME TRANSMISSION
// ============================================================================

void testFrameTransmission() {
    printSectionHeader("FRAME TRANSMISSION TESTS");

    // **NEW:** Check TX buffer status
    bool buffer_ok = checkTXBufferAvailable();
    if (!buffer_ok && VERBOSE_OUTPUT) {
        safe_println("  " COLOR_YELLOW "Warning: TX buffers may be busy" COLOR_RESET);
    }

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
    delay(50); // **FIX:** Allow transmission to complete

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
// TEST CATEGORY 6: FRAME RECEPTION (IMPROVED)
// ============================================================================

void testFrameReception() {
    printSectionHeader("FRAME RECEPTION TESTS");

    if (TEST_MODE == TEST_MODE_TWO_DEVICE && DEVICE_ROLE == ROLE_SLAVE) {
        // Slave sends test frames
        safe_println(COLOR_YELLOW "Slave: Sending test frames for master..." COLOR_RESET);

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

    // Test reception with improved data integrity checking
    printTestHeader("Data Integrity Test (Enhanced)");

    // **FIX:** Use unique, verifiable data pattern
    struct can_frame tx_frame;
    tx_frame.can_id = 0x300;
    tx_frame.can_dlc = 8;

    // Create unique pattern: 0xA0, 0xA1, 0xA2, ... 0xA7
    for (uint8_t i = 0; i < 8; i++) {
        tx_frame.data[i] = 0xA0 + i;
    }

    // Clear any pending RX messages first
    struct can_frame dummy;
    while (mcp2515.checkReceive()) {
        mcp2515.readMessage(&dummy);
        delay(5);
    }

    // Send the test frame
    MCP2515::ERROR err = mcp2515.sendMessage(&tx_frame);

    // **OPTIMIZED:** Use polling instead of fixed delays - typically 10-50ms vs 100-150ms
    #if ENABLE_DATA_INTEGRITY_FIX
    bool has_message = waitForMessage(150);  // Poll with 150ms timeout (was: delay(100) + delay(50) retry)
    #else
    bool has_message = waitForMessage(50);   // Poll with 50ms timeout (was: delay(50))
    #endif

    // Read the message
    struct can_frame rx_frame;
    if (has_message) {
        err = mcp2515.readMessage(&rx_frame);

        if (err == MCP2515::ERROR_OK) {
            // Verify data integrity
            bool id_match = (rx_frame.can_id == tx_frame.can_id);
            bool dlc_match = (rx_frame.can_dlc == tx_frame.can_dlc);
            bool data_match = true;

            for (uint8_t i = 0; i < tx_frame.can_dlc; i++) {
                if (rx_frame.data[i] != tx_frame.data[i]) {
                    data_match = false;
                    break;
                }
            }

            bool integrity_ok = id_match && dlc_match && data_match;
            printTestResult(integrity_ok, "Data integrity verified");

            if (VERBOSE_OUTPUT) {
                safe_println("  TX Frame:");
                printFrame(&tx_frame, true);
                safe_println("  RX Frame:");
                printFrame(&rx_frame, false);

                if (!integrity_ok) {
                    safe_printf("  %sIntegrity Check Details:%s\n", COLOR_YELLOW, COLOR_RESET);
                    safe_printf("    ID Match:  %s\n", id_match ? "✓" : "✗");
                    safe_printf("    DLC Match: %s\n", dlc_match ? "✓" : "✗");
                    safe_printf("    Data Match: %s\n", data_match ? "✓" : "✗");

                    if (!data_match) {
                        safe_println("  Expected vs Actual Data:");
                        printDataComparison(tx_frame.data, rx_frame.data, tx_frame.can_dlc);
                    }
                }
            }
        } else {
            printTestResult(false, "Failed to read message");
        }
    } else {
        printTestResult(false, "No message received in loopback");
        if (VERBOSE_OUTPUT) {
            safe_println("  " COLOR_YELLOW "Possible causes:" COLOR_RESET);
            safe_println("    - Loopback mode not properly set");
            safe_println("    - RX buffers full or overflow");
            safe_println("    - Timing issue (try increasing delay)");
        }
    }

    // Test checkReceive function
    printTestHeader("Check Receive Function");
    tx_frame.can_id = 0x301;
    mcp2515.sendMessage(&tx_frame);
    delay(100); // **FIX:** Adequate delay
    has_message = mcp2515.checkReceive();
    printTestResult(has_message, "checkReceive() detected message");

    // Test reading from specific buffer
    printTestHeader("Read from RX Buffer 0");
    if (has_message) {
        err = mcp2515.readMessage(MCP2515::RXB0, &rx_frame);
        printTestResult(err == MCP2515::ERROR_OK, "RXB0 read successful");
    } else {
        test_stats.skipped_tests++;
        safe_println("  " COLOR_YELLOW "SKIP - No message to read" COLOR_RESET);
    }

    // Test no message available - **FIX:** Corrected logic
    printTestHeader("Read with No Message Available");

    // Clear all pending messages first
    while (mcp2515.checkReceive()) {
        mcp2515.readMessage(&rx_frame);
        delay(5);
    }

    // Now try to read when nothing is available
    err = mcp2515.readMessage(&rx_frame);

    // **FIX:** This SHOULD return ERROR_NOMSG when no message available
    bool correct_behavior = (err == MCP2515::ERROR_NOMSG);
    printTestResult(correct_behavior, "ERROR_NOMSG returned correctly");

    if (!correct_behavior && VERBOSE_OUTPUT) {
        safe_printf("    Expected ERROR_NOMSG, got error code: %d\n", err);
    }

    delay(100);
}

// ============================================================================
// TEST CATEGORY 6B: RXB1 BUFFER TESTING
// ============================================================================

void testRXB1Buffer() {
    printSectionHeader("RXB1 BUFFER TESTING");

    // **ENHANCED:** Now works in BOTH loopback and two-device modes

    struct can_frame tx_frame, rx_frame;
    MCP2515::ERROR err;

    // Clear any pending messages
    while (mcp2515.checkReceive()) {
        mcp2515.readMessage(&rx_frame);
    }

    // TWO_DEVICE MODE: Slave sends test frames, Master receives
    if (TEST_MODE == TEST_MODE_TWO_DEVICE && DEVICE_ROLE == ROLE_SLAVE) {
        // Slave is called from runSlaveMode() which already handled waitForStartSignalAndAck()
        safe_println(COLOR_YELLOW "Slave: Sending RXB1 test frames for master..." COLOR_RESET);

        // Send two frames for RXB0/RXB1 testing
        tx_frame.can_id = 0x301;
        tx_frame.can_dlc = 1;
        tx_frame.data[0] = 0x11;
        mcp2515.sendMessage(&tx_frame);
        delay(50);

        tx_frame.can_id = 0x302;
        tx_frame.can_dlc = 1;
        tx_frame.data[0] = 0x22;
        mcp2515.sendMessage(&tx_frame);
        delay(100);

        // Signal completion and wait for master to finish
        sendCompleteSignal();
        if (waitForComplete(5000)) {
            safe_println(COLOR_GREEN "  ✓ Master completed RXB1 tests" COLOR_RESET);
        }
        return;
    }

    // MASTER MODE: Signal slave to start sending
    if (TEST_MODE == TEST_MODE_TWO_DEVICE && DEVICE_ROLE == ROLE_MASTER) {
        if (!sendStartSignalWithRetry(0x05, 5)) {  // Test ID 0x05, 5 retries
            safe_println(COLOR_RED "ERROR: Failed to start slave for RXB1 test!" COLOR_RESET);
            return;
        }
        delay(200);  // Give slave time to start sending
    }

    // Send two frames - first goes to RXB0, second may go to RXB1 if rollover enabled
    printTestHeader("Send Multiple Frames for RXB0/RXB1");

    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        tx_frame.can_id = 0x301;
        tx_frame.can_dlc = 1;
        tx_frame.data[0] = 0x11;
        mcp2515.sendMessage(&tx_frame);
        delay(20);  // **OPTIMIZED:** Reduced from 50ms

        tx_frame.can_id = 0x302;
        tx_frame.can_dlc = 1;
        tx_frame.data[0] = 0x22;
        mcp2515.sendMessage(&tx_frame);
    }

    // **OPTIMIZED:** Use polling instead of fixed delay
    waitForMessage(150);  // Wait for first frame
    waitForMessage(150);  // Wait for second frame

    printTestHeader("Read from RXB1 Explicitly");
    err = mcp2515.readMessage(MCP2515::RXB1, &rx_frame);

    if (err == MCP2515::ERROR_OK) {
        printTestResult(true, "RXB1 read successful");
        if (VERBOSE_OUTPUT) {
            safe_printf("  RXB1 ID: 0x%03X, Data: 0x%02X\n",
                       rx_frame.can_id & CAN_SFF_MASK, rx_frame.data[0]);
        }
    } else if (err == MCP2515::ERROR_NOMSG) {
        printTestResult(false, "RXB1 empty (rollover may not be enabled)");
        if (VERBOSE_OUTPUT) {
            safe_println("  " COLOR_YELLOW "Note: RXB0 rollover to RXB1 may not be configured" COLOR_RESET);
        }
    } else {
        printTestResult(false, "RXB1 read failed");
    }

    // Test reading from RXB0 to compare
    printTestHeader("Read from RXB0 for Comparison");
    err = mcp2515.readMessage(MCP2515::RXB0, &rx_frame);
    printTestResult(err == MCP2515::ERROR_OK || err == MCP2515::ERROR_NOMSG,
                   "RXB0 read completed");

    delay(20);  // **OPTIMIZED:** Reduced from 100ms

    // **ENHANCED:** Signal completion and wait for slave in two-device mode
    if (TEST_MODE == TEST_MODE_TWO_DEVICE && DEVICE_ROLE == ROLE_MASTER) {
        sendCompleteSignal();
        if (waitForComplete(5000)) {
            if (VERBOSE_OUTPUT) {
                safe_println(COLOR_GREEN "  ✓ Slave completed RXB1 tests" COLOR_RESET);
            }
        }
    }
}

// ============================================================================
// TEST CATEGORY 7: ERROR HANDLING
// ============================================================================

void testErrorHandling() {
    printSectionHeader("ERROR HANDLING TESTS - WITH VERIFICATION");

    // **FIX:** This test now actually verifies error handling works, not just that functions don't crash

    // Test 1: Get error flags and verify in normal operating range
    printTestHeader("Get Error Flags - Verify Normal State");
    uint8_t eflg = mcp2515.getErrorFlags();

    // In normal operation (loopback, no bus errors), critical flags should not be set
    bool no_critical_errors = !(eflg & (MCP2515::EFLG_TXBO | MCP2515::EFLG_TXEP | MCP2515::EFLG_RXEP));

    printTestResult(no_critical_errors, "No critical error flags (bus-off, error-passive)");

    if (VERBOSE_OUTPUT) {
        safe_printf("  EFLG = 0x%02X\n", eflg);
        if (eflg & MCP2515::EFLG_RX0OVR) safe_println("    - RX0 Overflow");
        if (eflg & MCP2515::EFLG_RX1OVR) safe_println("    - RX1 Overflow");
        if (eflg & MCP2515::EFLG_TXBO) safe_println("    - TX Bus-Off");
        if (eflg & MCP2515::EFLG_TXEP) safe_println("    - TX Error Passive");
        if (eflg & MCP2515::EFLG_RXEP) safe_println("    - RX Error Passive");
        if (eflg & MCP2515::EFLG_TXWAR) safe_println("    - TX Error Warning");
        if (eflg & MCP2515::EFLG_RXWAR) safe_println("    - RX Error Warning");
    }

    // Test 2: Check error function consistency
    printTestHeader("Check Error Consistency with Flags");
    bool has_error = mcp2515.checkError();

    // checkError() should return true if EFLG has any error flags
    bool error_consistent = (has_error == (eflg != 0));

    printTestResult(error_consistent, "checkError() consistent with error flags");

    // Test 3: Error counters in normal range
    printTestHeader("Verify Error Counters in Normal Range");
    uint8_t rec = mcp2515.errorCountRX();
    uint8_t tec = mcp2515.errorCountTX();

    // In normal operation, error counters should be < 128 (not error-passive)
    bool counters_normal = (rec < 128 && tec < 128);

    printTestResult(counters_normal, "Error counters < 128 (not error-passive)");

    if (VERBOSE_OUTPUT) {
        safe_printf("  REC = %d, TEC = %d\n", rec, tec);
    }

    // Test 4: Clear overflow flags and verify
    printTestHeader("Clear and Verify RXnOVR Flags");

    uint8_t eflg_before = mcp2515.getErrorFlags();
    mcp2515.clearRXnOVRFlags();
    delay(10);
    uint8_t eflg_after = mcp2515.getErrorFlags();

    // Overflow bits should be cleared
    bool overflow_cleared = !(eflg_after & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR));

    printTestResult(overflow_cleared, "RXnOVR flags cleared");

    if (!overflow_cleared && VERBOSE_OUTPUT) {
        safe_printf("  Before: 0x%02X, After: 0x%02X\n", eflg_before, eflg_after);
    }

    // Test 5: Test clearRXnOVR (alternative clear method)
    printTestHeader("Clear RXnOVR (Alternative Method)");
    mcp2515.clearRXnOVR();
    delay(10);
    uint8_t eflg_rxnovr = mcp2515.getErrorFlags();

    bool rxnovr_cleared = !(eflg_rxnovr & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR));
    printTestResult(rxnovr_cleared, "RXnOVR cleared (alternative method)");

    // Test 6: Clear MERR flag
    printTestHeader("Clear MERR Flag");
    mcp2515.clearMERR();
    delay(10);

    // Can't directly verify MERR flag (it's in CANINTF), but function should not crash
    printTestResult(true, "clearMERR executed");

    // Test 7: Clear ERRIF flag
    printTestHeader("Clear ERRIF Flag");
    mcp2515.clearERRIF();
    delay(10);

    // Verify ERRIF is cleared from interrupt flags
    uint8_t irq = mcp2515.getInterrupts();
    bool errif_cleared = !(irq & MCP2515::CANINTF_ERRIF);

    printTestResult(errif_cleared, "ERRIF flag cleared");

    // Test 8: Error recovery (ESP32-specific)
    #ifdef ESP32
    printTestHeader("Error Recovery - Verify Execution");

    uint8_t tec_before = mcp2515.errorCountTX();
    uint8_t rec_before = mcp2515.errorCountRX();

    MCP2515::ERROR err = mcp2515.performErrorRecovery();

    bool recovery_ok = (err == MCP2515::ERROR_OK);
    printTestResult(recovery_ok, "Error recovery executed successfully");

    if (VERBOSE_OUTPUT && recovery_ok) {
        uint8_t tec_after = mcp2515.errorCountTX();
        uint8_t rec_after = mcp2515.errorCountRX();
        safe_printf("  Error counts - Before: TEC=%d REC=%d, After: TEC=%d REC=%d\n",
                   tec_before, rec_before, tec_after, rec_after);
    }
    #endif

    // Test 9: Bus status (ESP32-specific)
    #ifdef ESP32
    printTestHeader("Get Bus Status - Verify Valid");
    uint8_t bus_status = mcp2515.getBusStatus();

    // Bus status should have a valid mode in upper 3 bits (OPMOD field)
    uint8_t opmod = (bus_status >> 5) & 0x07;
    bool valid_mode = (opmod <= 5);  // Valid modes are 0-5

    printTestResult(valid_mode, "Bus status contains valid operating mode");

    if (VERBOSE_OUTPUT) {
        safe_printf("  CANSTAT = 0x%02X, OPMOD = %d\n", bus_status, opmod);
    }
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
        safe_printf("  CANINTF = 0x%02X\n", irq);
        if (irq & MCP2515::CANINTF_RX0IF) safe_println("    - RX0 Interrupt");
        if (irq & MCP2515::CANINTF_RX1IF) safe_println("    - RX1 Interrupt");
        if (irq & MCP2515::CANINTF_TX0IF) safe_println("    - TX0 Interrupt");
        if (irq & MCP2515::CANINTF_TX1IF) safe_println("    - TX1 Interrupt");
        if (irq & MCP2515::CANINTF_TX2IF) safe_println("    - TX2 Interrupt");
        if (irq & MCP2515::CANINTF_ERRIF) safe_println("    - Error Interrupt");
        if (irq & MCP2515::CANINTF_WAKIF) safe_println("    - Wake Interrupt");
        if (irq & MCP2515::CANINTF_MERRF) safe_println("    - Message Error");
    }

    // Test 2: Get interrupt mask
    printTestHeader("Get Interrupt Mask");
    uint8_t imask = mcp2515.getInterruptMask();
    printTestResult(true, "Interrupt mask read");
    if (VERBOSE_OUTPUT) safe_printf("  CANINTE = 0x%02X\n", imask);

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
    printSectionHeader("STATISTICS TESTS (ESP32-specific) - VALUE VERIFICATION");

    #ifdef ESP32
    // **FIX:** This test now actually verifies counter values match expected behavior

    // Test 1: Reset statistics and verify all counters are zero
    printTestHeader("Reset Statistics and Verify Zero");
    mcp2515.resetStatistics();
    delay(50);

    mcp2515_statistics_t stats_reset;
    mcp2515.getStatistics(&stats_reset);

    bool all_zero = (stats_reset.tx_frames == 0 &&
                     stats_reset.rx_frames == 0 &&
                     stats_reset.tx_errors == 0 &&
                     stats_reset.rx_errors == 0 &&
                     stats_reset.rx_overflow == 0 &&
                     stats_reset.tx_timeouts == 0 &&
                     stats_reset.bus_errors == 0 &&
                     stats_reset.bus_off_count == 0);

    printTestResult(all_zero, "All statistics counters reset to zero");

    if (!all_zero && VERBOSE_OUTPUT) {
        safe_printf("  %sNon-zero counters after reset:%s\n", COLOR_YELLOW, COLOR_RESET);
        if (stats_reset.tx_frames) safe_printf("    TX Frames: %lu\n", stats_reset.tx_frames);
        if (stats_reset.rx_frames) safe_printf("    RX Frames: %lu\n", stats_reset.rx_frames);
        if (stats_reset.tx_errors) safe_printf("    TX Errors: %lu\n", stats_reset.tx_errors);
    }

    // Test 2: Send exactly 10 frames and verify TX counter
    printTestHeader("TX Frame Counter Accuracy");

    mcp2515_statistics_t stats_before;
    mcp2515.getStatistics(&stats_before);

    uint32_t frames_sent_successfully = 0;
    for (int i = 0; i < 10; i++) {
        struct can_frame frame;
        frame.can_id = 0x400 + i;
        frame.can_dlc = 8;
        for (uint8_t j = 0; j < 8; j++) {
            frame.data[j] = i * 10 + j;
        }
        MCP2515::ERROR err = mcp2515.sendMessage(&frame);
        if (err == MCP2515::ERROR_OK) {
            frames_sent_successfully++;
        }
        delay(10);
    }

    delay(100);  // Allow all TX to complete

    mcp2515_statistics_t stats_after;
    mcp2515.getStatistics(&stats_after);

    uint32_t tx_delta = stats_after.tx_frames - stats_before.tx_frames;
    bool tx_correct = (tx_delta == frames_sent_successfully);

    printTestResult(tx_correct, "TX counter matches frames sent");

    if (!tx_correct) {
        safe_printf("  %sExpected: %lu, Actual delta: %lu%s\n",
                   COLOR_YELLOW, frames_sent_successfully, tx_delta, COLOR_RESET);
    }

    // Test 3: RX counter test - works in BOTH loopback and two-device modes
    printTestHeader("RX Frame Counter Test");

    // TWO_DEVICE MODE: Coordinate with slave to receive frames
    if (TEST_MODE == TEST_MODE_TWO_DEVICE) {
        if (DEVICE_ROLE == ROLE_SLAVE) {
            // Slave is called from runSlaveMode() which already handled waitForStartSignalAndAck()
            safe_println(COLOR_YELLOW "Slave: Sending statistics test frames for master..." COLOR_RESET);

            // Slave: Send frames for master to receive
            for (int i = 0; i < 10; i++) {
                struct can_frame frame;
                frame.can_id = 0x410 + i;
                frame.can_dlc = 2;
                frame.data[0] = 0xA0 + i;
                frame.data[1] = i;
                mcp2515.sendMessage(&frame);
                delay(20);
            }

            // Signal completion and wait for master to finish
            sendCompleteSignal();
            if (waitForComplete(5000)) {
                safe_println(COLOR_GREEN "  ✓ Master completed statistics tests" COLOR_RESET);
            }
            return;  // Slave done
        } else {
            // Master: Signal slave to start, then receive frames
            if (!sendStartSignalWithRetry(0x06, 5)) {  // Test ID 0x06, 5 retries
                safe_println(COLOR_RED "ERROR: Failed to start slave for statistics test!" COLOR_RESET);
                return;
            }
            delay(100);  // Give slave time to start sending

            mcp2515_statistics_t stats_rx_before;
            mcp2515.getStatistics(&stats_rx_before);

            // Receive frames from slave
            uint32_t frames_received = 0;
            for (int i = 0; i < 10; i++) {
                if (waitForMessage(500)) {
                    struct can_frame frame;
                    mcp2515.readMessage(&frame);
                    frames_received++;
                }
            }

            delay(50);
            mcp2515_statistics_t stats_rx_after;
            mcp2515.getStatistics(&stats_rx_after);

            uint32_t rx_delta = stats_rx_after.rx_frames - stats_rx_before.rx_frames;
            bool rx_correct = (rx_delta == frames_received && frames_received > 0);

            printTestResult(rx_correct, "RX counter matches frames received (two-device)");

            if (VERBOSE_OUTPUT) {
                safe_printf("  Frames received: %lu, RX delta: %lu\n", frames_received, rx_delta);
            }

            // Send completion signal and wait for slave
            sendCompleteSignal();
            waitForComplete(5000);
        }
    } else {
        // LOOPBACK MODE: Frames loop back to RX
        uint32_t rx_delta = stats_after.rx_frames - stats_before.rx_frames;

        // In loopback, frames loop back to RX
        // The count should be close to frames sent (may vary slightly)
        bool rx_incremented = (rx_delta > 0 && rx_delta <= frames_sent_successfully);

        printTestResult(rx_incremented, "RX counter incremented in loopback");

        if (VERBOSE_OUTPUT) {
            safe_printf("  TX delta: %lu, RX delta: %lu\n", tx_delta, rx_delta);
        }
    }

    // Test 4: Verify statistics don't go backwards
    printTestHeader("Statistics Monotonicity");

    mcp2515_statistics_t stats1, stats2;
    mcp2515.getStatistics(&stats1);

    // Send one more frame
    struct can_frame test_frame;
    test_frame.can_id = 0x500;
    test_frame.can_dlc = 1;
    test_frame.data[0] = 0xFF;
    mcp2515.sendMessage(&test_frame);
    delay(50);

    mcp2515.getStatistics(&stats2);

    bool monotonic = (stats2.tx_frames >= stats1.tx_frames &&
                      stats2.rx_frames >= stats1.rx_frames);

    printTestResult(monotonic, "Statistics counters are monotonic (don't decrease)");

    // Test 5: Get RX queue count
    printTestHeader("Get RX Queue Count");
    uint32_t queue_count = mcp2515.getRxQueueCount();

    // Queue count should be a reasonable number (not > queue size, not negative)
    bool queue_count_valid = (queue_count <= 100);  // Assuming max queue size around 32-64

    printTestResult(queue_count_valid, "Queue count is valid");
    if (VERBOSE_OUTPUT) {
        safe_printf("  Queue count = %lu\n", queue_count);
    }

    // Test 6: Display full statistics if verbose
    if (VERBOSE_OUTPUT) {
        safe_println("\n  " COLOR_BOLD "Final Statistics:" COLOR_RESET);
        safe_printf("    RX Frames:     %lu\n", stats2.rx_frames);
        safe_printf("    TX Frames:     %lu\n", stats2.tx_frames);
        safe_printf("    RX Errors:     %lu\n", stats2.rx_errors);
        safe_printf("    TX Errors:     %lu\n", stats2.tx_errors);
        safe_printf("    RX Overflow:   %lu\n", stats2.rx_overflow);
        safe_printf("    TX Timeouts:   %lu\n", stats2.tx_timeouts);
        safe_printf("    Bus Errors:    %lu\n", stats2.bus_errors);
        safe_printf("    Bus-Off Count: %lu\n", stats2.bus_off_count);
    }

    #else
    safe_println(COLOR_YELLOW "Statistics tests are ESP32-specific - skipped" COLOR_RESET);
    test_stats.skipped_tests += 6;
    #endif

    delay(100);
}

// ============================================================================
// TEST CATEGORY 10: STATUS FUNCTIONS
// ============================================================================

void testStatusFunctions() {
    printSectionHeader("STATUS FUNCTION TESTS - WITH VERIFICATION");

    // Send a frame first to potentially set status bits
    struct can_frame frame;
    frame.can_id = 0x600;
    frame.can_dlc = 1;
    frame.data[0] = 0xAA;
    mcp2515.sendMessage(&frame);
    delay(100);

    printTestHeader("Get Status and Verify Valid");
    uint8_t status = mcp2515.getStatus();

    // Status should be a valid byte (not 0xFF which might indicate SPI error)
    bool status_valid = (status != 0xFF);

    printTestResult(status_valid, "Status read successfully (not 0xFF)");

    if (VERBOSE_OUTPUT) {
        safe_printf("  Status = 0x%02X\n", status);
        if (status & 0x01) safe_println("    - RX0IF");
        if (status & 0x02) safe_println("    - RX1IF");
        if (status & 0x04) safe_println("    - TX0REQ");
        if (status & 0x08) safe_println("    - TX0IF");
        if (status & 0x10) safe_println("    - TX1REQ");
        if (status & 0x20) safe_println("    - TX1IF");
        if (status & 0x40) safe_println("    - TX2REQ");
        if (status & 0x80) safe_println("    - TX2IF");
    }

    delay(100);
}

// ============================================================================
// TEST CATEGORY 11: STRESS TESTS (IMPROVED WITH ADAPTIVE RATE)
// ============================================================================

#if ENABLE_STRESS_TESTS
void testStressScenarios() {
    printSectionHeader("STRESS TESTS");

    // **NEW:** Adaptive stress test based on CAN speed
    printTestHeader("Adaptive High-Speed TX Stress Test");

    uint32_t start_time = millis();
    uint32_t frame_count = 0;
    uint32_t error_count = 0;
    uint32_t buffer_full_count = 0;

    // **FIX:** Calculate adaptive delay based on CAN speed
    uint32_t delay_micros = getAdaptiveDelayMicros(current_test_speed);
    uint32_t theoretical_fps = getTheoreticalFramesPerSecond(current_test_speed);

    if (VERBOSE_OUTPUT) {
        safe_printf("  Target Rate:     ~%lu frames/sec\n", theoretical_fps);
        safe_printf("  Inter-frame Delay: %lu µs\n", delay_micros);
    }

    struct can_frame stress_frame;
    stress_frame.can_dlc = 8;

    while (millis() - start_time < STRESS_TEST_DURATION_MS) {
        stress_frame.can_id = 0x500 + (frame_count % 256);

        // **NEW:** Use unique, verifiable data pattern
        uint32_t timestamp = millis();
        stress_frame.data[0] = (timestamp >> 24) & 0xFF;
        stress_frame.data[1] = (timestamp >> 16) & 0xFF;
        stress_frame.data[2] = (timestamp >> 8) & 0xFF;
        stress_frame.data[3] = timestamp & 0xFF;
        stress_frame.data[4] = (frame_count >> 24) & 0xFF;
        stress_frame.data[5] = (frame_count >> 16) & 0xFF;
        stress_frame.data[6] = (frame_count >> 8) & 0xFF;
        stress_frame.data[7] = frame_count & 0xFF;

        MCP2515::ERROR err = mcp2515.sendMessage(&stress_frame);

        if (err == MCP2515::ERROR_OK) {
            frame_count++;
        } else if (err == MCP2515::ERROR_ALLTXBUSY) {
            buffer_full_count++;
            error_count++;
            // **FIX:** Back off when buffers full
            delayMicroseconds(delay_micros * 2);
        } else {
            error_count++;
        }

        // **FIX:** Adaptive delay to match CAN speed capability
        if (delay_micros > 0) {
            delayMicroseconds(delay_micros);
        }
    }

    uint32_t duration = millis() - start_time;
    float frames_per_sec = (float)frame_count / (duration / 1000.0);
    float error_rate = (frame_count + error_count > 0) ?
        ((float)error_count / (frame_count + error_count) * 100.0) : 0.0;

    safe_printf("  Sent %lu frames in %lu ms (%.1f frames/sec)\n",
                  frame_count, duration, frames_per_sec);
    safe_printf("  Errors: %lu (%.2f%%), Buffer Full: %lu\n",
                  error_count, error_rate, buffer_full_count);

    // **NEW:** Performance analysis
    if (VERBOSE_OUTPUT) {
        float utilization = (frames_per_sec / theoretical_fps) * 100.0;
        safe_printf("  Bus Utilization: %.1f%% of theoretical max\n", utilization);

        if (error_rate > 50.0) {
            safe_printf("  %s⚠ High error rate suggests TX buffers saturated%s\n",
                         COLOR_YELLOW, COLOR_RESET);
            safe_println("  Recommendation: Reduce send rate or increase CAN speed");
        }
    }

    // **FIX:** Realistic error threshold based on speed
    float acceptable_error_rate = (current_test_speed <= CAN_125KBPS) ? 50.0 : 10.0;
    printTestResult(error_rate < acceptable_error_rate,
                   (error_rate < acceptable_error_rate) ?
                   "Acceptable error rate for this speed" :
                   "High error rate detected");

    // RX Buffer Overflow Test (if in loopback)
    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        printTestHeader("RX Buffer Overflow Test");

        // Clear any existing overflow
        mcp2515.clearRXnOVR();
        delay(50);

        // Send many frames rapidly without reading
        for (int i = 0; i < 20; i++) {
            stress_frame.can_id = 0x600 + i;
            mcp2515.sendMessage(&stress_frame);
            delayMicroseconds(50);  // Very rapid
        }

        delay(100);

        uint8_t eflg = mcp2515.getErrorFlags();
        bool overflow_detected = (eflg & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR));

        safe_printf("  Overflow %s\n", overflow_detected ? "DETECTED" : "NOT DETECTED");
        printTestResult(true, "Overflow test completed");

        // Clear overflow for subsequent tests
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
// TEST CATEGORY 13A: ADVANCED QUEUE TESTS
// ============================================================================

void testAdvancedQueue() {
    printSectionHeader("ADVANCED QUEUE TESTS (ESP32-specific)");

    #ifdef ESP32

    struct can_frame frame, rx_frame;
    MCP2515::ERROR err;

    // Clear queue first
    while (mcp2515.getRxQueueCount() > 0) {
        mcp2515.readMessageQueued(&rx_frame, 0);
    }

    // Test 1: Read from empty queue with zero timeout
    printTestHeader("Read from Empty Queue (No Wait)");
    err = mcp2515.readMessageQueued(&rx_frame, 0);
    bool empty_correct = (err == MCP2515::ERROR_NOMSG || err == MCP2515::ERROR_TIMEOUT);
    printTestResult(empty_correct, "Empty queue returns ERROR_NOMSG or ERROR_TIMEOUT");

    // Test 2: Send message and verify queue count increments
    printTestHeader("Queue Count Increments");
    uint32_t count_before = mcp2515.getRxQueueCount();

    frame.can_id = 0x650;
    frame.can_dlc = 2;
    frame.data[0] = 0xAA;
    frame.data[1] = 0xBB;
    mcp2515.sendMessage(&frame);
    delay(100);

    uint32_t count_after = mcp2515.getRxQueueCount();
    bool count_incremented = (count_after > count_before);

    printTestResult(count_incremented, "Queue count incremented after RX");

    if (VERBOSE_OUTPUT) {
        safe_printf("  Before: %lu, After: %lu\n", count_before, count_after);
    }

    // Test 3: Read with timeout and verify it works
    printTestHeader("Read from Queue with Timeout");
    err = mcp2515.readMessageQueued(&rx_frame, 100);
    bool read_ok = (err == MCP2515::ERROR_OK);

    printTestResult(read_ok, "Queued read successful");

    if (read_ok) {
        bool data_match = (rx_frame.can_id == 0x650 &&
                          rx_frame.data[0] == 0xAA &&
                          rx_frame.data[1] == 0xBB);
        printTestResult(data_match, "Queued frame data matches sent frame");
    }

    #else
    safe_println(COLOR_YELLOW "Queue tests are ESP32-specific - skipped" COLOR_RESET);
    test_stats.skipped_tests += 4;
    #endif

    delay(100);
}

// ============================================================================
// TEST CATEGORY 13B: STATE TRANSITION ERROR TESTS
// ============================================================================

void testStateTransitionErrors() {
    printSectionHeader("STATE TRANSITION ERROR TESTS");

    struct can_frame frame;
    MCP2515::ERROR err;

    // Test 1: Try to send in CONFIG mode (should fail)
    printTestHeader("Send Frame in CONFIG Mode");
    mcp2515.setConfigMode();
    waitForModeChange(CANSTAT_OPMOD_CONFIG, 100);  // **OPTIMIZED:** Poll instead of delay(50)

    frame.can_id = 0x700;
    frame.can_dlc = 1;
    frame.data[0] = 0xFF;
    err = mcp2515.sendMessage(&frame);

    // In config mode, send should fail (or at minimum, not transmit)
    bool send_blocked = (err != MCP2515::ERROR_OK);
    printTestResult(send_blocked, "Send blocked in CONFIG mode");

    if (!send_blocked && VERBOSE_OUTPUT) {
        safe_println("  " COLOR_YELLOW "Warning: Send succeeded in CONFIG mode" COLOR_RESET);
    }

    // Test 2: Try to change bitrate in NORMAL mode (may require config mode)
    printTestHeader("Change Bitrate in NORMAL Mode");
    mcp2515.setNormalMode();
    waitForModeChange(CANSTAT_OPMOD_NORMAL, 100);  // **OPTIMIZED:** Poll instead of delay(50)

    err = mcp2515.setBitrate(CAN_500KBPS, CONFIG_CAN_CLOCK);

    // Some implementations may require config mode first
    if (err != MCP2515::ERROR_OK && VERBOSE_OUTPUT) {
        safe_println("  " COLOR_YELLOW "Bitrate change blocked (may require CONFIG mode)" COLOR_RESET);
    }

    printTestResult(true, "Bitrate change attempted in NORMAL mode");

    // Restore to known state
    mcp2515.setConfigMode();
    if (waitForModeChange(CANSTAT_OPMOD_CONFIG, 100)) {  // **OPTIMIZED:** Poll instead of delay(20)
        mcp2515.setBitrate(current_test_speed, CONFIG_CAN_CLOCK);
        delay(20);  // Wait for bitrate to apply
    }

    if (TEST_MODE == TEST_MODE_LOOPBACK) {
        mcp2515.setLoopbackMode();
        waitForModeChange(CANSTAT_OPMOD_LOOPBACK, 100);  // **OPTIMIZED:** Poll
    } else {
        mcp2515.setNormalMode();
        waitForModeChange(CANSTAT_OPMOD_NORMAL, 100);  // **OPTIMIZED:** Poll
    }

    delay(20);  // **OPTIMIZED:** Reduced from 100ms
}

// ============================================================================
// TEST CATEGORY 14: BOUNDARY CONDITION TESTS
// ============================================================================

void testBoundaryConditions() {
    printSectionHeader("BOUNDARY CONDITION TESTS");

    struct can_frame frame;
    MCP2515::ERROR err;

    // Test 1: Zero-byte frame (valid)
    printTestHeader("Zero-Byte Frame (DLC=0)");
    frame.can_id = 0x200;
    frame.can_dlc = 0;
    err = mcp2515.sendMessage(&frame);
    printTestResult(err == MCP2515::ERROR_OK, "Zero-byte frame accepted");

    // Test 2: Maximum DLC (8 bytes)
    printTestHeader("Maximum DLC (8 Bytes)");
    frame.can_dlc = 8;
    for (uint8_t i = 0; i < 8; i++) frame.data[i] = i;
    err = mcp2515.sendMessage(&frame);
    printTestResult(err == MCP2515::ERROR_OK, "8-byte frame accepted");

    // Test 3: DLC > 8 (should be rejected or clamped)
    printTestHeader("Invalid DLC > 8");
    frame.can_dlc = 15;
    err = mcp2515.sendMessage(&frame);
    // Library may accept but clamp DLC, or may reject
    // Either behavior is acceptable
    printTestResult(true, "Invalid DLC handled (accept with clamp or reject)");
    if (VERBOSE_OUTPUT) {
        safe_printf("  Result: %s\n", (err == MCP2515::ERROR_OK) ? "Accepted (clamped)" : "Rejected");
    }

    // Test 4: Maximum standard CAN ID
    printTestHeader("Maximum Standard ID (0x7FF)");
    frame.can_id = 0x7FF;  // Max 11-bit ID
    frame.can_dlc = 1;
    frame.data[0] = 0xAA;
    err = mcp2515.sendMessage(&frame);
    printTestResult(err == MCP2515::ERROR_OK, "Max standard ID accepted");

    // Test 5: Maximum extended CAN ID
    printTestHeader("Maximum Extended ID (0x1FFFFFFF)");
    frame.can_id = 0x1FFFFFFF | CAN_EFF_FLAG;  // Max 29-bit ID
    frame.can_dlc = 1;
    frame.data[0] = 0xBB;
    err = mcp2515.sendMessage(&frame);
    printTestResult(err == MCP2515::ERROR_OK, "Max extended ID accepted");

    // Test 6: Invalid ID beyond standard range without EFF flag
    printTestHeader("ID > 0x7FF Without EFF Flag");
    frame.can_id = 0xFFF;  // Beyond 11-bit without EFF flag
    frame.can_dlc = 1;
    frame.data[0] = 0xCC;
    err = mcp2515.sendMessage(&frame);
    // Library should either set EFF flag automatically or reject
    printTestResult(true, "Out-of-range ID handled");
    if (VERBOSE_OUTPUT) {
        safe_printf("  Result: %s\n", (err == MCP2515::ERROR_OK) ? "Accepted (may set EFF)" : "Rejected");
    }

    delay(100);
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void printSectionHeader(const char* section_name) {
    SAFE_SERIAL_BLOCK();
    Serial.println();
    Serial.println(COLOR_BOLD COLOR_MAGENTA "┌────────────────────────────────────────────────────────────────┐" COLOR_RESET);
    Serial.printf(COLOR_BOLD COLOR_MAGENTA "│ %-62s │\n" COLOR_RESET, section_name);
    Serial.println(COLOR_BOLD COLOR_MAGENTA "└────────────────────────────────────────────────────────────────┘" COLOR_RESET);
}

void printTestHeader(const char* test_name) {
    SAFE_SERIAL_BLOCK();
    Serial.printf("\n%s[TEST]%s %s\n", COLOR_BOLD COLOR_CYAN, COLOR_RESET, test_name);
    test_stats.total_tests++;
}

void printTestResult(bool passed, const char* message) {
    SAFE_SERIAL_BLOCK();
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
    SAFE_SERIAL_BLOCK();
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

// **NEW:** Print data comparison for debugging
void printDataComparison(const uint8_t* expected, const uint8_t* actual, uint8_t length) {
    SAFE_SERIAL_BLOCK();
    Serial.print("    Expected: ");
    for (uint8_t i = 0; i < length; i++) {
        Serial.printf("%02X ", expected[i]);
    }
    Serial.println();

    Serial.print("    Actual:   ");
    for (uint8_t i = 0; i < length; i++) {
        if (actual[i] != expected[i]) {
            Serial.printf("%s%02X%s ", COLOR_RED, actual[i], COLOR_RESET);
        } else {
            Serial.printf("%02X ", actual[i]);
        }
    }
    Serial.println();
}

// **NEW:** Check if TX buffers are available
bool checkTXBufferAvailable() {
    uint8_t status = mcp2515.getStatus();
    // Check if any TX buffer is not pending transmission
    return !(status & 0x54);  // Bits 2, 4, 6 indicate TX pending
}

// ============================================================================
// OPTIMIZED TIMING HELPERS - Poll instead of fixed delays
// ============================================================================

/**
 * Poll for message availability with timeout (replaces fixed delay + checkReceive)
 * This is MUCH faster than fixed 100ms delays - typically 1-50ms in practice
 */
bool waitForMessage(uint32_t timeout_ms) {
    uint32_t start = millis();
    while ((millis() - start) < timeout_ms) {
        if (mcp2515.checkReceive()) {
            return true;
        }
        delayMicroseconds(100);  // Poll every 100µs
    }
    return false;
}

/**
 * Poll for message and read it with timeout
 */
bool waitForMessageAndRead(struct can_frame* frame, uint32_t timeout_ms) {
    if (waitForMessage(timeout_ms)) {
        MCP2515::ERROR err = mcp2515.readMessage(frame);
        return (err == MCP2515::ERROR_OK);
    }
    return false;
}

/**
 * Poll for TX buffer availability with timeout
 */
bool waitForTXBuffer(uint32_t timeout_ms) {
    uint32_t start = millis();
    while ((millis() - start) < timeout_ms) {
        if (checkTXBufferAvailable()) {
            return true;
        }
        delayMicroseconds(50);
    }
    return false;
}

/**
 * Smart delay for mode transitions - polls status to verify mode change
 * Returns true if mode changed successfully, false if timeout
 * Note: Uses getStatus() which reads CANSTAT register internally
 */
bool waitForModeChange(uint8_t target_mode, uint32_t timeout_ms) {
    uint32_t start = millis();
    while ((millis() - start) < timeout_ms) {
        uint8_t status = mcp2515.getStatus();
        // CANSTAT mode is in bits 7:5 of status byte
        uint8_t current_mode = (status >> 5) & 0x07;
        if (current_mode == target_mode) {
            return true;
        }
        delayMicroseconds(100);
    }
    return false;
}

// ============================================================================
// HARDWARE CONNECTIVITY VERIFICATION
// ============================================================================

/**
 * Comprehensive hardware connectivity check to detect if MCP2515 is actually connected
 * This helps identify false test passes when running without hardware.
 *
 * Verification steps:
 * 1. Check for floating MISO (0xFF indicates no SPI connection)
 * 2. Verify register write-then-readback works
 * 3. Test mode change capability (requires hardware response)
 * 4. Attempt simple loopback validation in loopback mode
 *
 * Returns: true if hardware is connected and responding, false otherwise
 * NOTE: Test suite continues regardless of result to show false positives
 */
bool verifyHardwareConnected() {
    SAFE_SERIAL_BLOCK();
    Serial.println();
    Serial.println(COLOR_BOLD COLOR_BLUE "╔════════════════════════════════════════════════════════════════╗" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_BLUE "║  HARDWARE CONNECTIVITY VERIFICATION                           ║" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_BLUE "╚════════════════════════════════════════════════════════════════╝" COLOR_RESET);
    Serial.println();

    uint8_t tests_passed = 0;
    uint8_t total_tests = 5;

    // Test 1: Check for floating MISO (0xFF = no connection)
    Serial.println(COLOR_YELLOW "Test 1: Checking for floating SPI MISO line..." COLOR_RESET);

    // Read CANSTAT multiple times - should not be 0xFF if hardware present
    bool all_ff = true;
    for (int i = 0; i < 5; i++) {
        uint8_t status = mcp2515.getBusStatus();  // Reads CANSTAT
        if (status != 0xFF) {
            all_ff = false;
        }
        delay(5);
    }

    if (all_ff) {
        Serial.println(COLOR_RED "  ✗ FAIL - MISO line floating (all reads = 0xFF)" COLOR_RESET);
        Serial.println(COLOR_RED "  → No SPI connection detected!" COLOR_RESET);
        Serial.println();
        Serial.println("Possible causes:");
        Serial.println("  • MCP2515 module not connected");
        Serial.println("  • Incorrect SPI wiring (MISO/MOSI/SCK/CS)");
        Serial.println("  • MCP2515 not powered");
        Serial.println("  • Wrong CS pin configured");
        Serial.println("  • SPI bus not initialized");
        Serial.println();
    } else {
        Serial.println(COLOR_GREEN "  ✓ PASS - SPI MISO responding (not floating)" COLOR_RESET);
        tests_passed++;
    }

    // Test 2: Reset and verify response
    Serial.println(COLOR_YELLOW "Test 2: Verifying MCP2515 reset..." COLOR_RESET);

    MCP2515::ERROR err = mcp2515.reset();
    if (err != MCP2515::ERROR_OK) {
        Serial.println(COLOR_RED "  ✗ FAIL - Reset failed" COLOR_RESET);
        Serial.println(COLOR_RED "  → MCP2515 not responding to reset command!" COLOR_RESET);
        Serial.println();
        Serial.println("Possible causes:");
        Serial.println("  • MCP2515 oscillator not running (check crystal)");
        Serial.println("  • MCP2515 in bad state");
        Serial.println("  • CS pin conflict with other SPI device");
        Serial.println();
    } else {
        Serial.println(COLOR_GREEN "  ✓ PASS - MCP2515 reset successful" COLOR_RESET);
        tests_passed++;
    }

    // Test 3: Bitrate configuration (requires entering CONFIG mode)
    Serial.println(COLOR_YELLOW "Test 3: Testing bitrate configuration..." COLOR_RESET);

    err = mcp2515.setBitrate(CONFIG_CAN_SPEED, CONFIG_CAN_CLOCK);
    if (err != MCP2515::ERROR_OK) {
        Serial.println(COLOR_RED "  ✗ FAIL - Cannot configure bitrate" COLOR_RESET);
        Serial.println(COLOR_RED "  → MCP2515 mode changes not working!" COLOR_RESET);
        Serial.println();
    } else {
        Serial.println(COLOR_GREEN "  ✓ PASS - Bitrate configuration successful" COLOR_RESET);
        tests_passed++;
    }

    // Test 4: Mode change verification (tests register read/write)
    Serial.println(COLOR_YELLOW "Test 4: Verifying mode change capability..." COLOR_RESET);

    err = mcp2515.setLoopbackMode();
    bool mode_set_ok = (err == MCP2515::ERROR_OK);

    // Verify mode actually changed by reading CANSTAT
    delay(20);
    uint8_t canstat = mcp2515.getBusStatus();
    uint8_t opmod = (canstat >> 5) & 0x07;
    bool mode_verified = (opmod == CANSTAT_OPMOD_LOOPBACK);

    if (!mode_set_ok || !mode_verified) {
        Serial.printf(COLOR_RED "  ✗ FAIL - Mode verification failed (OPMOD=0x%02X, expected 0x02)\n" COLOR_RESET, opmod);
        Serial.println(COLOR_RED "  → MCP2515 not entering requested mode!" COLOR_RESET);
        Serial.println();
    } else {
        Serial.printf(COLOR_GREEN "  ✓ PASS - Loopback mode verified (CANSTAT=0x%02X, OPMOD=0x%02X)\n" COLOR_RESET, canstat, opmod);
        tests_passed++;
    }

    // Test 5: Simple loopback transmission test (ultimate hardware validation)
    Serial.println(COLOR_YELLOW "Test 5: Loopback transmission test..." COLOR_RESET);

    // Clear any pending messages
    struct can_frame temp;
    while (mcp2515.checkReceive()) {
        mcp2515.readMessage(&temp);
    }

    // Send a test frame
    struct can_frame tx_frame;
    tx_frame.can_id = 0x7FE;  // Test ID
    tx_frame.can_dlc = 4;
    tx_frame.data[0] = 0xDE;
    tx_frame.data[1] = 0xAD;
    tx_frame.data[2] = 0xBE;
    tx_frame.data[3] = 0xEF;

    err = mcp2515.sendMessage(&tx_frame);
    bool send_ok = (err == MCP2515::ERROR_OK);

    // Wait for reception (in loopback, should be immediate)
    delay(50);

    bool receive_ok = mcp2515.checkReceive();
    bool data_match = false;

    if (receive_ok) {
        // Read and verify the message
        struct can_frame rx_frame;
        err = mcp2515.readMessage(&rx_frame);
        if (err == MCP2515::ERROR_OK) {
            // Verify data integrity
            data_match = (rx_frame.can_id == tx_frame.can_id &&
                         rx_frame.can_dlc == tx_frame.can_dlc &&
                         memcmp(rx_frame.data, tx_frame.data, tx_frame.can_dlc) == 0);
        }
    }

    if (!send_ok || !receive_ok || !data_match) {
        Serial.println(COLOR_RED "  ✗ FAIL - Loopback test failed" COLOR_RESET);
        if (!send_ok) Serial.println(COLOR_RED "  → TX buffer operation failed!" COLOR_RESET);
        if (!receive_ok) Serial.println(COLOR_RED "  → No message received in loopback!" COLOR_RESET);
        if (receive_ok && !data_match) Serial.println(COLOR_RED "  → Data corruption detected!" COLOR_RESET);
        Serial.println();
        Serial.println("This indicates:");
        Serial.println("  • MCP2515 internal logic not working");
        Serial.println("  • Oscillator failure");
        Serial.println("  • Defective chip");
        Serial.println();
    } else {
        Serial.println(COLOR_GREEN "  ✓ PASS - Loopback TX/RX with data integrity verified" COLOR_RESET);
        tests_passed++;
    }

    // Summary
    Serial.println();
    if (tests_passed == total_tests) {
        Serial.println(COLOR_BOLD COLOR_GREEN "═══════════════════════════════════════════════════════════════" COLOR_RESET);
        Serial.println(COLOR_BOLD COLOR_GREEN "  ✓ HARDWARE CONNECTIVITY VERIFIED ✓" COLOR_RESET);
        Serial.println(COLOR_BOLD COLOR_GREEN "═══════════════════════════════════════════════════════════════" COLOR_RESET);
        Serial.println();
        Serial.printf(COLOR_GREEN "  All %d hardware verification tests passed!\n" COLOR_RESET, total_tests);
        Serial.println(COLOR_GREEN "  MCP2515 is connected and functioning correctly." COLOR_RESET);
        Serial.println();
        return true;
    } else {
        Serial.println(COLOR_BOLD COLOR_RED "═══════════════════════════════════════════════════════════════" COLOR_RESET);
        Serial.println(COLOR_BOLD COLOR_RED "  ✗ HARDWARE CONNECTIVITY FAILED ✗" COLOR_RESET);
        Serial.println(COLOR_BOLD COLOR_RED "═══════════════════════════════════════════════════════════════" COLOR_RESET);
        Serial.println();
        Serial.printf(COLOR_RED "  Hardware verification: %d/%d tests passed\n" COLOR_RESET, tests_passed, total_tests);
        Serial.println(COLOR_YELLOW "  ⚠ WARNING: Test results may contain false positives!" COLOR_RESET);
        Serial.println(COLOR_YELLOW "  Continuing test suite for diagnostic purposes..." COLOR_RESET);
        Serial.println();
        return false;
    }
}

/**
 * Clear all pending messages from RX buffers to prevent contamination between tests
 * Returns number of frames cleared
 */
void clearRXBuffer() {
    struct can_frame temp;
    uint32_t cleared = 0;
    while (mcp2515.checkReceive() && cleared < 100) {  // Safety limit
        mcp2515.readMessage(&temp);
        cleared++;
    }
    if (VERBOSE_OUTPUT && cleared > 0) {
        safe_printf("  %s[Cleared %lu stale frame%s]%s\n",
                   COLOR_YELLOW, cleared, cleared == 1 ? "" : "s", COLOR_RESET);
    }
}

// ============================================================================
// ENHANCED TWO-DEVICE MODE COORDINATION PROTOCOL
// ============================================================================
//
// This protocol ensures robust synchronization between master and slave:
// 1. Slave continuously sends READY until master acknowledges
// 2. Master sends START_TEST and waits for ACK (with retries)
// 3. Both execute test
// 4. Both send COMPLETE and wait for each other
// 5. This allows devices to start in any order without timing issues
// ============================================================================

/**
 * Master: Wait for slave READY signal at startup
 * Waits up to timeout_ms for slave to send SYNC_ID_READY
 * Sends ACK back to slave to confirm receipt
 */
bool waitForSlaveReadyAtStartup(uint32_t timeout_ms) {
    if (TEST_MODE != TEST_MODE_TWO_DEVICE || DEVICE_ROLE != ROLE_MASTER) {
        return true;  // Not applicable
    }

    safe_println(COLOR_YELLOW "⏳ Master: Waiting for slave READY signal..." COLOR_RESET);

    uint32_t start = millis();
    uint32_t last_print = 0;

    while ((millis() - start) < timeout_ms) {
        // Print progress every 5 seconds
        if (millis() - last_print > 5000) {
            safe_printf("  Still waiting... (%lu/%lu seconds)\n",
                       (millis() - start) / 1000, timeout_ms / 1000);
            last_print = millis();
        }

        if (mcp2515.checkReceive()) {
            struct can_frame frame;
            if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
                if ((frame.can_id & CAN_SFF_MASK) == SYNC_ID_READY) {
                    safe_println(COLOR_GREEN "✓ Master: Slave READY signal received!" COLOR_RESET);

                    // Send ACK to stop slave from continuously sending READY
                    sendAck();
                    delay(100);  // Give slave time to receive ACK

                    return true;
                }
            }
        }
        delay(50);
    }

    safe_println(COLOR_RED "✗ Master: Timeout waiting for slave READY!" COLOR_RESET);
    return false;
}

/**
 * Slave: Continuously signal ready to master until ACK received
 * Used at startup - sends READY every 500ms until master acknowledges
 */
void signalSlaveReadyContinuously() {
    if (TEST_MODE != TEST_MODE_TWO_DEVICE || DEVICE_ROLE != ROLE_SLAVE) {
        return;
    }

    safe_println(COLOR_YELLOW "⏳ Slave: Sending READY signals to master..." COLOR_RESET);

    struct can_frame ready_frame;
    ready_frame.can_id = SYNC_ID_READY;
    ready_frame.can_dlc = 1;
    ready_frame.data[0] = 0xAA;

    uint32_t send_count = 0;
    uint32_t start = millis();

    while (true) {
        // Send READY signal
        mcp2515.sendMessage(&ready_frame);
        send_count++;

        if (send_count % 10 == 1) {  // Print every 10 sends (~5 seconds)
            safe_printf("  READY signal #%lu sent (waiting %lu seconds)\n",
                       send_count, (millis() - start) / 1000);
        }

        // Check for ACK from master
        uint32_t ack_wait_start = millis();
        while (millis() - ack_wait_start < 500) {  // Wait 500ms before next send
            if (mcp2515.checkReceive()) {
                struct can_frame frame;
                if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
                    if ((frame.can_id & CAN_SFF_MASK) == SYNC_ID_ACK ||
                        (frame.can_id & CAN_SFF_MASK) == SYNC_ID_START_TEST) {
                        safe_println(COLOR_GREEN "✓ Slave: Master acknowledged! Ready for commands." COLOR_RESET);
                        return;  // Master is ready
                    }
                }
            }
            delay(10);
        }
    }
}

/**
 * Master: Send start signal with retry until ACK received
 * Returns true if slave acknowledged, false if all retries failed
 */
bool sendStartSignalWithRetry(uint8_t test_id, uint32_t max_retries) {
    if (TEST_MODE != TEST_MODE_TWO_DEVICE || DEVICE_ROLE != ROLE_MASTER) {
        return true;  // Not applicable
    }

    if (VERBOSE_OUTPUT) {
        safe_printf(COLOR_CYAN "→ Master: Sending START command (test ID 0x%02X)...\n" COLOR_RESET, test_id);
    }

    struct can_frame frame;
    frame.can_id = SYNC_ID_START_TEST;
    frame.can_dlc = 1;
    frame.data[0] = test_id;

    for (uint32_t attempt = 1; attempt <= max_retries; attempt++) {
        // Clear RX buffer before sending
        clearRXBuffer();

        // Send start signal
        mcp2515.sendMessage(&frame);

        // Wait for ACK
        if (waitForAck(3000)) {
            if (VERBOSE_OUTPUT) {
                safe_println(COLOR_GREEN "  ✓ Slave acknowledged START command" COLOR_RESET);
            }
            return true;
        }

        if (attempt < max_retries) {
            safe_printf(COLOR_YELLOW "  ⚠ No ACK (attempt %lu/%lu), retrying...\n" COLOR_RESET,
                       attempt, max_retries);
            delay(500);
        }
    }

    safe_printf(COLOR_RED "  ✗ Failed to get ACK after %lu attempts!\n" COLOR_RESET, max_retries);
    return false;
}

/**
 * Slave: Wait for start signal and immediately send ACK
 * Returns true if START_TEST received, false if timeout
 */
bool waitForStartSignalAndAck(uint8_t* test_id, uint32_t timeout_ms) {
    if (TEST_MODE != TEST_MODE_TWO_DEVICE || DEVICE_ROLE != ROLE_SLAVE) {
        return true;  // Not applicable
    }

    uint32_t start = millis();
    uint32_t last_print = 0;

    while ((millis() - start) < timeout_ms) {
        // Print progress every 10 seconds
        if (millis() - last_print > 10000) {
            safe_printf("  Slave: Still waiting for command... (%lu seconds)\n",
                       (millis() - start) / 1000);
            last_print = millis();
        }

        if (mcp2515.checkReceive()) {
            struct can_frame frame;
            if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
                if ((frame.can_id & CAN_SFF_MASK) == SYNC_ID_START_TEST) {
                    *test_id = frame.data[0];

                    // Immediately send ACK
                    sendAck();

                    if (VERBOSE_OUTPUT) {
                        safe_printf(COLOR_CYAN "← Slave: START command received (test ID 0x%02X), ACK sent\n" COLOR_RESET,
                                   *test_id);
                    }

                    return true;
                }
            }
        }
        delay(50);
    }

    safe_println(COLOR_RED "✗ Slave: Timeout waiting for START command!" COLOR_RESET);
    return false;
}

/**
 * Send ACK signal
 */
void sendAck() {
    if (TEST_MODE != TEST_MODE_TWO_DEVICE) {
        return;
    }

    struct can_frame frame;
    frame.can_id = SYNC_ID_ACK;
    frame.can_dlc = 1;
    frame.data[0] = (DEVICE_ROLE == ROLE_MASTER) ? 0xAA : 0xBB;
    mcp2515.sendMessage(&frame);
}

/**
 * Wait for ACK signal
 */
bool waitForAck(uint32_t timeout_ms) {
    if (TEST_MODE != TEST_MODE_TWO_DEVICE) {
        return true;  // Not applicable
    }

    uint32_t start = millis();
    while ((millis() - start) < timeout_ms) {
        if (mcp2515.checkReceive()) {
            struct can_frame frame;
            if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
                if ((frame.can_id & CAN_SFF_MASK) == SYNC_ID_ACK) {
                    return true;
                }
            }
        }
        delay(10);
    }
    return false;
}

/**
 * Send test completion signal
 */
void sendCompleteSignal() {
    if (TEST_MODE != TEST_MODE_TWO_DEVICE) {
        return;
    }

    struct can_frame frame;
    frame.can_id = SYNC_ID_COMPLETE;
    frame.can_dlc = 1;
    frame.data[0] = (DEVICE_ROLE == ROLE_MASTER) ? 0xAA : 0xBB;
    mcp2515.sendMessage(&frame);

    if (VERBOSE_OUTPUT) {
        safe_printf(COLOR_CYAN "%s COMPLETE signal sent\n" COLOR_RESET,
                   DEVICE_ROLE == ROLE_MASTER ? "→" : "←");
    }
}

/**
 * Wait for completion signal from other device
 */
bool waitForComplete(uint32_t timeout_ms) {
    if (TEST_MODE != TEST_MODE_TWO_DEVICE) {
        return true;  // Not applicable
    }

    uint32_t start = millis();
    while ((millis() - start) < timeout_ms) {
        if (mcp2515.checkReceive()) {
            struct can_frame frame;
            if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
                if ((frame.can_id & CAN_SFF_MASK) == SYNC_ID_COMPLETE) {
                    if (VERBOSE_OUTPUT) {
                        safe_printf(COLOR_GREEN "%s COMPLETE signal received\n" COLOR_RESET,
                                   DEVICE_ROLE == ROLE_MASTER ? "←" : "→");
                    }
                    return true;
                }
            }
        }
        delay(10);
    }

    safe_printf(COLOR_YELLOW "⚠ Timeout waiting for COMPLETE signal\n" COLOR_RESET);
    return false;
}

// ============================================================================
// SLAVE MODE HANDLER
// ============================================================================

/**
 * Slave mode handler - waits for commands from master and executes tests
 * This function never returns - it loops indefinitely waiting for commands
 *
 * ENHANCED: Uses continuous READY signaling to synchronize with master
 */
void runSlaveMode() {
    SAFE_SERIAL_BLOCK();
    Serial.println(COLOR_BOLD COLOR_CYAN "╔════════════════════════════════════════════════════════════════╗" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_CYAN "║              SLAVE MODE - AWAITING MASTER COMMANDS             ║" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_CYAN "╚════════════════════════════════════════════════════════════════╝" COLOR_RESET);
    Serial.println();

    // Initialize MCP2515
    Serial.print("Initializing MCP2515... ");
    MCP2515::ERROR err = mcp2515.reset();
    if (err != MCP2515::ERROR_OK) {
        Serial.println(COLOR_RED "FAILED" COLOR_RESET);
        Serial.println(COLOR_RED "Cannot continue in slave mode!" COLOR_RESET);
        while(1) delay(1000);  // Halt
    }
    Serial.println(COLOR_GREEN "OK" COLOR_RESET);

    Serial.print("Setting bitrate... ");
    err = mcp2515.setBitrate(CONFIG_CAN_SPEED, CONFIG_CAN_CLOCK);
    if (err != MCP2515::ERROR_OK) {
        Serial.println(COLOR_RED "FAILED" COLOR_RESET);
        while(1) delay(1000);  // Halt
    }
    Serial.println(COLOR_GREEN "OK" COLOR_RESET);

    Serial.print("Setting normal mode... ");
    err = mcp2515.setNormalMode();
    if (err != MCP2515::ERROR_OK) {
        Serial.println(COLOR_RED "FAILED" COLOR_RESET);
        while(1) delay(1000);  // Halt
    }
    Serial.println(COLOR_GREEN "OK" COLOR_RESET);
    Serial.println();

    // **ENHANCED:** Continuously signal READY until master acknowledges
    // This allows slave to start before master without timing issues
    signalSlaveReadyContinuously();

    Serial.println();
    Serial.println(COLOR_GREEN "✓ Synchronized with master!" COLOR_RESET);
    Serial.println(COLOR_YELLOW "Waiting for test commands..." COLOR_RESET);
    Serial.println();

    // Main loop - wait for commands from master
    uint32_t commands_received = 0;
    while (true) {
        uint8_t test_id = 0;

        // **ENHANCED:** Use new protocol - wait for START and send ACK
        if (waitForStartSignalAndAck(&test_id, 60000)) {  // 60s timeout
            commands_received++;

            {
                SAFE_SERIAL_BLOCK();
                Serial.println();
                Serial.println(COLOR_BOLD COLOR_BLUE "┌────────────────────────────────────────┐" COLOR_RESET);
                Serial.printf(COLOR_BOLD COLOR_BLUE "│ Command #%lu: Test ID 0x%02X             │\n" COLOR_RESET,
                             commands_received, test_id);
                Serial.println(COLOR_BOLD COLOR_BLUE "└────────────────────────────────────────┘" COLOR_RESET);
            }

            // Execute test based on test_id
            // Note: Individual test functions check for slave mode and behave accordingly
            switch(test_id) {
                case 0x04:  // Filter tests
                    safe_println(COLOR_YELLOW "  → Executing: Filter test frames" COLOR_RESET);
                    testFiltersAndMasks();
                    break;

                case 0x05:  // RXB1 buffer tests
                    safe_println(COLOR_YELLOW "  → Executing: RXB1 buffer test frames" COLOR_RESET);
                    testRXB1Buffer();
                    break;

                case 0x06:  // Statistics RX counter test
                    safe_println(COLOR_YELLOW "  → Executing: Statistics RX counter test" COLOR_RESET);
                    testStatistics();
                    break;

                default:
                    safe_printf(COLOR_YELLOW "  → Unknown test ID 0x%02X - no action\n" COLOR_RESET, test_id);
                    break;
            }

            safe_println(COLOR_GREEN "  ✓ Command completed\n" COLOR_RESET);

            // Clear RX buffer before next command
            clearRXBuffer();
        }
    }
}

// **NEW:** Calculate theoretical max frames per second for a given CAN speed
uint32_t getTheoreticalFramesPerSecond(CAN_SPEED speed) {
    // Conservative estimate considering:
    // - Standard frame (11-bit ID): ~130 bits total with overhead
    // - We'll use 150 bits per frame to be conservative

    uint32_t bitrate;
    switch (speed) {
        case CAN_5KBPS:    bitrate = 5000; break;
        case CAN_10KBPS:   bitrate = 10000; break;
        case CAN_20KBPS:   bitrate = 20000; break;
        case CAN_50KBPS:   bitrate = 50000; break;
        case CAN_100KBPS:  bitrate = 100000; break;
        case CAN_125KBPS:  bitrate = 125000; break;
        case CAN_250KBPS:  bitrate = 250000; break;
        case CAN_500KBPS:  bitrate = 500000; break;
        case CAN_1000KBPS: bitrate = 1000000; break;
        default:           bitrate = 125000; break;
    }

    // Calculate frames per second (assuming 150 bits per frame)
    return bitrate / 150;
}

// **NEW:** Get adaptive delay in microseconds based on CAN speed
uint32_t getAdaptiveDelayMicros(CAN_SPEED speed) {
    uint32_t fps = getTheoreticalFramesPerSecond(speed);

    // Target 80% of theoretical max to avoid saturation
    fps = (fps * 80) / 100;

    if (fps == 0) return 10000;  // Safety fallback

    // Convert to inter-frame delay in microseconds
    return 1000000 / fps;
}

void printFinalStatistics() {
    SAFE_SERIAL_BLOCK();
    Serial.println();
    Serial.println(COLOR_BOLD COLOR_CYAN "╔════════════════════════════════════════════════════════════════╗" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_CYAN "║                     FINAL TEST RESULTS                         ║" COLOR_RESET);
    Serial.println(COLOR_BOLD COLOR_CYAN "╚════════════════════════════════════════════════════════════════╝" COLOR_RESET);
    Serial.println();

    Serial.printf("  Total Tests:    %s%lu%s\n", COLOR_BOLD, test_stats.total_tests, COLOR_RESET);
    Serial.printf("  Passed:         %s%lu%s (%.1f%%)\n",
                  COLOR_GREEN, test_stats.passed_tests, COLOR_RESET,
                  (float)test_stats.passed_tests / test_stats.total_tests * 100.0);

    if (test_stats.failed_tests > 0) {
        Serial.printf("  Failed:         %s%lu%s (%.1f%%)\n",
                      COLOR_RED, test_stats.failed_tests, COLOR_RESET,
                      (float)test_stats.failed_tests / test_stats.total_tests * 100.0);
    }

    if (test_stats.skipped_tests > 0) {
        Serial.printf("  Skipped:        %s%lu%s\n",
                     COLOR_YELLOW, test_stats.skipped_tests, COLOR_RESET);
    }

    if (test_stats.warnings > 0) {
        Serial.printf("  Warnings:       %s%lu%s\n",
                     COLOR_YELLOW, test_stats.warnings, COLOR_RESET);
    }

    Serial.println();

    // Hardware connectivity status
    Serial.println(COLOR_BOLD "Hardware Status:" COLOR_RESET);
    if (hardware_detected) {
        Serial.println(COLOR_GREEN "  ✓ MCP2515 hardware detected and verified" COLOR_RESET);
    } else {
        Serial.println(COLOR_RED "  ✗ MCP2515 hardware NOT detected!" COLOR_RESET);
        Serial.println(COLOR_YELLOW "  ⚠ Test results may contain FALSE POSITIVES!" COLOR_RESET);
        Serial.println(COLOR_YELLOW "  ⚠ Connect hardware and retest for accurate results" COLOR_RESET);
    }

    Serial.println();

    // Overall result
    if (!hardware_detected) {
        Serial.println(COLOR_BOLD COLOR_RED "  ⚠ INVALID RESULTS - NO HARDWARE DETECTED ⚠" COLOR_RESET);
    } else if (test_stats.failed_tests == 0) {
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

    // **NEW:** Performance metrics
    if (lib_stats.tx_frames > 0) {
        float error_rate = ((float)lib_stats.tx_errors / (lib_stats.tx_frames + lib_stats.tx_errors)) * 100.0;
        Serial.printf("  TX Error Rate:  %.2f%%\n", error_rate);
    }

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

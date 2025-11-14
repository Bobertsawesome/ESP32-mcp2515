/**
 * @file ESP32_CAN_advanced.ino
 * @brief ESP32 Advanced CAN Example with FreeRTOS Tasks
 *
 * This example demonstrates advanced features:
 * - Separate FreeRTOS tasks for TX and RX
 * - CAN filtering
 * - Error recovery
 * - Power management
 * - Statistics monitoring
 *
 * Hardware Setup:
 * - MCP2515 CS   -> GPIO 5
 * - MCP2515 INT  -> GPIO 4
 * - MCP2515 MOSI -> GPIO 23
 * - MCP2515 MISO -> GPIO 19
 * - MCP2515 SCK  -> GPIO 18
 * - LED          -> GPIO 2 (built-in LED)
 */

#include <mcp2515.h>

// Pin definitions
#define CAN_CS_PIN    GPIO_NUM_5
#define CAN_INT_PIN   GPIO_NUM_4
#define LED_PIN       GPIO_NUM_2

// CAN IDs for filtering
#define CAN_ID_SENSOR1   0x100
#define CAN_ID_SENSOR2   0x101
#define CAN_ID_COMMAND   0x200

// Task handles
TaskHandle_t rxTaskHandle = NULL;
TaskHandle_t txTaskHandle = NULL;
TaskHandle_t monitorTaskHandle = NULL;

// MCP2515 instance with full configuration
MCP2515 mcp2515(CAN_CS_PIN, CAN_INT_PIN);

// Shared data (protected by FreeRTOS)
SemaphoreHandle_t dataMutex = NULL;
volatile uint32_t sensor1_value = 0;
volatile uint32_t sensor2_value = 0;
volatile bool led_state = false;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("========================================");
    Serial.println("ESP32 MCP2515 Advanced CAN Example");
    Serial.println("FreeRTOS Multi-Task with Filtering");
    Serial.println("========================================");

    // Configure LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Create mutex for shared data
    dataMutex = xSemaphoreCreateMutex();

    // Initialize MCP2515
    Serial.print("Initializing MCP2515... ");
    if (mcp2515.reset() != MCP2515::ERROR_OK) {
        Serial.println("FAILED!");
        while (1) delay(1000);
    }
    Serial.println("OK");

    // Set bitrate
    Serial.print("Setting bitrate... ");
    if (mcp2515.setBitrate(CAN_125KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) {
        Serial.println("FAILED!");
        while (1) delay(1000);
    }
    Serial.println("OK");

    // Configure filters to accept specific IDs
    Serial.println("Configuring CAN filters...");

    // Filter 0: Accept sensor 1 (0x100)
    mcp2515.setFilter(MCP2515::RXF0, false, CAN_ID_SENSOR1);

    // Filter 1: Accept sensor 2 (0x101)
    mcp2515.setFilter(MCP2515::RXF1, false, CAN_ID_SENSOR2);

    // Filter 2: Accept commands (0x200)
    mcp2515.setFilter(MCP2515::RXF2, false, CAN_ID_COMMAND);

    // Set masks to check all bits
    mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
    mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);

    Serial.println("Filters configured");

    // Set normal mode
    Serial.print("Setting normal mode... ");
    if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
        Serial.println("FAILED!");
        while (1) delay(1000);
    }
    Serial.println("OK");

    // Create FreeRTOS tasks
    Serial.println("\nCreating FreeRTOS tasks...");

    xTaskCreatePinnedToCore(
        rxTask,           // Task function
        "CAN_RX",         // Task name
        4096,             // Stack size
        NULL,             // Parameters
        2,                // Priority
        &rxTaskHandle,    // Task handle
        1                 // Core 1
    );

    xTaskCreatePinnedToCore(
        txTask,
        "CAN_TX",
        4096,
        NULL,
        2,
        &txTaskHandle,
        0                 // Core 0
    );

    xTaskCreatePinnedToCore(
        monitorTask,
        "Monitor",
        4096,
        NULL,
        1,
        &monitorTaskHandle,
        1                 // Core 1
    );

    Serial.println("All systems ready!");
    Serial.println("========================================\n");
}

void loop() {
    // Main loop does nothing - all work in FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// RX Task - Receives and processes CAN frames
void rxTask(void* pvParameters) {
    struct can_frame frame;

    Serial.println("[RX Task] Started on core " + String(xPortGetCoreID()));

    while (true) {
        // Wait for frame with 100ms timeout
        MCP2515::ERROR result = mcp2515.readMessageQueued(&frame, 100);

        if (result == MCP2515::ERROR_OK) {
            // Process based on CAN ID
            if ((frame.can_id & CAN_EFF_MASK) == CAN_ID_SENSOR1 && frame.can_dlc == 4) {
                uint32_t value = (frame.data[0] << 24) | (frame.data[1] << 16) |
                                 (frame.data[2] << 8) | frame.data[3];

                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    sensor1_value = value;
                    xSemaphoreGive(dataMutex);
                }

                Serial.printf("[RX] Sensor1: %lu\n", value);
            }
            else if ((frame.can_id & CAN_EFF_MASK) == CAN_ID_SENSOR2 && frame.can_dlc == 4) {
                uint32_t value = (frame.data[0] << 24) | (frame.data[1] << 16) |
                                 (frame.data[2] << 8) | frame.data[3];

                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    sensor2_value = value;
                    xSemaphoreGive(dataMutex);
                }

                Serial.printf("[RX] Sensor2: %lu\n", value);
            }
            else if ((frame.can_id & CAN_EFF_MASK) == CAN_ID_COMMAND && frame.can_dlc >= 1) {
                // Command byte 0: LED control
                if (frame.data[0] == 0x01) {
                    digitalWrite(LED_PIN, HIGH);
                    led_state = true;
                    Serial.println("[RX] Command: LED ON");
                } else if (frame.data[0] == 0x00) {
                    digitalWrite(LED_PIN, LOW);
                    led_state = false;
                    Serial.println("[RX] Command: LED OFF");
                }
            }
        } else if (result != MCP2515::ERROR_TIMEOUT) {
            Serial.printf("[RX] Error: %d\n", result);
        }

        // Small yield to other tasks
        taskYIELD();
    }
}

// TX Task - Sends periodic status messages
void txTask(void* pvParameters) {
    struct can_frame frame;
    uint32_t counter = 0;

    Serial.println("[TX Task] Started on core " + String(xPortGetCoreID()));

    while (true) {
        // Send status frame every second
        frame.can_id = 0x300;  // Status message ID
        frame.can_dlc = 8;

        // Counter
        frame.data[0] = (counter >> 24) & 0xFF;
        frame.data[1] = (counter >> 16) & 0xFF;
        frame.data[2] = (counter >> 8) & 0xFF;
        frame.data[3] = counter & 0xFF;

        // LED state
        frame.data[4] = led_state ? 0x01 : 0x00;

        // Free heap
        uint32_t free_heap = ESP.getFreeHeap();
        frame.data[5] = (free_heap >> 16) & 0xFF;
        frame.data[6] = (free_heap >> 8) & 0xFF;
        frame.data[7] = free_heap & 0xFF;

        MCP2515::ERROR result = mcp2515.sendMessage(&frame);

        if (result == MCP2515::ERROR_OK) {
            Serial.printf("[TX] Status #%lu sent\n", counter);
        } else {
            Serial.printf("[TX] Send failed: %d\n", result);

            // Attempt error recovery
            if (result == MCP2515::ERROR_ALLTXBUSY) {
                mcp2515.performErrorRecovery();
            }
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Monitor Task - Displays statistics
void monitorTask(void* pvParameters) {
    Serial.println("[Monitor Task] Started on core " + String(xPortGetCoreID()));

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));  // Every 5 seconds

        Serial.println("\n========== CAN Statistics ==========");

        // Get statistics
        mcp2515_statistics_t stats;
        mcp2515.getStatistics(&stats);

        Serial.printf("RX Frames:     %lu\n", stats.rx_frames);
        Serial.printf("TX Frames:     %lu\n", stats.tx_frames);
        Serial.printf("RX Queue Size: %lu\n", mcp2515.getRxQueueCount());
        Serial.printf("RX Errors:     %lu\n", stats.rx_errors);
        Serial.printf("TX Errors:     %lu\n", stats.tx_errors);
        Serial.printf("RX Overflows:  %lu\n", stats.rx_overflow);
        Serial.printf("Bus Errors:    %lu\n", stats.bus_errors);
        Serial.printf("Bus-Off:       %lu\n", stats.bus_off_count);

        // Error counters
        Serial.printf("\nError Counts - RX: %d, TX: %d\n",
                      mcp2515.errorCountRX(),
                      mcp2515.errorCountTX());

        // Sensor values
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            Serial.printf("\nSensor Values:\n");
            Serial.printf("  Sensor 1: %lu\n", sensor1_value);
            Serial.printf("  Sensor 2: %lu\n", sensor2_value);
            xSemaphoreGive(dataMutex);
        }

        // System info
        Serial.printf("\nSystem Info:\n");
        Serial.printf("  Free Heap: %lu bytes\n", ESP.getFreeHeap());
        Serial.printf("  Min Free Heap: %lu bytes\n", ESP.getMinFreeHeap());
        Serial.printf("  LED State: %s\n", led_state ? "ON" : "OFF");

        Serial.println("====================================\n");
    }
}

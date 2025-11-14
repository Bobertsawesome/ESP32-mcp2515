/**
 * @file ESP32_CAN_read.ino
 * @brief ESP32 Arduino CAN Frame Reception Example
 *
 * This example demonstrates interrupt-driven CAN reception on ESP32
 * using the refactored MCP2515 library with FreeRTOS queues.
 *
 * Hardware Setup:
 * - MCP2515 CS   -> GPIO 5
 * - MCP2515 INT  -> GPIO 4
 * - MCP2515 MOSI -> GPIO 23
 * - MCP2515 MISO -> GPIO 19
 * - MCP2515 SCK  -> GPIO 18
 * - CAN_H and CAN_L connected to CAN bus with 120Ω termination
 */

#include <mcp2515.h>

// Pin definitions
#define CAN_CS_PIN    GPIO_NUM_5
#define CAN_INT_PIN   GPIO_NUM_4

// Create MCP2515 instance with interrupt support
MCP2515 mcp2515(CAN_CS_PIN, CAN_INT_PIN);

void setup() {
    Serial.begin(115200);
    while (!Serial);  // Wait for serial connection

    Serial.println("========================================");
    Serial.println("ESP32 MCP2515 CAN Receive Example");
    Serial.println("Interrupt-Driven with FreeRTOS Queue");
    Serial.println("========================================");

    // Initialize MCP2515
    Serial.print("Initializing MCP2515... ");
    if (mcp2515.reset() != MCP2515::ERROR_OK) {
        Serial.println("FAILED!");
        Serial.println("Check SPI connections!");
        while (1) delay(1000);
    }
    Serial.println("OK");

    // Set bitrate (125 kbps @ 16 MHz crystal)
    Serial.print("Setting bitrate to 125 kbps... ");
    if (mcp2515.setBitrate(CAN_125KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) {
        Serial.println("FAILED!");
        while (1) delay(1000);
    }
    Serial.println("OK");

    // Set normal mode (send ACKs, fully active on bus)
    Serial.print("Setting normal mode... ");
    if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
        Serial.println("FAILED!");
        while (1) delay(1000);
    }
    Serial.println("OK");

    Serial.println("\n--- Waiting for CAN frames ---");
    Serial.println("Format: [ID] [DLC] [DATA]");
    Serial.println();
}

void loop() {
    struct can_frame frame;

    // Read from queue with 100ms timeout
    // This is non-blocking and works with interrupt-driven reception
    MCP2515::ERROR result = mcp2515.readMessageQueued(&frame, 100);

    if (result == MCP2515::ERROR_OK) {
        // Print CAN ID (in hex)
        Serial.print("0x");
        if (frame.can_id < 0x100) Serial.print("0");
        if (frame.can_id < 0x10) Serial.print("0");
        Serial.print(frame.can_id, HEX);
        Serial.print("  ");

        // Print DLC
        Serial.print(frame.can_dlc);
        Serial.print("  ");

        // Print data bytes
        for (int i = 0; i < frame.can_dlc; i++) {
            if (frame.data[i] < 0x10) Serial.print("0");
            Serial.print(frame.data[i], HEX);
            Serial.print(" ");
        }

        // Show if extended frame or RTR
        if (frame.can_id & CAN_EFF_FLAG) Serial.print(" [EXT]");
        if (frame.can_id & CAN_RTR_FLAG) Serial.print(" [RTR]");

        Serial.println();
    }

    // Print statistics every 5 seconds
    static unsigned long last_stats = 0;
    if (millis() - last_stats > 5000) {
        last_stats = millis();

        mcp2515_statistics_t stats;
        mcp2515.getStatistics(&stats);

        Serial.println("\n--- Statistics ---");
        Serial.printf("RX Frames: %lu\n", stats.rx_frames);
        Serial.printf("RX Queue: %lu frames waiting\n", mcp2515.getRxQueueCount());
        Serial.printf("RX Errors: %lu\n", stats.rx_errors);
        Serial.printf("RX Overflows: %lu\n", stats.rx_overflow);
        Serial.printf("Bus Errors: %lu\n", stats.bus_errors);
        Serial.printf("Bus-Off Count: %lu\n", stats.bus_off_count);
        Serial.println();
    }
}

/**
 * @file ESP32_CAN_write.ino
 * @brief ESP32 Arduino CAN Frame Transmission Example
 *
 * This example demonstrates CAN frame transmission on ESP32
 * using the refactored MCP2515 library with thread-safe operations.
 *
 * Hardware Setup:
 * - MCP2515 CS   -> GPIO 5
 * - MCP2515 MOSI -> GPIO 23
 * - MCP2515 MISO -> GPIO 19
 * - MCP2515 SCK  -> GPIO 18
 * - CAN_H and CAN_L connected to CAN bus with 120Ω termination
 */

#include <mcp2515.h>

// Pin definitions
#define CAN_CS_PIN    GPIO_NUM_5

// Create MCP2515 instance (no interrupts needed for TX only)
MCP2515 mcp2515(CAN_CS_PIN, GPIO_NUM_NC);

// Counter for demo messages
uint32_t message_counter = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("========================================");
    Serial.println("ESP32 MCP2515 CAN Transmit Example");
    Serial.println("Thread-Safe Operations");
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

    // Set normal mode
    Serial.print("Setting normal mode... ");
    if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
        Serial.println("FAILED!");
        while (1) delay(1000);
    }
    Serial.println("OK");

    Serial.println("\n--- Transmitting CAN frames ---");
    Serial.println();
}

void loop() {
    struct can_frame frame;

    // Example 1: Simple data frame
    frame.can_id = 0x123;  // Standard 11-bit ID
    frame.can_dlc = 8;     // 8 bytes of data

    // Fill with incrementing pattern
    for (int i = 0; i < 8; i++) {
        frame.data[i] = (message_counter + i) & 0xFF;
    }

    // Send message
    MCP2515::ERROR result = mcp2515.sendMessage(&frame);

    if (result == MCP2515::ERROR_OK) {
        Serial.print("Sent message #");
        Serial.print(message_counter);
        Serial.print(" - ID: 0x");
        Serial.print(frame.can_id, HEX);
        Serial.print(" Data: ");
        for (int i = 0; i < frame.can_dlc; i++) {
            if (frame.data[i] < 0x10) Serial.print("0");
            Serial.print(frame.data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        message_counter++;
    } else {
        Serial.print("Send failed with error: ");
        Serial.println(result);

        // Check for specific errors
        if (result == MCP2515::ERROR_ALLTXBUSY) {
            Serial.println("All TX buffers busy - bus may be disconnected");
        }
    }

    // Send at 10 Hz
    delay(100);

    // Every 50 messages, send an extended frame
    if (message_counter % 50 == 0 && message_counter > 0) {
        struct can_frame ext_frame;
        ext_frame.can_id = 0x12345678 | CAN_EFF_FLAG;  // 29-bit extended ID
        ext_frame.can_dlc = 4;
        ext_frame.data[0] = (message_counter >> 24) & 0xFF;
        ext_frame.data[1] = (message_counter >> 16) & 0xFF;
        ext_frame.data[2] = (message_counter >> 8) & 0xFF;
        ext_frame.data[3] = message_counter & 0xFF;

        if (mcp2515.sendMessage(&ext_frame) == MCP2515::ERROR_OK) {
            Serial.println(">>> Sent extended frame!");
        }
    }

    // Print statistics every 10 seconds
    static unsigned long last_stats = 0;
    if (millis() - last_stats > 10000) {
        last_stats = millis();

        mcp2515_statistics_t stats;
        mcp2515.getStatistics(&stats);

        Serial.println("\n--- Statistics ---");
        Serial.printf("TX Frames: %lu\n", stats.tx_frames);
        Serial.printf("TX Errors: %lu\n", stats.tx_errors);
        Serial.printf("TX Timeouts: %lu\n", stats.tx_timeouts);
        Serial.printf("Error Counts - RX: %d, TX: %d\n",
                      mcp2515.errorCountRX(),
                      mcp2515.errorCountTX());
        Serial.println();
    }
}

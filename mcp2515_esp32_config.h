/**
 * @file mcp2515_esp32_config.h
 * @brief ESP32-specific configuration for MCP2515 CAN Controller Library
 *
 * This header provides ESP32 platform optimizations including:
 * - FreeRTOS integration
 * - ESP32 SPI driver support
 * - Interrupt handling with IRAM placement
 * - DMA configuration
 * - Power management
 *
 * @author ESP32-MCP2515 Refactored Library
 * @date 2025
 */

#ifndef MCP2515_ESP32_CONFIG_H_
#define MCP2515_ESP32_CONFIG_H_

#ifdef ESP32

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <esp_attr.h>

// ===========================
// ESP32 Variant Detection
// ===========================

// Detect ESP32 chip variant for proper pin and peripheral mapping
#if CONFIG_IDF_TARGET_ESP32
    #define MCP2515_CHIP_ESP32_CLASSIC  1
#elif CONFIG_IDF_TARGET_ESP32S2
    #define MCP2515_CHIP_ESP32S2        1
#elif CONFIG_IDF_TARGET_ESP32S3
    #define MCP2515_CHIP_ESP32S3        1
#elif CONFIG_IDF_TARGET_ESP32C3
    #define MCP2515_CHIP_ESP32C3        1
#elif CONFIG_IDF_TARGET_ESP32C6
    #define MCP2515_CHIP_ESP32C6        1
#elif CONFIG_IDF_TARGET_ESP32H2
    #define MCP2515_CHIP_ESP32H2        1
#else
    // Fallback: Assume classic ESP32 for Arduino IDE without explicit target
    #define MCP2515_CHIP_ESP32_CLASSIC  1
#endif

// ===========================
// ESP32 SPI Configuration
// ===========================

/** Default SPI host for MCP2515
 * - ESP32 classic: VSPI_HOST (SPI3)
 * - ESP32-S2/S3/C3/C6/H2: SPI2_HOST or SPI3_HOST
 */
#ifndef MCP2515_SPI_HOST
    #if defined(MCP2515_CHIP_ESP32_CLASSIC) && defined(VSPI_HOST)
        // Classic ESP32 with VSPI_HOST available
        #define MCP2515_SPI_HOST        VSPI_HOST
    #elif defined(SPI3_HOST)
        // Use SPI3_HOST if available (newer ESP-IDF/Arduino)
        #define MCP2515_SPI_HOST        SPI3_HOST
    #elif defined(SPI2_HOST)
        // Fallback to SPI2_HOST
        #define MCP2515_SPI_HOST        SPI2_HOST
    #else
        #error "No SPI host definition found for this ESP32 variant"
    #endif
#endif

/** Default SPI clock speed (10 MHz) */
#ifndef MCP2515_SPI_CLOCK_SPEED
#define MCP2515_SPI_CLOCK_SPEED (10 * 1000 * 1000)
#endif

/** SPI queue size for transactions */
#ifndef MCP2515_SPI_QUEUE_SIZE
#define MCP2515_SPI_QUEUE_SIZE  7
#endif

/** Enable SPI DMA for transfers */
#ifndef MCP2515_USE_SPI_DMA
#define MCP2515_USE_SPI_DMA     1
#endif

/** DMA channel for SPI (SPI_DMA_CH_AUTO for automatic) */
#ifndef MCP2515_SPI_DMA_CHAN
#define MCP2515_SPI_DMA_CHAN    SPI_DMA_CH_AUTO
#endif

// ===========================
// Default ESP32 Pin Mappings (Variant-Specific)
// ===========================

/**
 * Default pin mappings for different ESP32 variants
 * These use the default VSPI/SPI3 pins for each chip
 * Users can override these with custom pin assignments
 */

#ifndef MCP2515_DEFAULT_MOSI
    #if defined(MCP2515_CHIP_ESP32_CLASSIC)
        // ESP32 Classic: VSPI MOSI = GPIO 23
        #define MCP2515_DEFAULT_MOSI    GPIO_NUM_23
    #elif defined(MCP2515_CHIP_ESP32S2)
        // ESP32-S2: Default SPI MOSI = GPIO 35
        #define MCP2515_DEFAULT_MOSI    GPIO_NUM_35
    #elif defined(MCP2515_CHIP_ESP32S3)
        // ESP32-S3: Default SPI MOSI = GPIO 11
        #define MCP2515_DEFAULT_MOSI    GPIO_NUM_11
    #elif defined(MCP2515_CHIP_ESP32C3)
        // ESP32-C3: Default SPI MOSI = GPIO 6
        #define MCP2515_DEFAULT_MOSI    GPIO_NUM_6
    #elif defined(MCP2515_CHIP_ESP32C6)
        // ESP32-C6: Default SPI MOSI = GPIO 19
        #define MCP2515_DEFAULT_MOSI    GPIO_NUM_19
    #else
        // Generic fallback
        #define MCP2515_DEFAULT_MOSI    GPIO_NUM_23
    #endif
#endif

#ifndef MCP2515_DEFAULT_MISO
    #if defined(MCP2515_CHIP_ESP32_CLASSIC)
        // ESP32 Classic: VSPI MISO = GPIO 19
        #define MCP2515_DEFAULT_MISO    GPIO_NUM_19
    #elif defined(MCP2515_CHIP_ESP32S2)
        // ESP32-S2: Default SPI MISO = GPIO 37
        #define MCP2515_DEFAULT_MISO    GPIO_NUM_37
    #elif defined(MCP2515_CHIP_ESP32S3)
        // ESP32-S3: Default SPI MISO = GPIO 13
        #define MCP2515_DEFAULT_MISO    GPIO_NUM_13
    #elif defined(MCP2515_CHIP_ESP32C3)
        // ESP32-C3: Default SPI MISO = GPIO 5
        #define MCP2515_DEFAULT_MISO    GPIO_NUM_5
    #elif defined(MCP2515_CHIP_ESP32C6)
        // ESP32-C6: Default SPI MISO = GPIO 20
        #define MCP2515_DEFAULT_MISO    GPIO_NUM_20
    #else
        // Generic fallback
        #define MCP2515_DEFAULT_MISO    GPIO_NUM_19
    #endif
#endif

#ifndef MCP2515_DEFAULT_SCK
    #if defined(MCP2515_CHIP_ESP32_CLASSIC)
        // ESP32 Classic: VSPI SCK = GPIO 18
        #define MCP2515_DEFAULT_SCK     GPIO_NUM_18
    #elif defined(MCP2515_CHIP_ESP32S2)
        // ESP32-S2: Default SPI SCK = GPIO 36
        #define MCP2515_DEFAULT_SCK     GPIO_NUM_36
    #elif defined(MCP2515_CHIP_ESP32S3)
        // ESP32-S3: Default SPI SCK = GPIO 12
        #define MCP2515_DEFAULT_SCK     GPIO_NUM_12
    #elif defined(MCP2515_CHIP_ESP32C3)
        // ESP32-C3: Default SPI SCK = GPIO 4
        #define MCP2515_DEFAULT_SCK     GPIO_NUM_4
    #elif defined(MCP2515_CHIP_ESP32C6)
        // ESP32-C6: Default SPI SCK = GPIO 18
        #define MCP2515_DEFAULT_SCK     GPIO_NUM_18
    #else
        // Generic fallback
        #define MCP2515_DEFAULT_SCK     GPIO_NUM_18
    #endif
#endif

#ifndef MCP2515_DEFAULT_CS
    #if defined(MCP2515_CHIP_ESP32_CLASSIC)
        // ESP32 Classic: VSPI CS = GPIO 5
        #define MCP2515_DEFAULT_CS      GPIO_NUM_5
    #elif defined(MCP2515_CHIP_ESP32S2)
        // ESP32-S2: Default SPI CS = GPIO 34
        #define MCP2515_DEFAULT_CS      GPIO_NUM_34
    #elif defined(MCP2515_CHIP_ESP32S3)
        // ESP32-S3: Default SPI CS = GPIO 10
        #define MCP2515_DEFAULT_CS      GPIO_NUM_10
    #elif defined(MCP2515_CHIP_ESP32C3)
        // ESP32-C3: Default SPI CS = GPIO 7
        #define MCP2515_DEFAULT_CS      GPIO_NUM_7
    #elif defined(MCP2515_CHIP_ESP32C6)
        // ESP32-C6: Default SPI CS = GPIO 22
        #define MCP2515_DEFAULT_CS      GPIO_NUM_22
    #else
        // Generic fallback
        #define MCP2515_DEFAULT_CS      GPIO_NUM_5
    #endif
#endif

#ifndef MCP2515_DEFAULT_INT
    #if defined(MCP2515_CHIP_ESP32_CLASSIC)
        // ESP32 Classic: GPIO 4 for interrupt
        #define MCP2515_DEFAULT_INT     GPIO_NUM_4
    #elif defined(MCP2515_CHIP_ESP32S2)
        // ESP32-S2: GPIO 33 for interrupt
        #define MCP2515_DEFAULT_INT     GPIO_NUM_33
    #elif defined(MCP2515_CHIP_ESP32S3)
        // ESP32-S3: GPIO 9 for interrupt
        #define MCP2515_DEFAULT_INT     GPIO_NUM_9
    #elif defined(MCP2515_CHIP_ESP32C3)
        // ESP32-C3: GPIO 8 for interrupt
        #define MCP2515_DEFAULT_INT     GPIO_NUM_8
    #elif defined(MCP2515_CHIP_ESP32C6)
        // ESP32-C6: GPIO 21 for interrupt
        #define MCP2515_DEFAULT_INT     GPIO_NUM_21
    #else
        // Generic fallback
        #define MCP2515_DEFAULT_INT     GPIO_NUM_4
    #endif
#endif

// ===========================
// FreeRTOS Configuration
// ===========================

/** Enable thread-safe operations with mutex */
#ifndef MCP2515_USE_MUTEX
#define MCP2515_USE_MUTEX       1
#endif

/** RX queue depth for received CAN frames */
#ifndef MCP2515_RX_QUEUE_SIZE
#define MCP2515_RX_QUEUE_SIZE   32
#endif

/** Mutex timeout in FreeRTOS ticks */
#ifndef MCP2515_MUTEX_TIMEOUT
#define MCP2515_MUTEX_TIMEOUT   pdMS_TO_TICKS(100)
#endif

/** Task priority for interrupt handling task */
#ifndef MCP2515_ISR_TASK_PRIORITY
#define MCP2515_ISR_TASK_PRIORITY   (configMAX_PRIORITIES - 2)
#endif

/** Stack size for ISR task in bytes */
#ifndef MCP2515_ISR_TASK_STACK_SIZE
#define MCP2515_ISR_TASK_STACK_SIZE 4096
#endif

/** Task core affinity (tskNO_AFFINITY for any core, 0 or 1 for specific core) */
#ifndef MCP2515_ISR_TASK_CORE
#define MCP2515_ISR_TASK_CORE   tskNO_AFFINITY
#endif

// ===========================
// Interrupt Configuration
// ===========================

/** Enable interrupt-driven reception */
#ifndef MCP2515_USE_INTERRUPTS
#define MCP2515_USE_INTERRUPTS  1
#endif

/** Interrupt edge type */
#ifndef MCP2515_INT_EDGE
#define MCP2515_INT_EDGE        GPIO_INTR_NEGEDGE
#endif

/** Place ISR in IRAM for low latency */
#ifndef MCP2515_ISR_IN_IRAM
#define MCP2515_ISR_IN_IRAM     1
#endif

#if MCP2515_ISR_IN_IRAM
#define MCP2515_IRAM_ATTR       IRAM_ATTR
#else
#define MCP2515_IRAM_ATTR
#endif

// ===========================
// Power Management
// ===========================

/** Enable ESP32 power management integration */
#ifndef MCP2515_USE_POWER_MGMT
#define MCP2515_USE_POWER_MGMT  1
#endif

/** Acquire power lock during active operations */
#ifndef MCP2515_USE_PM_LOCKS
#define MCP2515_USE_PM_LOCKS    1
#endif

// ===========================
// Logging Configuration
// ===========================

/** Log tag for ESP-IDF logging */
#ifndef MCP2515_LOG_TAG
#define MCP2515_LOG_TAG         "MCP2515"
#endif

/** Default log level (ESP_LOG_INFO, ESP_LOG_DEBUG, etc.) */
#ifndef MCP2515_LOG_LEVEL
#define MCP2515_LOG_LEVEL       ESP_LOG_INFO
#endif

// ===========================
// Performance Optimization
// ===========================

/** Use register caching to reduce SPI reads */
#ifndef MCP2515_USE_REGISTER_CACHE
#define MCP2515_USE_REGISTER_CACHE  0
#endif

/** Optimize for speed over size */
#ifndef MCP2515_OPTIMIZE_SPEED
#define MCP2515_OPTIMIZE_SPEED  1
#endif

/** Enable compile-time optimizations */
#if MCP2515_OPTIMIZE_SPEED
#define MCP2515_INLINE          inline __attribute__((always_inline))
#else
#define MCP2515_INLINE          inline
#endif

// ===========================
// Memory Configuration
// ===========================

/** Allocate buffers in DMA-capable memory */
#ifndef MCP2515_USE_DMA_MEMORY
#define MCP2515_USE_DMA_MEMORY  1
#endif

/** Maximum SPI transaction size */
#ifndef MCP2515_MAX_TRANSFER_SIZE
#define MCP2515_MAX_TRANSFER_SIZE   64
#endif

// ===========================
// Feature Flags
// ===========================

/** Enable CAN frame statistics tracking */
#ifndef MCP2515_ENABLE_STATISTICS
#define MCP2515_ENABLE_STATISTICS   1
#endif

/** Enable error recovery mechanisms */
#ifndef MCP2515_ENABLE_ERROR_RECOVERY
#define MCP2515_ENABLE_ERROR_RECOVERY   1
#endif

/** Enable automatic bus-off recovery */
#ifndef MCP2515_AUTO_BUS_OFF_RECOVERY
#define MCP2515_AUTO_BUS_OFF_RECOVERY   1
#endif

// ===========================
// Compatibility Defines
// ===========================

/** Use ESP32 native SPI driver */
#define MCP2515_ESP32_NATIVE_SPI    1

/** Support Arduino-ESP32 compatibility */
#if defined(ARDUINO)
#define MCP2515_ARDUINO_COMPAT      1
#else
#define MCP2515_ARDUINO_COMPAT      0
#endif

#endif /* ESP32 */

#endif /* MCP2515_ESP32_CONFIG_H_ */

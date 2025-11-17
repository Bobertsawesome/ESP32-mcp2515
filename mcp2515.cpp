/**
 * @file mcp2515.cpp
 * @brief ESP32-optimized MCP2515 CAN Controller Implementation
 *
 * Refactored implementation with ESP32 native features
 */

#ifdef ESP32
    #ifdef ARDUINO
        #include "Arduino.h"
    #else
        #include <esp_system.h>
        #include <esp_timer.h>
        #define millis() (esp_timer_get_time() / 1000ULL)
        #define delay(ms) vTaskDelay(pdMS_TO_TICKS(ms))
        #define digitalWrite(pin, val) gpio_set_level((gpio_num_t)(pin), (val))
        #define pinMode(pin, mode) /* handled in init */
        #define HIGH 1
        #define LOW 0
        #define OUTPUT GPIO_MODE_OUTPUT
    #endif
#else
    #include "Arduino.h"
#endif

#include "mcp2515.h"

const struct MCP2515::TXBn_REGS MCP2515::TXB[MCP2515::N_TXBUFFERS] = {
    {MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA},
    {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA},
    {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA}
};

const struct MCP2515::RXBn_REGS MCP2515::RXB[N_RXBUFFERS] = {
    {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF},
    {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF}
};

// ===========================================
// Constructors and Destructor
// ===========================================

#ifdef ARDUINO
MCP2515::MCP2515(const uint8_t _CS, const uint32_t _SPI_CLOCK, SPIClass * _SPI)
{
#ifdef ESP32
    initialized = false;
    use_interrupts = false;
    spi_mutex = NULL;
    isr_semaphore = NULL;
    rx_queue = NULL;
    isr_task_handle = NULL;
    int_pin = GPIO_NUM_NC;
    spi_handle = NULL;  // Initialize to NULL for Arduino-ESP32 mode
    shutdown_requested = false;
    memset(&statistics, 0, sizeof(statistics));
    statistics_mutex = portMUX_INITIALIZER_UNLOCKED;

    // Create recursive mutex for thread safety (allows nested locking)
    spi_mutex = xSemaphoreCreateRecursiveMutex();
#endif

    if (_SPI != nullptr) {
        SPIn = _SPI;
    }
    else {
        SPIn = &SPI;
        SPIn->begin();
    }

    SPICS = _CS;
    SPI_CLOCK = _SPI_CLOCK;
    pinMode(SPICS, OUTPUT);
    digitalWrite(SPICS, HIGH);

#ifdef ESP32
    initialized = true;
#endif
}
#else
MCP2515::MCP2515(const uint8_t _CS, const uint32_t _SPI_CLOCK)
{
    SPICS = _CS;
    SPI_CLOCK = _SPI_CLOCK;

    initialized = false;
    use_interrupts = false;
    spi_mutex = NULL;
    isr_semaphore = NULL;
    rx_queue = NULL;
    isr_task_handle = NULL;
    int_pin = GPIO_NUM_NC;
    spi_handle = NULL;  // Initialize to NULL for native ESP-IDF mode
    shutdown_requested = false;
    memset(&statistics, 0, sizeof(statistics));
    statistics_mutex = portMUX_INITIALIZER_UNLOCKED;

    // Create default config for native ESP-IDF SPI initialization
    mcp2515_esp32_config_t config = {
        .spi_host = MCP2515_SPI_HOST,
        .spi_clock_speed = _SPI_CLOCK,
        .pins = {
            .miso = MCP2515_DEFAULT_MISO,
            .mosi = MCP2515_DEFAULT_MOSI,
            .sclk = MCP2515_DEFAULT_SCK,
            .cs = (gpio_num_t)_CS,
            .irq = GPIO_NUM_NC
        },
        .use_interrupts = false,
        .use_mutex = false,
        .rx_queue_size = 0,
        .isr_task_priority = 0,
        .isr_task_stack_size = 0
    };

    // Initialize SPI for native ESP-IDF
    if (initSPI(&config) != ERROR_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "SPI initialization failed in basic constructor");
        return;
    }

    initialized = true;
}
#endif

#ifdef ESP32
MCP2515::MCP2515(const mcp2515_esp32_config_t* config)
{
    initialized = false;
    use_interrupts = config->use_interrupts;
    spi_mutex = NULL;
    isr_semaphore = NULL;
    rx_queue = NULL;
    isr_task_handle = NULL;
    int_pin = config->pins.irq;
    spi_handle = NULL;  // Initialize to NULL before initSPI
    shutdown_requested = false;
    memset(&statistics, 0, sizeof(statistics));
    statistics_mutex = portMUX_INITIALIZER_UNLOCKED;

    SPICS = config->pins.cs;
    SPI_CLOCK = config->spi_clock_speed;

    // Initialize SPI
    if (initSPI(config) != ERROR_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "SPI initialization failed");
        return;
    }

    // Initialize interrupts if enabled
    if (use_interrupts && int_pin != GPIO_NUM_NC) {
        if (initInterrupts(int_pin) != ERROR_OK) {
            ESP_LOGE(MCP2515_LOG_TAG, "Interrupt initialization failed");
            return;
        }
    }

    initialized = true;
}

MCP2515::MCP2515(gpio_num_t cs_pin, gpio_num_t int_pin)
{
    initialized = false;
    use_interrupts = (int_pin != GPIO_NUM_NC);
    spi_mutex = NULL;
    isr_semaphore = NULL;
    rx_queue = NULL;
    isr_task_handle = NULL;
    this->int_pin = int_pin;
    spi_handle = NULL;  // Initialize to NULL before initSPI
    shutdown_requested = false;
    memset(&statistics, 0, sizeof(statistics));
    statistics_mutex = portMUX_INITIALIZER_UNLOCKED;

    SPICS = cs_pin;
    SPI_CLOCK = MCP2515_SPI_CLOCK_SPEED;

#ifdef ARDUINO
    // Arduino mode: Create mutex here (initSPI is no-op for Arduino)
    spi_mutex = xSemaphoreCreateRecursiveMutex();
    if (!spi_mutex) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to create SPI recursive mutex");
        initialized = false;
        return;
    }

    // Initialize Arduino SPI
    SPIn = &SPI;
    SPIn->begin();
    pinMode(SPICS, OUTPUT);
    digitalWrite(SPICS, HIGH);
#endif

    // Create default config
    mcp2515_esp32_config_t config = {
        .spi_host = MCP2515_SPI_HOST,
        .spi_clock_speed = MCP2515_SPI_CLOCK_SPEED,
        .pins = {
            .miso = MCP2515_DEFAULT_MISO,
            .mosi = MCP2515_DEFAULT_MOSI,
            .sclk = MCP2515_DEFAULT_SCK,
            .cs = cs_pin,
            .irq = int_pin
        },
        .use_interrupts = use_interrupts,
        .use_mutex = true,
        .rx_queue_size = MCP2515_RX_QUEUE_SIZE,
        .isr_task_priority = MCP2515_ISR_TASK_PRIORITY,
        .isr_task_stack_size = MCP2515_ISR_TASK_STACK_SIZE
    };

    // Initialize SPI
    if (initSPI(&config) != ERROR_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "SPI initialization failed");
        return;
    }

    // Initialize interrupts if enabled
    if (use_interrupts && int_pin != GPIO_NUM_NC) {
        if (initInterrupts(int_pin) != ERROR_OK) {
            ESP_LOGE(MCP2515_LOG_TAG, "Interrupt initialization failed");
            return;
        }
    }

    initialized = true;
}
#endif

MCP2515::~MCP2515()
{
#ifdef ESP32
    // Signal ISR task to stop
    shutdown_requested = true;

    // Wake up ISR task if it's waiting on semaphore
    if (isr_semaphore != NULL) {
        xSemaphoreGive(isr_semaphore);
    }

    // Wait for ISR task to finish (with timeout)
    if (isr_task_handle != NULL) {
        // Wait up to 100ms for task to exit cleanly
        for (int i = 0; i < 10; i++) {
            eTaskState state = eTaskGetState(isr_task_handle);
            if (state == eDeleted || state == eInvalid) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        // Force delete if still running
        vTaskDelete(isr_task_handle);
        isr_task_handle = NULL;
    }

    // NOW safe to remove interrupt handler
    if (int_pin != GPIO_NUM_NC) {
        gpio_isr_handler_remove(int_pin);
        // NOTE: NOT calling gpio_uninstall_isr_service() because it's a global
        // operation that would break other GPIO ISRs. Let ESP-IDF manage it.
    }

    // NOW safe to delete FreeRTOS objects
    if (spi_mutex != NULL) {
        vSemaphoreDelete(spi_mutex);
        spi_mutex = NULL;
    }
    if (isr_semaphore != NULL) {
        vSemaphoreDelete(isr_semaphore);
        isr_semaphore = NULL;
    }
    if (rx_queue != NULL) {
        vQueueDelete(rx_queue);
        rx_queue = NULL;
    }

#ifndef ARDUINO
    // Remove SPI device
    if (spi_handle != NULL) {
        spi_bus_remove_device(spi_handle);
        spi_handle = NULL;
    }
#endif

    initialized = false;
#endif
}

// ===========================================
// SPI Communication Methods
// ===========================================

#if defined(ESP32) && !defined(ARDUINO)
// Native ESP-IDF SPI transfer helper
inline uint8_t MCP2515::spiTransfer(uint8_t data) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;  // 8 bits
    t.tx_data[0] = data;
    t.rx_data[0] = 0;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;

    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "SPI transfer failed");
        return 0xFF;
    }
    return t.rx_data[0];
}
#define SPI_TRANSFER(x) spiTransfer(x)
#else
// Arduino SPI transfer
#define SPI_TRANSFER(x) SPIn->transfer(x)
#endif

void IRAM_ATTR MCP2515::startSPI() {
#ifdef ESP32
    #ifdef ARDUINO
        SPIn->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
        digitalWrite(SPICS, LOW);
    #else
        // Native ESP32: CS is handled automatically by driver, nothing to do
    #endif
#else
    SPIn->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(SPICS, LOW);
#endif
}

void IRAM_ATTR MCP2515::endSPI() {
#ifdef ESP32
    #ifdef ARDUINO
        digitalWrite(SPICS, HIGH);
        SPIn->endTransaction();
    #else
        // Native ESP32: CS is handled automatically by driver, nothing to do
    #endif
#else
    digitalWrite(SPICS, HIGH);
    SPIn->endTransaction();
#endif
}

MCP2515::ERROR MCP2515::reset(void)
{
    startSPI();
    SPI_TRANSFER(INSTRUCTION_RESET);
    endSPI();

    delay(10);

    uint8_t zeros[14];
    memset(zeros, 0, sizeof(zeros));
    ERROR err;
    if ((err = setRegisters(MCP_TXB0CTRL, zeros, 14)) != ERROR_OK) return err;
    if ((err = setRegisters(MCP_TXB1CTRL, zeros, 14)) != ERROR_OK) return err;
    if ((err = setRegisters(MCP_TXB2CTRL, zeros, 14)) != ERROR_OK) return err;

    if ((err = setRegister(MCP_RXB0CTRL, 0)) != ERROR_OK) return err;
    if ((err = setRegister(MCP_RXB1CTRL, 0)) != ERROR_OK) return err;

    if ((err = setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF)) != ERROR_OK) return err;

    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    if ((err = modifyRegister(MCP_RXB0CTRL,
                   RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT)) != ERROR_OK) return err;
    if ((err = modifyRegister(MCP_RXB1CTRL,
                   RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT)) != ERROR_OK) return err;

    // clear filters and masks
    // do not filter any standard frames for RXF0 used by RXB0
    // do not filter any extended frames for RXF1 used by RXB1
    RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    for (int i=0; i<6; i++) {
        bool ext = (i == 1);
        ERROR result = setFilter(filters[i], ext, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }

    MASK masks[] = {MASK0, MASK1};
    for (int i=0; i<2; i++) {
        ERROR result = setFilterMask(masks[i], true, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }

    return ERROR_OK;
}

uint8_t IRAM_ATTR MCP2515::readRegister(const REGISTER reg)
{
#ifdef ESP32
    if (acquireMutex(MCP2515_MUTEX_TIMEOUT) != ERROR_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to acquire mutex in readRegister");
        return 0xFF;
    }
#endif

    startSPI();
    SPI_TRANSFER(INSTRUCTION_READ);
    SPI_TRANSFER(reg);
    uint8_t ret = SPI_TRANSFER(0x00);
    endSPI();

#ifdef ESP32
    releaseMutex();
#endif

    return ret;
}

MCP2515::ERROR IRAM_ATTR MCP2515::readRegisters(const REGISTER reg, uint8_t values[], const uint8_t n)
{
#ifdef ESP32
    ERROR err = acquireMutex(MCP2515_MUTEX_TIMEOUT);
    if (err != ERROR_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to acquire mutex in readRegisters");
        return ERROR_MUTEX;
    }
#endif

    startSPI();
    SPI_TRANSFER(INSTRUCTION_READ);
    SPI_TRANSFER(reg);
    // mcp2515 has auto-increment of address-pointer
    for (uint8_t i=0; i<n; i++) {
        values[i] = SPI_TRANSFER(0x00);
    }
    endSPI();

#ifdef ESP32
    releaseMutex();
#endif

    return ERROR_OK;
}

MCP2515::ERROR IRAM_ATTR MCP2515::setRegister(const REGISTER reg, const uint8_t value)
{
#ifdef ESP32
    ERROR err = acquireMutex(MCP2515_MUTEX_TIMEOUT);
    if (err != ERROR_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to acquire mutex in setRegister");
        return ERROR_MUTEX;
    }
#endif

    startSPI();
    SPI_TRANSFER(INSTRUCTION_WRITE);
    SPI_TRANSFER(reg);
    SPI_TRANSFER(value);
    endSPI();

#ifdef ESP32
    releaseMutex();
#endif

    return ERROR_OK;
}

MCP2515::ERROR IRAM_ATTR MCP2515::setRegisters(const REGISTER reg, const uint8_t values[], const uint8_t n)
{
#ifdef ESP32
    ERROR err = acquireMutex(MCP2515_MUTEX_TIMEOUT);
    if (err != ERROR_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to acquire mutex in setRegisters");
        return ERROR_MUTEX;
    }
#endif

    startSPI();
    SPI_TRANSFER(INSTRUCTION_WRITE);
    SPI_TRANSFER(reg);
    for (uint8_t i=0; i<n; i++) {
        SPI_TRANSFER(values[i]);
    }
    endSPI();

#ifdef ESP32
    releaseMutex();
#endif

    return ERROR_OK;
}

MCP2515::ERROR IRAM_ATTR MCP2515::modifyRegister(const REGISTER reg, const uint8_t mask, const uint8_t data)
{
#ifdef ESP32
    ERROR err = acquireMutex(MCP2515_MUTEX_TIMEOUT);
    if (err != ERROR_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to acquire mutex in modifyRegister");
        return ERROR_MUTEX;
    }
#endif

    startSPI();
    SPI_TRANSFER(INSTRUCTION_BITMOD);
    SPI_TRANSFER(reg);
    SPI_TRANSFER(mask);
    SPI_TRANSFER(data);
    endSPI();

#ifdef ESP32
    releaseMutex();
#endif

    return ERROR_OK;
}

uint8_t IRAM_ATTR MCP2515::getStatus(void)
{
#ifdef ESP32
    if (acquireMutex(MCP2515_MUTEX_TIMEOUT) != ERROR_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to acquire mutex in getStatus");
        return 0xFF;
    }
#endif

    startSPI();
    SPI_TRANSFER(INSTRUCTION_READ_STATUS);
    uint8_t i = SPI_TRANSFER(0x00);
    endSPI();

#ifdef ESP32
    releaseMutex();
#endif

    return i;
}

MCP2515::ERROR MCP2515::setConfigMode()
{
    return setMode(CANCTRL_REQOP_CONFIG);
}

MCP2515::ERROR MCP2515::setListenOnlyMode()
{
    return setMode(CANCTRL_REQOP_LISTENONLY);
}

MCP2515::ERROR MCP2515::setSleepMode()
{
    return setMode(CANCTRL_REQOP_SLEEP);
}

MCP2515::ERROR MCP2515::setLoopbackMode()
{
    return setMode(CANCTRL_REQOP_LOOPBACK);
}

MCP2515::ERROR MCP2515::setNormalMode()
{
    return setMode(CANCTRL_REQOP_NORMAL);
}

MCP2515::ERROR MCP2515::setNormalOneShotMode()
{
    return setMode(CANCTRL_REQOP_OSM);
}

MCP2515::ERROR MCP2515::setMode(const CANCTRL_REQOP_MODE mode)
{
    ERROR err;
    if ((err = modifyRegister(MCP_CANCTRL, CANCTRL_REQOP | CANCTRL_OSM, mode)) != ERROR_OK) return err;

    // Sleep mode cannot be verified via SPI read - reading CANSTAT wakes the chip
    // Per MCP2515 datasheet Section 7.5: "The MCP2515 wakes up into Listen-Only mode"
    // Any SPI activity (including reading CANSTAT) causes immediate wake-up.
    // For Sleep mode, trust that modifyRegister() succeeded above.
    if (mode == CANCTRL_REQOP_SLEEP) {
        return ERROR_OK;
    }

    // Use delta-time pattern to prevent infinite loop when millis() overflows at 49.7 days
    unsigned long startTime = millis();
    const unsigned long timeout_ms = 10;
    bool modeMatch = false;
    while ((millis() - startTime) < timeout_ms) {
        uint8_t newmode = readRegister(MCP_CANSTAT);
        newmode &= CANSTAT_OPMOD;

        // One-Shot mode (CANCTRL_REQOP_OSM = 0x08) is a mode modifier, not a distinct mode value.
        // It sets the OSM bit in CANCTRL but CANSTAT will show Normal mode (0x00).
        // For OSM verification, check that CANSTAT mode bits are 0x00 (Normal).
        if (mode == CANCTRL_REQOP_OSM) {
            modeMatch = (newmode == CANCTRL_REQOP_NORMAL);
        } else {
            modeMatch = (newmode == mode);
        }

        if (modeMatch) {
            break;
        }
    }

    return modeMatch ? ERROR_OK : ERROR_FAIL;

}

MCP2515::ERROR MCP2515::setBitrate(const CAN_SPEED canSpeed)
{
    return setBitrate(canSpeed, MCP_16MHZ);
}

MCP2515::ERROR MCP2515::setBitrate(const CAN_SPEED canSpeed, CAN_CLOCK canClock)
{
    ERROR error = setConfigMode();
    if (error != ERROR_OK) {
        return error;
    }

    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock)
    {
        case (MCP_8MHZ):
        switch (canSpeed)
        {
            case (CAN_5KBPS):                                               //   5KBPS
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10KBPS
            cfg1 = MCP_8MHz_10kBPS_CFG1;
            cfg2 = MCP_8MHz_10kBPS_CFG2;
            cfg3 = MCP_8MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20KBPS
            cfg1 = MCP_8MHz_20kBPS_CFG1;
            cfg2 = MCP_8MHz_20kBPS_CFG2;
            cfg3 = MCP_8MHz_20kBPS_CFG3;
            break;

            case (CAN_31K25BPS):                                            //  31.25KBPS
            cfg1 = MCP_8MHz_31k25BPS_CFG1;
            cfg2 = MCP_8MHz_31k25BPS_CFG2;
            cfg3 = MCP_8MHz_31k25BPS_CFG3;
            break;

            case (CAN_33KBPS):                                              //  33.333KBPS
            cfg1 = MCP_8MHz_33k3BPS_CFG1;
            cfg2 = MCP_8MHz_33k3BPS_CFG2;
            cfg3 = MCP_8MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_8MHz_40kBPS_CFG1;
            cfg2 = MCP_8MHz_40kBPS_CFG2;
            cfg3 = MCP_8MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_8MHz_50kBPS_CFG1;
            cfg2 = MCP_8MHz_50kBPS_CFG2;
            cfg3 = MCP_8MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_8MHz_80kBPS_CFG1;
            cfg2 = MCP_8MHz_80kBPS_CFG2;
            cfg3 = MCP_8MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_8MHz_100kBPS_CFG1;
            cfg2 = MCP_8MHz_100kBPS_CFG2;
            cfg3 = MCP_8MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_8MHz_125kBPS_CFG1;
            cfg2 = MCP_8MHz_125kBPS_CFG2;
            cfg3 = MCP_8MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_8MHz_200kBPS_CFG1;
            cfg2 = MCP_8MHz_200kBPS_CFG2;
            cfg3 = MCP_8MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_8MHz_250kBPS_CFG1;
            cfg2 = MCP_8MHz_250kBPS_CFG2;
            cfg3 = MCP_8MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_8MHz_500kBPS_CFG1;
            cfg2 = MCP_8MHz_500kBPS_CFG2;
            cfg3 = MCP_8MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_8MHz_1000kBPS_CFG1;
            cfg2 = MCP_8MHz_1000kBPS_CFG2;
            cfg3 = MCP_8MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (MCP_16MHZ):
        switch (canSpeed)
        {
            case (CAN_5KBPS):                                               //   5Kbps
            cfg1 = MCP_16MHz_5kBPS_CFG1;
            cfg2 = MCP_16MHz_5kBPS_CFG2;
            cfg3 = MCP_16MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10Kbps
            cfg1 = MCP_16MHz_10kBPS_CFG1;
            cfg2 = MCP_16MHz_10kBPS_CFG2;
            cfg3 = MCP_16MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20Kbps
            cfg1 = MCP_16MHz_20kBPS_CFG1;
            cfg2 = MCP_16MHz_20kBPS_CFG2;
            cfg3 = MCP_16MHz_20kBPS_CFG3;
            break;

            case (CAN_33KBPS):                                              //  33.333Kbps
            cfg1 = MCP_16MHz_33k3BPS_CFG1;
            cfg2 = MCP_16MHz_33k3BPS_CFG2;
            cfg3 = MCP_16MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_16MHz_40kBPS_CFG1;
            cfg2 = MCP_16MHz_40kBPS_CFG2;
            cfg3 = MCP_16MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_16MHz_50kBPS_CFG1;
            cfg2 = MCP_16MHz_50kBPS_CFG2;
            cfg3 = MCP_16MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_16MHz_80kBPS_CFG1;
            cfg2 = MCP_16MHz_80kBPS_CFG2;
            cfg3 = MCP_16MHz_80kBPS_CFG3;
            break;

            case (CAN_83K3BPS):                                             //  83.333Kbps
            cfg1 = MCP_16MHz_83k3BPS_CFG1;
            cfg2 = MCP_16MHz_83k3BPS_CFG2;
            cfg3 = MCP_16MHz_83k3BPS_CFG3;
            break;

            case (CAN_95KBPS):                                              //  95Kbps
            cfg1 = MCP_16MHz_95kBPS_CFG1;
            cfg2 = MCP_16MHz_95kBPS_CFG2;
            cfg3 = MCP_16MHz_95kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_16MHz_100kBPS_CFG1;
            cfg2 = MCP_16MHz_100kBPS_CFG2;
            cfg3 = MCP_16MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_16MHz_125kBPS_CFG1;
            cfg2 = MCP_16MHz_125kBPS_CFG2;
            cfg3 = MCP_16MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_16MHz_200kBPS_CFG1;
            cfg2 = MCP_16MHz_200kBPS_CFG2;
            cfg3 = MCP_16MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_16MHz_250kBPS_CFG1;
            cfg2 = MCP_16MHz_250kBPS_CFG2;
            cfg3 = MCP_16MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_16MHz_500kBPS_CFG1;
            cfg2 = MCP_16MHz_500kBPS_CFG2;
            cfg3 = MCP_16MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_16MHz_1000kBPS_CFG1;
            cfg2 = MCP_16MHz_1000kBPS_CFG2;
            cfg3 = MCP_16MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (MCP_20MHZ):
        switch (canSpeed)
        {
            case (CAN_33KBPS):                                              //  33.333Kbps
            cfg1 = MCP_20MHz_33k3BPS_CFG1;
            cfg2 = MCP_20MHz_33k3BPS_CFG2;
            cfg3 = MCP_20MHz_33k3BPS_CFG3;
	    break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_20MHz_40kBPS_CFG1;
            cfg2 = MCP_20MHz_40kBPS_CFG2;
            cfg3 = MCP_20MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_20MHz_50kBPS_CFG1;
            cfg2 = MCP_20MHz_50kBPS_CFG2;
            cfg3 = MCP_20MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_20MHz_80kBPS_CFG1;
            cfg2 = MCP_20MHz_80kBPS_CFG2;
            cfg3 = MCP_20MHz_80kBPS_CFG3;
            break;

            case (CAN_83K3BPS):                                             //  83.333Kbps
            cfg1 = MCP_20MHz_83k3BPS_CFG1;
            cfg2 = MCP_20MHz_83k3BPS_CFG2;
            cfg3 = MCP_20MHz_83k3BPS_CFG3;
	    break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_20MHz_100kBPS_CFG1;
            cfg2 = MCP_20MHz_100kBPS_CFG2;
            cfg3 = MCP_20MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_20MHz_125kBPS_CFG1;
            cfg2 = MCP_20MHz_125kBPS_CFG2;
            cfg3 = MCP_20MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_20MHz_200kBPS_CFG1;
            cfg2 = MCP_20MHz_200kBPS_CFG2;
            cfg3 = MCP_20MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_20MHz_250kBPS_CFG1;
            cfg2 = MCP_20MHz_250kBPS_CFG2;
            cfg3 = MCP_20MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_20MHz_500kBPS_CFG1;
            cfg2 = MCP_20MHz_500kBPS_CFG2;
            cfg3 = MCP_20MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_20MHz_1000kBPS_CFG1;
            cfg2 = MCP_20MHz_1000kBPS_CFG2;
            cfg3 = MCP_20MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        default:
        set = 0;
        break;
    }

    if (set) {
        ERROR err;
        if ((err = setRegister(MCP_CNF1, cfg1)) != ERROR_OK) return err;
        if ((err = setRegister(MCP_CNF2, cfg2)) != ERROR_OK) return err;
        if ((err = setRegister(MCP_CNF3, cfg3)) != ERROR_OK) return err;
        return ERROR_OK;
    }
    else {
        return ERROR_FAIL;
    }
}

MCP2515::ERROR MCP2515::setClkOut(const CAN_CLKOUT divisor)
{
    ERROR err;
    if (divisor == CLKOUT_DISABLE) {
	/* Turn off CLKEN */
	if ((err = modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, 0x00)) != ERROR_OK) return err;

	/* Turn on CLKOUT for SOF */
	if ((err = modifyRegister(MCP_CNF3, CNF3_SOF, CNF3_SOF)) != ERROR_OK) return err;
        return ERROR_OK;
    }

    /* Set the prescaler (CLKPRE) */
    if ((err = modifyRegister(MCP_CANCTRL, CANCTRL_CLKPRE, divisor)) != ERROR_OK) return err;

    /* Turn on CLKEN */
    if ((err = modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, CANCTRL_CLKEN)) != ERROR_OK) return err;

    /* Turn off CLKOUT for SOF */
    if ((err = modifyRegister(MCP_CNF3, CNF3_SOF, 0x00)) != ERROR_OK) return err;
    return ERROR_OK;
}

void MCP2515::prepareId(uint8_t *buffer, const bool ext, const uint32_t id)
{
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if (ext) {
        buffer[MCP_EID0] = (uint8_t) (canid & 0xFF);
        buffer[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        buffer[MCP_SIDL] = (uint8_t) (canid & 0x03);
        buffer[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
        buffer[MCP_SIDH] = (uint8_t) (canid >> 5);
    } else {
        buffer[MCP_SIDH] = (uint8_t) (canid >> 3);
        buffer[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        buffer[MCP_EID0] = 0;
        buffer[MCP_EID8] = 0;
    }
}

MCP2515::ERROR MCP2515::setFilterMask(const MASK mask, const bool ext, const uint32_t ulData)
{
    // Save current mode
    uint8_t current_mode = readRegister(MCP_CANSTAT) & CANSTAT_OPMOD;

    ERROR res = setConfigMode();
    if (res != ERROR_OK) {
        return res;
    }

    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);

    REGISTER reg;
    switch (mask) {
        case MASK0: reg = MCP_RXM0SIDH; break;
        case MASK1: reg = MCP_RXM1SIDH; break;
        default:
            return ERROR_FAIL;
    }

    ERROR err;
    if ((err = setRegisters(reg, tbufdata, 4)) != ERROR_OK) return err;

    // Restore original mode
    return setMode((CANCTRL_REQOP_MODE)current_mode);
}

MCP2515::ERROR MCP2515::setFilter(const RXF num, const bool ext, const uint32_t ulData)
{
    // Save current mode
    uint8_t current_mode = readRegister(MCP_CANSTAT) & CANSTAT_OPMOD;

    ERROR res = setConfigMode();
    if (res != ERROR_OK) {
        return res;
    }

    REGISTER reg;

    switch (num) {
        case RXF0: reg = MCP_RXF0SIDH; break;
        case RXF1: reg = MCP_RXF1SIDH; break;
        case RXF2: reg = MCP_RXF2SIDH; break;
        case RXF3: reg = MCP_RXF3SIDH; break;
        case RXF4: reg = MCP_RXF4SIDH; break;
        case RXF5: reg = MCP_RXF5SIDH; break;
        default:
            return ERROR_FAIL;
    }

    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);
    ERROR err;
    if ((err = setRegisters(reg, tbufdata, 4)) != ERROR_OK) return err;

    // Restore original mode
    return setMode((CANCTRL_REQOP_MODE)current_mode);
}

MCP2515::ERROR MCP2515::sendMessage(const TXBn txbn, const struct can_frame *frame)
{
    // Validate frame pointer to prevent crash on null dereference
    if (frame == nullptr) {
        return ERROR_FAILTX;
    }

    if (frame->can_dlc > CAN_MAX_DLEN) {
        return ERROR_FAILTX;
    }

    const struct TXBn_REGS *txbuf = &TXB[txbn];

    uint8_t data[13];

    bool ext = (frame->can_id & CAN_EFF_FLAG);
    bool rtr = (frame->can_id & CAN_RTR_FLAG);
    uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    prepareId(data, ext, id);

    data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

    memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

    // Use optimized LOAD TX BUFFER instruction (saves 1 SPI byte vs standard WRITE)
    uint8_t load_instruction;
    switch (txbn) {
        case TXB0: load_instruction = INSTRUCTION_LOAD_TX0; break;
        case TXB1: load_instruction = INSTRUCTION_LOAD_TX1; break;
        case TXB2: load_instruction = INSTRUCTION_LOAD_TX2; break;
        default: return ERROR_FAILTX;
    }

#ifdef ESP32
    if (acquireMutex(MCP2515_MUTEX_TIMEOUT) != ERROR_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to acquire mutex in sendMessage");
        return ERROR_FAILTX;
    }
#endif

    startSPI();
    SPI_TRANSFER(load_instruction);  // Points to TXBnSIDH, no address byte needed
    for (uint8_t i = 0; i < (5 + frame->can_dlc); i++) {
        SPI_TRANSFER(data[i]);
    }
    endSPI();

#ifdef ESP32
    releaseMutex();
#endif

    ERROR err;
    if ((err = modifyRegister(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ)) != ERROR_OK) return err;

    uint8_t ctrl = readRegister(txbuf->CTRL);
    if ((ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) != 0) {
#ifdef ESP32
        portENTER_CRITICAL(&statistics_mutex);
        statistics.tx_errors++;
        portEXIT_CRITICAL(&statistics_mutex);
#endif
        return ERROR_FAILTX;
    }

#ifdef ESP32
    portENTER_CRITICAL(&statistics_mutex);
    statistics.tx_frames++;
    portEXIT_CRITICAL(&statistics_mutex);
#endif
    return ERROR_OK;
}

MCP2515::ERROR MCP2515::sendMessage(const struct can_frame *frame)
{
    // Validate frame pointer to prevent crash on null dereference
    if (frame == nullptr) {
        return ERROR_FAILTX;
    }

    if (frame->can_dlc > CAN_MAX_DLEN) {
        return ERROR_FAILTX;
    }

#ifdef ESP32
    // Acquire mutex for entire buffer selection + send operation
    // This prevents race condition where two tasks select same buffer
    if (acquireMutex(MCP2515_MUTEX_TIMEOUT) != ERROR_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to acquire mutex in sendMessage");
        portENTER_CRITICAL(&statistics_mutex);
        statistics.tx_errors++;
        portEXIT_CRITICAL(&statistics_mutex);
        return ERROR_FAILTX;
    }
#endif

    TXBn txBuffers[N_TXBUFFERS] = {TXB0, TXB1, TXB2};
    ERROR result = ERROR_ALLTXBUSY;

    for (int i=0; i<N_TXBUFFERS; i++) {
        const struct TXBn_REGS *txbuf = &TXB[txBuffers[i]];
        uint8_t ctrlval = readRegister(txbuf->CTRL);  // Recursive mutex - OK
        if ( (ctrlval & TXB_TXREQ) == 0 ) {
            result = sendMessage(txBuffers[i], frame);  // Recursive mutex - OK
            break;
        }
    }

#ifdef ESP32
    releaseMutex();

    if (result == ERROR_ALLTXBUSY) {
        portENTER_CRITICAL(&statistics_mutex);
        statistics.tx_errors++;
        portEXIT_CRITICAL(&statistics_mutex);
    }
#endif

    return result;
}

MCP2515::ERROR MCP2515::setTransmitPriority(const TXBn txbn, const uint8_t priority)
{
    // Validate priority (0-3, where 3 is highest)
    if (priority > 3) {
        return ERROR_FAIL;
    }

    const struct TXBn_REGS *txbuf = &TXB[txbn];

    // Check that buffer is not currently transmitting
    uint8_t ctrl = readRegister(txbuf->CTRL);
    if (ctrl & TXB_TXREQ) {
        return ERROR_FAIL;  // Cannot change priority while transmission is pending
    }

    // Set TXP[1:0] bits (bits 1:0 of TXBnCTRL)
    ERROR err;
    if ((err = modifyRegister(txbuf->CTRL, TXB_TXP, priority)) != ERROR_OK) return err;

    return ERROR_OK;
}

MCP2515::ERROR MCP2515::abortTransmission(const TXBn txbn)
{
    const struct TXBn_REGS *txbuf = &TXB[txbn];

    // Clear the TXREQ bit to abort transmission
    ERROR err;
    if ((err = modifyRegister(txbuf->CTRL, TXB_TXREQ, 0)) != ERROR_OK) return err;

    // Check if abort was successful by reading ABTF flag
    uint8_t ctrl = readRegister(txbuf->CTRL);
    if (ctrl & TXB_ABTF) {
        // Clear the abort flag
        if ((err = modifyRegister(txbuf->CTRL, TXB_ABTF, 0)) != ERROR_OK) return err;
    }

    return ERROR_OK;
}

MCP2515::ERROR MCP2515::abortAllTransmissions(void)
{
    // Set ABAT bit in CANCTRL register to abort all pending transmissions
    ERROR err;
    if ((err = modifyRegister(MCP_CANCTRL, CANCTRL_ABAT, CANCTRL_ABAT)) != ERROR_OK) return err;

    // Wait for abort to complete (ABAT bit is automatically cleared by hardware)
    // Use delta-time pattern to prevent infinite loop when millis() overflows at 49.7 days
    unsigned long startTime = millis();
    const unsigned long timeout_ms = 10;
    while ((millis() - startTime) < timeout_ms) {
        uint8_t ctrl = readRegister(MCP_CANCTRL);
        if ((ctrl & CANCTRL_ABAT) == 0) {
            break;  // Abort completed
        }
    }

    // Clear TX buffer states to make them usable again
    // Per MCP2515 datasheet: after abort, both ABTF and TXREQ remain set
    for (int i = 0; i < N_TXBUFFERS; i++) {
        const struct TXBn_REGS *txbuf = &TXB[i];

        // Clear ABTF (abort flag) - required to prevent ERROR_FAILTX
        if ((err = modifyRegister(txbuf->CTRL, TXB_ABTF, 0)) != ERROR_OK) return err;

        // Clear TXREQ (transmit request) - required to make buffer available
        // Without this, all buffers remain "busy" and sendMessage() fails
        if ((err = modifyRegister(txbuf->CTRL, TXB_TXREQ, 0)) != ERROR_OK) return err;
    }

    return ERROR_OK;
}

MCP2515::ERROR IRAM_ATTR MCP2515::readMessage(const RXBn rxbn, struct can_frame *frame)
{
    // Validate frame pointer to prevent crash on null dereference
    if (frame == nullptr) {
        return ERROR_FAIL;
    }

    const struct RXBn_REGS *rxb = &RXB[rxbn];

    uint8_t tbufdata[5];

    // Use optimized READ RX BUFFER instruction (saves 1 SPI byte vs standard READ)
    // INSTRUCTION_READ_RX0 (0x90) for RXB0, INSTRUCTION_READ_RX1 (0x94) for RXB1
#ifdef ESP32
    if (acquireMutex(MCP2515_MUTEX_TIMEOUT) != ERROR_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to acquire mutex in readMessage");
        return ERROR_FAIL;
    }
#endif

    startSPI();
    SPI_TRANSFER(rxbn == RXB0 ? INSTRUCTION_READ_RX0 : INSTRUCTION_READ_RX1);
    // MCP2515 auto-increments address pointer
    for (uint8_t i = 0; i < 5; i++) {
        tbufdata[i] = SPI_TRANSFER(0x00);
    }
    endSPI();

#ifdef ESP32
    releaseMutex();
#endif

    uint32_t id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK ) {
        id = (id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        id = (id<<8) + tbufdata[MCP_EID8];
        id = (id<<8) + tbufdata[MCP_EID0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK);
    if (dlc > CAN_MAX_DLEN) {
        return ERROR_FAIL;
    }

    uint8_t ctrl = readRegister(rxb->CTRL);
    if (ctrl & RXBnCTRL_RTR) {
        id |= CAN_RTR_FLAG;
    }

    frame->can_id = id;
    frame->can_dlc = dlc;

    // Read data bytes using optimized READ RX BUFFER command starting at D0
    // This is a separate SPI transaction starting at the data field
#ifdef ESP32
    if (acquireMutex(MCP2515_MUTEX_TIMEOUT) != ERROR_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to acquire mutex in readMessage (data)");
        return ERROR_FAIL;
    }
#endif

    startSPI();
    // Use 0x92 for RXB0 data, 0x96 for RXB1 data (starts at D0)
    SPI_TRANSFER((rxbn == RXB0 ? INSTRUCTION_READ_RX0 : INSTRUCTION_READ_RX1) | 0x02);
    for (uint8_t i = 0; i < dlc; i++) {
        frame->data[i] = SPI_TRANSFER(0x00);
    }
    endSPI();

#ifdef ESP32
    releaseMutex();
#endif

    ERROR err;
    if ((err = modifyRegister(MCP_CANINTF, rxb->CANINTF_RXnIF, 0)) != ERROR_OK) return err;

    return ERROR_OK;
}

MCP2515::ERROR IRAM_ATTR MCP2515::readMessage(struct can_frame *frame)
{
    // Validate frame pointer to prevent crash on null dereference
    if (frame == nullptr) {
        return ERROR_NOMSG;
    }

    ERROR rc;
    uint8_t stat = getStatus();

    if ( stat & STAT_RX0IF ) {
        rc = readMessage(RXB0, frame);
    } else if ( stat & STAT_RX1IF ) {
        rc = readMessage(RXB1, frame);
    } else {
        rc = ERROR_NOMSG;
    }

    return rc;
}

uint8_t MCP2515::getFilterHit(const RXBn rxbn)
{
    const struct RXBn_REGS *rxb = &RXB[rxbn];
    uint8_t ctrl = readRegister(rxb->CTRL);

    // Extract FILHIT bits from RXBnCTRL register
    // RXB0: FILHIT[1:0] in bits [1:0] - indicates filters 0-1 (or rollover from RXB1)
    // RXB1: FILHIT[2:0] in bits [2:0] - indicates filters 3-5
    if (rxbn == RXB0) {
        return (ctrl & RXB0CTRL_FILHIT_MASK);  // Returns 0-3
    } else {
        return (ctrl & RXB1CTRL_FILHIT_MASK);  // Returns 0-7, typically 3-5
    }
}

bool MCP2515::checkReceive(void)
{
    uint8_t res = getStatus();
    if ( res & STAT_RXIF_MASK ) {
        return true;
    } else {
        return false;
    }
}

bool MCP2515::checkError(void)
{
    uint8_t eflg = getErrorFlags();

    if ( eflg & EFLG_ERRORMASK ) {
        return true;
    } else {
        return false;
    }
}

uint8_t IRAM_ATTR MCP2515::getErrorFlags(void)
{
    return readRegister(MCP_EFLG);
}

void MCP2515::clearRXnOVRFlags(void)
{
	(void)modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
}

uint8_t IRAM_ATTR MCP2515::getInterrupts(void)
{
    return readRegister(MCP_CANINTF);
}

void MCP2515::clearInterrupts(void)
{
    (void)setRegister(MCP_CANINTF, 0);
}

uint8_t MCP2515::getInterruptMask(void)
{
    return readRegister(MCP_CANINTE);
}

void IRAM_ATTR MCP2515::clearTXInterrupts(void)
{
    (void)modifyRegister(MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);
}

void MCP2515::clearRXnOVR(void)
{
	uint8_t eflg = getErrorFlags();
	if (eflg != 0) {
		clearRXnOVRFlags();
		clearInterrupts();
		//modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
	}
	
}

void MCP2515::clearMERR()
{
	//modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
	//clearInterrupts();
	(void)modifyRegister(MCP_CANINTF, CANINTF_MERRF, 0);
}

void IRAM_ATTR MCP2515::clearERRIF()
{
    //modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
    //clearInterrupts();
    (void)modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
}

uint8_t MCP2515::errorCountRX(void)                             
{
    return readRegister(MCP_REC);
}

uint8_t MCP2515::errorCountTX(void)
{
    return readRegister(MCP_TEC);
}

#ifdef ESP32
// ===========================================
// ESP32-Specific Implementations
// ===========================================

MCP2515::ERROR MCP2515::initSPI(const mcp2515_esp32_config_t* config)
{
#ifdef ARDUINO
    // Arduino-ESP32 uses SPIClass
    return ERROR_OK;
#else
    // Native ESP-IDF SPI initialization
    spi_bus_config_t buscfg = {
        .mosi_io_num = config->pins.mosi,
        .miso_io_num = config->pins.miso,
        .sclk_io_num = config->pins.sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MCP2515_MAX_TRANSFER_SIZE,
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,  // SPI mode 0
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = config->spi_clock_speed,
        .input_delay_ns = 0,
        .spics_io_num = config->pins.cs,
        .flags = 0,
        .queue_size = MCP2515_SPI_QUEUE_SIZE,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    // Initialize SPI bus
    esp_err_t ret = spi_bus_initialize(config->spi_host, &buscfg, MCP2515_SPI_DMA_CHAN);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(MCP2515_LOG_TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ERROR_FAILINIT;
    }

    // Add device to SPI bus
    ret = spi_bus_add_device(config->spi_host, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return ERROR_FAILINIT;
    }

    // PSRAM + DMA safety check (fail initialization if conflict detected)
    #if CONFIG_SPIRAM_USE_MALLOC
        #if MCP2515_SPI_DMA_CHAN != SPI_DMA_DISABLED
            ESP_LOGE(MCP2515_LOG_TAG, "CRITICAL: PSRAM enabled but SPI DMA is also enabled!");
            ESP_LOGE(MCP2515_LOG_TAG, "DMA cannot access PSRAM - this WILL cause crashes");
            ESP_LOGE(MCP2515_LOG_TAG, "Fix: Disable PSRAM OR set MCP2515_SPI_DMA_CHAN=SPI_DMA_DISABLED");
            return ERROR_PSRAM;  // Fail hard to prevent crashes
        #endif
    #endif

    // Create recursive mutex if requested (allows nested locking)
    if (config->use_mutex) {
        spi_mutex = xSemaphoreCreateRecursiveMutex();
        if (spi_mutex == NULL) {
            ESP_LOGE(MCP2515_LOG_TAG, "Failed to create SPI recursive mutex");
            return ERROR_FAILINIT;
        }
    }

    ESP_LOGI(MCP2515_LOG_TAG, "SPI initialized successfully");
    return ERROR_OK;
#endif
}

MCP2515::ERROR MCP2515::initInterrupts(gpio_num_t int_pin)
{
    if (int_pin == GPIO_NUM_NC) {
        return ERROR_OK;
    }

    // Configure interrupt pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << int_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = MCP2515_INT_EDGE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "GPIO config failed");
        return ERROR_FAILINIT;
    }

    // Create semaphore for ISR notification
    isr_semaphore = xSemaphoreCreateBinary();
    if (isr_semaphore == NULL) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to create ISR semaphore");
        return ERROR_FAILINIT;
    }

    // Create RX queue
    rx_queue = xQueueCreate(MCP2515_RX_QUEUE_SIZE, sizeof(struct can_frame));
    if (rx_queue == NULL) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to create RX queue");
        return ERROR_FAILINIT;
    }

    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(MCP2515_LOG_TAG, "GPIO ISR service install failed");
        return ERROR_FAILINIT;
    }

    // Add ISR handler
    ret = gpio_isr_handler_add(int_pin, isrHandler, (void*)this);
    if (ret != ESP_OK) {
        ESP_LOGE(MCP2515_LOG_TAG, "GPIO ISR handler add failed");
        return ERROR_FAILINIT;
    }

    // Create ISR processing task
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        isrTask,
        "mcp2515_isr",
        MCP2515_ISR_TASK_STACK_SIZE,
        (void*)this,
        MCP2515_ISR_TASK_PRIORITY,
        &isr_task_handle,
        MCP2515_ISR_TASK_CORE
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(MCP2515_LOG_TAG, "Failed to create ISR task");
        return ERROR_FAILINIT;
    }

    ESP_LOGI(MCP2515_LOG_TAG, "Interrupts initialized on GPIO %d", int_pin);
    return ERROR_OK;
}

void IRAM_ATTR MCP2515::isrHandler(void* arg)
{
    MCP2515* mcp = static_cast<MCP2515*>(arg);

    // Null check - if semaphore not ready yet or already deleted, return
    if (mcp->isr_semaphore == NULL) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Give semaphore to wake up ISR task
    xSemaphoreGiveFromISR(mcp->isr_semaphore, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void MCP2515::isrTask(void* pvParameters)
{
    MCP2515* mcp = static_cast<MCP2515*>(pvParameters);

    ESP_LOGI(MCP2515_LOG_TAG, "ISR task started");

    while (!mcp->shutdown_requested) {
        // Wait for ISR semaphore with timeout to allow checking shutdown flag
        if (xSemaphoreTake(mcp->isr_semaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Check shutdown flag again before processing
            if (!mcp->shutdown_requested) {
                mcp->processInterrupts();
            }
        }
    }

    ESP_LOGI(MCP2515_LOG_TAG, "ISR task exiting");
}

void MCP2515::processInterrupts()
{
    uint8_t irq = getInterrupts();

    // Handle RX interrupts
    if (irq & (CANINTF_RX0IF | CANINTF_RX1IF)) {
        struct can_frame frame;

        while (readMessage(&frame) == ERROR_OK) {
            portENTER_CRITICAL(&statistics_mutex);
            statistics.rx_frames++;
            portEXIT_CRITICAL(&statistics_mutex);

            // Add to queue
            if (xQueueSend(rx_queue, &frame, 0) != pdTRUE) {
                portENTER_CRITICAL(&statistics_mutex);
                statistics.rx_overflow++;
                portEXIT_CRITICAL(&statistics_mutex);
                ESP_LOGW(MCP2515_LOG_TAG, "RX queue full, frame dropped");
            }
        }
    }

    // Handle error interrupts
    if (irq & CANINTF_ERRIF) {
        uint8_t eflg = getErrorFlags();

        portENTER_CRITICAL(&statistics_mutex);
        statistics.bus_errors++;
        if (eflg & EFLG_RX0OVR) statistics.rx_overflow++;
        if (eflg & EFLG_RX1OVR) statistics.rx_overflow++;
        if (eflg & EFLG_TXBO) statistics.bus_off_count++;
        portEXIT_CRITICAL(&statistics_mutex);

        clearERRIF();

#if MCP2515_AUTO_BUS_OFF_RECOVERY
        if (eflg & EFLG_TXBO) {
            performErrorRecovery();
        }
#endif
    }

    // Handle TX complete interrupts
    if (irq & (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF)) {
        clearTXInterrupts();
    }
}

MCP2515::ERROR MCP2515::acquireMutex(TickType_t timeout)
{
    if (spi_mutex == NULL) {
        return ERROR_OK;  // No mutex configured
    }

    if (xSemaphoreTakeRecursive(spi_mutex, timeout) != pdTRUE) {
        return ERROR_MUTEX;
    }

    return ERROR_OK;
}

void MCP2515::releaseMutex()
{
    if (spi_mutex != NULL) {
        xSemaphoreGiveRecursive(spi_mutex);
    }
}

MCP2515::ERROR MCP2515::readMessageQueued(struct can_frame *frame, uint32_t timeout_ms)
{
    // Validate frame pointer to prevent crash on null dereference
    if (frame == nullptr) {
        return ERROR_NOMSG;
    }

    if (!use_interrupts || rx_queue == NULL) {
        // Fall back to polling
        return readMessage(frame);
    }

    TickType_t timeout_ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);

    if (xQueueReceive(rx_queue, frame, timeout_ticks) == pdTRUE) {
        return ERROR_OK;
    }

    return (timeout_ms == 0) ? ERROR_NOMSG : ERROR_TIMEOUT;
}

uint32_t MCP2515::getRxQueueCount(void)
{
    if (rx_queue == NULL) {
        return 0;
    }
    return uxQueueMessagesWaiting(rx_queue);
}

void MCP2515::getStatistics(mcp2515_statistics_t* stats)
{
    if (stats != NULL) {
        // Use spinlock to prevent torn reads from ISR task on dual-core ESP32
        portENTER_CRITICAL(&statistics_mutex);
        memcpy(stats, &statistics, sizeof(mcp2515_statistics_t));
        portEXIT_CRITICAL(&statistics_mutex);
    }
}

void MCP2515::resetStatistics(void)
{
    memset(&statistics, 0, sizeof(statistics));
}

bool MCP2515::isInitialized(void)
{
    return initialized;
}

MCP2515::ERROR MCP2515::setInterruptMode(bool enable)
{
    use_interrupts = enable;

    if (enable && int_pin != GPIO_NUM_NC) {
        gpio_intr_enable(int_pin);
    } else if (int_pin != GPIO_NUM_NC) {
        gpio_intr_disable(int_pin);
    }

    return ERROR_OK;
}

MCP2515::ERROR MCP2515::performErrorRecovery(void)
{
    ESP_LOGW(MCP2515_LOG_TAG, "Performing error recovery");

    // Get error counts and log them (call functions directly to avoid unused variable warnings)
    ESP_LOGI(MCP2515_LOG_TAG, "Error counts - RX: %d, TX: %d", errorCountRX(), errorCountTX());

    // Clear all error flags
    clearRXnOVR();
    clearMERR();
    clearERRIF();

    // Reset if in bus-off state
    uint8_t eflg = getErrorFlags();
    if (eflg & EFLG_TXBO) {
        ESP_LOGW(MCP2515_LOG_TAG, "Bus-off detected, resetting controller");

        // Save current mode
        uint8_t current_mode = readRegister(MCP_CANSTAT) & CANSTAT_OPMOD;

        // Reset controller
        ERROR err = reset();
        if (err != ERROR_OK) {
            return err;
        }

        // Restore mode
        if (current_mode == CANCTRL_REQOP_NORMAL) {
            return setNormalMode();
        } else if (current_mode == CANCTRL_REQOP_LISTENONLY) {
            return setListenOnlyMode();
        }
    }

    return ERROR_OK;
}

uint8_t MCP2515::getBusStatus(void)
{
    return readRegister(MCP_CANSTAT);
}

#endif /* ESP32 */
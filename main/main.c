/**
 * ESP32 SX1262 LoRa Communication
 * Simple point-to-point LoRa communication between two ESP32 nodes
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sx1262_hal.h"
#include "sx1262_driver.h"

static const char *TAG = "MAIN";

// ============================================
// PIN CONFIGURATION
// Configure these pins according to your wiring
// ============================================
#define SX1262_SPI_CS    5   // Chip Select
#define SX1262_SPI_MOSI  23  // Master Out Slave In
#define SX1262_SPI_MISO  19  // Master In Slave Out
#define SX1262_SPI_SCK   18  // SPI Clock
#define SX1262_BUSY      4   // Busy signal (must be LOW before SPI operations)
#define SX1262_RST       2   // Reset pin (active low)
#define SX1262_RXEN      26  // RX Enable (controls RF switch)
#define SX1262_TXEN      25  // TX Enable (controls RF switch)
#define SX1262_DIO1      32  // Interrupt pin
#define SX1262_DIO2      33  // Control pin
// DIO3 is not present on Waveshare Core1262 board

// ============================================
// LoRa PARAMETERS
// Both units must use the same parameters!
// ============================================
typedef struct {
    uint32_t frequency;         // RF frequency in Hz (e.g., 868100000 for 868.1 MHz)
    uint8_t spreading_factor;   // SF5-SF12 (default: 7)
    uint8_t bandwidth;          // Bandwidth option
    uint8_t coding_rate;        // Coding rate
    uint8_t tx_power;           // TX power in dBm (0-22)
} lora_config_t;

// Communication settings - MUST match UART module exactly
#define NETWORK_ID     0      // Must match cfg_reg[5] from UART module
#define NODE_ADDRESS   1      // Use 1, UART module uses 0
#define AIR_SPEED      2400   // 2400 bps - matches UART module air_speed parameter

static lora_config_t lora_config = {
    .frequency = 868100000,           // 868.1 MHz
    .spreading_factor = 10,          // SF10 for maximum range
    .bandwidth = 0,                   // 7.8 kHz (BW_0) for low data rate  
    .coding_rate = SX1262_LORA_CR_4_8, // Most robust
    .tx_power = 22                    // 22 dBm - maximum power
};

// ============================================
// LoRa HELPER FUNCTIONS
// ============================================

/**
 * Initialize LoRa module
 */
esp_err_t lora_init(const lora_config_t *config)
{
    ESP_LOGI(TAG, "Initializing LoRa...");
    
    // Reset module
    ESP_LOGI(TAG, "Resetting module...");
    sx1262_hal_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Check BUSY pin status
    bool busy_after_reset = gpio_get_level(SX1262_BUSY);
    ESP_LOGI(TAG, "BUSY pin after reset: %s", busy_after_reset ? "HIGH" : "LOW");
    
    // Check chip status and mode
    ESP_LOGI(TAG, "Checking chip status and mode...");
    sx1262_check_status_and_mode();
    
    // Set regulator mode (LDO = 0x01, use DCDC if available = 0x00)
    uint8_t reg_cmd[2] = {SX1262_OPCODE_SET_REGULATOR_MODE, 0x01};
    sx1262_hal_write(reg_cmd, 2);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Clear all IRQ flags
    sx1262_clear_irq_status(0xFFFF);
    
    // Configure IRQ masks for TX done and RX done
    uint16_t irq_mask = SX1262_IRQ_TX_DONE | SX1262_IRQ_RX_DONE | SX1262_IRQ_TIMEOUT | SX1262_IRQ_CRC_ERROR;
    uint8_t irq_cmd[5] = {SX1262_OPCODE_SET_DIO_IRQ_PARAMS, 
                          (uint8_t)(irq_mask >> 8), (uint8_t)(irq_mask), 
                          (uint8_t)(irq_mask >> 8), (uint8_t)(irq_mask)};
    sx1262_hal_write(irq_cmd, 5);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Set to standby XOSC (host controlled, never sleep)
    sx1262_set_standby();
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure as LoRa packet type (LoRa = 0x01)
    sx1262_set_packet_type(0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Set RF frequency
    sx1262_set_rf_frequency(config->frequency);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Set LoRa parameters (SF, BW, CR)
    sx1262_set_lora_params(
        config->spreading_factor,
        config->bandwidth,
        config->coding_rate
    );
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // DIO2 as RF switch control (set by many reference designs)
    uint8_t dio2_cmd[2] = { SX1262_OPCODE_SET_DIO2_AS_RF_SWITCH_CTRL, 0x01 };
    sx1262_hal_transfer(dio2_cmd, NULL, sizeof(dio2_cmd));
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure LoRa packet parameters
    // Preamble: 8 symbols, Header: explicit, Payload: variable length, CRC: on
    // Format: [opcode] [preamble_H] [preamble_L] [header] [max_length] [crc] [invert_iq]
    uint8_t cmd[7] = {
        SX1262_OPCODE_SET_LORA_PACKET_PARAMS,
        0x00, 0x08,  // Preamble length: 8 symbols
        0x00,         // Header: explicit (0x00)
        0xFF,         // Max payload length: 255
        0x01,         // CRC: enabled (0x01)
        0x00          // Invert IQ: disabled (0x00)
    };
    sx1262_hal_transfer(cmd, NULL, 7);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Set PA configuration for HP PA
    // Params: PaDutyCycle=0x04, HpMax=0x07, DeviceSel=0x00, PaLut=0x01 per app note
    uint8_t pa_cmd[5] = {SX1262_OPCODE_SET_PA_CONFIG, 0x04, 0x07, 0x00, 0x01};
    sx1262_hal_transfer(pa_cmd, NULL, 5);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "Configured for ultra-long range:");
    ESP_LOGI(TAG, "  - Frequency: %lu Hz", config->frequency);
    ESP_LOGI(TAG, "  - Spreading Factor: %d (SF10 - max range)", config->spreading_factor);
    ESP_LOGI(TAG, "  - Bandwidth: 7.8 kHz (lowest data rate)");
    ESP_LOGI(TAG, "  - Coding Rate: 4/8 (most robust)");
    ESP_LOGI(TAG, "  - TX Power: %d dBm (maximum)", config->tx_power);
    
    // Set TX parameters (ramp time: 0x07 = 200 us typical; keep 0x07 as used in driver)
    sx1262_set_tx_params(config->tx_power);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Set buffer base addresses (TX at 0, RX at 128 to avoid overlap)
    uint8_t buf_cmd[3] = {SX1262_OPCODE_SET_BUFFER_BASE_ADDRESS, 0x00, 0x80};
    sx1262_hal_transfer(buf_cmd, NULL, 3);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "LoRa initialized");
    return ESP_OK;
}

/**
 * Send data over LoRa
 */
esp_err_t lora_send(uint8_t *data, uint8_t len)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Preparing to send %d bytes", len);
    
    // Clear IRQ status
    sx1262_clear_irq_status(0xFFFF);
    
    ESP_LOGD(TAG, "Writing %d bytes to buffer", len);
    
    // Write data to SX1262 buffer
    ret = sx1262_write_buffer(data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write buffer");
        return ret;
    }
    
    // Enable TX mode
    ret = sx1262_set_tx(1000);  // 1 second timeout
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set TX");
        return ret;
    }
    
    ESP_LOGI(TAG, "Waiting for TX completion...");
    
    // Wait for transmission to complete
    int timeout = 5000;  // 5 second timeout
    uint16_t irq_status;
    
    ESP_LOGD(TAG, "Monitoring IRQ status...");
    
    while (timeout-- > 0) {
        irq_status = sx1262_get_irq_status();
        
        if (irq_status & SX1262_IRQ_TX_DONE) {
            sx1262_clear_irq_status(SX1262_IRQ_TX_DONE);
            ESP_LOGI(TAG, "✓ TX completed successfully!");
            return ESP_OK;
        }
        
        if (irq_status & SX1262_IRQ_TIMEOUT) {
            ESP_LOGE(TAG, "TX timeout IRQ received");
            sx1262_clear_irq_status(SX1262_IRQ_TIMEOUT);
            return ESP_ERR_TIMEOUT;
        } 
        
        if (irq_status & SX1262_IRQ_CRC_ERROR) {
            ESP_LOGE(TAG, "CRC error on TX");
            sx1262_clear_irq_status(SX1262_IRQ_CRC_ERROR);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Final diagnostic check
    irq_status = sx1262_get_irq_status();
    
    ESP_LOGE(TAG, "==============================================");
    ESP_LOGE(TAG, "TX TIMEOUT - Diagnostic Information:");
    ESP_LOGE(TAG, "  Final IRQ: 0x%04X", irq_status);
    ESP_LOGE(TAG, "==============================================");
    
    // Get chip mode
    uint8_t chip_mode = sx1262_get_chip_mode();
    const char* mode_names[] = {
        "SLEEP", "RC_RUN", "HXTAL", "RESET", 
        "STDBY_RC", "STDBY_XOSC", "FS", "RX", "TX"
    };
    ESP_LOGE(TAG, "  Chip Mode: %d (%s)", chip_mode, mode_names[chip_mode < 9 ? chip_mode : 8]);
    
    // Get chip error
    uint16_t chip_error = sx1262_get_chip_error();
    ESP_LOGE(TAG, "  Chip Error: 0x%04X", chip_error);
    
    // Return to STDBY_XOSC and avoid sleep
    ESP_LOGW(TAG, "Resetting to STDBY_XOSC mode...");
    sx1262_clear_irq_status(0xFFFF);
    vTaskDelay(pdMS_TO_TICKS(10));
    sx1262_set_standby();
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Check mode again
    chip_mode = sx1262_get_chip_mode();
    ESP_LOGI(TAG, "  After reset, Chip Mode: %d (%s)", chip_mode, mode_names[chip_mode < 9 ? chip_mode : 8]);
    
    return ESP_ERR_TIMEOUT;
}

/**
 * Receive data from LoRa
 */
esp_err_t lora_receive(uint8_t *data, uint8_t *len)
{
    esp_err_t ret;
    
    // Enable RX mode
    ret = sx1262_set_rx(1000);  // 1 second timeout
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for data to arrive
    int timeout = 5000;  // 5 second timeout
    while (timeout-- > 0) {
        uint16_t irq_status = sx1262_get_irq_status();
        if (irq_status & SX1262_IRQ_RX_DONE) {
            sx1262_clear_irq_status(SX1262_IRQ_RX_DONE);
            
            // Read received data - this now returns actual packet length
            ret = sx1262_read_buffer(data, len);
            if (ret == ESP_OK && *len > 0) {
                ESP_LOGI(TAG, "RX completed: %d bytes", *len);
                return ESP_OK;
            }
        }
        
        if (irq_status & SX1262_IRQ_TIMEOUT) {
            ESP_LOGE(TAG, "RX timeout IRQ");
            sx1262_clear_irq_status(SX1262_IRQ_TIMEOUT);
            return ESP_ERR_TIMEOUT;
        }
        
        if (irq_status & SX1262_IRQ_CRC_ERROR) {
            ESP_LOGE(TAG, "CRC error on RX");
            sx1262_clear_irq_status(SX1262_IRQ_CRC_ERROR);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGE(TAG, "RX timeout");
    return ESP_ERR_TIMEOUT;
}

// ============================================
// MAIN APPLICATION
// ============================================

void app_main(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "=== ESP32 LoRa Communication ===");
    
    // Initialize NVS (Non-Volatile Storage)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Configure GPIO pins
    ESP_LOGI(TAG, "Configuring GPIO pins...");
    sx1262_hal_config_t hal_config = {
        .spi_cs = SX1262_SPI_CS,
        .spi_mosi = SX1262_SPI_MOSI,
        .spi_miso = SX1262_SPI_MISO,
        .spi_sck = SX1262_SPI_SCK,
        .busy = SX1262_BUSY,
        .rst = SX1262_RST,
        .rxen = SX1262_RXEN,
        .txen = SX1262_TXEN,
        .dio1 = SX1262_DIO1,
        .dio2 = SX1262_DIO2,
        .dio3 = GPIO_NUM_NC  // Not present on Waveshare Core1262
    };
    
    // Initialize HAL (Hardware Abstraction Layer)
    ret = sx1262_hal_init(&hal_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HAL initialization failed");
        return;
    }
    
    // Initialize driver
    ret = sx1262_driver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Driver initialization failed");
        return;
    }
    
    // Initialize LoRa
    ret = lora_init(&lora_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LoRa initialization failed");
        return;
    }
    
    ESP_LOGI(TAG, "\n============================================");
    ESP_LOGI(TAG, "  LoRa Configuration (Matching UART Module)");
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "Frequency: %lu Hz (868.1 MHz)", lora_config.frequency);
    ESP_LOGI(TAG, "Spreading Factor: %d (SF%d)", lora_config.spreading_factor, lora_config.spreading_factor);
    ESP_LOGI(TAG, "Bandwidth: 7.8 kHz");
    ESP_LOGI(TAG, "Coding Rate: 4/8");
    ESP_LOGI(TAG, "TX Power: %d dBm", lora_config.tx_power);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Network ID: %d (must match)", NETWORK_ID);
    ESP_LOGI(TAG, "Node Address: %d", NODE_ADDRESS);
    ESP_LOGI(TAG, "Air Speed: %d bps", AIR_SPEED);
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "Node Ready! Waiting for messages...\n");
    
    // Main communication loop
    uint8_t tx_buffer[] = "Hello LoRa!";
    uint8_t rx_buffer[255];
    uint8_t rx_len;
    int tx_count = 0;
    
    while (1) {
        // === RECEIVE (Listen first) ===
        ESP_LOGI(TAG, "[RX] Listening for data from UART module...");
        ret = lora_receive(rx_buffer, &rx_len);
        if (ret == ESP_OK) {
            // UART module sends: [dest_addr_H][dest_addr_L][freq_offset][src_addr_H][src_addr_L][src_freq_offset][payload]
            // Try to parse it
            rx_buffer[rx_len] = '\0';
            ESP_LOGI(TAG, "✓ RX successful: %d bytes", rx_len);
            ESP_LOGI(TAG, "  Data: %.*s", rx_len > 6 ? rx_len - 6 : rx_len, &rx_buffer[6]);
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));  // Wait 2 seconds
        
        // === TRANSMIT ===
        ESP_LOGI(TAG, "[TX #%d] Sending to UART module: %s", ++tx_count, tx_buffer);
        ret = lora_send(tx_buffer, strlen((char*)tx_buffer));
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ TX successful");
        } else {
            ESP_LOGE(TAG, "✗ TX failed");
        }
        
        vTaskDelay(pdMS_TO_TICKS(15000));  // Wait 15 seconds
    }
}

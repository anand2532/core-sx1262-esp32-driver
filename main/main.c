/**
 * ESP32 SX1262 LoRa Communication
 * Simple RX-only receiver to test module operation
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sx1262_hal.h"
#include "sx1262_driver.h"

static const char *TAG = "MAIN";

// Pin Configuration
#define SX1262_SPI_CS    5   // Chip Select
#define SX1262_SPI_MOSI  23  // SPI Data Out
#define SX1262_SPI_MISO  19  // SPI Data In
#define SX1262_SPI_SCK   18  // SPI Clock
#define SX1262_BUSY      4   // Busy Signal (MUST be LOW for SPI)
#define SX1262_RST       2   // Reset (active LOW)
#define SX1262_DIO1      32  // Interrupt Pin
#define SX1262_DIO2      33  // RF Switch Control

void app_main(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "=== SX1262 RX Test Started ===");
    
    // Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // Configure HAL
    sx1262_hal_config_t hal_config = {
        .spi_cs = SX1262_SPI_CS,
        .spi_mosi = SX1262_SPI_MOSI,
        .spi_miso = SX1262_SPI_MISO,
        .spi_sck = SX1262_SPI_SCK,
        .busy = SX1262_BUSY,
        .rst = SX1262_RST,
        .rxen = GPIO_NUM_NC,  // Not used
        .txen = GPIO_NUM_NC,  // Not used
        .dio1 = SX1262_DIO1,
        .dio2 = SX1262_DIO2,
        .dio3 = GPIO_NUM_NC
    };
    
    // Initialize
    ESP_ERROR_CHECK(sx1262_hal_init(&hal_config));
    ESP_ERROR_CHECK(sx1262_driver_init());
    
    // Reset module
    ESP_LOGI(TAG, "Resetting module...");
    gpio_set_level(SX1262_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(SX1262_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    int busy = gpio_get_level(SX1262_BUSY);
    ESP_LOGI(TAG, "BUSY pin after reset: %d", busy);
    
    if (busy == 1) {
        ESP_LOGE(TAG, "ERROR: BUSY pin stuck HIGH!");
        ESP_LOGE(TAG, "Check wiring: GPIO4 -> BUSY pin");
        return;
    }
    
    // Initialize module
    ESP_LOGI(TAG, "Initializing SX1262...");
    
    // Wait for BUSY to be LOW
    int busy_timeout = 1000;
    while (gpio_get_level(SX1262_BUSY) == 1 && busy_timeout-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    if (gpio_get_level(SX1262_BUSY) == 1) {
        ESP_LOGE(TAG, "BUSY stuck HIGH - cannot initialize!");
        return;
    }
    
    // Set regulator mode (LDO)
    uint8_t cmd[2] = {SX1262_OPCODE_SET_REGULATOR_MODE, 0x01};
    ret = sx1262_hal_write(cmd, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set regulator mode");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Clear IRQ
    ret = sx1262_clear_irq_status(0xFFFF);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear IRQ");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Set standby
    ret = sx1262_set_standby();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set standby");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Set LoRa packet type
    ret = sx1262_set_packet_type(0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set packet type");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Enable DIO2 as RF switch (let chip control RF internally)
    uint8_t dio2_cmd[2] = {SX1262_OPCODE_SET_DIO2_AS_RF_SWITCH_CTRL, 0x01};
    ret = sx1262_hal_transfer(dio2_cmd, NULL, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DIO2");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Set frequency to 868.1 MHz
    uint32_t freq = 868100000;
    uint32_t rf_freq = (uint32_t)((double)freq / 0.95367431640625);
    uint8_t freq_cmd[5] = {
        SX1262_OPCODE_SET_RF_FREQUENCY,
        (uint8_t)(rf_freq >> 24),
        (uint8_t)(rf_freq >> 16),
        (uint8_t)(rf_freq >> 8),
        (uint8_t)(rf_freq)
    };
    ret = sx1262_hal_transfer(freq_cmd, NULL, 5);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set frequency");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Set LoRa parameters: SF10, BW7.8kHz, CR4/8
    uint8_t lora_cmd[5] = {
        SX1262_OPCODE_SET_LORA_PARAMS,
        10,  // SF10
        0,   // BW 7.8kHz
        SX1262_LORA_CR_4_8,
        1    // Low data rate optimize
    };
    ret = sx1262_hal_transfer(lora_cmd, NULL, 5);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa params");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Set packet parameters
    uint8_t pkt_cmd[7] = {
        SX1262_OPCODE_SET_LORA_PACKET_PARAMS,
        0x00, 0x08,  // Preamble
        0x00,        // Explicit header
        0xFF,        // Max payload
        0x01,        // CRC enabled
        0x00         // Invert IQ disabled
    };
    ret = sx1262_hal_transfer(pkt_cmd, NULL, 7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set packet params");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Set buffer base addresses
    uint8_t buf_cmd[3] = {SX1262_OPCODE_SET_BUFFER_BASE_ADDRESS, 0x00, 0x80};
    ret = sx1262_hal_transfer(buf_cmd, NULL, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set buffer addresses");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    
    ESP_LOGI(TAG, "=== Configuration Complete ===");
    ESP_LOGI(TAG, "Frequency: 868.1 MHz");
    ESP_LOGI(TAG, "SF: 10, BW: 7.8 kHz, CR: 4/8");
    ESP_LOGI(TAG, "Starting RX mode...");
    
    // Enter RX mode
    uint32_t timeout = 1000;  // 1 second timeout
    uint32_t timeout_value = (timeout * 64) / 1000;
    uint8_t rx_cmd[5] = {
        SX1262_OPCODE_SET_RX,
        (uint8_t)(timeout_value >> 16),
        (uint8_t)(timeout_value >> 8),
        (uint8_t)(timeout_value),
        0x00
    };
    ret = sx1262_hal_transfer(rx_cmd, NULL, 5);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter RX mode");
        return;
    }
    
    // Wait for data
    ESP_LOGI(TAG, "Listening for data...");
    
    while(1) {
        uint16_t irq = sx1262_get_irq_status();
        
        if (irq & SX1262_IRQ_RX_DONE) {
            ESP_LOGI(TAG, "RX DONE! Data received");
            sx1262_clear_irq_status(SX1262_IRQ_RX_DONE);
            
            // Read buffer status to get length
            uint8_t tx_status[3] = {SX1262_OPCODE_GET_RX_BUFFER_STATUS, 0x00, 0x00};
            uint8_t rx_status[3] = {0};
            sx1262_hal_transfer(tx_status, rx_status, 3);
            
            uint8_t len = rx_status[1];
            uint8_t offset = rx_status[2];
            ESP_LOGI(TAG, "Received: %d bytes (offset: %d)", len, offset);
            
            if (len > 0) {
                // Limit length to prevent buffer overflow
                uint8_t read_len = (len > 240) ? 240 : len;
                
                // Read actual data
                uint8_t tx_read[3 + 240] = {0};
                uint8_t rx_read[3 + 240] = {0};
                tx_read[0] = SX1262_OPCODE_READ_BUFFER;
                tx_read[1] = offset;
                tx_read[2] = 0x00; // dummy
                sx1262_hal_transfer(tx_read, rx_read, 3 + read_len);
                
                // Print data as hex
                ESP_LOGI(TAG, "Data (hex, first %d of %d bytes):", read_len, len);
                for (int i = 0; i < read_len; i++) {
                    if (i % 16 == 0) printf("\n  %04X: ", i);
                    printf("%02X ", rx_read[3 + i]);
                }
                printf("\n");
                
                // Print data as ASCII (if printable)
                ESP_LOGI(TAG, "Data (ASCII):");
                printf("  ");
                for (int i = 0; i < read_len && i < 80; i++) {
                    uint8_t c = rx_read[3 + i];
                    if (c >= 32 && c < 127) {
                        printf("%c", c);
                    } else {
                        printf(".");
                    }
                }
                printf("\n");
            }
            
            // Re-enter RX
            ret = sx1262_hal_transfer(rx_cmd, NULL, 5);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to re-enter RX mode");
            }
        }
        
        if (irq & SX1262_IRQ_TIMEOUT) {
            ESP_LOGI(TAG, "RX timeout, re-entering RX mode...");
            sx1262_clear_irq_status(SX1262_IRQ_TIMEOUT);
            ret = sx1262_hal_transfer(rx_cmd, NULL, 5);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to re-enter RX mode after timeout");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * ESP32 SX1262 LoRa Communication - SIMPLIFIED TEST
 * Only RX mode to test operation mode changes
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

// PIN CONFIGURATION
#define SX1262_SPI_CS    5
#define SX1262_SPI_MOSI  23
#define SX1262_SPI_MISO  19
#define SX1262_SPI_SCK   18
#define SX1262_BUSY      4
#define SX1262_RST       2
#define SX1262_RXEN      26
#define SX1262_TXEN      25
#define SX1262_DIO1      32
#define SX1262_DIO2      33

// Simplified test - no LoRa config needed

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32 SX1262 RX Only Test ===");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
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
        .dio3 = GPIO_NUM_NC
    };
    
    // Initialize HAL
    ESP_LOGI(TAG, "Initializing HAL...");
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
    
    ESP_LOGI(TAG, "=== Testing Module States ===");
    
    // Test 1: Check initial GPIO state
    ESP_LOGI(TAG, "\n--- Test 1: GPIO Pin States ---");
    ESP_LOGI(TAG, "Before any commands:");
    ESP_LOGI(TAG, "  TXEN (GPIO%d): %d", SX1262_TXEN, gpio_get_level(SX1262_TXEN));
    ESP_LOGI(TAG, "  RXEN (GPIO%d): %d", SX1262_RXEN, gpio_get_level(SX1262_RXEN));
    ESP_LOGI(TAG, "  BUSY (GPIO%d): %d", SX1262_BUSY, gpio_get_level(SX1262_BUSY));
    
    // Test 2: Manually set GPIO pins
    ESP_LOGI(TAG, "\n--- Test 2: Manual GPIO Control ---");
    gpio_set_level(SX1262_TXEN, 1);
    gpio_set_level(SX1262_RXEN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "After setting TXEN=HIGH, RXEN=LOW:");
    ESP_LOGI(TAG, "  TXEN (GPIO%d): %d", SX1262_TXEN, gpio_get_level(SX1262_TXEN));
    ESP_LOGI(TAG, "  RXEN (GPIO%d): %d", SX1262_RXEN, gpio_get_level(SX1262_RXEN));
    
    gpio_set_level(SX1262_TXEN, 0);
    gpio_set_level(SX1262_RXEN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "After setting TXEN=LOW, RXEN=HIGH:");
    ESP_LOGI(TAG, "  TXEN (GPIO%d): %d", SX1262_TXEN, gpio_get_level(SX1262_TXEN));
    ESP_LOGI(TAG, "  RXEN (GPIO%d): %d", SX1262_RXEN, gpio_get_level(SX1262_RXEN));
    
    // Test 3: Initialize LoRa module
    ESP_LOGI(TAG, "\n--- Test 3: LoRa Initialization ---");
    ESP_LOGI(TAG, "Resetting module...");
    gpio_set_level(SX1262_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(SX1262_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    int busy = gpio_get_level(SX1262_BUSY);
    ESP_LOGI(TAG, "BUSY pin after reset: %d (should be 0)", busy);
    
    if (busy == 1) {
        ESP_LOGE(TAG, "BUSY pin is stuck HIGH - hardware issue!");
        ESP_LOGE(TAG, "Cannot proceed with module initialization");
        ESP_LOGI(TAG, "\n=== DIAGNOSIS ===");
        ESP_LOGI(TAG, "Your SX1262 module has a HARDWARE PROBLEM:");
        ESP_LOGI(TAG, "1. Check BUSY pin (GPIO4) wiring");
        ESP_LOGI(TAG, "2. Check power supply (needs stable 3.3V)");
        ESP_LOGI(TAG, "3. Check ground connection");
        ESP_LOGI(TAG, "4. Module may be damaged");
        return;
    }
    
    // Basic commands only if BUSY is OK
    ESP_LOGI(TAG, "BUSY is LOW - proceeding with initialization...");
    
    // Set regulator mode
    uint8_t reg_cmd[2] = {SX1262_OPCODE_SET_REGULATOR_MODE, 0x01};
    sx1262_hal_write(reg_cmd, 2);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Set standby
    sx1262_set_standby();
    vTaskDelay(pdMS_TO_TICKS(10));
    
    uint8_t chip_mode = sx1262_get_chip_mode();
    ESP_LOGI(TAG, "Chip mode after STANDBY: %d", chip_mode);
    
    // Set packet type to LoRa
    sx1262_set_packet_type(0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    chip_mode = sx1262_get_chip_mode();
    ESP_LOGI(TAG, "Chip mode after SET_PACKET_TYPE: %d", chip_mode);
    
    ESP_LOGI(TAG, "\n=== DIAGNOSTIC SUMMARY ===");
    
    if (gpio_get_level(SX1262_TXEN) == 0 && gpio_get_level(SX1262_RXEN) == 1) {
        ESP_LOGI(TAG, "✓ GPIO pins ARE responding");
        ESP_LOGI(TAG, "✓ HAL functions work");
    } else {
        ESP_LOGE(TAG, "✗ GPIO pins NOT responding");
        ESP_LOGE(TAG, "✗ Hardware issue with ESP32 or wiring");
    }
    
    ESP_LOGI(TAG, "\nModule ready for RX mode testing");
    ESP_LOGI(TAG, "Complete initialization and then enter continuous RX mode...");
    
    vTaskDelay(pdMS_TO_TICKS(5000));
}

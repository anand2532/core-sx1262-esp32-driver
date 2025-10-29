#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sx1262_hal.h"
#include "sx1262_driver.h"

// Set 1 for transmitter example, 0 for receiver example
#ifndef APP_TX_MODE
#define APP_TX_MODE 0
#endif

// ESP32 default VSPI mapping (change if needed):
// SCK=GPIO18, MOSI=GPIO23, MISO=GPIO19, CS=GPIO5
// BUSY=GPIO4, RESET=GPIO27, DIO1=GPIO26
// RXEN/TXEN are not used (RF switch via DIO2)

static const char *TAG = "APP";

static void radio_init(void)
{
    sx1262_hal_config_t cfg = {
        .spi_cs = 5,
        .spi_mosi = 23,
        .spi_miso = 19,
        .spi_sck = 18,
        .busy = 4,
        .rst = 27,
        .rxen = GPIO_NUM_NC,
        .txen = GPIO_NUM_NC,
        .dio1 = 26,
        .dio2 = GPIO_NUM_NC,
        .dio3 = GPIO_NUM_NC,
    };
    ESP_ERROR_CHECK(sx1262_hal_init(&cfg));
    ESP_ERROR_CHECK(sx1262_driver_init());

    // Configure to match UART module settings: try 868.1 MHz, SF10, BW 7.8 kHz, CR 4/8, 20 dBm
    ESP_ERROR_CHECK(sx1262_configure_default(868100000));
    ESP_ERROR_CHECK(sx1262_set_lora_params(10, SX1262_LORA_BW_7_8, SX1262_LORA_CR_4_8));
    ESP_ERROR_CHECK(sx1262_set_tx_power(20));
    // Sync word: try 0x3444 (public). If still no link, try 0x1424 (private)
    ESP_ERROR_CHECK(sx1262_set_sync_word(0x3444));
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting SX1262 example (APP_TX_MODE=%d)", APP_TX_MODE);
    radio_init();

#if APP_TX_MODE
    const char *msg = "Hello from ESP32 SX1262";
    while (1) {
        ESP_ERROR_CHECK(sx1262_write_payload((const uint8_t *)msg, strlen(msg)));
        ESP_ERROR_CHECK(sx1262_tx(5000));
        ESP_LOGI(TAG, "TX sent: %s", msg);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#else
    uint8_t buf[255];
    uint8_t len = 0;
    // Clear stale IRQs then enter continuous RX
    uint16_t stale = sx1262_get_irq();
    if (stale) { sx1262_clear_irq(stale); }
    ESP_ERROR_CHECK(sx1262_enter_rx(0)); // continuous RX
    ESP_LOGI(TAG, "RX started (continuous)");
    while (1) {
        uint16_t irq = sx1262_get_irq();
        if (irq) {
            ESP_LOGI(TAG, "IRQ=0x%04X", irq);
            if (irq & 0x0002) { // RX_DONE
                if (sx1262_read_payload(buf, &len) == ESP_OK && len > 0) {
                    buf[len] = '\0';
                    ESP_LOGI(TAG, "RX len=%u data=%s", len, (char *)buf);
                }
            }
            sx1262_clear_irq(irq);
        }
        // Periodic GPIO state debug
        static int ctr = 0;
        if ((ctr++ % 10) == 0) { // ~500 ms if loop delay is 50 ms
            bool busy = sx1262_hal_get_busy_level();
            bool dio1 = sx1262_hal_get_dio1();
            bool rst  = sx1262_hal_get_rst_level();
            ESP_LOGI(TAG, "GPIO: BUSY=%d DIO1=%d RST=%d", busy, dio1, rst);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
#endif
}
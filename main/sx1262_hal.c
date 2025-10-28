#include "sx1262_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SX1262_HAL";

// Static variables
static spi_device_handle_t spi_handle = NULL;
static gpio_num_t gpio_busy = GPIO_NUM_NC;
static gpio_num_t gpio_rst = GPIO_NUM_NC;
static gpio_num_t gpio_rxen = GPIO_NUM_NC;
static gpio_num_t gpio_txen = GPIO_NUM_NC;
static gpio_num_t gpio_dio1 = GPIO_NUM_NC;
static gpio_num_t gpio_dio2 = GPIO_NUM_NC;
static gpio_num_t gpio_dio3 = GPIO_NUM_NC;

esp_err_t sx1262_hal_gpio_init(const sx1262_hal_config_t *config)
{
    gpio_config_t io_conf = {0};
    
    // Configure BUSY pin as input
    io_conf.pin_bit_mask = (1ULL << config->busy);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Configure RESET pin as output
    io_conf.pin_bit_mask = (1ULL << config->rst);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(config->rst, 1); // Keep high during operation
    
    // Configure RXEN pin as output
    io_conf.pin_bit_mask = (1ULL << config->rxen);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Configure TXEN pin as output
    io_conf.pin_bit_mask = (1ULL << config->txen);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Configure DIO1 pin as input with interrupt on rising edge
    io_conf.pin_bit_mask = (1ULL << config->dio1);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Install GPIO ISR service and add ISR handler for DIO1
    gpio_install_isr_service(0);
    gpio_isr_handler_add(config->dio1, NULL, NULL);
    
    // Configure DIO2 pin as input
    if (config->dio2 != GPIO_NUM_NC) {
        io_conf.pin_bit_mask = (1ULL << config->dio2);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }
    
    // Configure DIO3 pin as input
    if (config->dio3 != GPIO_NUM_NC) {
        io_conf.pin_bit_mask = (1ULL << config->dio3);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }
    
    // Store pin numbers
    gpio_busy = config->busy;
    gpio_rst = config->rst;
    gpio_rxen = config->rxen;
    gpio_txen = config->txen;
    gpio_dio1 = config->dio1;
    if (config->dio2 != GPIO_NUM_NC) {
        gpio_dio2 = config->dio2;
    }
    if (config->dio3 != GPIO_NUM_NC) {
        gpio_dio3 = config->dio3;
    }
    
    return ESP_OK;
}

esp_err_t sx1262_hal_init(const sx1262_hal_config_t *config)
{
    esp_err_t ret = ESP_OK;
    
    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = config->spi_miso,
        .mosi_io_num = config->spi_mosi,
        .sclk_io_num = config->spi_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure SPI device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,  // 8MHz (max is 18MHz)
        .mode = 0,                           // SPI mode 0
        .spics_io_num = config->spi_cs,
        .queue_size = 7,
        .flags = 0,
        .pre_cb = NULL,
    };
    
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize GPIO pins
    ret = sx1262_hal_gpio_init(config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "HAL initialized successfully");
    return ESP_OK;
}

esp_err_t sx1262_hal_deinit(void)
{
    if (spi_handle) {
        spi_bus_remove_device(spi_handle);
        spi_bus_free(HSPI_HOST);
        spi_handle = NULL;
    }
    return ESP_OK;
}

void sx1262_hal_wait_busy(void)
{
    int timeout = 1000; // Maximum wait time in ms
    int initial_busy = gpio_get_level(gpio_busy);
    
    // Don't wait if already not busy
    if (initial_busy == 0) {
        return;
    }
    
    int counter = 0;
    while (gpio_get_level(gpio_busy) == 1 && timeout-- > 0) {
        if (counter++ % 100 == 0) {
            ESP_LOGW(TAG, "BUSY pin still HIGH (timeout in %d ms)", timeout);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    if (timeout <= 0) {
        ESP_LOGE(TAG, "BUSY pin timeout - stuck at level: %d", gpio_get_level(gpio_busy));
    }
}

void sx1262_hal_reset(void)
{
    gpio_set_level(gpio_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(1));  // Wait 1ms
    gpio_set_level(gpio_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait 100ms for reset to complete
}

esp_err_t sx1262_hal_write(uint8_t *buffer, uint8_t size)
{
    // Wait for BUSY to be LOW before starting
    int busy_timeout = 1000;
    while (gpio_get_level(gpio_busy) == 1 && busy_timeout-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    if (busy_timeout <= 0) {
        ESP_LOGE(TAG, "BUSY pin HIGH before write - cannot proceed");
        return ESP_ERR_TIMEOUT;
    }
    
    // Create SPI transaction
    spi_transaction_t t = {
        .length = size * 8,
        .tx_buffer = buffer,
        .rx_buffer = NULL,
        .flags = 0,
    };
    
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for module to process the command
    vTaskDelay(pdMS_TO_TICKS(1));
    
    return ESP_OK;
}

esp_err_t sx1262_hal_read(uint8_t *buffer, uint8_t size)
{
    // Wait for BUSY to be LOW (chip is ready for SPI)
    int busy_timeout = 1000;
    while (gpio_get_level(gpio_busy) == 1 && busy_timeout-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    if (busy_timeout <= 0) {
        ESP_LOGE(TAG, "BUSY pin still HIGH before read - SPI may hang");
        return ESP_ERR_TIMEOUT;
    }
    
    // Allocate a dummy buffer to send during read (SPI needs TX data)
    uint8_t dummy_tx[size];
    memset(dummy_tx, 0xFF, size); // Send 0xFF while reading
    
    spi_transaction_t t = {
        .length = size * 8,
        .rx_buffer = buffer,
        .tx_buffer = dummy_tx,
    };
    
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI read failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Small delay after read
    vTaskDelay(pdMS_TO_TICKS(1));
    
    return ESP_OK;
}

esp_err_t sx1262_hal_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    // Wait for BUSY to be LOW
    int busy_timeout = 1000;
    while (gpio_get_level(gpio_busy) == 1 && busy_timeout-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (busy_timeout <= 0) {
        ESP_LOGE(TAG, "BUSY pin HIGH before transfer");
        return ESP_ERR_TIMEOUT;
    }

    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transfer failed: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

void sx1262_hal_set_tx_mode(void)
{
    gpio_set_level(gpio_txen, 1);
    gpio_set_level(gpio_rxen, 0);
}

void sx1262_hal_set_rx_mode(void)
{
    gpio_set_level(gpio_rxen, 1);
    gpio_set_level(gpio_txen, 0);
}

bool sx1262_hal_get_dio1(void)
{
    if (gpio_dio1 != GPIO_NUM_NC) {
        return gpio_get_level(gpio_dio1);
    }
    return false;
}

bool sx1262_hal_get_dio2(void)
{
    if (gpio_dio2 != GPIO_NUM_NC) {
        return gpio_get_level(gpio_dio2);
    }
    return false;
}

bool sx1262_hal_get_dio3(void)
{
    if (gpio_dio3 != GPIO_NUM_NC) {
        return gpio_get_level(gpio_dio3);
    }
    return false;
}


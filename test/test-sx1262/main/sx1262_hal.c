#include "sx1262_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SX1262_HAL";

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

    io_conf.pin_bit_mask = (1ULL << config->busy);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.pin_bit_mask = (1ULL << config->rst);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(config->rst, 1);

    if (config->rxen != GPIO_NUM_NC) {
        io_conf.pin_bit_mask = (1ULL << config->rxen);
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        gpio_set_level(config->rxen, 0);
    }

    if (config->txen != GPIO_NUM_NC) {
        io_conf.pin_bit_mask = (1ULL << config->txen);
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        gpio_set_level(config->txen, 0);
    }

    io_conf.pin_bit_mask = (1ULL << config->dio1);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    if (config->dio2 != GPIO_NUM_NC) {
        io_conf.pin_bit_mask = (1ULL << config->dio2);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }
    if (config->dio3 != GPIO_NUM_NC) {
        io_conf.pin_bit_mask = (1ULL << config->dio3);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }

    gpio_busy = config->busy;
    gpio_rst = config->rst;
    gpio_rxen = config->rxen;
    gpio_txen = config->txen;
    gpio_dio1 = config->dio1;
    gpio_dio2 = config->dio2;
    gpio_dio3 = config->dio3;

    return ESP_OK;
}

esp_err_t sx1262_hal_init(const sx1262_hal_config_t *config)
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .miso_io_num = config->spi_miso,
        .mosi_io_num = config->spi_mosi,
        .sclk_io_num = config->spi_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ret = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = config->spi_cs,
        .queue_size = 5,
    };
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = sx1262_hal_gpio_init(config);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "HAL init OK");
    return ESP_OK;
}

esp_err_t sx1262_hal_deinit(void)
{
    if (spi_handle) {
        spi_bus_remove_device(spi_handle);
        spi_handle = NULL;
    }
    spi_bus_free(VSPI_HOST);
    return ESP_OK;
}

void sx1262_hal_wait_busy(void)
{
    int timeout = 1000;
    while (gpio_get_level(gpio_busy) == 1 && timeout-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void sx1262_hal_reset(void)
{
    gpio_set_level(gpio_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(gpio_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

esp_err_t sx1262_hal_write(uint8_t *buffer, uint8_t size)
{
    sx1262_hal_wait_busy();
    spi_transaction_t t = {
        .length = size * 8,
        .tx_buffer = buffer,
    };
    return spi_device_transmit(spi_handle, &t);
}

esp_err_t sx1262_hal_read(uint8_t *buffer, uint8_t size)
{
    sx1262_hal_wait_busy();
    uint8_t dummy[size];
    memset(dummy, 0x00, sizeof(dummy));
    spi_transaction_t t = {
        .length = size * 8,
        .tx_buffer = dummy,
        .rx_buffer = buffer,
    };
    return spi_device_transmit(spi_handle, &t);
}

esp_err_t sx1262_hal_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    sx1262_hal_wait_busy();
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    sx1262_hal_wait_busy();
    return ret;
}

void sx1262_hal_set_tx_mode(void)
{
    if (gpio_txen != GPIO_NUM_NC) gpio_set_level(gpio_txen, 1);
    if (gpio_rxen != GPIO_NUM_NC) gpio_set_level(gpio_rxen, 0);
}

void sx1262_hal_set_rx_mode(void)
{
    if (gpio_txen != GPIO_NUM_NC) gpio_set_level(gpio_txen, 0);
    if (gpio_rxen != GPIO_NUM_NC) gpio_set_level(gpio_rxen, 1);
}

bool sx1262_hal_get_dio1(void)
{
    return gpio_dio1 != GPIO_NUM_NC ? gpio_get_level(gpio_dio1) : false;
}

bool sx1262_hal_get_dio2(void)
{
    return gpio_dio2 != GPIO_NUM_NC ? gpio_get_level(gpio_dio2) : false;
}

bool sx1262_hal_get_dio3(void)
{
    return gpio_dio3 != GPIO_NUM_NC ? gpio_get_level(gpio_dio3) : false;
}



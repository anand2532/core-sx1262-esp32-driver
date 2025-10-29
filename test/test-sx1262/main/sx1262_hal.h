#ifndef SX1262_HAL_H
#define SX1262_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t spi_cs;
    gpio_num_t spi_mosi;
    gpio_num_t spi_miso;
    gpio_num_t spi_sck;
    gpio_num_t busy;
    gpio_num_t rst;
    gpio_num_t rxen;
    gpio_num_t txen;
    gpio_num_t dio1;
    gpio_num_t dio2;
    gpio_num_t dio3;
} sx1262_hal_config_t;

esp_err_t sx1262_hal_init(const sx1262_hal_config_t *config);
esp_err_t sx1262_hal_deinit(void);

esp_err_t sx1262_hal_read(uint8_t *buffer, uint8_t size);
esp_err_t sx1262_hal_write(uint8_t *buffer, uint8_t size);
esp_err_t sx1262_hal_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len);

void sx1262_hal_wait_busy(void);
void sx1262_hal_reset(void);
esp_err_t sx1262_hal_gpio_init(const sx1262_hal_config_t *config);

void sx1262_hal_set_tx_mode(void);
void sx1262_hal_set_rx_mode(void);

bool sx1262_hal_get_dio1(void);
bool sx1262_hal_get_dio2(void);
bool sx1262_hal_get_dio3(void);

#ifdef __cplusplus
}
#endif

#endif // SX1262_HAL_H



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

/**
 * Pin Configuration Structure
 */
typedef struct {
    gpio_num_t spi_cs;          // SPI Chip Select
    gpio_num_t spi_mosi;        // SPI MOSI
    gpio_num_t spi_miso;        // SPI MISO
    gpio_num_t spi_sck;         // SPI Clock
    gpio_num_t busy;            // BUSY pin
    gpio_num_t rst;             // RESET pin
    gpio_num_t rxen;            // RX Enable
    gpio_num_t txen;            // TX Enable
    gpio_num_t dio1;            // DIO1 interrupt
    gpio_num_t dio2;            // DIO2
    gpio_num_t dio3;            // DIO3
} sx1262_hal_config_t;

/**
 * Initialize the HAL layer
 */
esp_err_t sx1262_hal_init(const sx1262_hal_config_t *config);

/**
 * Deinitialize the HAL layer
 */
esp_err_t sx1262_hal_deinit(void);

/**
 * Read from SX1262
 */
esp_err_t sx1262_hal_read(uint8_t *buffer, uint8_t size);

/**
 * Write to SX1262
 */
esp_err_t sx1262_hal_write(uint8_t *buffer, uint8_t size);

/**
 * Wait for BUSY pin to be low
 */
void sx1262_hal_wait_busy(void);

/**
 * Reset the module
 */
void sx1262_hal_reset(void);

/**
 * Configure GPIO pins
 */
esp_err_t sx1262_hal_gpio_init(const sx1262_hal_config_t *config);

/**
 * Set TX mode (TXEN high, RXEN low)
 */
void sx1262_hal_set_tx_mode(void);

/**
 * Set RX mode (RXEN high, TXEN low)
 */
void sx1262_hal_set_rx_mode(void);

/**
 * Read DIO1 pin state
 */
bool sx1262_hal_get_dio1(void);

/**
 * Read DIO2 pin state
 */
bool sx1262_hal_get_dio2(void);

/**
 * Read DIO3 pin state
 */
bool sx1262_hal_get_dio3(void);

#ifdef __cplusplus
}
#endif

#endif // SX1262_HAL_H


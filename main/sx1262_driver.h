#ifndef SX1262_DRIVER_H
#define SX1262_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * SX1262 Return Values
 */
#define SX1262_OK             0
#define SX1262_ERROR          -1

/**
 * SX1262 Opcodes
 */
#define SX1262_OPCODE_WRITE_REGISTER              0x00
#define SX1262_OPCODE_READ_REGISTER               0x1D
#define SX1262_OPCODE_WRITE_BUFFER                0x0E
#define SX1262_OPCODE_READ_BUFFER                 0x1E
#define SX1262_OPCODE_GET_STATUS_REG              0x03
#define SX1262_OPCODE_SET_SLEEP                   0x84
#define SX1262_OPCODE_SET_STANDBY                 0x80
#define SX1262_OPCODE_SET_RX                      0x82
#define SX1262_OPCODE_SET_TX                      0x83
#define SX1262_OPCODE_GET_RX_BUFFER_STATUS        0x13
#define SX1262_OPCODE_SET_DIO2_AS_RF_SWITCH_CTRL  0x9D
#define SX1262_OPCODE_SET_DIO3_AS_TCXO_CTRL       0x97
#define SX1262_OPCODE_GET_IRQ_STATUS              0x12
#define SX1262_OPCODE_CLR_IRQ_STATUS              0x02
#define SX1262_OPCODE_SET_LORA_PARAMS             0x8B
#define SX1262_OPCODE_SET_PACKET_TYPE             0x8A
#define SX1262_OPCODE_WRITE_BUFFER                0x0E
#define SX1262_OPCODE_GET_STATUS                  0xC0
#define SX1262_OPCODE_GET_STATUS_REG              0x03
#define SX1262_OPCODE_SET_PA_CONFIG               0x95
#define SX1262_OPCODE_SET_REGULATOR_MODE          0x96
#define SX1262_OPCODE_SET_RF_FREQUENCY            0x86
#define SX1262_OPCODE_SET_TX_PARAMS               0x8E
#define SX1262_OPCODE_SET_DIO_IRQ_PARAMS          0x08
#define SX1262_OPCODE_SET_REGULATOR_MODE          0x96
#define SX1262_OPCODE_SET_PA_CONFIG               0x95
#define SX1262_OPCODE_GET_PACKET_STATUS           0x14

/**
 * SX1262 Registers
 */
#define SX1262_REG_LR_WHITENING_INITIAL_MSB       0x06B8
#define SX1262_REG_LR_CRC_INITIAL_VALUE          0x06BC
#define SX1262_REG_LR_SYNC_WORD                   0x0740
#define SX1262_REG_CALIBRATED_IMAGE               0x08DB
#define SX1262_REG_IMAGE_CAL_STATUS               0x02E0

/**
 * LoRa Parameters
 */
#define SX1262_LORA_BW_7                         0
#define SX1262_LORA_BW_10                        1
#define SX1262_LORA_BW_15                        2
#define SX1262_LORA_BW_20                        3
#define SX1262_LORA_BW_31                        4
#define SX1262_LORA_BW_41                        5
#define SX1262_LORA_BW_62                        6
#define SX1262_LORA_BW_125                       7
#define SX1262_LORA_BW_250                       8
#define SX1262_LORA_BW_500                       9

#define SX1262_LORA_CR_4_5                       1
#define SX1262_LORA_CR_4_6                       2
#define SX1262_LORA_CR_4_7                       3
#define SX1262_LORA_CR_4_8                       4

#define SX1262_LORA_LOW_DATA_RATE_OPTIMIZE_ON     1
#define SX1262_LORA_LOW_DATA_RATE_OPTIMIZE_OFF    0

#define SX1262_LORA_HEADER_EXPLICIT               0
#define SX1262_LORA_HEADER_IMPLICIT               1

#define SX1262_PACKET_VARIABLE_LENGTH             0
#define SX1262_PACKET_FIXED_LENGTH                1

/**
 * IRQ Masks
 */
#define SX1262_IRQ_TX_DONE                        0x01
#define SX1262_IRQ_RX_DONE                        0x02
#define SX1262_IRQ_TIMEOUT                        0x04
#define SX1262_IRQ_CRC_ERROR                      0x08
#define SX1262_IRQ_HEADER_ERROR                   0x10
#define SX1262_IRQ_HEADER_VALID                   0x20
#define SX1262_IRQ_SYNCWORD_VALID                 0x40

/**
 * Function Declarations
 */
esp_err_t sx1262_driver_init(void);
esp_err_t sx1262_driver_deinit(void);
esp_err_t sx1262_set_sleep(void);
esp_err_t sx1262_set_standby(void);
esp_err_t sx1262_set_packet_type(uint8_t packet_type);
esp_err_t sx1262_set_rf_frequency(uint32_t frequency);
esp_err_t sx1262_set_lora_params(uint8_t spreading_factor, uint8_t bandwidth, uint8_t coding_rate);
esp_err_t sx1262_set_tx_params(uint8_t tx_power);
esp_err_t sx1262_set_rx(uint32_t timeout_in_ms);
esp_err_t sx1262_set_tx(uint32_t timeout_in_ms);
esp_err_t sx1262_read_buffer(uint8_t *buffer, uint8_t size);
esp_err_t sx1262_write_buffer(uint8_t *buffer, uint8_t size);
uint16_t sx1262_get_irq_status(void);
esp_err_t sx1262_clear_irq_status(uint16_t irq_mask);
esp_err_t sx1262_set_dio_irq_params(uint16_t irq_mask);
esp_err_t sx1262_set_regulator_mode(uint8_t regulator_mode);
esp_err_t sx1262_get_status(void);
uint16_t sx1262_get_chip_error(void);
esp_err_t sx1262_check_status_and_mode(void);
uint8_t sx1262_get_chip_mode(void);

#ifdef __cplusplus
}
#endif

#endif // SX1262_DRIVER_H


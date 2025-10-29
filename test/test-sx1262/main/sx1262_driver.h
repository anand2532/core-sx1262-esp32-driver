#ifndef SX1262_DRIVER_H
#define SX1262_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SX1262_OPCODE_WRITE_REGISTER              0x0D
#define SX1262_OPCODE_READ_REGISTER               0x1D
#define SX1262_OPCODE_WRITE_BUFFER                0x0E
#define SX1262_OPCODE_READ_BUFFER                 0x1E
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
#define SX1262_OPCODE_GET_STATUS                  0xC0
#define SX1262_OPCODE_SET_PA_CONFIG               0x95
#define SX1262_OPCODE_SET_REGULATOR_MODE          0x96
#define SX1262_OPCODE_SET_RF_FREQUENCY            0x86
#define SX1262_OPCODE_SET_TX_PARAMS               0x8E
#define SX1262_OPCODE_SET_DIO_IRQ_PARAMS          0x08
#define SX1262_OPCODE_GET_PACKET_STATUS           0x14
#define SX1262_OPCODE_SET_LORA_PACKET_PARAMS      0x8C
#define SX1262_OPCODE_SET_BUFFER_BASE_ADDRESS     0x8F

#define SX1262_REG_LR_SYNC_WORD                   0x0740

#define SX1262_LORA_BW_7_8                        0x00
#define SX1262_LORA_BW_125                        0x04
#define SX1262_LORA_BW_250                        0x05
#define SX1262_LORA_BW_500                        0x06

#define SX1262_LORA_CR_4_5                        0x01
#define SX1262_LORA_CR_4_6                        0x02
#define SX1262_LORA_CR_4_7                        0x03
#define SX1262_LORA_CR_4_8                        0x04

esp_err_t sx1262_driver_init(void);
esp_err_t sx1262_configure_default(uint32_t freq_hz);
esp_err_t sx1262_write_payload(const uint8_t *data, uint8_t len);
esp_err_t sx1262_tx(uint32_t timeout_ms);
esp_err_t sx1262_enter_rx(uint32_t timeout_ms);
esp_err_t sx1262_read_payload(uint8_t *data, uint8_t *len);
uint16_t sx1262_get_irq(void);
void sx1262_clear_irq(uint16_t mask);

#ifdef __cplusplus
}
#endif

#endif // SX1262_DRIVER_H



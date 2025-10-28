#include <string.h>
#include "sx1262_driver.h"
#include "sx1262_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SX1262_DRIVER";

// Frequency step: 0.95367431640625 Hz per step
#define FREQ_STEP  0.95367431640625

esp_err_t sx1262_driver_init(void)
{
    ESP_LOGI(TAG, "Initializing SX1262 driver");
    return ESP_OK;
}

esp_err_t sx1262_driver_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing SX1262 driver");
    return ESP_OK;
}

esp_err_t sx1262_set_sleep(void)
{
    uint8_t opcode = SX1262_OPCODE_SET_SLEEP;
    sx1262_hal_write(&opcode, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    return ESP_OK;
}

esp_err_t sx1262_set_standby(void)
{
    uint8_t cmd[2] = {SX1262_OPCODE_SET_STANDBY, 0x00}; // STDBY_XOSC
    sx1262_hal_write(cmd, 2);
    vTaskDelay(pdMS_TO_TICKS(2));
    return ESP_OK;
}

esp_err_t sx1262_set_packet_type(uint8_t packet_type)
{
    uint8_t cmd[2] = {SX1262_OPCODE_SET_PACKET_TYPE, packet_type};
    sx1262_hal_write(cmd, 2);
    return ESP_OK;
}

esp_err_t sx1262_set_dio1_parameters(uint16_t irq_mask)
{
    uint8_t cmd[9] = {
        0x0A,  // SET_DIO1_AS_RF_SWITCH_CTRL (custom, needs to be checked)
        (uint8_t)(irq_mask >> 8),
        (uint8_t)(irq_mask),
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    sx1262_hal_write(cmd, 9);
    return ESP_OK;
}

esp_err_t sx1262_set_lora_packet_params(uint16_t preamble_len, uint8_t header_type, uint8_t payload_len, bool crc_on)
{
    uint8_t cmd[7] = {
        0x8C,  // SET_LORA_PACKET_PARAMS
        (uint8_t)(preamble_len >> 8),
        (uint8_t)(preamble_len),
        0x01,  // Header (explicit)
        payload_len,
        crc_on ? 0x01 : 0x00,
        0x00
    };
    sx1262_hal_write(cmd, 7);
    return ESP_OK;
}

esp_err_t sx1262_set_buffer_base_address(uint8_t tx_base_addr, uint8_t rx_base_addr)
{
    uint8_t cmd[3] = {0x8F, tx_base_addr, rx_base_addr};  // SET_BUFFER_BASE_ADDRESS
    sx1262_hal_write(cmd, 3);
    return ESP_OK;
}

esp_err_t sx1262_set_rf_frequency(uint32_t frequency)
{
    uint32_t rf_freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
    uint8_t frame[5] = {
        SX1262_OPCODE_SET_RF_FREQUENCY,
        (uint8_t)(rf_freq >> 24),
        (uint8_t)(rf_freq >> 16),
        (uint8_t)(rf_freq >> 8),
        (uint8_t)(rf_freq)
    };
    sx1262_hal_transfer(frame, NULL, sizeof(frame));
    return ESP_OK;
}

esp_err_t sx1262_set_lora_params(uint8_t spreading_factor, uint8_t bandwidth, uint8_t coding_rate)
{
    uint8_t frame[5] = {
        SX1262_OPCODE_SET_LORA_PARAMS,
        spreading_factor,
        bandwidth,
        coding_rate,
        (spreading_factor >= 11) ? SX1262_LORA_LOW_DATA_RATE_OPTIMIZE_ON : SX1262_LORA_LOW_DATA_RATE_OPTIMIZE_OFF
    };
    sx1262_hal_transfer(frame, NULL, sizeof(frame));
    return ESP_OK;
}

esp_err_t sx1262_set_tx_params(uint8_t tx_power)
{
    uint8_t frame[3] = {SX1262_OPCODE_SET_TX_PARAMS, tx_power, 0x07};
    sx1262_hal_transfer(frame, NULL, sizeof(frame));
    return ESP_OK;
}

esp_err_t sx1262_set_rx(uint32_t timeout_in_ms)
{
    uint32_t timeout = (timeout_in_ms * 64) / 1000;
    if (timeout > 0xFFFFFF) timeout = 0xFFFFFF;
    
    uint8_t cmd[5] = {
        SX1262_OPCODE_SET_RX,
        (uint8_t)(timeout >> 16),
        (uint8_t)(timeout >> 8),
        (uint8_t)(timeout),
        0x00  // rx_boosted_if_dtc1
    };
    
    sx1262_hal_transfer(cmd, NULL, sizeof(cmd));
    sx1262_hal_set_rx_mode();
    
    return ESP_OK;
}

esp_err_t sx1262_set_tx(uint32_t timeout_in_ms)
{
    uint32_t timeout = (timeout_in_ms * 64) / 1000;
    if (timeout > 0xFFFFFF) timeout = 0xFFFFFF;
    
    uint8_t cmd[5] = {
        SX1262_OPCODE_SET_TX,
        (uint8_t)(timeout >> 16),
        (uint8_t)(timeout >> 8),
        (uint8_t)(timeout),
        0x00  // Ramp time
    };
    
    sx1262_hal_transfer(cmd, NULL, sizeof(cmd));
    sx1262_hal_set_tx_mode();
    
    return ESP_OK;
}

esp_err_t sx1262_read_buffer(uint8_t *buffer, uint8_t size)
{
    // 1) Get RX buffer status (single transfer)
    uint8_t tx_status[3] = { SX1262_OPCODE_GET_RX_BUFFER_STATUS, 0x00, 0x00 };
    uint8_t rx_status[3] = {0};
    sx1262_hal_transfer(tx_status, rx_status, sizeof(tx_status));
    uint8_t rx_start_buffer_pointer = rx_status[2]; // rx_status[1]=len, [2]=ptr

    // 2) Read buffer: opcode + offset + dummy, read back status+data
    uint8_t tx[3 + 255] = {0};
    uint8_t rx[3 + 255] = {0};
    tx[0] = SX1262_OPCODE_READ_BUFFER;
    tx[1] = rx_start_buffer_pointer;
    tx[2] = 0x00; // dummy
    sx1262_hal_transfer(tx, rx, 3 + size);
    // rx[0]=status, rx[1]=first data? Per datasheet after dummy, data starts.
    memcpy(buffer, &rx[3], size);
    
    return ESP_OK;
}

esp_err_t sx1262_write_buffer(uint8_t *buffer, uint8_t size)
{
    // Single transaction: opcode + offset + payload
    uint8_t frame[2 + 255] = {0};
    frame[0] = SX1262_OPCODE_WRITE_BUFFER;
    frame[1] = 0x00; // offset 0
    memcpy(&frame[2], buffer, size);
    sx1262_hal_transfer(frame, NULL, 2 + size);
    
    return ESP_OK;
}

uint16_t sx1262_get_irq_status(void)
{
    uint8_t tx[3] = { SX1262_OPCODE_GET_IRQ_STATUS, 0x00, 0x00 };
    uint8_t rx[3] = {0};
    // Full-duplex single transaction: opcode + 2 dummy, read 3 bytes (Status + IRQ)
    sx1262_hal_transfer(tx, rx, sizeof(tx));

    // According to datasheet GetIrqStatus response:
    // Byte 0: Status (CMD_ERROR, etc.)
    // Byte 1: IRQ[15:8] (high byte)
    // Byte 2: IRQ[7:0]  (low byte)
    uint16_t irq_status = ((uint16_t)rx[1] << 8) | rx[2];
    
    // Check for suspicious patterns - but don't filter them yet
    ESP_LOGI(TAG, "IRQ raw: [0x%02X] [0x%02X] [0x%02X] → IRQ=0x%04X", 
             rx[0], rx[1], rx[2], irq_status);
    
    return irq_status;
}

esp_err_t sx1262_clear_irq_status(uint16_t irq_mask)
{
    uint8_t frame[3] = {
        SX1262_OPCODE_CLR_IRQ_STATUS,
        (uint8_t)(irq_mask >> 8),
        (uint8_t)(irq_mask)
    };
    sx1262_hal_transfer(frame, NULL, sizeof(frame));
    return ESP_OK;
}

esp_err_t sx1262_set_dio_irq_params(uint16_t irq_mask)
{
    uint8_t cmd[5] = {
        SX1262_OPCODE_SET_DIO_IRQ_PARAMS,
        (uint8_t)(irq_mask >> 8),
        (uint8_t)(irq_mask),
        (uint8_t)(irq_mask >> 8),
        (uint8_t)(irq_mask)
    };
    
    sx1262_hal_write(cmd, 5);
    return ESP_OK;
}

esp_err_t sx1262_set_regulator_mode(uint8_t regulator_mode)
{
    uint8_t cmd[2] = {SX1262_OPCODE_SET_REGULATOR_MODE, regulator_mode};
    sx1262_hal_write(cmd, 2);
    return ESP_OK;
}

esp_err_t sx1262_get_status(void)
{
    uint8_t cmd = SX1262_OPCODE_GET_STATUS;
    uint8_t status[1];
    sx1262_hal_write(&cmd, 1);
    sx1262_hal_read(status, 1);
    
    ESP_LOGD(TAG, "Chip Status: 0x%02X", status[0]);
    
    return ESP_OK;
}

esp_err_t sx1262_check_status_and_mode(void)
{
    // Single transaction: opcode + dummy byte, read status
    uint8_t tx[2] = { SX1262_OPCODE_GET_STATUS, 0x00 };
    uint8_t rx[2] = {0};
    sx1262_hal_transfer(tx, rx, sizeof(tx));
    
    uint8_t chip_mode = (rx[0] >> 2) & 0x07;
    uint8_t chip_status = rx[0] & 0x03;
    
    const char* mode_str[] = {
        "Sleep", "RC running", "HXTAL", "Reset", 
        "Standby RC", "Standby XOSC", "FS", "RX", "TX"
    };
    
    ESP_LOGI(TAG, "Chip Status: 0x%02X", rx[0]);
    ESP_LOGI(TAG, "  - Mode: %s (%d)", mode_str[chip_mode % 9], chip_mode);
    ESP_LOGI(TAG, "  - Status bits: 0x%02X", chip_status);
    
    return ESP_OK;
}

uint8_t sx1262_get_chip_mode(void)
{
    uint8_t tx[2] = { SX1262_OPCODE_GET_STATUS, 0x00 };
    uint8_t rx[2] = {0};
    sx1262_hal_transfer(tx, rx, sizeof(tx));
    
    return (rx[0] >> 2) & 0x07;
}

uint16_t sx1262_get_chip_error(void)
{
    uint8_t cmd = 0x17;  // GET_ERROR
    uint8_t error;
    sx1262_hal_write(&cmd, 1);
    sx1262_hal_read(&error, 1);
    return error;
}


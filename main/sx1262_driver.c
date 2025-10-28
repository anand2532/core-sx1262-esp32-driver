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
    uint8_t cmd[5] = {
        SX1262_OPCODE_SET_RF_FREQUENCY,
        (uint8_t)(rf_freq >> 24),
        (uint8_t)(rf_freq >> 16),
        (uint8_t)(rf_freq >> 8),
        (uint8_t)(rf_freq)
    };
    sx1262_hal_write(cmd, 5);
    return ESP_OK;
}

esp_err_t sx1262_set_lora_params(uint8_t spreading_factor, uint8_t bandwidth, uint8_t coding_rate)
{
    uint8_t cmd[5] = {
        SX1262_OPCODE_SET_LORA_PARAMS,
        spreading_factor,
        bandwidth,
        coding_rate,
        SX1262_LORA_LOW_DATA_RATE_OPTIMIZE_OFF
    };
    
    // Enable low data rate optimization for SF11 and SF12
    if (spreading_factor >= 11) {
        cmd[4] = SX1262_LORA_LOW_DATA_RATE_OPTIMIZE_ON;
    }
    
    sx1262_hal_write(cmd, 5);
    return ESP_OK;
}

esp_err_t sx1262_set_tx_params(uint8_t tx_power)
{
    uint8_t cmd[3] = {SX1262_OPCODE_SET_TX_PARAMS, tx_power, 0x07};
    sx1262_hal_write(cmd, 3);
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
    
    sx1262_hal_write(cmd, 5);
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
    
    sx1262_hal_write(cmd, 5);
    sx1262_hal_set_tx_mode();
    
    return ESP_OK;
}

esp_err_t sx1262_read_buffer(uint8_t *buffer, uint8_t size)
{
    uint8_t cmd[3];
    
    // Get RX buffer status
    uint8_t status_cmd = SX1262_OPCODE_GET_RX_BUFFER_STATUS;
    sx1262_hal_write(&status_cmd, 1);
    
    uint8_t status[2];
    sx1262_hal_read(status, 2);
    
    uint8_t rx_start_buffer_pointer = status[1];
    
    // Read data from buffer
    cmd[0] = SX1262_OPCODE_READ_BUFFER;
    cmd[1] = rx_start_buffer_pointer;
    cmd[2] = size;
    
    sx1262_hal_write(cmd, 3);
    sx1262_hal_read(buffer, size);
    
    return ESP_OK;
}

esp_err_t sx1262_write_buffer(uint8_t *buffer, uint8_t size)
{
    uint8_t cmd[2];
    
    cmd[0] = SX1262_OPCODE_WRITE_BUFFER;
    cmd[1] = 0x00;  // Offset
    
    sx1262_hal_write(cmd, 2);
    sx1262_hal_write(buffer, size);
    
    return ESP_OK;
}

uint16_t sx1262_get_irq_status(void)
{
    uint8_t cmd = SX1262_OPCODE_GET_IRQ_STATUS;
    uint8_t status[3];
    
    // Write command
    sx1262_hal_write(&cmd, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Read response (should be 3 bytes: Status[7:0], IRQ_0[7:0], IRQ_0[15:8])
    sx1262_hal_read(status, 3);
    
    // IRQ is in bytes 1 and 2
    uint16_t irq_status = ((uint16_t)status[2] << 8) | status[1];
    
    if ((status[1] == 0xA8 && status[2] == 0xA8) || (status[0] == 0xA8 && status[1] == 0xA8)) {
        // Bad read - log the raw bytes
        ESP_LOGW(TAG, "Bad IRQ read: 0x%02X 0x%02X 0x%02X - module may not be responding correctly", 
                 status[0], status[1], status[2]);
    } else {
        ESP_LOGD(TAG, "IRQ Status: [0x%02X] [0x%02X] [0x%02X] → 0x%04X", 
                 status[0], status[1], status[2], irq_status);
    }
    
    return irq_status;
}

esp_err_t sx1262_clear_irq_status(uint16_t irq_mask)
{
    uint8_t cmd[3] = {
        SX1262_OPCODE_CLR_IRQ_STATUS,
        (uint8_t)(irq_mask >> 8),
        (uint8_t)(irq_mask)
    };
    
    sx1262_hal_write(cmd, 3);
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
    uint8_t cmd = SX1262_OPCODE_GET_STATUS;
    uint8_t status[1];
    
    sx1262_hal_write(&cmd, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    sx1262_hal_read(status, 1);
    
    uint8_t chip_mode = (status[0] >> 2) & 0x07;
    uint8_t chip_status = status[0] & 0x03;
    
    const char* mode_str[] = {
        "Sleep", "RC running", "HXTAL", "Reset", 
        "Standby RC", "Standby XOSC", "FS", "RX", "TX"
    };
    
    ESP_LOGI(TAG, "Chip Status: 0x%02X", status[0]);
    ESP_LOGI(TAG, "  - Mode: %s (%d)", mode_str[chip_mode % 9], chip_mode);
    ESP_LOGI(TAG, "  - Status bits: 0x%02X", chip_status);
    
    return ESP_OK;
}

uint8_t sx1262_get_chip_mode(void)
{
    uint8_t cmd = SX1262_OPCODE_GET_STATUS;
    uint8_t status[1];
    
    sx1262_hal_write(&cmd, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    sx1262_hal_read(status, 1);
    
    return (status[0] >> 2) & 0x07;
}

uint16_t sx1262_get_chip_error(void)
{
    uint8_t cmd = 0x17;  // GET_ERROR
    uint8_t error;
    sx1262_hal_write(&cmd, 1);
    sx1262_hal_read(&error, 1);
    return error;
}


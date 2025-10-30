#include <string.h>
#include "sx1262_driver.h"
#include "sx1262_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SX1262_DRV";

static esp_err_t send_simple(const uint8_t *buf, uint8_t len)
{
    return sx1262_hal_write((uint8_t *)buf, len);
}

esp_err_t sx1262_driver_init(void)
{
    // Hardware reset
    sx1262_hal_reset();

    // Sanity check: Try to read chip status to verify SPI communication
    uint8_t status_cmd[2] = { SX1262_OPCODE_GET_STATUS, 0x00 };
    uint8_t status_resp[2] = {0};
    esp_err_t ret = sx1262_hal_transfer(status_cmd, status_resp, sizeof(status_cmd));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Chip status after reset: 0x%02X", status_resp[0]);
        uint8_t chip_mode = (status_resp[0] >> 4) & 0x07;
        ESP_LOGI(TAG, "Chip mode: %d (0=Sleep, 2=Standby RC, 3=Standby XOSC)", chip_mode);
    } else {
        ESP_LOGE(TAG, "Failed to read chip status! SPI communication may be broken.");
    }

    // DIO2 controls RF switch
    uint8_t rf_switch_cmd[2] = { SX1262_OPCODE_SET_DIO2_AS_RF_SWITCH_CTRL, 0x01 };
    ESP_ERROR_CHECK(send_simple(rf_switch_cmd, sizeof(rf_switch_cmd)));

    // Packet type LoRa
    uint8_t pkt_type[2] = { SX1262_OPCODE_SET_PACKET_TYPE, 0x01 };
    ESP_ERROR_CHECK(send_simple(pkt_type, sizeof(pkt_type)));

    // Stop timer on SyncWord/Header detection (RX)
    uint8_t stop_on_preamble[2] = { 0x9F, 0x00 };
    ESP_ERROR_CHECK(send_simple(stop_on_preamble, sizeof(stop_on_preamble)));

    // Regulator mode: DC-DC (0x01) if available; else LDO (0x00)
    uint8_t reg_mode[2] = { SX1262_OPCODE_SET_REGULATOR_MODE, 0x01 };
    ESP_ERROR_CHECK(send_simple(reg_mode, sizeof(reg_mode)));

    // PA config (duty, hpMax, device, paLut)
    uint8_t pa_cfg[5] = { SX1262_OPCODE_SET_PA_CONFIG, 0x04, 0x07, 0x00, 0x01 };
    ESP_ERROR_CHECK(send_simple(pa_cfg, sizeof(pa_cfg)));

    // Base addresses
    uint8_t base_addr[3] = { SX1262_OPCODE_SET_BUFFER_BASE_ADDRESS, 0x00, 0x00 };
    ESP_ERROR_CHECK(send_simple(base_addr, sizeof(base_addr)));

    // Enable DIO1 interrupts for RX_DONE, TX_DONE, TIMEOUT, CRC_ERROR
    // Irq bits (LSB): bit0=TX_DONE, bit1=RX_DONE, bit2=TIMEOUT, bit3=CRC_ERROR
    uint8_t irq_lsb = 0x0F;
    uint8_t dio_irq[9] = {
        SX1262_OPCODE_SET_DIO_IRQ_PARAMS,
        0x00, irq_lsb,      // IrqMask MSB/LSB
        0x00, irq_lsb,      // Map to DIO1
        0x00, 0x00,         // DIO2 mask
        0x00, 0x00          // DIO3 mask
    };
    ESP_ERROR_CHECK(send_simple(dio_irq, sizeof(dio_irq)));

    ESP_LOGI(TAG, "Driver init OK");
    return ESP_OK;
}

static esp_err_t set_rf_freq(uint32_t freq_hz)
{
    // rfFreqSteps = freq / (32e6 / 2^25) = freq / 0.95367431640625
    uint32_t steps = (uint32_t)((double)freq_hz / 0.95367431640625);
    uint8_t cmd[5] = { SX1262_OPCODE_SET_RF_FREQUENCY,
                       (uint8_t)(steps >> 24), (uint8_t)(steps >> 16),
                       (uint8_t)(steps >> 8), (uint8_t)(steps) };
    return send_simple(cmd, sizeof(cmd));
}

static esp_err_t set_lora_mod(uint8_t sf, uint8_t bw, uint8_t cr)
{
    uint8_t ldo = (sf >= 11) ? 1 : 0;
    uint8_t cmd[5] = { SX1262_OPCODE_SET_LORA_PARAMS, sf, bw, cr, ldo };
    return send_simple(cmd, sizeof(cmd));
}

esp_err_t sx1262_configure_default(uint32_t freq_hz)
{
    ESP_ERROR_CHECK(set_rf_freq(freq_hz));

    // BW=125kHz, CR=4/5, SF7 defaults
    ESP_ERROR_CHECK(set_lora_mod(7, SX1262_LORA_BW_125, SX1262_LORA_CR_4_5));

    // TX params: power=22dBm, ramp 40us
    uint8_t txp[3] = { SX1262_OPCODE_SET_TX_PARAMS, 22, 0x02 };
    ESP_ERROR_CHECK(send_simple(txp, sizeof(txp)));

    // LoRa packet params: preamble 12, explicit header, payload len 0xFF, CRC OFF, IQ normal
    uint8_t pkt[7] = { SX1262_OPCODE_SET_LORA_PACKET_PARAMS, 0x00, 0x0C, 0x00, 0xFF, 0x00, 0x00 };
    ESP_ERROR_CHECK(send_simple(pkt, sizeof(pkt)));

    return ESP_OK;
}

esp_err_t sx1262_set_rf_frequency(uint32_t freq_hz)
{
    return set_rf_freq(freq_hz);
}

esp_err_t sx1262_set_lora_params(uint8_t spreading_factor, uint8_t bandwidth, uint8_t coding_rate)
{
    return set_lora_mod(spreading_factor, bandwidth, coding_rate);
}

esp_err_t sx1262_set_tx_power(uint8_t dbm)
{
    uint8_t txp[3] = { SX1262_OPCODE_SET_TX_PARAMS, dbm, 0x02 };
    return send_simple(txp, sizeof(txp));
}

esp_err_t sx1262_set_sync_word(uint16_t sync_word)
{
    // Write LoRa sync word register 0x0740 in a single transaction
    uint8_t frame[3 + 2] = {
        SX1262_OPCODE_WRITE_REGISTER,
        (uint8_t)(SX1262_REG_LR_SYNC_WORD >> 8),
        (uint8_t)(SX1262_REG_LR_SYNC_WORD & 0xFF),
        (uint8_t)(sync_word >> 8),
        (uint8_t)(sync_word & 0xFF)
    };
    return sx1262_hal_transfer(frame, NULL, sizeof(frame));
}

esp_err_t sx1262_set_standby(uint8_t mode_rc_or_xosc)
{
    uint8_t cmd[2] = { SX1262_OPCODE_SET_STANDBY, (mode_rc_or_xosc ? 0x01 : 0x00) };
    return send_simple(cmd, sizeof(cmd));
}

esp_err_t sx1262_set_fs(void)
{
    uint8_t cmd = SX1262_OPCODE_SET_FS;
    return send_simple(&cmd, 1);
}

uint8_t sx1262_get_status_raw(void)
{
    uint8_t tx[2] = { SX1262_OPCODE_GET_STATUS, 0x00 };
    uint8_t rx[2] = {0};
    sx1262_hal_transfer(tx, rx, sizeof(tx));
    return rx[0];
}

esp_err_t sx1262_get_device_errors(uint16_t *errors)
{
    if (!errors) return ESP_ERR_INVALID_ARG;
    uint8_t tx[3] = { SX1262_OPCODE_GET_DEVICE_ERRORS, 0x00, 0x00 };
    uint8_t rx[3] = {0};
    esp_err_t ret = sx1262_hal_transfer(tx, rx, sizeof(tx));
    if (ret != ESP_OK) return ret;
    *errors = ((uint16_t)rx[1] << 8) | rx[2];
    return ESP_OK;
}

esp_err_t sx1262_clear_device_errors(void)
{
    uint8_t cmd = SX1262_OPCODE_CLEAR_DEVICE_ERRORS;
    return send_simple(&cmd, 1);
}

esp_err_t sx1262_read_register(uint16_t addr, uint8_t *buf, uint8_t len)
{
    if (!buf || len == 0) return ESP_ERR_INVALID_ARG;
    uint8_t tx[4 + 255] = {0};
    uint8_t rx[4 + 255] = {0};
    tx[0] = SX1262_OPCODE_READ_REGISTER;
    tx[1] = (uint8_t)(addr >> 8);
    tx[2] = (uint8_t)(addr & 0xFF);
    tx[3] = 0x00; // dummy
    sx1262_hal_transfer(tx, rx, 4 + len);
    memcpy(buf, &rx[4], len);
    return ESP_OK;
}

esp_err_t sx1262_write_register(uint16_t addr, const uint8_t *buf, uint8_t len)
{
    if (!buf || len == 0) return ESP_ERR_INVALID_ARG;
    uint8_t frame[3 + 255] = {0};
    frame[0] = SX1262_OPCODE_WRITE_REGISTER;
    frame[1] = (uint8_t)(addr >> 8);
    frame[2] = (uint8_t)(addr & 0xFF);
    memcpy(&frame[3], buf, len);
    return sx1262_hal_transfer(frame, NULL, 3 + len);
}

esp_err_t sx1262_write_payload(const uint8_t *data, uint8_t len)
{
    if (len > 255) len = 255;
    uint8_t frame[2 + 255] = {0};
    frame[0] = SX1262_OPCODE_WRITE_BUFFER;
    frame[1] = 0x00;
    memcpy(&frame[2], data, len);
    return sx1262_hal_transfer(frame, NULL, 2 + len);
}

esp_err_t sx1262_tx(uint32_t timeout_ms)
{
    sx1262_hal_set_tx_mode();
    uint32_t ticks = (timeout_ms * 64) / 1000;
    if (ticks > 0xFFFFFF) ticks = 0xFFFFFF;
    uint8_t cmd[4] = { SX1262_OPCODE_SET_TX, (uint8_t)(ticks >> 16), (uint8_t)(ticks >> 8), (uint8_t)ticks };
    return sx1262_hal_transfer(cmd, NULL, sizeof(cmd));
}

esp_err_t sx1262_enter_rx(uint32_t timeout_ms)
{
    sx1262_hal_set_rx_mode();
    uint32_t ticks = (timeout_ms * 64) / 1000;
    if (ticks == 0) ticks = 0xFFFFFF; // no timeout
    if (ticks > 0xFFFFFF) ticks = 0xFFFFFF;
    uint8_t cmd[4] = { SX1262_OPCODE_SET_RX, (uint8_t)(ticks >> 16), (uint8_t)(ticks >> 8), (uint8_t)ticks };
    return sx1262_hal_transfer(cmd, NULL, sizeof(cmd));
}

esp_err_t sx1262_read_payload(uint8_t *data, uint8_t *len)
{
    uint8_t status[3] = { SX1262_OPCODE_GET_RX_BUFFER_STATUS, 0x00, 0x00 };
    uint8_t resp[3] = {0};
    ESP_ERROR_CHECK(sx1262_hal_transfer(status, resp, sizeof(status)));
    uint8_t pay_len = resp[1];
    uint8_t start = resp[2];
    if (pay_len == 0) { *len = 0; return ESP_OK; }
    uint8_t tx[3 + 255] = {0};
    uint8_t rx[3 + 255] = {0};
    tx[0] = SX1262_OPCODE_READ_BUFFER;
    tx[1] = start;
    tx[2] = 0x00;
    ESP_ERROR_CHECK(sx1262_hal_transfer(tx, rx, 3 + pay_len));
    memcpy(data, &rx[3], pay_len);
    *len = pay_len;
    return ESP_OK;
}

uint16_t sx1262_get_irq(void)
{
    // Send opcode + 3 dummies, read back 4 bytes (status + IRQ[15:8] + IRQ[7:0])
    uint8_t tx[4] = { SX1262_OPCODE_GET_IRQ_STATUS, 0x00, 0x00, 0x00 };
    uint8_t rx[4] = {0};
    sx1262_hal_transfer(tx, rx, sizeof(tx));
    return ((uint16_t)rx[2] << 8) | rx[3];
}

void sx1262_clear_irq(uint16_t mask)
{
    uint8_t cmd[3] = { SX1262_OPCODE_CLR_IRQ_STATUS, (uint8_t)(mask >> 8), (uint8_t)mask };
    sx1262_hal_transfer(cmd, NULL, sizeof(cmd));
}



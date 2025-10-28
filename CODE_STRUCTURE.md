# Code Structure - ESP32 LoRa Communication

## Overview

This project implements **simple point-to-point LoRa communication** between two ESP32 nodes using the Waveshare Core1262-868M module.

## File Structure

```
.
├── main/
│   ├── main.c              # Application code (pin config, init, TX/RX loop)
│   ├── sx1262_hal.h/c      # Hardware layer (SPI, GPIO, BUSY pin)
│   ├── sx1262_driver.h/c   # LoRa driver (set frequency, SF, BW, etc.)
│   └── CMakeLists.txt      # Build configuration
├── CMakeLists.txt
└── README.md
```

## How It Works

### 1. Hardware Layer (`sx1262_hal.c/h`)
**What it does:**
- Initializes SPI communication with SX1262
- Configures GPIO pins (CS, MOSI, MISO, SCK, BUSY, RST, RXEN, TXEN, DIO1, DIO2)
- **Critical:** Waits for BUSY pin to be LOW before any SPI operation
- Controls RF switch via RXEN/TXEN pins

**Key functions:**
- `sx1262_hal_init()` - Initialize hardware
- `sx1262_hal_wait_busy()` - Wait for module to be ready
- `sx1262_hal_reset()` - Reset the module
- `sx1262_hal_set_tx_mode()` / `sx1262_hal_set_rx_mode()` - Switch RF mode

### 2. Driver Layer (`sx1262_driver.c/h`)
**What it does:**
- Configures LoRa parameters (frequency, spreading factor, bandwidth, coding rate)
- Handles TX/RX operations
- Manages interrupt status

**Key functions:**
- `sx1262_set_rf_frequency()` - Set frequency (e.g., 868.1 MHz)
- `sx1262_set_lora_params()` - Set SF, BW, CR
- `sx1262_set_tx()` / `sx1262_set_rx()` - Enter TX/RX mode
- `sx1262_write_buffer()` / `sx1262_read_buffer()` - Data handling
- `sx1262_get_irq_status()` - Check interrupts (TX done, RX done)

### 3. Application Layer (`main.c`)
**What it does:**
- Configures pin numbers
- Sets LoRa parameters
- Initializes hardware and driver
- Runs TX/RX loop

**Key functions:**
- `lora_init()` - Initialize LoRa module
- `lora_send()` - Send data
- `lora_receive()` - Receive data
- `app_main()` - Main application loop

## Pin Configuration (Waveshare Core1262)

**Required Pins:**
- SPI: CS (GPIO5), MOSI (GPIO23), MISO (GPIO19), SCK (GPIO18)
- Control: BUSY (GPIO4), RST (GPIO2), RXEN (GPIO26), TXEN (GPIO25)
- Optional: DIO1 (GPIO32), DIO2 (GPIO33)
- **Note:** DIO3 is NOT present on this board (set to GPIO_NUM_NC)

## How TX Works

1. Write data to buffer: `sx1262_write_buffer(data, len)`
2. Set TX mode: `sx1262_set_tx(timeout)` → This sets TXEN HIGH, RXEN LOW
3. Wait for completion: Check IRQ status for TX_DONE flag
4. Clear interrupt: `sx1262_clear_irq_status()`

## How RX Works

1. Set RX mode: `sx1262_set_rx(timeout)` → This sets RXEN HIGH, TXEN LOW
2. Wait for data: Check IRQ status for RX_DONE flag
3. Read buffer: `sx1262_read_buffer()` to get data
4. Clear interrupt: `sx1262_clear_irq_status()`

## Important Notes

1. **BUSY Pin is Critical:** ALWAYS check BUSY pin is LOW before SPI operations
2. **RF Switch Control:** RXEN/TXEN pins control the internal RF switch
3. **Same Parameters:** Both units MUST use identical parameters (frequency, SF, BW, CR)
4. **Antenna Required:** Both units need properly connected antennas
5. **SPI Speed:** Maximum 18 MHz (configured as 8 MHz for safety)

## Code Organization

- **Separation of Concerns:** HAL (hardware) → Driver (LoRa) → App (logic)
- **Clean Functions:** Each function does one thing
- **Good Comments:** Explains what and why, not how
- **No Unused Code:** Removed LoRaWAN complexity
- **Readable:** Clear function names and variable names

## Usage Example

```c
// 1. Configure pins
sx1262_hal_config_t hal_config = {
    .spi_cs = 5,
    // ... other pins
};

// 2. Initialize
sx1262_hal_init(&hal_config);
lora_init(&lora_config);

// 3. Send data
lora_send(data, length);

// 4. Receive data
lora_receive(buffer, &length);
```

## Building

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```


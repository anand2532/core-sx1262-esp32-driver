# ESP32 SX1262 LoRa Communication

This project integrates the Waveshare Core1262-868M LoRa module with ESP32 using the ESP-IDF framework for **simple point-to-point LoRa communication**.

## Overview

The Core1262-868M is a LoRa core module based on the Semtech SX1262 chip. This project provides:

- **Hardware Abstraction Layer (HAL)**: Low-level SPI and GPIO control
- **SX1262 Driver**: Register operations and BUSY pin handling  
- **Simple LoRa Communication**: Basic TX/RX functionality for point-to-point communication
- **Modular Design**: Clean, easy-to-understand code structure

> **Note**: This is a **simple LoRa implementation** for direct communication between two units, **not LoRaWAN**. No gateways or network infrastructure needed!

## Project Structure

```
.
├── CMakeLists.txt
├── README.md
└── main/
    ├── CMakeLists.txt
    ├── main.c              # Main application with simple LoRa functions
    ├── sx1262_hal.h/c      # Hardware abstraction layer
    └── sx1262_driver.h/c   # SX1262 low-level driver
```

## Hardware Connections

### Pin Configuration

Connect the Core1262-868M module to ESP32 as follows:

| Core1262-868M Pin | ESP32 GPIO | Function | Description |
|-------------------|------------|----------|-------------|
| MOSI | GPIO23 | SPI MOSI | Master Out, Slave In |
| MISO | GPIO19 | SPI MISO | Master In, Slave Out |
| CLK | GPIO18 | SPI CLK | SPI Clock |
| CS | GPIO5 | SPI CS | Chip Select |
| BUSY | GPIO4 | Input | Busy status signal |
| RST | GPIO2 | Output | Reset signal |
| RXEN | GPIO26 | Output | RX Enable |
| TXEN | GPIO25 | Output | TX Enable |
| DIO1 | GPIO32 | Input | Interrupt pin |
| DIO2 | GPIO33 | Input | Control pin |
| DIO3 | GPIO35 | Input | TCXO control |

### Default Pin Configuration (configurable in `main.c`)

```c
#define SX1262_SPI_CS    5   // GPIO5
#define SX1262_SPI_MOSI 23  // GPIO23
#define SX1262_SPI_MISO 19  // GPIO19
#define SX1262_SPI_SCK  18  // GPIO18
#define SX1262_BUSY     4   // GPIO4
#define SX1262_RST      2   // GPIO2
#define SX1262_RXEN     26  // GPIO26
#define SX1262_TXEN     25  // GPIO25
#define SX1262_DIO1     32  // GPIO32
#define SX1262_DIO2     33  // GPIO33
#define SX1262_DIO3     35  // GPIO35
```

### Pin Descriptions

- **SPI Interface**: Standard SPI communication (MOSI, MISO, CLK, CS)
- **BUSY**: Indicates when the chip is busy. **MUST** be low before any SPI operation
- **RST**: Active low reset pin. Keep high during operation
- **RXEN/TXEN**: Control pins for RF switch. Used to switch between transmit/receive modes
- **DIO1**: Typically used for interrupts (TX done, RX done, etc.)
- **DIO2**: Often used for RX enable control
- **DIO3**: Used for TCXO power control

## Build and Flash

### Prerequisites

- ESP-IDF v4.4 or higher
- Python 3.6 or higher

### Build Steps

```bash
# Set up ESP-IDF environment
. $HOME/esp/esp-idf/export.sh

# Navigate to project directory
cd /path/to/sx1262_esp32

# Build the project
idf.py build

# Flash to device
idf.py -p /dev/ttyUSB0 flash

# Monitor output
idf.py -p /dev/ttyUSB0 monitor
```

## Configuration

### LoRa Parameters

Edit `main.c` to configure your LoRa parameters:

```c
static lora_config_t lora_config = {
    .frequency = 868100000,           // 868.1 MHz (EU868)
    .spreading_factor = 7,           // SF7
    .bandwidth = SX1262_LORA_BW_125, // 125 kHz
    .coding_rate = SX1262_LORA_CR_4_5,
    .tx_power = 14                    // 14 dBm
};
```

### Frequency Bands

- **EU868**: 868.1 MHz, 868.3 MHz, 868.5 MHz
- **US915**: 902 - 928 MHz
- **AS433**: 433 MHz

### Spreading Factors

- SF5 - SF12 (default: SF7)
- Higher SF = Longer range, slower data rate, higher sensitivity

### Bandwidth Options

- 7.8 kHz - Lowest bandwidth, best sensitivity
- 10.4 kHz
- 15.6 kHz
- 20.8 kHz
- 31.25 kHz
- 41.7 kHz
- 62.5 kHz
- **125 kHz** (recommended, default)
- 250 kHz
- 500 kHz - Highest bandwidth, faster data rate

### Coding Rate

- **CR_4_5**: 5/4 coding rate (faster, less robust)
- **CR_4_6**: 6/4 coding rate
- **CR_4_7**: 7/4 coding rate  
- **CR_4_8**: 8/4 coding rate (slower, more robust)

### TX Power

- Range: 0-22 dBm (default: 14 dBm)
- Higher power = Longer range, more power consumption

## Usage

The application implements a simple TX/RX loop:

```c
// Initialize LoRa
lora_init(&lora_config);

// Send data
uint8_t data[] = "Hello LoRa!";
lora_send(data, strlen(data));

// Receive data
uint8_t rx_buffer[255];
uint8_t rx_len;
lora_receive(rx_buffer, &rx_len);
```

### Main Functions

**`lora_init(config)`**: Initialize the LoRa module with specified parameters
**`lora_send(data, len)`**: Send data over LoRa
**`lora_receive(data, len)`**: Receive data from LoRa

## How It Works

1. **Initialization**: Module is reset and configured with LoRa parameters
2. **TX Mode**: Data is written to buffer, module is set to TX mode
3. **RX Mode**: Module listens for incoming LoRa packets
4. **IRQ Handling**: TX/RX completion is detected via interrupt status

## Important Notes

1. **BUSY Pin**: Always check BUSY pin is LOW before any SPI operation
2. **RF Mode**: Switch between TX/RX modes using RXEN/TXEN pins
3. **SPI Speed**: Maximum SPI clock is 18 MHz (configured as 8 MHz)
4. **Timing**: Proper delays are crucial for correct operation
5. **Antenna**: Ensure antenna is properly connected
6. **Same Settings**: Both units must use the same frequency, spreading factor, bandwidth, and coding rate

## Troubleshooting

### Common Issues

1. **SPI Communication Failed**
   - Check BUSY pin is LOW
   - Verify SPI pins are correctly connected
   - Check SPI clock speed (max 18 MHz)

2. **Module Not Responding**
   - Verify RST pin is held HIGH
   - Check power supply (3.3V)
   - Reset the module

3. **No Communication Between Units**
   - Ensure both units use **identical** parameters (frequency, SF, BW, CR)
   - Check antenna is connected on both units
   - Verify units are within range
   - Try increasing TX power
   - Try lower spreading factor (SF7 instead of SF12)

4. **TX/RX Not Working**
   - Check RXEN/TXEN pin configuration
   - Verify antenna is connected
   - Check RF frequency settings
   - Ensure radio parameters match on both devices

### Parameter Selection Guide

**For Maximum Range:**
- Spreading Factor: 12 (SF12)
- Bandwidth: 125 kHz
- Coding Rate: 4/8
- TX Power: 22 dBm

**For Higher Data Rate:**
- Spreading Factor: 7 (SF7)
- Bandwidth: 500 kHz
- Coding Rate: 4/5
- TX Power: 14 dBm

**For Balanced Performance:**
- Spreading Factor: 9 (SF9)
- Bandwidth: 125 kHz
- Coding Rate: 4/7
- TX Power: 17 dBm

## Example: Two ESP32 Units Communicating

**Unit 1 (Transmitter):**
- Flash the same firmware
- Both units will alternate between TX and RX

**Unit 2 (Receiver):**
- Flash the same firmware
- Both units will alternate between TX and RX

Both units can send and receive messages to each other!

## Resources

- [SX1262 Datasheet](https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1262)
- [LoRa Modulation Basics](https://www.semtech.com/lora/what-is-lora)
- [Waveshare Core1262-868M Wiki](https://www.waveshare.com/wiki/Core1262-868M)

## License

This project is provided as-is for educational purposes.

## Contact

For issues and questions, please refer to the ESP-IDF documentation and SX1262 datasheet.

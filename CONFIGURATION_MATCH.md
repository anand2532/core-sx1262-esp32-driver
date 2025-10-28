# Configuration Matching with UART Module

## Overview
This document explains how to match the SPI-based SX1262 configuration with a UART-based module.

## UART Module Configuration (Your Other Module)
```python
cfg_reg[3] = high_addr           # High byte of node address
cfg_reg[4] = low_addr            # Low byte of node address  
cfg_reg[5] = net_id_temp         # Network ID (must match both modules)
cfg_reg[6] = SX126X_UART_BAUDRATE_115200 + air_speed_temp  # UART speed + air data rate
cfg_reg[7] = buffer_size_temp + power_temp + 0x20  # Buffer, power, RSSI noise
cfg_reg[8] = freq_temp           # Frequency setting
cfg_reg[9] = 0x43 + rssi_temp    # RSSI output configuration
cfg_reg[10] = h_crypt            # High byte of encryption key
cfg_reg[11] = l_crypt            # Low byte of encryption key

# airspeed = 2400 bps
```

## Corresponding SPI Configuration

### For Direct SX1262 (SPI Mode):
```c
// Matched Configuration
static lora_config_t lora_config = {
    .frequency = 433000000,      // Match freq_temp from other module
    .spreading_factor = 10,     // SF10 for ultra-long range
    .bandwidth = SX1262_LORA_BW_7_8,  // 7.8 kHz - ultra low rate
    .coding_rate = SX1262_LORA_CR_4_8, // Most robust
    .tx_power = 20               // 20 dBm (0x20 = power_temp)
};
```

### Key Parameters to Match:

| UART Config | SPI Equivalent | Value |
|-------------|---------------|-------|
| `cfg_reg[5]` (net_id) | Network ID | Must match both modules |
| `cfg_reg[3:4]` (address) | Node address | Unique per module |
| `cfg_reg[6]` (air_speed) | Bandwidth + Spreading Factor | SF10 + 7.8kHz |
| `cfg_reg[7]` (power) | TX Power | 20 dBm |
| `cfg_reg[8]` (freq) | Frequency | Same on both modules |
| `cfg_reg[10:11]` (crypt) | Encryption | (If using encryption) |

## Configuration Values

### Air Speed: 2400 bps
This corresponds to:
- **Spreading Factor**: 10 (SF10)
- **Bandwidth**: 7.8 kHz  
- **Data Rate**: ~2400 bps
- **Range**: Maximum possible with SX1262

### Frequency Setting
Configure `freq_temp` to match between both modules. Common values:
- 433 MHz: `0x6C`
- 868 MHz: `0xD4`
- 915 MHz: `0xE5`

### TX Power: 20 dBm
- Maximum power setting
- Requires proper power supply (3.3V @ 45mA)
- Best range but higher current consumption

## Matching Checklist

✅ Both modules must have:
- **Same network ID** (cfg_reg[5])
- **Same frequency** (cfg_reg[8])
- **Same air speed settings** (bandwidth + SF)
- **Same coding rate** (for compatibility)
- **Same encryption key** (if using encryption)

## Network Setup Example

### Module A (UART-based):
```
Network ID: 0
Node Address: 0x0001
Frequency: 433 MHz
Air Speed: 2400 bps
```

### Module B (SPI-based - this code):
```c
#define NETWORK_ID 0
#define NODE_ADDRESS 2  
#define AIR_SPEED 2400
lora_config.frequency = 433000000;
```

## Testing Communication

1. **Power both modules**
2. **Set same frequency on both**
3. **Module A (UART)**: Configure with network settings
4. **Module B (SPI)**: Flash with matched settings
5. **Test TX**: Module B sends → Module A receives
6. **Test RX**: Module A sends → Module B receives

## Expected Data Rate
With 2400 bps air speed:
- **Effective data rate**: ~300 bytes/sec
- **Per packet overhead**: ~20 bytes
- **Maximum packet size**: ~250 bytes
- **Transmission time**: ~100-200ms per packet

## Troubleshooting

### Modules can't communicate:
1. Verify **same frequency** on both
2. Check **same network ID**
3. Ensure **same spreading factor** and bandwidth
4. Verify **TX power** is sufficient
5. Check **antennas** are connected
6. Ensure **range** is within limits

### Range is too short:
1. Increase TX power (max 20 dBm)
2. Use higher spreading factor (SF10)
3. Lower bandwidth (7.8 kHz)
4. Improve antenna connections
5. Check for interference

### Packet loss:
1. Reduce interference sources
2. Increase spreading factor
3. Lower data rate further
4. Check power supply stability
5. Verify antenna quality

## Current Configuration

This SPI module is now configured for:
- **Maximum range**: SF10 + 7.8kHz + CR 4/8
- **Low data rate**: ~2400 bps effective
- **High power**: 20 dBm TX
- **Maximum robustness**: Coding rate 4/8

This should match your UART module with airspeed 2400!


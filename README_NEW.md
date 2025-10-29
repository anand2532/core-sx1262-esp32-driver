# ESP32 SX1262 Clean Implementation

## What Changed

### Key Finding
Your GPIO25/GPIO26 (TXEN/RXEN) pins are not working. This is either:
1. Wiring issue - pins not connected to module
2. Waveshare Core1262 controls RF switch internally via DIO2
3. Hardware defect

### Solution
New code **removes TXEN/RXEN control** and uses **DIO2** for RF switch control instead.

## Pin Configuration

```c
SPI_CS:    GPIO5
SPI_MOSI:  GPIO23
SPI_MISO:  GPIO19
SPI_SCK:   GPIO18
BUSY:      GPIO4    (MUST work - signals module busy)
RST:       GPIO2    (Hardware reset)
DIO1:      GPIO32   (Interrupt)
DIO2:      GPIO33   (RF Switch Control - now automatic)
```

**TXEN and RXEN are NOT USED** - module handles RF switching internally.

## What This Code Does

1. Initializes SX1262 module
2. Configures as LoRa mode
3. Sets frequency: 868.1 MHz
4. Sets parameters: SF10, BW7.8kHz, CR4/8
5. Enters RX mode continuously
6. Waits for data from UART module

## Building & Flashing

```bash
cd /home/anand/personal/sx1262_esp32
. $HOME/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Expected Output

```
I MAIN: === SX1262 RX Test Started ===
I MAIN: Resetting module...
I MAIN: BUSY pin after reset: 0
I MAIN: Initializing SX1262...
I MAIN: === Configuration Complete ===
I MAIN: Frequency: 868.1 MHz
I MAIN: SF: 10, BW: 7.8 kHz, CR: 4/8
I MAIN: Starting RX mode...
I MAIN: Listening for data...
```

## What to Test

1. Flash this code to ESP32
2. Module should enter RX mode
3. Send data from UART module
4. Check if ESP32 receives data

## If Still Not Working

Check these hardware issues:

1. **BUSY pin (GPIO4)**: Must be LOW after reset
   - If HIGH, module is not responding
   - Check power supply (needs stable 3.3V)
   - Check ground connection

2. **SPI communication**: Chip should respond
   - If chip mode doesn't change, SPI wiring issue
   - Check MOSI/MISO/SCK connections

3. **Module power**: SX1262 needs adequate power
   - Check 3.3V supply
   - Add decoupling capacitors

## Next Steps

Once RX mode works:
1. Module receives data successfully
2. Then implement TX mode (with DIO2 RF control only)
3. Add full duplex communication

## Important Notes

- This code is RX-only for testing
- No TXEN/RXEN control (removed)
- RF switch controlled by DIO2 (chip-internal)
- Simple and clean implementation


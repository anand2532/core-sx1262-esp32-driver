# CoreSX1262 Schematic Analysis

Based on the schematic file found in the project.

## Key Findings

From the schematic extraction, the module has:

### SX1262 Chip Connections (U3)
- NSS (SPI Chip Select)
- SCK (SPI Clock)
- MOSI (SPI Data Out)
- MISO (SPI Data In)
- NRESET (Reset)
- BUSY (Busy signal)
- DIO1, DIO2, DIO3 (Digital IO pins)

### RF Switch Circuit
- There is an RF switch controlled by **RX_EN** and **TX_EN** signals
- These appear to be generated from the DIO pins or controlled separately

### Connector Pinout
From the schematic showing connector pins:
```
Pin 1-12: Various signals
Pin 13: DIO1
Pin 14: BUSY
Pin 15: RST
Pin 16: MISO
Pin 17: MOSI
Pin 18: CLK
Pin 19: CS
```

**IMPORTANT**: The schematic shows RX_EN and TX_EN signals exist on the board, but they may be:
1. **Internal to the board** (automatically controlled by DIO2)
2. **Not on the connector** (only available for internal RF switch)
3. **Require jumpers** (need to be connected)

## What This Means

### Most Likely Scenario
The Waveshare Core1262 board has **TXEN/RXEN controlled internally by DIO2** and these signals are NOT brought out to the connector. The RF switch on the board is controlled by the SX1262 chip itself, not by external MCU GPIO.

### Why Your GPIO Didn't Work
- GPIO25/GPIO26 → TXEN/RXEN wires are probably **not connected on your board**
- These signals may only exist inside the module's circuitry
- They're controlled by the SX1262 via DIO2

### The Solution
**DIO2 controlled RF switch** (which we already implemented) is the **correct approach** for this module.

## Verification Needed

Please check your physical module:

1. **Look for silkscreen labels** on the connector:
   - Does it show TXEN/RXEN on any pins?
   - Or only SPI, RST, BUSY, DIO1, etc.?

2. **Check with multimeter**:
   - Are pins 25 and 26 on your ESP32 actually connected to anything?
   - Measure resistance between GPIO25 and module connector

3. **Check board variant**:
   - Does your board have "CoreSX1262" or "Core1262" label?
   - Some variants have different pinouts

## Recommended Pin Mapping (Based on Schematic)

```c
// SPI Interface
CS:   GPIO5
CLK:  GPIO18
MOSI: GPIO23
MISO: GPIO19

// Control Pins
RST:  GPIO2
BUSY: GPIO4
DIO1: GPIO32
DIO2: GPIO33  ← Used for RF switch control (automatic)
DIO3: Not used (or NC)

// RF Control
TXEN: NOT CONNECTED (chip controls via DIO2)
RXEN: NOT CONNECTED (chip controls via DIO2)
```

## Conclusion

Your current implementation using **DIO2 for RF switch control** is likely **CORRECT**. The TXEN/RXEN pins don't need external control - the chip handles it automatically.

**Next Step**: Test the RX mode code we just created - it should work because it uses the internal DIO2 control approach.


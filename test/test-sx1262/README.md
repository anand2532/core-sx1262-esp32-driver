# SX1262 ESP-IDF example (Waveshare Core1262)

Pin connections (ESP32 DevKit → Core1262):
- SPI SCK (GPIO18) → CLK
- SPI MOSI (GPIO23) → MOSI
- SPI MISO (GPIO19) → MISO
- SPI CS (GPIO5) → NSS
- BUSY (GPIO4) → BUSY
- RESET (GPIO27) → NRESET
- DIO1 (GPIO26) → DIO1
- ANT → Antenna connector (SMA)
- 3V3 → 3V3, GND → GND

Notes
- RF switch is controlled internally via DIO2 (SetDIO2AsRfSwitchCtrl). RXEN/TXEN external pins not required.
- Default LoRa region is EU868 (868.1 MHz). Change in `main/main.c` if needed.

Build
```
idf.py build
```

Run
- Default is receiver (APP_TX_MODE=0). To build transmitter:
```
idf.py -DAPP_TX_MODE=1 build
```

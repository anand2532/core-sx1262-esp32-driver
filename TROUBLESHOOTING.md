# SX1262 Troubleshooting Guide

## Current Issue: BUSY Pin Stuck HIGH

### Symptoms
- BUSY pin timeout errors continuously
- Module not responding to commands
- TX/RX not working

### Likely Causes

#### 1. **Hardware Wiring Issues**
**Check:**
- Is BUSY pin correctly connected to GPIO4?
- Is VDD (3.3V) connected and stable?
- Is GND properly connected?
- Are all SPI pins correctly wired?

**Expected pinout for Waveshare Core1262-868M:**
```
Core1262    ESP32      Function
MOSI    →  GPIO23    SPI MOSI
MISO    →  GPIO19    SPI MISO
CLK     →  GPIO18    SPI Clock
CS      →  GPIO5      SPI Chip Select
BUSY    →  GPIO4     Busy signal (IMPORTANT!)
RST     →  GPIO2     Reset
RXEN    →  GPIO26    RX Enable
TXEN    →  GPIO25    TX Enable
DIO1    →  GPIO32    Interrupt
DIO2    →  GPIO33    Control
```

#### 2. **Power Issues**
- Module requires 3.3V power supply
- Check if VDD is stable (measure with multimeter)
- Ensure sufficient current capacity (module draws ~45mA when transmitting)

#### 3. **Module Not Responding**
Possible reasons:
- Module damaged or defective
- Module still in sleep mode
- Missing proper reset sequence

### Diagnostic Commands to Test

#### Test 1: Check BUSY Pin After Reset
The code now logs the BUSY pin status after reset. 
Expected: BUSY should be LOW after reset
If HIGH: Module is not responding or not powered

#### Test 2: Measure BUSY Pin With Multimeter
1. Connect multimeter to BUSY pin (GPIO4)
2. Reset the module
3. Should read close to 0V (LOW) or 3.3V (HIGH)
4. If stuck at 3.3V constantly: Module not responding or pin issue

#### Test 3: Check SPI Communication
Add this test to check basic SPI:
```c
// Test SPI communication
uint8_t test_cmd = 0xC0; // GET_STATUS
uint8_t result[3];
sx1262_hal_write(&test_cmd, 1);
sx1262_hal_read(result, 3);
ESP_LOGI(TAG, "Status result: 0x%02X 0x%02X 0x%02X", result[0], result[1], result[2]);
```

### Solutions to Try

#### Solution 1: Verify Hardware Connections
Double-check all wiring, especially:
- BUSY pin connection
- Power connections (VDD, GND)
- SPI pin connections

#### Solution 2: Try Different Reset Sequence
Some modules need a longer reset pulse or multiple resets:
```c
void sx1262_hal_reset(void)
{
    gpio_set_level(gpio_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(10));  // Longer reset
    gpio_set_level(gpio_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(150)); // Longer delay
}
```

#### Solution 3: Check if BUSY Pin is Actually Working
Test the GPIO pin itself:
```c
// Test if GPIO4 can read properly
bool busy_test = gpio_get_level(SX1262_BUSY);
ESP_LOGI(TAG, "BUSY pin reads: %d", busy_test);
```

#### Solution 4: Try Slower SPI Speed
If timing issues:
```c
// In sx1262_hal_init, change SPI speed
.clock_speed_hz = 1 * 1000 * 1000,  // 1MHz instead of 8MHz
```

#### Solution 5: Check if Module is Actually Present
Use a second ESP32 or logic analyzer to monitor the SPI bus and see if:
- Chip Select is being asserted
- MOSI/MISO lines are active
- Clock is running

### Expected Normal Operation

After successful initialization, you should see:
```
I (xx) MAIN: BUSY pin after reset: LOW  ← This should be LOW
I (xx) MAIN: Initializing LoRa...
I (xx) MAIN: LoRa initialized
I (xx) MAIN: [TX #1] Sending: Hello LoRa!
I (xx) MAIN: ✓ TX successful
```

### Current Debugging Status

The enhanced logging will show:
1. BUSY pin state after reset
2. BUSY pin state before each write
3. Better timeout messages with current state

### Next Steps

1. **Flash the new firmware** and check the BUSY pin status after reset
2. **Report the BUSY pin status** you see in the logs
3. **Check hardware connections** with a multimeter
4. **Try the diagnostic tests** above

### If BUSY Pin is Always HIGH

This indicates one of:
- Module not powered
- Module defective
- BUSY pin not connected properly
- Module in error state that won't clear

In this case, the module might need a hardware replacement or the issue is in the PCB/cabling.


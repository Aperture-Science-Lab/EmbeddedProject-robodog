# 🔧 UART Fix - Quick Reference Card

## What Changed?

**UART Port**: **UART0** on GP16/GP17 (these pins are dedicated to UART0)

## New Wiring (UART0 on GP16/GP17)

### Pico W → Nano RP2040 Connect

| Pico Pin | GPIO | Function | → | Nano Pin | GPIO |
|----------|------|----------|---|----------|------|
| Pin 22 | **GP16** | UART0_TX | → | Pin 17 | RX (GPIO1) |
| Pin 23 | **GP17** | UART0_RX | ← | Pin 16 | TX (GPIO0) |
| GND | GND | Ground | ⟷ | Pin 14 | GND |
| VSYS/5V | 5V | Power | → | Pin 15 | VIN |

## Files Changed

✅ `sensor_hub.h` - UART0 config  
✅ `sensor_hub.cpp` - Improved initialization  
✅ `nano_imu.ino` - Updated docs  
✅ `main_controller.cpp` - Updated help text  
✅ `docs/WIRING_DIAGRAM.md` - All pin references  
✅ `docs/UART_TROUBLESHOOTING.md` - Complete guide (NEW)

## Quick Test

1. **Upload Nano code** (nano_imu.ino)
2. **Rebuild Pico code** 
   ```bash
   cd spotmicro-rp2040/build
   cmake ..
   make main_controller
   ```
3. **Flash Pico** with main_controller.uf2
4. **Check serial output** - should see "Nano RP2040 Connect: CONNECTED"
5. **Test command**: Type `SENSOR` in serial console

## Expected Output

```
Nano RP2040 Connect: ONLINE
IMU Status: OK
Roll: 0.00°  Pitch: 0.00°  Yaw: 0.00°
```

## If It Doesn't Work

1. Check power (5V on Nano Pin 15)
2. Verify wiring (TX → RX, RX → TX)
3. Common ground connected
4. Both at 115200 baud
5. See `docs/UART_TROUBLESHOOTING.md` for details

---
**⚡ Remember**: TX goes to RX, RX goes to TX (crossover)

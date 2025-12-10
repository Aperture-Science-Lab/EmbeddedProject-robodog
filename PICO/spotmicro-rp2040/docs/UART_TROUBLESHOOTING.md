# UART Connection Troubleshooting Guide
## Nano RP2040 Connect ↔ Pico W Communication

## Problem Fixed
The UART connection between the IMU on Nano and the PICO was not working due to pin conflicts.

---

## Changes Made

### 1. **UART Port Changed: UART0 → UART1**
   - **Problem**: UART0 is typically used by the Pico SDK for USB serial communication
   - **Solution**: Moved to UART1 to avoid conflicts
   - **Impact**: Requires rewiring the UART connections

### 2. **New Pin Assignments**

#### PICO W Side:
| Change | Description |
|--------|-------------|
| **UART Port** | Using **UART0** on GP16/GP17 (these pins can only be UART0) |
| **TX Pin** | GP16 (UART0_TX) → Nano GPIO1/RX (Pin 17) |
| **RX Pin** | GP17 (UART0_RX) ← Nano GPIO0/TX (Pin 16) |

#### Nano RP2040 Connect Side:
| Pin | Function | Connection |
|-----|----------|------------|
| Pin 16 (GPIO0, TX) | Transmit | → Pico GP17 (UART1_RX) |
| Pin 17 (GPIO1, RX) | Receive | ← Pico GP16 (UART1_TX) |

#### Nano RP2040 Connect Side:
- **Pin 16 (GPIO0, TX)** → Pico GP9 (UART1_RX)
- **Pin 17 (GPIO1, RX)** ← Pico GP8 (UART1_TX)
- **Pin 15 (VIN)** ← 5V from LM2596S
- **Pin 14 (GND)** ← Common Ground

### 3. **UART Configuration Improvements**
   - Hardware flow control **explicitly disabled**
   - UART format set to **8N1** (8 data bits, no parity, 1 stop bit)
   - Baud rate: **115200**
   - FIFO enabled for better performance
   - Line ending translation disabled for raw data

---

## Wiring Instructions

### Step 1: Disconnect Power
- Turn off the robot completely
- Disconnect battery or power supply

### Step 2: Rewire UART Connection

**From Pico W:**
1. **GP16 (Pin 22)** → White wire → Nano Pin 17 (GPIO1/RX)
2. **GP17 (Pin 23)** → Green wire → Nano Pin 16 (GPIO0/TX)

**From Nano:**
1. **Pin 16 (GPIO0/TX)** → Green wire → Pico GP17
2. **Pin 17 (GPIO1/RX)** → White wire → Pico GP16
3. **Pin 15 (VIN)** → Red wire → 5V from LM2596S
4. **Pin 14 (GND)** → Black wire → Common Ground

---

## Testing Procedure

### 1. Upload Nano Code
```bash
# In Arduino IDE:
# - Open: nano_imu/nano_imu.ino
# - Board: "Arduino Nano RP2040 Connect"
# - Port: Select your Nano's COM port
# - Upload the sketch
```

### 2. Monitor Nano Serial Output
```
Tools → Serial Monitor → 115200 baud

Expected output:
  SpotMicro Smart IMU Sensor Hub
  Nano RP2040 Connect
  ==============================
  Initializing LSM6DSOX IMU... OK!
  Calibrating IMU... Keep robot still!
  Calibration complete!
  Ready! Waiting for commands...
```

### 3. Build and Flash Pico Code
```bash
cd spotmicro-rp2040
mkdir -p build
cd build
cmake ..
make main_controller
```

Copy `main_controller.uf2` to your Pico W in bootloader mode.

### 4. Check Pico Serial Output
```
Expected during initialization:
  Initializing Sensor Hub...
    UART0: GP16 (TX), GP17 (RX) @ 115200 baud
    Actual baud rate: 115200
    Ultrasonic 1 (Left): TRIG=GP6, ECHO=GP7
    Ultrasonic 2 (Right): TRIG=GP8, ECHO=GP9
    Checking Nano connection...
    Nano RP2040 Connect: CONNECTED
  Sensor Hub initialized.
```

### 5. Test Communication
In the Pico serial console, type:
```
SENSOR
```

Expected output:
```
╔══════════════════════════════════════════════════════╗
║               SENSOR STATUS                          ║
╠══════════════════════════════════════════════════════╣
║ Nano RP2040 Connect: ONLINE                          ║
╠══════════════════════════════════════════════════════╣
║ IMU (LSM6DSOX):                                      ║
║   Status:      OK                                    ║
║   Roll:        +0.00°                                ║
║   Pitch:       +0.00°                                ║
║   Yaw:         +0.00°                                ║
║   Accel (g):   X=+0.00 Y=+0.00 Z=+1.00               ║
║   Gyro (dps):  X=+0.0 Y=+0.0 Z=+0.0                  ║
╚══════════════════════════════════════════════════════╝
```

---

## Troubleshooting

### Issue: "Nano RP2040 Connect: NOT DETECTED"

**Check these:**

1. **Power Supply**
   - Nano Pin 15 should have 5V
   - Check with multimeter
   - Common ground must be connected

2. **Wiring**
   - Verify TX/RX are not swapped
   - Check continuity of wires
   - No loose connections

3. **Baud Rate**
   - Both should be at 115200
   - Check Nano Serial Monitor shows correct baud

4. **Nano is Running**
   - Check LED on Nano is lit
   - Connect to USB and check Serial Monitor
   - Should see IMU messages

5. **UART Pins**
   - Verify GP8 and GP9 are not used elsewhere
   - Check CMakeLists.txt if you have custom configs

### Issue: Getting Garbled Data

**Possible causes:**
1. **Baud rate mismatch** - Must be 115200 on both sides
2. **Electrical interference** - Keep wires short, away from servos
3. **Ground loop** - Ensure only ONE common ground point
4. **Bad wires** - Try different jumper wires

### Issue: Intermittent Connection

**Solutions:**
1. **Add pull-up resistors** on RX/TX lines (4.7kΩ to 3.3V)
2. **Shorter wires** - Keep UART wires under 30cm
3. **Twisted pair** - Twist TX/RX wires together
4. **Shielded cable** - Use shielded cable near motors

### Issue: Nano Code Uploads But No Serial Output

**Check:**
1. Serial Monitor is on correct port
2. Baud rate is 115200
3. Line ending set to "Newline" or "Both NL & CR"
4. Try pressing Reset button on Nano

---

## UART Communication Protocol

### Commands (Pico → Nano):
- `STATUS_REQUEST` - Request full sensor status
- `IMU_CALIBRATE` - Calibrate IMU offsets
- `IMU_RESET` - Reset sensor fusion
- `IMU_STREAM_ON` - Enable continuous streaming
- `IMU_STREAM_OFF` - Disable continuous streaming
- `PING` - Connection test

### Responses (Nano → Pico):
- `IMU,<roll>,<pitch>,<yaw>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>`
- `STATUS,<imu_ok>,<roll>,<pitch>,<yaw>,<temp>`
- `INFO,<message>`
- `ERROR,<message>`
- `PONG` - Response to PING

### Data Format:
- ASCII text, comma-separated values
- Line ending: `\n` (newline)
- Update rate: 50 Hz (when streaming enabled)

---

## Advanced Debugging

### Enable Verbose UART Debug on Pico

In `sensor_hub.cpp`, add debug prints:
```cpp
static void process_uart_line(const char* line) {
    if (strlen(line) == 0) return;
    
    // DEBUG: Print received line
    printf("[UART RX] %s\n", line);
    
    // Try to parse as different message types
    if (parse_imu_data(line)) return;
    if (parse_status_data(line)) return;
    parse_info_message(line);
}
```

### Enable Verbose Debug on Nano

The Nano already echoes all UART traffic to USB Serial for debugging.
Connect USB cable to Nano and open Serial Monitor at 115200 baud.

### Logic Analyzer

For detailed timing analysis:
1. Use a logic analyzer on TX/RX lines
2. Set to 115200 baud, 8N1, async serial
3. Check for timing issues, parity errors, framing errors

---

## Performance Metrics

### Expected Latency:
- Command → Response: < 10ms
- IMU update rate: 100 Hz internal, 50 Hz transmission
- UART transmission time: ~1ms per message

### Bandwidth Usage:
- Single IMU message: ~50 bytes
- At 50 Hz: 2500 bytes/sec
- UART capacity at 115200 baud: 11520 bytes/sec
- Utilization: ~22% (plenty of headroom)

---

## Files Modified

1. `sensor_hub.h` - Changed UART0 → UART1, updated pins
2. `sensor_hub.cpp` - Improved UART initialization
3. `nano_imu.ino` - Updated documentation
4. `docs/WIRING_DIAGRAM.md` - Updated pin assignments
5. `docs/UART_TROUBLESHOOTING.md` - This file (new)

---

## Contact & Support

If issues persist after following this guide:
1. Check all connections with multimeter
2. Try a loopback test (connect TX to RX on same board)
3. Test with minimal code (just UART echo)
4. Check for hardware damage

**Good luck! 🤖**

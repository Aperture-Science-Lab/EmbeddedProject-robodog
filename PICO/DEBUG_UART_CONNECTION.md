# UART Connection Debug Guide

## Current Status
- ✅ Nano is powered (LED on)
- ❌ Pico shows "Nano RP2040 Connect: OFFLINE"
- ✅ Ultrasonic sensors working (41.6cm, 44.9cm readings)

## Problem: UART Communication Not Working

### Step 1: Check Nano Serial Output

**Action:** Connect USB cable to Nano and open Serial Monitor

**Arduino IDE:**
1. Tools → Port → Select your Nano's COM port
2. Tools → Serial Monitor
3. Set baud rate to **115200**

**Expected Output:**
```
SpotMicro Smart IMU Sensor Hub
Nano RP2040 Connect
==============================
Initializing LSM6DSOX IMU... OK!
  Accelerometer sample rate: 104.00 Hz
  Gyroscope sample rate: 104.00 Hz
Calibrating IMU... Keep robot still!
INFO,CALIBRATING
Calibration complete!
INFO,READY

Ready! Waiting for commands...
Commands: STATUS_REQUEST, IMU_CALIBRATE, IMU_RESET, IMU_STREAM_ON/OFF
```

**If you see this:** Nano code is running correctly ✓

**If you see nothing or errors:** 
- Check if you uploaded the code to the Nano
- Verify you selected the correct board: "Arduino Nano RP2040 Connect"

---

### Step 2: Test Nano Response (USB Serial)

**While connected to Nano via USB:**

Type in Serial Monitor:
```
PING
```

**Expected Response:**
```
PONG
```

**If this works:** Nano is functioning ✓

---

### Step 3: Check UART Wiring

**Verify Physical Connections:**

| From Pico | To Nano | Wire Color | Voltage |
|-----------|---------|------------|---------|
| GP16 (Pin 22) | Pin 17 (GPIO1/RX) | White/Blue | 3.3V |
| GP17 (Pin 23) | Pin 16 (GPIO0/TX) | Green/Yellow | 3.3V |
| GND | Pin 14 (GND) | Black | 0V |
| VSYS (5V) | Pin 15 (VIN) | Red | 5V |

**Critical Check with Multimeter:**

1. **Power Check:**
   - Nano Pin 15 (VIN) should read **5V**
   - If not, Nano won't work properly

2. **Ground Check:**
   - Continuity between Pico GND and Nano Pin 14
   - Must have common ground!

3. **Signal Check (requires logic analyzer or oscilloscope):**
   - Pico GP16 should show 3.3V when idle
   - Nano Pin 16 (TX) should show 3.3V when idle

**Visual Inspection:**
- [ ] Wires are firmly connected (no loose connections)
- [ ] No wires swapped (TX → RX, RX → TX)
- [ ] No damaged wires
- [ ] No short circuits between pins

---

### Step 4: Manual UART Test from Pico

**Add debug code to sensor_hub.cpp** (temporarily):

Find `sensor_hub_init()` and after the PING test, add:

```cpp
// Test UART transmission
printf("  Sending test command...\n");
uart_puts(IMU_UART_ID, "PING\n");
sleep_ms(200);

// Check for any received data
printf("  Checking for response...\n");
while (uart_is_readable(IMU_UART_ID)) {
    char c = uart_getc(IMU_UART_ID);
    printf("    Received: 0x%02X ('%c')\n", c, c);
}
```

**Rebuild and flash Pico.**

**Expected Output:**
```
Sending test command...
Checking for response...
  Received: 0x50 ('P')
  Received: 0x4F ('O')
  Received: 0x4E ('N')
  Received: 0x47 ('G')
  Received: 0x0A ('\n')
```

**If you see nothing:** UART connection is broken

---

### Step 5: Check UART Hardware

**Verify UART1 is configured correctly:**

In your Pico code, check `sensor_hub.cpp`:
```cpp
#define IMU_UART_ID     uart1    // Should be uart1, not uart0!
#define IMU_UART_TX_PIN 16       // GP16
#define IMU_UART_RX_PIN 17       // GP17
```

**Verify pins are set correctly:**
```cpp
gpio_set_function(16, GPIO_FUNC_UART);  // GP16 = UART1_TX
gpio_set_function(17, GPIO_FUNC_UART);  // GP17 = UART1_RX
```

---

### Step 6: Loopback Test (Isolate Problem)

**Test 1: Pico UART Loopback**

Temporarily connect:
- GP16 (TX) → GP17 (RX) directly

Send "PING" from Pico - should receive it back.

**Test 2: Nano UART Loopback**

On Nano, modify code temporarily in `loop()`:
```cpp
// Echo back everything received
while (Serial1.available()) {
    char c = Serial1.read();
    Serial1.write(c);  // Echo back
    Serial.write(c);   // Show on USB
}
```

Connect TX to RX on Nano (Pin 16 to Pin 17) - should echo.

---

### Step 7: Common Issues & Solutions

#### Issue 1: Wrong UART Port
**Symptom:** No communication
**Fix:** Verify using **UART1** not UART0
- Check `#define IMU_UART_ID uart1`

#### Issue 2: Swapped TX/RX
**Symptom:** No communication
**Fix:** Remember crossover:
- Pico TX (GP16) → Nano RX (Pin 17)
- Pico RX (GP17) ← Nano TX (Pin 16)

#### Issue 3: Missing Common Ground
**Symptom:** Garbage data or no communication
**Fix:** Ensure GND connected between boards

#### Issue 4: Voltage Issue
**Symptom:** Nano not powered or unstable
**Fix:** Check 5V supply to Nano Pin 15

#### Issue 5: Baud Rate Mismatch
**Symptom:** Garbage characters
**Fix:** Both should be 115200
- Check Pico: `uart_init(uart1, 115200)`
- Check Nano: `Serial1.begin(115200)`

#### Issue 6: Wrong GPIO Function
**Symptom:** No communication
**Fix:** Verify pins assigned to UART:
```cpp
gpio_set_function(16, GPIO_FUNC_UART);
gpio_set_function(17, GPIO_FUNC_UART);
```

#### Issue 7: Electrical Interference
**Symptom:** Intermittent communication
**Fix:** 
- Keep UART wires away from servo wires
- Use twisted pair for TX/RX
- Keep wires short (<30cm)

---

### Step 8: Check for Pin Conflicts

**Verify GP16/GP17 not used elsewhere:**

Search your code for any other uses of GP16 or GP17:
```bash
grep -r "GP16\|16\)" spotmicro-rp2040/
grep -r "GP17\|17\)" spotmicro-rp2040/
```

Make sure they're ONLY used for UART1.

---

### Step 9: Enable Verbose UART Debug

**In Nano code** (`main.cpp`), add to loop:
```cpp
static unsigned long last_debug = 0;
if (millis() - last_debug > 1000) {
    last_debug = millis();
    Serial.print("Alive... Serial1 available: ");
    Serial.println(Serial1.available());
}
```

**In Pico code** (`sensor_hub.cpp`), add to `process_uart_rx()`:
```cpp
// At the start of the function
static int byte_count = 0;
if (uart_is_readable(IMU_UART_ID)) {
    printf("[UART] Bytes available: %d\n", byte_count++);
}
```

---

## Quick Checklist

- [ ] Nano shows startup message on USB Serial
- [ ] Nano responds to PING on USB Serial
- [ ] Pico code uses UART1 (not UART0)
- [ ] GP16 connected to Nano Pin 17 (RX)
- [ ] GP17 connected to Nano Pin 16 (TX)
- [ ] Common ground connected
- [ ] 5V power to Nano Pin 15
- [ ] Both set to 115200 baud
- [ ] No other code using GP16/GP17
- [ ] Wires firmly connected, no damage

---

## What to Report Back

Please check and report:

1. **Nano USB Serial output** (copy/paste what you see)
2. **Does Nano respond to PING over USB?**
3. **Multimeter readings:**
   - Nano Pin 15: ___V (should be 5V)
   - Nano Pin 16: ___V (should be ~3.3V)
   - Nano Pin 17: ___V (should be ~3.3V)
   - Pico GP16: ___V (should be ~3.3V)
   - Pico GP17: ___V (should be ~3.3V)
4. **Wire connections verified:** Yes/No
5. **Common ground connected:** Yes/No

---

## Most Likely Causes (in order)

1. **TX/RX swapped** (90% of UART issues)
2. **No common ground** 
3. **Loose wire connection**
4. **Wrong UART port in code** (UART0 vs UART1)
5. **Nano code not uploaded or crashed**

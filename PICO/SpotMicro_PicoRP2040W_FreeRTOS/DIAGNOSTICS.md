# SpotMicro Pico W - Diagnostic Checklist

## 🔴 Problem: Serial Monitor Won't Open / No Output

### Step 1: Verify Firmware is Actually Running

**What to check:**
1. After flashing SpotMicro.uf2, does the Pico W **immediately reboot**?
   - You should see COM port briefly appear, then disappear
2. Watch Device Manager → Ports (COM & LPT) while plugging in USB
   - Should see "USB Serial Device" or "Pico" appear
3. If nothing appears in Device Manager, **hardware may be dead** or USB cable is bad

**If Pico isn't detected:**
- Try different USB cable
- Try different USB port
- Try pressing RST button on Pico to reboot
- Check USB cable is **data cable** (not power-only)

---

### Step 2: Check Serial Port is Correct

**Find the COM port:**
```powershell
# List all COM ports
Get-WmiObject Win32_SerialPort | Select-Object Name, Description
```

**Expected output:**
```
Name Description
---- -----------
COM6 USB Serial Device (Pico W)
```

If you see multiple COM ports, try the one labeled "USB Serial" first.

---

### Step 3: Verify Baud Rate and Terminal Settings

**PuTTY Configuration:**
- **Speed (baud)**: `115200` ← Must be exact
- **Data bits**: `8`
- **Stop bits**: `1`
- **Parity**: `None`
- **Flow control**: `None`
- **Serial line**: `COM6` (your port)

**Do NOT use auto baud rate** - set to exactly 115200

---

### Step 4: Boot Sequence Diagnostic Output

If everything is working, you should see **exactly this sequence** over serial:

```
=========================================
=== SpotMicro FreeRTOS Starting...   ===
=========================================
[BOOT STAGE 0] Serial initialized - code is RUNNING
[BOOT STAGE 1] Creating FreeRTOS sync objects...
[OK] RTOS objects created
[BOOT STAGE 2] Initializing I2C0...
[OK] I2C0 initialized (SDA=GP4, SCL=GP5)
[BOOT STAGE 3] Scanning I2C devices...
[FOUND] LCD at 0x27
[FOUND] PCA9685 at 0x40
[BOOT STAGE 4a] Initializing LCD...
[OK] LCD initialized
[BOOT STAGE 4b] Initializing PCA9685...
[OK] PCA9685 initialized
[BOOT STAGE 5] Initializing 12 servos...
[OK] 12 servos ready
[BOOT STAGE 6] Loading settings from flash...
[OK] Settings loaded
[BOOT STAGE 7] Setting neutral pose...
[OK] Neutral pose set
[BOOT STAGE 8] Initializing kinematics...
[OK] Kinematics ready
[BOOT STAGE 9] Initializing tail motor...
[OK] Motor ready
[BOOT STAGE 10] Initializing sensor hub...
[OK] Sensors ready
[BOOT STAGE 11] Creating FreeRTOS tasks...
[OK] Tasks created
[BOOT COMPLETE] Starting FreeRTOS Scheduler...
If you see this, initialization succeeded!
===== SWITCHING TO FREERTOS TASKS =====

Blink!
Blink!
Blink!
```

---

### Step 5: Identify Where Hang Occurs

**If output STOPS at a particular stage:**

#### Stops at **Stage 0**: Serial not initialized
- **Problem**: `stdio_init_all()` is failing
- **Check**: USB cable connection, Pico firmware/bootloader
- **Solution**: Try factory reset: Hold BOOTSEL + press RST

#### Stops at **Stage 1**: FreeRTOS objects creation
- **Problem**: Insufficient RAM or FreeRTOS configuration issue
- **Check**: `FreeRTOSConfig.h` heap size (configTOTAL_HEAP_SIZE)
- **Solution**: Reduce heap if needed, or check `xSemaphoreCreateRecursiveMutex()` return value

#### Stops at **Stage 2**: I2C initialization
- **Problem**: I2C GPIO conflict or configuration
- **Check**: No other code is using GP4 (SDA) or GP5 (SCL)
- **Solution**: Verify pin definitions in `main_controller.cpp`

#### Stops at **Stage 3**: I2C device scan
- **Problem**: I2C bus timeout, missing pull-up resistors, or dead devices
- **Check**: 
  - Wiring: SDA=GP4, SCL=GP5, both to LCD and PCA9685
  - Pull-ups: Should see 4.7kΩ resistors on SDA and SCL
  - Power: LCD and PCA9685 have correct 5V power
- **Solution**: Check wiring diagram in WIRING_DIAGRAM.md

#### Stops at **Stage 4a/4b**: LCD or PCA9685 initialization
- **Problem**: Device detected but initialization command failed
- **Check**: 
  - I2C address correct: LCD=0x27, PCA9685=0x40
  - Power supply to devices
  - Check address jumpers on LCD and PCA9685 modules
- **Solution**: 
  - LCD missing? Not critical, continues to PCA
  - PCA missing? **Critical - cannot proceed without servo driver**

#### Stops at **Stage 5**: Servo initialization
- **Problem**: PCA9685 not responding to commands
- **Check**: PCA9685 address, I2C communication
- **Solution**: Verify PCA9685 wiring and power

#### Stops at **Stage 6-11**: Other initialization
- These are usually safe (mostly configuration)
- Check if specific driver has issues

#### Reaches "SWITCHING TO FREERTOS" but no "Blink!"
- **Problem**: FreeRTOS scheduler not running or task not executing
- **Check**: FreeRTOS configuration, task stack size (1024 should be enough)
- **Solution**: Check `vBlinkTask` is actually being called

---

## 🔧 Quick Fixes to Try

### Fix 1: Factory Reset Pico W
```
1. Hold BOOTSEL button
2. Press and release RST button
3. Pico appears as "RPI-RP2" mass storage device
4. Drag SpotMicro.uf2 to the drive
5. Pico reboots automatically
```

### Fix 2: Check Device Manager
```powershell
# In Device Manager:
1. Ports (COM & LPT) → Right-click USB Serial Device
2. Properties → Port Settings
3. Verify: 115200, 8 bits, 1 stop bit, no parity
4. Click "Advanced" → Ensure no weird settings
```

### Fix 3: Try Different Serial Program
- **PuTTY**: https://www.putty.org/ (recommended)
- **Tera Term**: https://teratermproject.github.io/
- **RealTerm**: https://realterm.simpleterm.com/

If one works and another doesn't, it's a terminal program issue, not the Pico.

### Fix 4: Reduce Initialization Complexity
If Stage 3 (I2C scan) hangs forever:
- Comment out the I2C device detection code
- Check if Pico can at least output "Serial initialized"
- Gradually uncomment sections to find the culprit

---

## 📊 Hardware Self-Test

### Test 1: Is Pico Actually Running Code?
- Pico W should blink onboard LED when plugged in (if loader available)
- If no sign of life, **hardware is broken or wrong firmware loaded**

### Test 2: Can You Access BOOTSEL Mode?
```
1. Plug in USB
2. Hold BOOTSEL button for 3 seconds
3. Pico should appear as mass storage "RPI-RP2"
4. If not, USB port or Pico is broken
```

### Test 3: Is I2C Bus Working?
Use i2cdetect on a separate test program:
```cpp
// Minimal test - just scan I2C addresses
uart_init(uart0, 115200);
// ... GPIO setup ...
i2c_init(i2c0, 100000);
// ... scan 0x00-0x7F and print any hits ...
```

If this works but full init doesn't, issue is in driver code (`lcd_16x2.cpp`, `pca9685.cpp`).

---

## 💡 Pro Tips

### Enable Detailed Logging
Edit `main_controller.cpp` to add more `uart_puts()` calls in driver code:
```cpp
// In drivers/lcd_16x2.cpp, add:
uart_puts(uart0, "[LCD] Starting init sequence...\n");
uart_puts(uart0, "[LCD] Writing initialization commands...\n");
uart_puts(uart0, "[LCD] Setting address 0x27...\n");
```

### Use Minimal Firmware for Testing
Comment out most initialization and test just I2C:
```cpp
// In main():
uart_puts(uart0, "[TEST] I2C only\n");
i2c_init(I2C_PORT, 100000);
// ... scan ...
// Skip all other init
while(1) sleep_ms(1000);  // Idle loop
```

### Monitor Real-Time Output
Open serial in **unbuffered mode**:
- PuTTY: Flow control = RTS/CTS (may help)
- Use `unbuffered` terminal option if available

---

## ⚙️ Next Steps Based on Output

| Symptom | Most Likely Cause | Fix |
|---------|-------------------|-----|
| No COM port appears | USB driver, cable, or bootloader | Try factory reset |
| Port appears then disappears | Firmware crashes immediately | Check Stage 0-1 |
| "Serial initialized" but hangs | I2C or FreeRTOS object creation | Check stages 2-3 |
| Serial shows stages but incomplete | Driver initialization failure | Check LCD/PCA9685 wiring |
| Reaches "SWITCHING TO FREERTOS" but no task output | Scheduler issue | Verify FreeRTOSConfig.h |

---

## 📞 Debug Contact Points

If none of this works, the issue is likely one of:
1. **I2C wiring** - Check WIRING_DIAGRAM.md
2. **Pico hardware** - Try different Pico or cable
3. **Driver code** - Check lcd_16x2.cpp, pca9685.cpp for blocking calls
4. **FreeRTOS config** - Check FreeRTOSConfig.h for incompatible settings

**Save the serial output and share it for further debugging!**

# SpotMicro Pico W - Flashing & Serial Setup Guide

## Quick Start (5 minutes)

### Step 1: Flash Firmware to Pico W

**Method A: Drag & Drop (Easiest)**
```
1. Connect Pico W to PC with USB cable
2. Hold BOOTSEL button on Pico while connecting (or press RST while holding BOOTSEL)
3. Wait for "RPI-RP2" mass storage device to appear in File Explorer
4. Copy SpotMicro.uf2 from build\ folder
5. Paste into RPI-RP2 drive
6. Pico reboots automatically (mass storage disappears)
7. Done! Firmware is loaded
```

**File location:**
```
C:\Users\Victus\Documents\Spot Micro\EmbeddedProject-robodog\PICO\SpotMicro_PicoRP2040W_FreeRTOS\build\SpotMicro.uf2
```

**Method B: Command Line**
```powershell
# Copy the file to Pico's mass storage
Copy-Item "build/SpotMicro.uf2" -Destination "D:/"  # D:/ is your RPI-RP2 drive

# Or using PowerShell (works on newer Windows 11)
robocopy "build" "D:/" "SpotMicro.uf2" /Z
```

---

### Step 2: Open Serial Monitor

**Option A: PuTTY (Recommended)**

1. **Download PuTTY**: https://www.putty.org/
   - Get `putty.exe` (or `putty-64bit.exe`)

2. **Find COM port**:
   - Right-click Start → Device Manager
   - Expand "Ports (COM & LPT)"
   - Look for "USB Serial Device (COM?)" or similar
   - Note the COM number (e.g., COM6)

3. **Configure PuTTY**:
   ```
   Session:
   └─ Connection type: Serial
   
   Serial line: COM6          [← YOUR PORT]
   Speed: 115200              [← EXACT - DON'T CHANGE]
   
   Connection → Serial:
   └─ Data bits: 8
   └─ Stop bits: 1
   └─ Parity: None
   └─ Flow control: None
   
   Session:
   └─ Saved Sessions: "SpotMicro" [← optional, saves your config]
   
   Click: [Open]
   ```

4. **Expected output** (appears immediately after flashing):
   ```
   =========================================
   === SpotMicro FreeRTOS Starting...   ===
   =========================================
   [BOOT STAGE 0] Serial initialized - code is RUNNING
   [BOOT STAGE 1] Creating FreeRTOS sync objects...
   ...
   ```

**Option B: VS Code Terminal**

If you have a serial extension:
```
Ctrl + Shift + P
→ Type "serial"
→ Select "Serial: Open Terminal"
→ Choose COM port
→ Set baud to 115200
```

**Option C: PowerShell One-Liner**
```powershell
$port = New-Object System.IO.Ports.SerialPort COM6,115200,None,8,One
$port.Open()
while ($port.IsOpen) { 
    $line = $null
    try { $line = $port.ReadLine() } catch {}
    if ($line) { Write-Host $line }
    Start-Sleep -Milliseconds 10
}
```

---

## Complete Boot Sequence

Here's what **should** happen step-by-step:

### Hardware Sequence
```
1. Connect USB → Windows detects "USB Serial Device (COM?)"
2. Pico appears in Device Manager with yellow warning? → Update driver
3. COM port appears in Device Manager → Ready to use
```

### Firmware Sequence (Serial Output)
```
[BOOT STAGE 0] ← Code is running, serial is initialized
[BOOT STAGE 1] ← Creating FreeRTOS mutexes
[BOOT STAGE 2] ← Initializing I2C bus (GPIO 4, 5)
[BOOT STAGE 3] ← Scanning for LCD (0x27) and PCA9685 (0x40)
[BOOT STAGE 4] ← Initializing LCD and PCA9685
[BOOT STAGE 5] ← Initializing 12 servos
[BOOT STAGE 6] ← Loading calibration settings
[BOOT STAGE 7] ← Moving servos to neutral pose
[BOOT STAGE 8] ← Initializing kinematics engine
[BOOT STAGE 9] ← Initializing tail motor GPIO
[BOOT STAGE 10] ← Initializing ultrasonic sensors
[BOOT STAGE 11] ← Creating FreeRTOS tasks
[BOOT COMPLETE] ← All init done, starting scheduler
===== SWITCHING TO FREERTOS TASKS =====

Blink!  ← Task running every 1 second
Blink!
Blink!
```

---

## Troubleshooting

### Problem: "Failed to open serial port COM6"

**Cause**: Port is in use by another application

**Fix**:
```powershell
# Close ALL instances of:
# - PuTTY
# - VS Code with serial extension
# - Arduino IDE
# - TeraTerm
# - Any other serial software

# Then try again
```

### Problem: No COM Port Appears

**Cause**: USB driver not installed, bad cable, or Pico bootloader corrupted

**Fix 1**: Update USB drivers
```
1. Device Manager → Right-click "Unknown Device" or USB port
2. Update driver → Browse my computer
3. Search for Pico drivers in C:\Users\[User]\.pico-sdk\
4. Reboot computer
```

**Fix 2**: Factory Reset
```
1. Hold BOOTSEL button while connecting USB
2. Wait for "RPI-RP2" mass storage to appear
3. Drag UF2Bootloader.uf2 to the drive (factory bootloader)
   [Download from pico-sdk if needed]
4. Reboots, then try flashing SpotMicro.uf2 again
```

**Fix 3**: Different USB cable
```
- Your cable might be power-only (no data lines)
- Try phone charging cable or known-good USB 3.0 cable
```

### Problem: Serial Opens but No Output

**Cause 1**: Wrong baud rate
```
✓ MUST be 115200 (not 9600, not auto)
✓ MUST be 8 data bits, 1 stop bit, no parity
```

**Fix**:
```
PuTTY → Connection → Serial:
  Speed: 115200
  Data bits: 8
  Stop bits: 1
  Parity: None
  Flow control: None
```

**Cause 2**: Firmware hasn't started yet
```
- Wait 2-3 seconds after opening serial
- Firmware startup is slow due to FreeRTOS initialization
```

**Cause 3**: Pico crashed during startup
```
- If you never see "[BOOT STAGE 0]", code isn't reaching serial init
- Try minimal test firmware (see DIAGNOSTICS.md)
```

### Problem: Partial Output Then Hangs

**Example**: Output stops at `[BOOT STAGE 3]`

**Cause**: I2C device not responding

**Fix**:
```
1. Check wiring: SDA=GP4, SCL=GP5
2. Verify I2C devices have power
3. Check pull-up resistors: 4.7kΩ on SDA and SCL
4. Use i2c_detect program to scan addresses
5. If LCD not found (Stage 4a), that's OK (skips it)
   If PCA9685 not found (Stage 4b), **MUST fix** (critical device)
```

### Problem: Baud Rate Garbage

**Example**:
```
üüüü ¦ßßßßß § üüüü
```

**Cause**: Baud rate mismatch

**Fix**: 
```
1. Double-check PuTTY speed: 115200 (not 9600, not 115200.5)
2. Try terminal program at different baud rates
3. Try external USB-UART adapter if available
```

---

## Advanced: Manual COM Port Configuration

If PuTTY isn't working, configure COM port directly:

```powershell
# List COM ports and their baud rates
Get-WmiObject Win32_SerialPort | Select-Object Name, BaudRate

# Configure COM6 to 115200
mode COM6: BAUD=115200 PARITY=N DATABITS=8 STOPBITS=1 FLOW=OFF

# Read from COM6 (minimal)
$port = New-Object System.IO.Ports.SerialPort("COM6", 115200)
$port.Open()
Write-Host $port.ReadLine()
$port.Close()
```

---

## Performance Checklist

| Check | Expected | Your Result |
|-------|----------|-------------|
| **USB Connection** | "USB Serial Device" in Device Manager | ☐ |
| **COM Port** | COM4, COM5, COM6, or similar | ☐ COM___ |
| **Baud Rate** | 115200 in PuTTY settings | ☐ |
| **Data Bits** | 8 | ☐ |
| **Stop Bits** | 1 | ☐ |
| **Parity** | None | ☐ |
| **Flow Control** | None | ☐ |
| **[BOOT STAGE 0]** | Appears within 1 second of opening serial | ☐ |
| **All stages complete** | Reaches "[BOOT COMPLETE]" | ☐ |
| **Blink task runs** | "Blink!" appears every 1 second | ☐ |

---

## If Everything Fails

1. **Take a screenshot** of the serial output (if any)
2. **Document**:
   - What you see in Device Manager
   - What PuTTY shows (or nothing)
   - What baud rate you're using
   - Where initialization stops (if it does)
3. **Check**:
   - WIRING_DIAGRAM.md for I2C connections
   - DIAGNOSTICS.md for boot sequence details
4. **Try**:
   - Factory reset (hold BOOTSEL during USB connect)
   - Different Pico W (hardware defect?)
   - Minimal test program (just print "Hello")

---

## Minimal Test Firmware

If standard firmware doesn't work, try this minimal version to test just serial:

```cpp
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    sleep_ms(100);
    
    uart_puts(uart0, "=== MINIMAL TEST ===\n");
    uart_puts(uart0, "If you see this, serial works!\n");
    
    int count = 0;
    while(1) {
        uart_printf(uart0, "Count: %d\n", count++);
        sleep_ms(1000);
    }
    
    return 0;
}
```

If you see "Count: 0, 1, 2..." then serial hardware is OK, and the issue is with main initialization.

---

## Reference: GPIO Connections

```
Pico W Physical Pinout:
┌─────────────────────────┐
│  ●●●●●●●●●●●●●●●●●●●  │
│  ●●●●●●●●●●●●●●●●●●●  │
│   USB                   │
└─────────────────────────┘

Critical pins for debugging:
GP0  (Pin 1)  ← Not used in new firmware
GP1  (Pin 2)  ← Not used in new firmware
GP4  (Pin 6)  ← I2C SDA (LCD + PCA9685)
GP5  (Pin 7)  ← I2C SCL (LCD + PCA9685)

USB provides:
- Power (5V on VBUS, pin 40)
- Data (D+/D-, built-in pins)
- Serial CDC (handled by stdio_init_all())
```

---

## Success!

Once you see:
```
[BOOT COMPLETE] Starting FreeRTOS Scheduler...
===== SWITCHING TO FREERTOS TASKS =====

Blink!
Blink!
```

Your Pico W is **working correctly**! 🎉

Next steps:
- Verify LCD displays "FreeRTOS Boot"
- Verify PCA9685 doesn't report errors
- Check servo positioning
- Enable additional FreeRTOS tasks

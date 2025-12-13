# SpotMicro Serial Debugging Guide

## Overview
The firmware now outputs debug messages via **UART0** (GPIO 0-1) which is more reliable than USB CDC. Both USB and UART0 are active simultaneously.

## Serial Output Methods

### Option 1: USB Serial Monitor (Built-in to VS Code)
1. **Connect** Pico W to PC via USB cable
2. **VS Code**: Press `Ctrl + Shift + ~` to open terminal
3. **Run**: 
   ```powershell
   pico_get_usb_devs  # Get COM port (if available)
   ```

### Option 2: PuTTY (Recommended for UART0)
1. **Download**: https://www.putty.org/
2. **Find COM Port**:
   - Device Manager → Ports (COM & LPT)
   - Look for "USB Serial Device" (Pico on USB) or "COM?" entry
3. **Configure PuTTY**:
   - **Connection type**: Serial
   - **Serial line**: COM4 (or your port)
   - **Speed**: 115200 baud
   - **Data bits**: 8
   - **Stop bits**: 1
   - **Parity**: None
   - **Flow control**: None
4. **Click**: Open

### Option 3: PowerShell COM Port Monitor
```powershell
$port = New-Object System.IO.Ports.SerialPort COM4,115200,None,8,One
$port.Open()
while ($port.IsOpen) { 
    try { Write-Host $port.ReadLine() } catch {} 
}
$port.Close()
```

### Option 4: Teensy Loader Serial Monitor
- If installed, can monitor at 115200 baud

## Hardware Connections for UART0 Debugging

### Pico W GPIO Pinout
```
GP0 (Pin 1)  ──── UART0 TX (transmit to computer)
GP1 (Pin 2)  ──── UART0 RX (receive from computer)
GND (Pin 3)  ──── Common ground with USB
```

### Optional: External Serial Adapter
If not using USB, connect a USB-to-UART adapter:
- **Adapter TX** → Pico **GP1 (RX)**
- **Adapter RX** → Pico **GP0 (TX)**
- **Adapter GND** → Pico **GND**
- Configure for **115200 baud, 8N1**

## Expected Serial Output

When the firmware boots correctly, you should see:

```
=========================================
=== SpotMicro FreeRTOS Starting...   ===
=========================================
Build: Dec 11 2025 16:01:32
[UART0] Serial output initialized
[INIT] Initializing I2C bus...
[INIT] I2C bus initialized (SDA=GP4, SCL=GP5)
[INIT] Checking I2C devices...
[SCAN] LCD found at 0x27
[SCAN] PCA9685 found at 0x40
[INIT] Device check complete: LCD=OK, PCA9685=OK
[INIT] Init LCD & PCA9685...
[OK] LCD Init Success!
[INIT] Proceeding to PCA9685...
[OK] PCA9685 initialized
[INIT] Initializing servos...
[OK] 12 servos initialized
[OK] Settings Loaded from Flash
[INIT] Setting neutral pose...
[INIT] Initializing kinematics...
[INIT] Initializing tail motor...
[OK] Tail motor initialized
[INIT] Initializing sensor hub...
[OK] Sensor hub initialized
[INFO] WiFi init skipped (commented out)
[INIT] Creating FreeRTOS tasks...
[INIT] All systems initialized successfully!
[INIT] Starting FreeRTOS Scheduler...
Blink!
Blink!
Blink!
```

## Troubleshooting

### No Output at All
1. **Check USB connection**: Try different USB cable or port
2. **Check COM port**: Device Manager should show "USB Serial Device (COM?)"
3. **Try UART0 with external adapter**: GP0/GP1 might have more reliable output
4. **Verify baud rate**: Must be exactly **115200**

### Garbage Characters
- **Baud rate mismatch**: Verify 115200, 8 data bits, 1 stop bit, no parity

### Only Partial Output (stops after a few lines)
- **USB enumeration delay**: Firmware gives USB 500ms to connect, then proceeds
- **Use UART0 instead**: More reliable for debugging

### Device Not Found (I2C scan shows "No LCD" or "No PCA9685")
1. Check I2C wiring: SDA=GP4, SCL=GP5
2. Verify pull-up resistors on SDA/SCL
3. Test I2C devices separately
4. Check device addresses: LCD=0x27, PCA9685=0x40

### Freezes at [INIT] message
1. I2C device timeout → check wiring
2. Servo initialization → PCA9685 not responding
3. Enable UART0 logging to see exactly where it hangs

## Code Reference

### debug_print() Function
Located in `main_controller.cpp`:
```cpp
void debug_print(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // Try USB first if available
    if (stdio_usb_connected()) {
        printf("%s", buffer);
        stdio_flush();
    }
    
    // Also use UART0 (GP0=TX, GP1=RX) for guaranteed output
    uart_puts(uart0, buffer);
}
```

This function writes to **both** USB and UART0 simultaneously, ensuring output on at least one channel.

## Performance Metrics

- **Boot time**: ~0.5-1 second (FreeRTOS startup + initialization)
- **Debug message latency**: <1ms (UART0 is very fast)
- **USB latency**: 10-50ms (buffered, depends on host polling)

## Disabling Debug Output
To reduce power/improve performance, remove `debug_print()` calls from critical sections and replace with:
```cpp
// Minimal output
#ifdef DEBUG
    debug_print("Message\n");
#endif
```

Then compile with `-DDEBUG` flag.

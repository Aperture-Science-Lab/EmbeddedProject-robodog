# Stereo Vision Wiring Guide

## Overview

This document describes how to connect the ESP32 stereo camera system to the Pico W main controller for obstacle detection and avoidance.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         SPOTMICRO ROBODOG                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌─────────────────┐              ┌─────────────────┐              │
│   │   ESP32-CAM     │    UART      │    Pico W       │              │
│   │  Stereo Camera  │◄────────────►│  Main Controller│              │
│   │                 │              │                 │              │
│   │  GPIO1 (TX) ────┼──────────────┼──► GP1 (RX)     │              │
│   │  GPIO3 (RX) ◄───┼──────────────┼─── GP0 (TX)     │              │
│   │  GND ───────────┼──────────────┼─── GND          │              │
│   │  5V ────────────┼──────────────┼─── VSYS         │              │
│   └─────────────────┘              └─────────────────┘              │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Wiring Connections

### ESP32-CAM to Pico W (UART Communication)

| ESP32-CAM Pin | Wire Color | Pico W Pin | Description |
|---------------|------------|------------|-------------|
| GPIO1 (TX)    | Yellow     | GP1        | Camera TX → Pico RX |
| GPIO3 (RX)    | Green      | GP0        | Pico TX → Camera RX |
| GND           | Black      | GND        | Common ground |
| 5V            | Red        | VSYS       | Power (optional*) |

*Note: ESP32-CAM can be powered separately via USB if preferred.

### Pin Assignments Summary (Pico W)

| Pin | Function | Connected To |
|-----|----------|--------------|
| GP0 | UART1 TX | ESP32 GPIO3 (RX) |
| GP1 | UART1 RX | ESP32 GPIO1 (TX) |
| GP4 | I2C0 SDA | PCA9685, LCD |
| GP5 | I2C0 SCL | PCA9685, LCD |
| GP6 | US1 Trig | Left Ultrasonic |
| GP7 | US1 Echo | Left Ultrasonic |
| GP8 | US2 Trig | Right Ultrasonic |
| GP9 | US2 Echo | Right Ultrasonic |
| GP10 | Motor IN1 | L298N (Tail) |
| GP11 | Motor IN2 | L298N (Tail) |
| GP16 | UART0 TX | Nano RP2040 (IMU) |
| GP17 | UART0 RX | Nano RP2040 (IMU) |

## ESP32-CAM Pinout (AI-Thinker Module)

```
                    ┌─────────────────┐
                    │     ESP32-CAM   │
                    │   (AI-Thinker)  │
                    │                 │
         5V ────────┤ 5V          3V3 ├
        GND ────────┤ GND        GPIO16├
         NC ────────┤ GPIO12     GPIO0 ├──── (Boot mode)
         NC ────────┤ GPIO13     GND   ├
         NC ────────┤ GPIO15     VCC   ├
         NC ────────┤ GPIO14     U0R   ├──── GPIO3 (RX) ◄── Pico GP0
         NC ────────┤ GPIO2      U0T   ├──── GPIO1 (TX) ──► Pico GP1
         NC ────────┤ GPIO4      GND   ├
                    │                 │
                    │   [CAMERA]      │
                    │      □          │
                    └─────────────────┘
```

## Stereo Camera Configuration

### Option 1: Single ESP32-CAM (Monocular)
- Uses edge detection for basic obstacle sensing
- Lower accuracy but simpler setup
- Good for initial testing

### Option 2: Dual ESP32-CAM (True Stereo)
- Master ESP32-CAM: Left camera + computation
- Slave ESP32-CAM: Right camera, sends to master via SPI
- Requires synchronization between cameras

### Option 3: ESP32-S3 with Dual Camera (Recommended)
- Single board with two camera ports
- Best synchronization and performance
- More complex firmware

## Communication Protocol

### Messages from ESP32 to Pico (UART @ 115200 baud)

```
DEPTH,<left_cm>,<center_cm>,<right_cm>
  - Zone distances in centimeters
  - Example: DEPTH,45.2,28.7,62.1

OBSTACLE,<zone>,<distance_cm>,<confidence>
  - Zone: LEFT, CENTER, or RIGHT
  - Confidence: 0-100%
  - Example: OBSTACLE,CENTER,28.7,85

HEARTBEAT,<frame_count>,<fps>
  - Status message sent every second
  - Example: HEARTBEAT,1234,9.5

STEREO_CAM_READY
  - Sent once when camera initializes
```

### Commands from Pico to ESP32

```
STATUS    - Request immediate status update
DEPTH     - Request depth data
```

## Testing the Connection

1. **Flash ESP32-CAM firmware:**
   ```bash
   cd PICO/cameras/esp32_stereo_camera
   pio run -t upload
   ```

2. **Flash Pico W firmware:**
   ```bash
   cd PICO/spotmicro-rp2040
   mkdir build && cd build
   cmake ..
   make main_controller
   # Copy main_controller.uf2 to Pico
   ```

3. **Test communication:**
   - Connect via USB serial to Pico W
   - Type `CAMERA` to see stereo vision status
   - Should show "CONNECTED" if ESP32 is sending data

4. **Test autonomous mode:**
   - Type `AUTO` to enable autonomous navigation
   - Robot will walk forward and avoid obstacles
   - Type `STOP` or `MANUAL` to disable

## Troubleshooting

### No camera connection
- Check TX/RX wiring (crossed correctly)
- Verify GND is connected
- Check ESP32 is powered and running
- Monitor ESP32 USB serial for debug output

### Incorrect depth readings
- Verify `STEREO_BASELINE_MM` matches your camera spacing
- Run calibration procedure (see calibration docs)
- Check lighting conditions

### Robot not avoiding obstacles
- Check `CAMERA` command shows obstacles detected
- Verify threshold settings in `stereo_vision.h`
- Test with ultrasonic sensors alone first

## Related Files

- `PICO/cameras/esp32_stereo_camera/` - ESP32 firmware
- `PICO/spotmicro-rp2040/stereo_vision.cpp` - Pico receiver
- `PICO/spotmicro-rp2040/stereo_vision.h` - API definitions
- `PICO/spotmicro-rp2040/main_controller.cpp` - Integration

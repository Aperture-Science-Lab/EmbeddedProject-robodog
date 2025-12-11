# SpotMicro RP2040 - Setup and Installation Guide

This project is a quadruped robot dog implementation using the Raspberry Pi Pico W. It features a custom gait engine, Wi-Fi connectivity, and sensor integration.

## Hardware Requirements

*   **Microcontroller:** Raspberry Pi Pico W
*   **Actuators:** 12x MG996R Servos (3 per leg)
*   **Drivers:**
    *   PCA9685 16-Channel PWM Driver (for servos)
    *   L298N H-Bridge (for tail motor)
*   **Sensors:**
    *   Arduino Nano RP2040 Connect (Smart IMU)
    *   2x HC-SR04 Ultrasonic Sensors
*   **Power:**
    *   12V Li-ion Battery (e.g., 3000mAh+)
    *   XL4016 Buck Converter (12V -> 6V for Servos)
    *   LM2596S Buck Converter (12V -> 5V for Logic)
*   **Display:** LCD 16x2 with I2C Backpack

## Wiring Setup

Please refer to the detailed [PROJECT_REPORT.md](PROJECT_REPORT.md) or [docs/WIRING_DIAGRAM.md](docs/WIRING_DIAGRAM.md) for pinout tables and diagrams.

**Quick Pinout Reference (Pico W):**
*   **I2C (PCA9685/LCD):** SDA=GP4, SCL=GP5
*   **IMU UART:** TX=GP16, RX=GP17
*   **Ultrasonics:** Left=GP6/7, Right=GP8/9
*   **Tail:** GP10/11

---

## Software Setup

### Prerequisites
*   [VS Code](https://code.visualstudio.com/)
*   [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
*   Create a `PICO_SDK_PATH` environment variable pointing to your SDK installation.
*   CMake and GCC-ARM-Embedded Toolchain.

### 1. Configuration (`wifi_config.h`)
Before building, you **must** configure your Wi-Fi and MQTT credentials.

1.  Open `wifi_config.h` in the root directory.
2.  Edit the following lines with your network details:

```c
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER_IP  "192.168.1.xxx"
```

### 2. Build Instructions

1.  Create a build directory:
    ```bash
    mkdir build
    cd build
    ```

2.  Run CMake:
    ```bash
    cmake ..
    ```

3.  Compile the project:
    ```bash
    make -j4
    ```
    *(On Windows with MinGW, use `mingw32-make` or standard MSVC tools if configured)*

4.  **Output:** This will generate a `spotmicro.uf2` file in the `build` folder.

### 3. Flashing the Pico W
1.  Hold the **BOOTSEL** button on your Pico W while plugging it into USB.
2.  A drive named `RPI-RP2` will appear.
3.  Drag and drop the `spotmicro.uf2` file into this drive.
4.  The Pico will reboot and run the code.

---

## Usage and Commands

The robot is controlled via MQTT topics (`spotmicro/cmd`). It can also be controlled mostly automatically if gait modes are enabled.

**MQTT Commands:**
*   `w` - Walk Forward
*   `b` - Walk Backward
*   `s` - Stop
*   `a` / `d` - Turn Left / Right
*   `n` - Stand Neutral
*   `SAVE` - Save current calibration to Flash
*   `LOAD` - Load calibration from Flash

**Calibration:**
Use the serial monitor or MQTT to fine-tune servo offsets if legs are not straight.
*   Format: `SH_FR 90` (Set Front Right Shoulder to 90 degrees)
*   Once aligned, send `SAVE`.

---

## Project Structure
*   `src/` - Source code headers
*   `docs/` - Documentation and Wiring
*   `project_report.md` - Detailed project summary
*   `main_controller.cpp` - Entry point and logic

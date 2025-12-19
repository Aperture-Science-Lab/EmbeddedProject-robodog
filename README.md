# SpotMicro RP2040 Quadruped Robot

This project implements a SpotMicro quadruped robot using a distributed microcontroller architecture (Raspberry Pi Pico W and Arduino Nano RP2040 Connect). It features a custom inverse kinematics model, sensor fusion for state estimation, and autonomous navigation capabilities.

## Project Resources

* **3D Models:** [Google Drive Repository](https://drive.google.com/drive/folders/1cT-X_47x2Xq5MXbdnsAX1cy5eyDPXmdt)
* **Base Chassis Model:** [SpotMicro V4 (Creality Cloud)](https://www.crealitycloud.com/model-detail/spotmicro-robotic-dog-v4)
* **Project Documentation:** [SpotMicroAI ReadTheDocs](https://spotmicroai.readthedocs.io/en/latest/)

## System Architecture

The system is divided into three parallel subsystems controlled via FreeRTOS on the main controller.

### Hardware Overview
* **Main Controller:** Raspberry Pi Pico W (Locomotion, WiFi, MQTT)
* **Coprocessor (IMU/Sensors):** Arduino Nano RP2040 Connect (State Estimation)
* **Actuation:** 12x MG996R Servos driven by PCA9685
* **Vision:** 2x OV9655 Stereo Cameras
* **Sensors:** HC-SR04 Ultrasonics, NEO-6M GPS, IR/PIR sensors, LDR

### Subsystem Logic
1.  **Core Balance & Locomotion (Sequential):** Handles Inverse Kinematics (IK) and active balancing using IMU data.
2.  **Autonomous Navigation (Parallel):** Processes ultrasonic and stereo vision data to detect obstacles and drop-offs.
3.  **Remote Control (Parallel):** Listens for MQTT commands via WiFi for teleoperation and state reporting.

---

## Wiring Documentation

### 1. Main Controller: Raspberry Pi Pico W

The Pico W acts as the central brain, handling gait generation and servo control.

#### Pin Assignment Table

| GPIO | Function | Connection | Note |
|------|----------|------------|------|
| **GP4** | I2C0 SDA | PCA9685 / LCD | Shared Bus |
| **GP5** | I2C0 SCL | PCA9685 / LCD | Shared Bus |
| **GP6** | Output | Ultrasonic 1 TRIG | Left Sensor |
| **GP7** | Input | Ultrasonic 1 ECHO | Left Sensor (Voltage Divider req.) |
| **GP8** | Output | Ultrasonic 2 TRIG | Right Sensor |
| **GP9** | Input | Ultrasonic 2 ECHO | Right Sensor (Voltage Divider req.) |
| **GP10**| Output | L298N IN1 | Tail Motor |
| **GP11**| Output | L298N IN2 | Tail Motor |
| **GP16**| UART0 TX | Nano RP2040 RX | IMU Data Link |
| **GP17**| UART0 RX | Nano RP2040 TX | IMU Data Link |

#### PCA9685 Servo Mapping
* **Front Right:** CH0 (Shoulder), CH1 (Elbow), CH2 (Wrist)
* **Front Left:** CH3 (Shoulder), CH4 (Elbow), CH5 (Wrist)
* **Rear Right:** CH6 (Shoulder), CH7 (Elbow), CH8 (Wrist)
* **Rear Left:** CH9 (Shoulder), CH10 (Elbow), CH11 (Wrist)

### 2. Coprocessor: Arduino Nano RP2040 Connect

Handles the LSM6DSOX IMU, GPS, and environmental sensors.

#### Pin Assignment Table

| Pin | Component | Note |
|-----|-----------|------|
| **A0** | LDR Module | Digital OUT -> A0 |
| **A1** | Green LED | Security/Status Indicator |
| **A2** | Red LED | Security/Status Indicator |
| **D2** | IR Front | LOW = Object Detected |
| **D3** | IR Back | LOW = Object Detected |
| **D4** | PIR Front | HIGH = Motion Detected |
| **D5** | PIR Back | HIGH = Motion Detected |
| **D6** | RGB LED (R) | Status Indication |
| **D7** | RGB LED (G) | Status Indication |
| **D10** | RGB LED (B) | Status Indication |
| **D8** | GPS TX | NEO-6M |
| **D9** | GPS RX | NEO-6M |
| **D0/D1**| Serial1 | UART Link to Pico W |

#### RGB Status Logic
* **OFF:** Path Clear
* **GREEN:** Front Obstacle Detected (IR Front)
* **RED:** Rear Obstacle Detected (IR Back)
* **BLUE:** Both Front/Rear Blocked

---

## Power Distribution

The robot uses a split-power rail system powered by a 12V 7000mAh Li-ion battery.

1.  **High Power Rail (6V):**
    * **Source:** XL4016 Buck Converter (12V -> 6V, 8A Max)
    * **Load:** PCA9685 V+ Terminal (Powers all 12 Servos) and Tail Motor.
2.  **Logic Rail (5V):**
    * **Source:** LM2596S Buck Converter (12V -> 5V, 3A Max)
    * **Load:** Raspberry Pi Pico W (VSYS), Arduino Nano RP2040 (VIN), Sensors, LCD.

```text
    [12V Battery] --+-- [XL4016] --> 6V --> [PCA9685 V+] --> Servos
                    |
                    +-- [LM2596S] --> 5V --> [Pico W / Nano / Sensors]
```
## Software Dependencies & References

### Core Libraries
* **OS Kernel:** [FreeRTOS Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel)
* **Main Kinematics:** [SpotMicroAI Kinematics](https://spotmicroai.readthedocs.io/en/latest/kinematic/)
* **State Estimation:** [ES-EKF (Self-Driving Car)](https://github.com/Aperture-Science-Lab/Self-Driving-Car)
* **Microcontroller Code:** [SpotMicroESP32 Port](https://github.com/michaelkubina/SpotMicroESP32)

### AI & Vision Modules
* **Speech Recognition:** [OpenAI Whisper](https://github.com/openai/whisper)
* **Stereo Vision:** [ESP32 Stereo Camera](https://github.com/jonathanrandall/esp32_stereo_camera)
* **Secondary Kinematics Reference:** [OpenQuadruped](https://github.com/OpenQuadruped/spot_mini_mini)

---

## Setup and Installation

### Prerequisites
* VS Code with CMake and GCC-ARM-Embedded Toolchain.
* Raspberry Pi Pico SDK.

### Configuration
1.  Open `src/wifi_config.h`.
2.  Update the following definitions:
    * `WIFI_SSID`
    * `WIFI_PASSWORD`
    * `MQTT_BROKER_IP`

### Build Instructions

```bash
mkdir build
cd build
cmake ..
make -j4

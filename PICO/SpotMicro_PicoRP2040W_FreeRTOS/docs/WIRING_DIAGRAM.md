# SpotMicro RP2040-W Complete Wiring Diagram

## Raspberry Pi Pico W (RP2040) Pinout Reference

```
                    ┌──────────────────────┐
                    │      USB PORT        │
                    │    ┌──────────┐      │
            ┌───────┴────┤          ├──────┴───────┐
     GP0  ──┤ 1          │  RP2040  │          40 ├── VBUS (5V from USB)
     GP1  ──┤ 2          │          │          39 ├── VSYS (1.8V-5.5V input)
     GND  ──┤ 3          │   PICO   │          38 ├── GND
     GP2  ──┤ 4          │    W     │          37 ├── 3V3_EN
     GP3  ──┤ 5          │          │          36 ├── 3V3 OUT
I2C0 SDA ──┤ 6  (GP4)   │          │   (GP28) 35 ├── ADC2
I2C0 SCL ──┤ 7  (GP5)   │          │   (GND)  34 ├── ADC_GND
     GND  ──┤ 8          │          │   (GP27) 33 ├── ADC1
     GP6  ──┤ 9          │          │   (GP26) 32 ├── ADC0
     GP7  ──┤ 10         │          │          31 ├── RUN (Reset)
     GP8  ──┤ 11         │          │   (GP22) 30 ├── GP22
     GP9  ──┤ 12         │          │          29 ├── GND
     GND  ──┤ 13         │          │   (GP21) 28 ├── GP21
    GP10  ──┤ 14         │          │   (GP20) 27 ├── GP20
    GP11  ──┤ 15         │          │   (GP19) 26 ├── GP19
    GP12  ──┤ 16         │          │   (GP18) 25 ├── GP18
    GP13  ──┤ 17         │          │          24 ├── GND
     GND  ──┤ 18         │          │   (GP17) 23 ├── GP17
    GP14  ──┤ 19         │          │   (GP16) 22 ├── GP16
    GP15  ──┤ 20         │          │          21 ├── GP15
            └────────────┴──────────┴──────────────┘
```

---

## Power Distribution Diagram

```
    ┌─────────────────────────────────────────────────────────────────────────┐
    │                         12V Li-ion Battery                              │
    │                         (7000mAh, 12V)                                  │
    └───────────────────────────────┬─────────────────────────────────────────┘
                                    │
                        ┌───────────┴───────────┐
                        │    ROCKER SWITCH      │
                        │      (10A ON/OFF)     │
                        └───────────┬───────────┘
                                    │
              ┌─────────────────────┼─────────────────────┐
              │                     │                     │
              ▼                     ▼                     ▼
    ┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐
    │   XL4016 PWM    │   │   LM2596S       │   │   L298N         │
    │   Step-Down     │   │   Step-Down     │   │   H-Bridge      │
    │   12V → 6V      │   │   12V → 5V      │   │   (12V direct)  │
    │   (8A, Servos)  │   │   (3A, Logic)   │   │                 │
    └────────┬────────┘   └────────┬────────┘   └────────┬────────┘
             │                     │                     │
             ▼                     │                     │
    ┌─────────────────┐            │                     │
    │   PCA9685       │            ▼                     ▼
    │   (V+ Terminal) │   ┌─────────────────┐   ┌─────────────────┐
    │   Powers 12     │   │  Pico W (VSYS)  │   │   DC Motor      │
    │   MG996R Servos │   │  LCD (VCC)      │   │   (Tail Wag)    │
    └─────────────────┘   │  HC-SR04 x2     │   └─────────────────┘
                          │  Nano RP2040    │
                          │  Connect (VIN)  │
                          └─────────────────┘
```

---

## Complete GPIO Pin Assignment Table

| GPIO Pin | Physical Pin | Function | Connected To | Wire Color (Suggested) |
|----------|--------------|----------|--------------|------------------------|
| **I2C Bus (Shared)** |
| GP4 | 6 | I2C0 SDA | PCA9685 SDA, LCD SDA | Blue |
| GP5 | 7 | I2C0 SCL | PCA9685 SCL, LCD SCL | Yellow |
| **Ultrasonic Sensor 1 (Left)** |
| GP6 | 9 | GPIO Output | HC-SR04 #1 TRIG | Orange |
| GP7 | 10 | GPIO Input | HC-SR04 #1 ECHO | Green |
| **Ultrasonic Sensor 2 (Right)** |
| GP8 | 11 | GPIO Output | HC-SR04 #2 TRIG | Orange |
| GP9 | 12 | GPIO Input | HC-SR04 #2 ECHO | Green |
| **H-Bridge Motor Control** |
| GP10 | 14 | GPIO Output | L298N IN1 | Purple |
| GP11 | 15 | GPIO Output | L298N IN2 | Gray |
| **Smart IMU (Nano RP2040 Connect via UART0)** |
| GP16 | 22 | UART0 TX | Nano GPIO1/RX (Pin 17) | White |
| GP17 | 23 | UART0 RX | Nano GPIO0/TX (Pin 16) | Green |
| **Power** |
| VSYS | 39 | Power In | 5V from LM2596S | Red |
| GND | 3, 8, 13, 18, 23, 28, 33, 38 | Ground | Common Ground | Black |
| 3V3 | 36 | 3.3V Out | (Available for sensors) | Red |
| VBUS | 40 | 5V USB | (When USB connected) | Red |

---

## Detailed Wiring Connections

### 1. I2C Bus (PCA9685 + LCD 16x2)

Both devices share the same I2C bus (I2C0) at different addresses.

```
                          ┌──────────────────────────────────────┐
                          │           I2C BUS (I2C0)             │
                          │                                      │
    Pico W                │     PCA9685              LCD 16x2    │
    ┌──────┐              │     ┌──────┐             ┌──────┐    │
    │ GP4  ├──────────────┼─────┤ SDA  │─────────────┤ SDA  │    │
    │ (SDA)│              │     │      │             │      │    │
    │      │              │     │ Addr │             │ Addr │    │
    │ GP5  ├──────────────┼─────┤ SCL  │─────────────┤ SCL  │    │
    │ (SCL)│              │     │ 0x40 │             │ 0x27 │    │
    │      │              │     │      │             │ or   │    │
    │ GND  ├──────────────┼─────┤ GND  │─────────────┤ 0x3F │    │
    │      │              │     │      │             │      │    │
    │ VSYS ├──5V──────────┼─────┤ VCC  │─────────────┤ VCC  │    │
    └──────┘              │     └──────┘             └──────┘    │
                          └──────────────────────────────────────┘

    Note: Use 4.7kΩ pull-up resistors on SDA/SCL if not built into modules
```

**PCA9685 Wiring Detail:**
```
    PCA9685 Module
    ┌────────────────────────────────────────────────────┐
    │                                                    │
    │  CONTROL SIDE              SERVO OUTPUT SIDE       │
    │  ┌─────────┐               ┌───────────────────┐   │
    │  │ GND ────┼── Pico GND    │ CH0  ── FR Shoulder│  │
    │  │ OE  ────┼── (NC or GND) │ CH1  ── FR Elbow   │  │
    │  │ SCL ────┼── Pico GP5    │ CH2  ── FR Wrist   │  │
    │  │ SDA ────┼── Pico GP4    │ CH3  ── FL Shoulder│  │
    │  │ VCC ────┼── 5V Logic    │ CH4  ── FL Elbow   │  │
    │  │ V+  ────┼── 6V (XL4016) │ CH5  ── FL Wrist   │  │
    │  └─────────┘               │ CH6  ── RR Shoulder│  │
    │                            │ CH7  ── RR Elbow   │  │
    │                            │ CH8  ── RR Wrist   │  │
    │                            │ CH9  ── RL Shoulder│  │
    │                            │ CH10 ── RL Elbow   │  │
    │                            │ CH11 ── RL Wrist   │  │
    │                            │ CH12 ── (unused)   │  │
    │                            │ CH13 ── (unused)   │  │
    │                            │ CH14 ── (unused)   │  │
    │                            │ CH15 ── (unused)   │  │
    │                            └───────────────────┘   │
    └────────────────────────────────────────────────────┘
```

---

## PCA9685 Servo Driver - Detailed Wiring Guide

### Module Overview

The PCA9685 is a 16-channel, 12-bit PWM driver that communicates via I2C. It allows control of up to 16 servos with only 2 GPIO pins from the Pico.

```
    ┌─────────────────────────────────────────────────────────────────────────┐
    │                        PCA9685 MODULE LAYOUT                            │
    │                                                                         │
    │    LEFT SIDE (Control)              RIGHT SIDE (Servo Outputs)          │
    │    ┌─────────────────┐              ┌─────────────────────────┐         │
    │    │ GND  ●──────────│              │ PWM  V+  GND  (per ch)  │         │
    │    │ OE   ●──────────│              │  │    │    │            │         │
    │    │ SCL  ●──────────│              │  ▼    ▼    ▼            │         │
    │    │ SDA  ●──────────│              │ ┌──┬──┬──┐ CH0          │         │
    │    │ VCC  ●──────────│              │ │S │+ │- │ CH1          │         │
    │    │ V+   ●──────────│              │ │  │  │  │ CH2          │         │
    │    └─────────────────┘              │ │  │  │  │ ...          │         │
    │                                     │ └──┴──┴──┘ CH15         │         │
    │    Power Terminal Block             └─────────────────────────┘         │
    │    ┌─────────────────┐                                                  │
    │    │ V+   GND        │  ◄── External Servo Power (6V from XL4016)      │
    │    │  ●     ●        │                                                  │
    │    └─────────────────┘                                                  │
    │                                                                         │
    └─────────────────────────────────────────────────────────────────────────┘
```

### Pin Descriptions

| Pin | Name | Function | Connection |
|-----|------|----------|------------|
| GND | Ground | Logic ground | Pico GND (Pin 3, 8, 13, etc.) |
| OE | Output Enable | Active LOW, enables outputs | Connect to GND or leave floating |
| SCL | I2C Clock | Serial clock | Pico GP5 (Pin 7) |
| SDA | I2C Data | Serial data | Pico GP4 (Pin 6) |
| VCC | Logic Power | 3.3V-5V logic supply | Pico 5V (VSYS) or 3.3V |
| V+ | Servo Power | 4.8V-6V servo supply | **6V from XL4016** (NOT from Pico!) |

### ⚠️ Critical: Dual Power Supply

The PCA9685 has **TWO separate power inputs**:

```
    ┌────────────────────────────────────────────────────────────────┐
    │                    POWER CONNECTIONS                           │
    │                                                                │
    │   ┌─────────────┐         ┌─────────────────────────────────┐  │
    │   │   PICO W    │         │          PCA9685                │  │
    │   │             │         │                                 │  │
    │   │  VSYS (5V) ─┼────────►│─ VCC  (Logic Power, 5V)         │  │
    │   │             │         │                                 │  │
    │   │  GND ───────┼────────►│─ GND  (Common Ground)           │  │
    │   │             │         │                                 │  │
    │   └─────────────┘         │                                 │  │
    │                           │                                 │  │
    │   ┌─────────────┐         │                                 │  │
    │   │   XL4016    │         │                                 │  │
    │   │  (12V→6V)   │         │                                 │  │
    │   │             │         │                                 │  │
    │   │  OUT+ (6V) ─┼────────►│─ V+   (Servo Power, 6V)         │  │
    │   │             │         │        Use screw terminal!      │  │
    │   │  OUT- ──────┼────────►│─ GND  (Common Ground)           │  │
    │   │             │         │                                 │  │
    │   └─────────────┘         └─────────────────────────────────┘  │
    │                                                                │
    │   ⚠️ V+ powers the servos directly - must handle 6A+ !        │
    │   ⚠️ VCC only powers the logic chip (few mA)                  │
    │   ⚠️ All GNDs must be connected together!                     │
    │                                                                │
    └────────────────────────────────────────────────────────────────┘
```

### I2C Wiring to Pico W

```
    PICO W                              PCA9685
    ┌──────────────┐                   ┌──────────────┐
    │              │                   │              │
    │  GP4 (SDA) ──┼───────────────────┼── SDA       │
    │  Pin 6       │                   │              │
    │              │                   │              │
    │  GP5 (SCL) ──┼───────────────────┼── SCL       │
    │  Pin 7       │                   │              │
    │              │                   │              │
    │  GND ────────┼───────────────────┼── GND       │
    │  Pin 3/8/13  │                   │              │
    │              │                   │              │
    │  VSYS (5V) ──┼───────────────────┼── VCC       │
    │  Pin 39      │                   │              │
    │              │                   │              │
    └──────────────┘                   │  OE ────┬── GND (to enable)
                                       │         │   or leave floating
                                       └─────────┴────────────────────┘

    I2C Address: 0x40 (default)
    I2C Speed: 400kHz (Fast Mode) or 100kHz (Standard)
```

### Servo Output Channels

Each channel has 3 pins in a row:

```
    SERVO CONNECTOR (3-pin header per channel)
    
    ┌─────┬─────┬─────┐
    │ PWM │ V+  │ GND │   ◄── Standard servo pinout
    │(Sig)│(Red)│(Blk)│
    └──┬──┴──┬──┴──┬──┘
       │     │     │
       │     │     └── Black/Brown wire (Ground)
       │     └── Red wire (Power, 4.8-6V)
       └── Orange/Yellow/White wire (Signal/PWM)

    
    MG996R Servo Wires:
    ┌────────────────────────────────────┐
    │  Brown  = GND (Ground)             │
    │  Red    = VCC (Power, 4.8-6V)      │
    │  Orange = Signal (PWM)             │
    └────────────────────────────────────┘
```

### SpotMicro Servo Channel Mapping

```
    ┌─────────────────────────────────────────────────────────────────────────┐
    │                     SPOTMICRO LEG SERVO ASSIGNMENT                      │
    │                                                                         │
    │                            FRONT                                        │
    │                              ▲                                          │
    │                              │                                          │
    │         FRONT LEFT (FL)      │      FRONT RIGHT (FR)                    │
    │         ┌───────────┐        │        ┌───────────┐                     │
    │         │ CH3: Shoulder      │        │ CH0: Shoulder                   │
    │         │ CH4: Elbow         │        │ CH1: Elbow                      │
    │         │ CH5: Wrist         │        │ CH2: Wrist                      │
    │         └───────────┘        │        └───────────┘                     │
    │              │               │               │                          │
    │              │      ┌────────┴────────┐      │                          │
    │              │      │                 │      │                          │
    │              └──────┤   ROBOT BODY    ├──────┘                          │
    │                     │                 │                                 │
    │              ┌──────┤   (PCA9685      ├──────┐                          │
    │              │      │    mounted      │      │                          │
    │              │      │    inside)      │      │                          │
    │              │      └────────┬────────┘      │                          │
    │         ┌───────────┐        │        ┌───────────┐                     │
    │         │ CH9: Shoulder      │        │ CH6: Shoulder                   │
    │         │ CH10: Elbow        │        │ CH7: Elbow                      │
    │         │ CH11: Wrist        │        │ CH8: Wrist                      │
    │         └───────────┘        │        └───────────┘                     │
    │         REAR LEFT (RL)       │       REAR RIGHT (RR)                    │
    │                              │                                          │
    │                              ▼                                          │
    │                            REAR                                         │
    │                                                                         │
    └─────────────────────────────────────────────────────────────────────────┘
```

### Channel Assignment Table

| Channel | Leg | Joint | Servo Wire Colors |
|---------|-----|-------|-------------------|
| **CH0** | Front Right (FR) | Shoulder | Brown=GND, Red=6V, Orange=Signal |
| **CH1** | Front Right (FR) | Elbow | Brown=GND, Red=6V, Orange=Signal |
| **CH2** | Front Right (FR) | Wrist | Brown=GND, Red=6V, Orange=Signal |
| **CH3** | Front Left (FL) | Shoulder | Brown=GND, Red=6V, Orange=Signal |
| **CH4** | Front Left (FL) | Elbow | Brown=GND, Red=6V, Orange=Signal |
| **CH5** | Front Left (FL) | Wrist | Brown=GND, Red=6V, Orange=Signal |
| **CH6** | Rear Right (RR) | Shoulder | Brown=GND, Red=6V, Orange=Signal |
| **CH7** | Rear Right (RR) | Elbow | Brown=GND, Red=6V, Orange=Signal |
| **CH8** | Rear Right (RR) | Wrist | Brown=GND, Red=6V, Orange=Signal |
| **CH9** | Rear Left (RL) | Shoulder | Brown=GND, Red=6V, Orange=Signal |
| **CH10** | Rear Left (RL) | Elbow | Brown=GND, Red=6V, Orange=Signal |
| **CH11** | Rear Left (RL) | Wrist | Brown=GND, Red=6V, Orange=Signal |
| **CH12-15** | - | Unused | - |

### Complete PCA9685 Wiring Diagram

```
                                    ┌─────────────────┐
                                    │  12V BATTERY    │
                                    └────────┬────────┘
                                             │
                        ┌────────────────────┼────────────────────┐
                        │                    │                    │
                        ▼                    ▼                    │
               ┌─────────────────┐  ┌─────────────────┐           │
               │    XL4016       │  │    LM2596S      │           │
               │   12V → 6V      │  │   12V → 5V      │           │
               │    (8A MAX)     │  │    (3A MAX)     │           │
               └────────┬────────┘  └────────┬────────┘           │
                        │                    │                    │
                        │ 6V                 │ 5V                 │
                        │                    │                    │
    ┌───────────────────┼────────────────────┼────────────────────┘
    │                   │                    │
    │                   │                    ▼
    │                   │           ┌─────────────────┐
    │                   │           │    PICO W       │
    │                   │           │    RP2040       │
    │                   │           │                 │
    │                   │           │  VSYS ◄── 5V    │
    │                   │           │  GP4 ──► SDA    │
    │                   │           │  GP5 ──► SCL    │
    │                   │           │  GND ──► GND    │
    │                   │           └────────┬────────┘
    │                   │                    │
    │                   │                    │ I2C Bus
    │                   │                    │
    │                   ▼                    ▼
    │   ┌───────────────────────────────────────────────────────────────┐
    │   │                         PCA9685                               │
    │   │  ┌─────────────────────────────────────────────────────────┐  │
    │   │  │                                                         │  │
    │   │  │   CONTROL PINS              SERVO OUTPUT HEADERS        │  │
    │   │  │   ┌─────────┐               ┌───────────────────────┐   │  │
    │   │  │   │ VCC ◄───┼── 5V (Pico)   │ CH0  [PWM V+ GND] ────┼───┼──┼──► FR Shoulder
    │   │  │   │ GND ◄───┼── GND         │ CH1  [PWM V+ GND] ────┼───┼──┼──► FR Elbow
    │   │  │   │ SCL ◄───┼── GP5         │ CH2  [PWM V+ GND] ────┼───┼──┼──► FR Wrist
    │   │  │   │ SDA ◄───┼── GP4         │ CH3  [PWM V+ GND] ────┼───┼──┼──► FL Shoulder
    │   │  │   │ OE  ◄───┼── GND         │ CH4  [PWM V+ GND] ────┼───┼──┼──► FL Elbow
    │   │  │   └─────────┘               │ CH5  [PWM V+ GND] ────┼───┼──┼──► FL Wrist
    │   │  │                             │ CH6  [PWM V+ GND] ────┼───┼──┼──► RR Shoulder
    │   │  │   POWER TERMINAL            │ CH7  [PWM V+ GND] ────┼───┼──┼──► RR Elbow
    │   │  │   ┌─────────┐               │ CH8  [PWM V+ GND] ────┼───┼──┼──► RR Wrist
    │   │  │   │ V+ ◄────┼── 6V (XL4016) │ CH9  [PWM V+ GND] ────┼───┼──┼──► RL Shoulder
    │   │  │   │ GND ◄───┼── GND         │ CH10 [PWM V+ GND] ────┼───┼──┼──► RL Elbow
    │   │  │   └─────────┘               │ CH11 [PWM V+ GND] ────┼───┼──┼──► RL Wrist
    │   │  │        ▲                    │ CH12 [PWM V+ GND]     │   │  │
    │   │  │        │                    │ CH13 [PWM V+ GND]     │   │  │
    │   │  │        │                    │ CH14 [PWM V+ GND]     │   │  │
    │   │  │        │                    │ CH15 [PWM V+ GND]     │   │  │
    │   │  │        │                    └───────────────────────┘   │  │
    │   │  └────────┼────────────────────────────────────────────────┘  │
    │   └───────────┼───────────────────────────────────────────────────┘
    │               │
    └───────────────┘
         6V from XL4016 powers all servo V+ pins through the V+ terminal
```

### MG996R Servo Specifications

| Parameter | Value |
|-----------|-------|
| Operating Voltage | 4.8V - 6V (use 6V for full torque) |
| Stall Torque | 9.4 kg·cm (at 4.8V), 11 kg·cm (at 6V) |
| Stall Current | ~900mA (at 6V) |
| Operating Current | 500-900mA (under load) |
| Idle Current | ~10mA |
| Rotation Range | 0° - 180° |
| PWM Signal | 500µs (0°) to 2500µs (180°) |

### Power Budget for 12 Servos

```
    ┌────────────────────────────────────────────────────────────────┐
    │                    SERVO POWER CALCULATION                     │
    │                                                                │
    │   Scenario              Current per Servo    Total (12 servos) │
    │   ────────────────────────────────────────────────────────────│
    │   Idle (standing)       ~10mA                ~120mA           │
    │   Light movement        ~200mA               ~2.4A            │
    │   Normal walking        ~400mA               ~4.8A            │
    │   Heavy load/stall      ~900mA               ~10.8A           │
    │                                                                │
    │   ⚠️ XL4016 rated for 8A continuous                           │
    │   ⚠️ Avoid stalling multiple servos simultaneously!           │
    │   ✅ Normal operation stays under 5A                          │
    │                                                                │
    └────────────────────────────────────────────────────────────────┘
```

### Troubleshooting PCA9685

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| No I2C detection | Wrong address | Check address jumpers, try 0x40-0x7F scan |
| Servos don't move | No V+ power | Check XL4016 output, verify 6V on terminal |
| Servos jitter | Insufficient power | Use thicker wires, add capacitor |
| Only some servos work | Bad connection | Check individual channel connections |
| Servos move erratically | GND not common | Connect all grounds together |
| PCA9685 gets hot | V+ on VCC pin | V+ is for servos only, VCC is logic (5V) |

### Wiring Checklist

```
    ☐ VCC connected to 5V (from Pico VSYS or LM2596S)
    ☐ V+ connected to 6V (from XL4016 step-down) 
    ☐ GND connected to common ground
    ☐ SDA connected to Pico GP4 (Pin 6)
    ☐ SCL connected to Pico GP5 (Pin 7)
    ☐ OE connected to GND (or left floating)
    ☐ All 12 servos plugged into CH0-CH11
    ☐ Servo wire orientation correct (Signal-Power-Ground)
    ☐ XL4016 adjusted to output exactly 6.0V
    ☐ Thick wires (18-20 AWG) used for V+ power lines
```

### 2. Ultrasonic Sensors (HC-SR04 x2)

```
    ULTRASONIC SENSOR 1 (LEFT)              ULTRASONIC SENSOR 2 (RIGHT)
    ┌──────────────────────┐                 ┌──────────────────────┐
    │       HC-SR04        │                 │       HC-SR04        │
    │  ┌────┬────┬────┬────┤                 │  ┌────┬────┬────┬────┤
    │  │VCC │TRIG│ECHO│GND │                 │  │VCC │TRIG│ECHO│GND │
    │  └──┬─┴──┬─┴──┬─┴──┬─┘                 │  └──┬─┴──┬─┴──┬─┴──┬─┘
    └─────│────│────│────│──┘                └─────│────│────│────│──┘
          │    │    │    │                         │    │    │    │
          │    │    │    │                         │    │    │    │
          ▼    ▼    ▼    ▼                         ▼    ▼    ▼    ▼
         5V   GP6  GP7  GND                       5V   GP8  GP9  GND
         │    │    │    │                         │    │    │    │
         │    │    │    │                         │    │    │    │
    ┌────┴────┴────┴────┴─────────────────────────┴────┴────┴────┴────┐
    │                          PICO W                                  │
    │  VSYS (5V)  GP6   GP7   GND                GP8   GP9            │
    └─────────────────────────────────────────────────────────────────┘

    ⚠️ IMPORTANT: HC-SR04 ECHO outputs 5V but Pico GPIO is 3.3V tolerant!
       Option 1: Use voltage divider (2.2kΩ + 3.3kΩ) on ECHO line
       Option 2: Use HC-SR04P (3.3V compatible version)
       Option 3: Use level shifter module
```

**Voltage Divider for ECHO (Recommended):**
```
    HC-SR04 ECHO ────┬──── 2.2kΩ ────┬──── Pico GPIO (GP7 or GP9)
                     │               │
                   3.3kΩ            GND
                     │
                    GND

    Calculation: 5V × (3.3kΩ / (2.2kΩ + 3.3kΩ)) = 3.0V (safe for Pico)
```

### 3. H-Bridge L298N (DC Motor for Tail)

```
    ┌─────────────────────────────────────────────────────┐
    │                    L298N MODULE                      │
    │  ┌─────────────────────────────────────────────┐    │
    │  │                                             │    │
    │  │  POWER SECTION        CONTROL SECTION       │    │
    │  │  ┌─────────────┐      ┌─────────────────┐   │    │
    │  │  │ +12V ───────┼──────┤ Battery 12V     │   │    │
    │  │  │ GND  ───────┼──────┤ Common GND      │   │    │
    │  │  │ +5V  ───────┼──────┤ (Output, can    │   │    │
    │  │  │      (OUT)  │      │  power logic)   │   │    │
    │  │  └─────────────┘      └─────────────────┘   │    │
    │  │                                             │    │
    │  │  LOGIC INPUTS         MOTOR OUTPUTS         │    │
    │  │  ┌─────────────┐      ┌─────────────────┐   │    │
    │  │  │ IN1 ────────┼──GP10│ OUT1 ──┐        │   │    │
    │  │  │ IN2 ────────┼──GP11│ OUT2 ──┼─ DC    │   │    │
    │  │  │ IN3 ────────┼──(NC)│ OUT3 ──┘ Motor  │   │    │
    │  │  │ IN4 ────────┼──(NC)│ OUT4 ── (Tail)  │   │    │
    │  │  │ ENA ────────┼──5V  │                 │   │    │
    │  │  │ ENB ────────┼──(NC)│                 │   │    │
    │  │  └─────────────┘      └─────────────────┘   │    │
    │  └─────────────────────────────────────────────┘    │
    └─────────────────────────────────────────────────────┘

    Pico W Connections:
    - GP10 → L298N IN1
    - GP11 → L298N IN2
    - GND  → L298N GND (common ground with Pico)
    - ENA jumper: Keep in place (always enabled) or connect to PWM for speed control

    Motor Direction Truth Table:
    ┌──────┬──────┬────────────┐
    │ IN1  │ IN2  │ Motor      │
    ├──────┼──────┼────────────┤
    │ LOW  │ LOW  │ STOP       │
    │ HIGH │ LOW  │ FORWARD    │
    │ LOW  │ HIGH │ BACKWARD   │
    │ HIGH │ HIGH │ BRAKE      │
    └──────┴──────┴────────────┘
```

### 4. LCD 16x2 I2C Display

```
    ┌────────────────────────────────────────┐
    │           LCD 16x2 with I2C            │
    │           Backpack (PCF8574)           │
    │  ┌──────────────────────────────────┐  │
    │  │   ┌──────────────────────────┐   │  │
    │  │   │ 16 Character x 2 Line    │   │  │
    │  │   │      LCD DISPLAY         │   │  │
    │  │   └──────────────────────────┘   │  │
    │  │                                  │  │
    │  │   I2C Backpack Pins:             │  │
    │  │   ┌────┬────┬────┬────┐          │  │
    │  │   │GND │VCC │SDA │SCL │          │  │
    │  │   └─┬──┴─┬──┴─┬──┴─┬──┘          │  │
    │  └─────│────│────│────│─────────────┘  │
    └────────│────│────│────│────────────────┘
             │    │    │    │
             ▼    ▼    ▼    ▼
            GND   5V  GP4  GP5
             │    │    │    │
    ┌────────┴────┴────┴────┴────────────────┐
    │              PICO W                     │
    │   GND    VSYS   GP4    GP5             │
    │                (SDA)  (SCL)            │
    └─────────────────────────────────────────┘

    I2C Address: 0x27 (most common) or 0x3F
    
    Contrast Adjustment:
    - Blue potentiometer on I2C backpack
    - Turn to adjust text visibility
```

### 5. Smart IMU - Arduino Nano RP2040 Connect

The Arduino Nano RP2040 Connect acts as a "Smart IMU" - it reads its onboard LSM6DSOX 
6-axis IMU (accelerometer + gyroscope) and sends processed orientation data to the 
Pico W via UART. This offloads sensor fusion calculations from the main controller.

```
    ┌─────────────────────────────────────────────────────────────────────────┐
    │                    ARDUINO NANO RP2040 CONNECT                          │
    │                         (Smart IMU Module)                              │
    │                                                                         │
    │    ┌─────────────────────────────────────────────────────────────────┐  │
    │    │                      USB-C PORT                                 │  │
    │    │                    ┌──────────┐                                 │  │
    │    │            ┌───────┴──────────┴───────┐                         │  │
    │    │     D13 ──┤ 1                      30 ├── D12                   │  │
    │    │    3.3V ──┤ 2                      29 ├── D11                   │  │
    │    │    AREF ──┤ 3                      28 ├── D10                   │  │
    │    │   A0/14 ──┤ 4                      27 ├── D9                    │  │
    │    │   A1/15 ──┤ 5                      26 ├── D8                    │  │
    │    │   A2/16 ──┤ 6                      25 ├── D7                    │  │
    │    │   A3/17 ──┤ 7                      24 ├── D6                    │  │
    │    │   A4/18 ──┤ 8    NANO RP2040       23 ├── D5                    │  │
    │    │   A5/19 ──┤ 9      CONNECT         22 ├── D4                    │  │
    │    │   A6/20 ──┤ 10                     21 ├── D3                    │  │
    │    │   A7/21 ──┤ 11                     20 ├── D2                    │  │
    │    │    VUSB ──┤ 12   [LSM6DSOX IMU]    19 ├── GND                   │  │
    │    │     RST ──┤ 13   [WiFi/BLE u-blox] 18 ├── RST                   │  │
    │    │     GND ──┤ 14   [Microphone]      17 ├── RX/GPIO1 ◄── Pico TX  │  │
    │    │     VIN ──┤ 15                     16 ├── TX/GPIO0 ──► Pico RX  │  │
    │    │            └──────────────────────────┘                         │  │
    │    └─────────────────────────────────────────────────────────────────┘  │
    │                                                                         │
    │    ONBOARD SENSORS:                                                     │
    │    • LSM6DSOX - 6-axis IMU (Accel + Gyro)                              │
    │    • Microphone (PDM)                                                   │
    │    • u-blox NINA-W102 (WiFi + BLE)                                     │
    │                                                                         │
    └─────────────────────────────────────────────────────────────────────────┘
```

**UART Wiring Between Pico W and Nano RP2040 Connect:**

```
    PICO W (Main Controller)              NANO RP2040 CONNECT (Smart IMU)
    ┌──────────────────────┐              ┌──────────────────────┐
    │                      │              │                      │
    │  GP16 (UART0 TX) ────┼──────────────┼──► RX (GPIO1/Pin 17) │
    │  Pin 22              │              │                      │
    │                      │              │                      │
    │  GP17 (UART0 RX) ◄───┼──────────────┼──── TX (GPIO0/Pin 16)│
    │  Pin 23              │              │                      │
    │                      │              │                      │
    │  GND ────────────────┼──────────────┼──── GND (Pin 14/19)  │
    │  Pin 18/23           │              │                      │
    │                      │              │                      │
    └──────────────────────┘              └──────────────────────┘

    UART Configuration:
    ┌────────────────────────────────────────────────────────────┐
    │  Baud Rate:   115200                                       │
    │  Data Bits:   8                                            │
    │  Stop Bits:   1                                            │
    │  Parity:      None                                         │
    │  Flow Ctrl:   None                                         │
    └────────────────────────────────────────────────────────────┘

    ⚠️ IMPORTANT: Both boards are 3.3V logic - NO level shifter needed!
    ⚠️ Connect GND between both boards (common ground required)
```

**Smart IMU Data Protocol (Example):**

```
    Nano RP2040 sends processed IMU data at ~100Hz:
    
    Format: "IMU,<roll>,<pitch>,<yaw>,<ax>,<ay>,<az>\n"
    
    Example: "IMU,2.35,-1.20,45.67,0.02,-0.01,9.81\n"
    
    Fields:
    ┌─────────┬────────────────────────────────────────────┐
    │ roll    │ Roll angle in degrees (-180 to +180)       │
    │ pitch   │ Pitch angle in degrees (-90 to +90)        │
    │ yaw     │ Yaw angle in degrees (0 to 360)            │
    │ ax      │ Linear acceleration X (m/s²)               │
    │ ay      │ Linear acceleration Y (m/s²)               │
    │ az      │ Linear acceleration Z (m/s²)               │
    └─────────┴────────────────────────────────────────────┘
```

**Power Options for Nano RP2040 Connect:**

```
    Option 1: USB Power (for development)
    ┌────────────────────────────────────────────┐
    │  USB-C ──► Nano RP2040                     │
    │  (Separate USB cable from Pico)            │
    └────────────────────────────────────────────┘

    Option 2: VIN Pin (recommended for deployment)
    ┌────────────────────────────────────────────┐
    │  5V from LM2596S ──► VIN (Pin 15)          │
    │  GND ──► GND (Pin 14)                      │
    │                                            │
    │  Note: VIN accepts 5V-21V input            │
    │        Internal regulator provides 3.3V    │
    └────────────────────────────────────────────┘

    Option 3: 3.3V Direct (if already regulated)
    ┌────────────────────────────────────────────┐
    │  3.3V ──► 3.3V (Pin 2)                     │
    │  GND ──► GND (Pin 14)                      │
    │                                            │
    │  ⚠️ Bypasses regulator - use with care!   │
    └────────────────────────────────────────────┘
```

**Complete Smart IMU Wiring Diagram:**

```
                    ┌─────────────────────────────────────────────────────┐
                    │              SMART IMU SUBSYSTEM                    │
                    │                                                     │
    From LM2596S    │    ┌────────────────────────────────────┐          │
    (5V)            │    │    ARDUINO NANO RP2040 CONNECT     │          │
        │           │    │                                    │          │
        │           │    │  VIN ◄────────────────────┐        │          │
        └──────────────►─┼──(Pin 15)                 │        │          │
                    │    │                           │        │          │
    From Pico W     │    │  GND ◄────────────────────┼────────┼──► GND   │
    (GND)           │    │  (Pin 14)                 │        │  (Common)│
        │           │    │                           │        │          │
        └──────────────►─┼───────────────────────────┘        │          │
                    │    │                                    │          │
    From Pico W     │    │  RX (GPIO1, Pin 17) ◄─────────────────────────┼──┐
    GP16 (TX)       │    │                                    │          │  │
        └──────────────►─┼────────────────────────────────────┼──────────┘  │
                    │    │                                    │             │
    To Pico W       │    │  TX (GPIO0, Pin 16) ──────────────────────────┼──┤
    GP17 (RX)       │    │                                    │          │  │
        ◄────────────────┼────────────────────────────────────┼──────────┘  │
                    │    │                                    │             │
                    │    │  ┌────────────────────────┐        │             │
                    │    │  │    ONBOARD IMU         │        │             │
                    │    │  │    LSM6DSOX            │        │             │
                    │    │  │  - 3-axis Accel        │        │             │
                    │    │  │  - 3-axis Gyro         │        │             │
                    │    │  │  - ±2/4/8/16g          │        │             │
                    │    │  │  - ±125 to ±2000 dps   │        │             │
                    │    │  └────────────────────────┘        │             │
                    │    │                                    │             │
                    │    └────────────────────────────────────┘             │
                    └───────────────────────────────────────────────────────┘
```

---

## Complete System Wiring Diagram

```
                                    ┌─────────────────┐
                                    │  12V BATTERY    │
                                    │   7000mAh       │
                                    └────────┬────────┘
                                             │
                                    ┌────────┴────────┐
                                    │  ROCKER SWITCH  │
                                    │     ON/OFF      │
                                    └────────┬────────┘
                                             │
           ┌─────────────────────────────────┼─────────────────────────────────┐
           │                                 │                                 │
           ▼                                 ▼                                 ▼
  ┌─────────────────┐              ┌─────────────────┐              ┌─────────────────┐
  │    XL4016       │              │    LM2596S      │              │     L298N       │
  │   12V → 6V      │              │   12V → 5V      │              │   H-Bridge      │
  │    (8A)         │              │    (3A)         │              │                 │
  └────────┬────────┘              └────────┬────────┘              └────────┬────────┘
           │                                │                                │
           │ 6V                             │ 5V                             │
           │                                │                          ┌─────┴─────┐
           │                    ┌───────────┼───────────┐              │           │
           │                    │           │           │              │  DC MOTOR │
           ▼                    ▼           ▼           ▼              │  (TAIL)   │
  ┌─────────────────┐   ┌───────────┐ ┌───────────┐ ┌───────────┐     └───────────┘
  │    PCA9685      │   │  PICO W   │ │  LCD      │ │HC-SR04 x2 │
  │  Servo Driver   │   │  RP2040   │ │  16x2     │ │Ultrasonic │
  │  (V+ = 6V)      │   │           │ │           │ │           │
  └────────┬────────┘   └─────┬─────┘ └─────┬─────┘ └─────┬─────┘
           │                  │             │             │
           │                  │             │             │
    ┌──────┴──────┐           │             │             │
    │             │           │             │             │
    ▼             ▼           │             │             │
┌───────┐    ┌───────┐        │             │             │
│SERVOS │    │SERVOS │        │             │             │
│CH0-5  │    │CH6-11 │        │             │             │
│(Front)│    │(Rear) │        │             │             │
└───────┘    └───────┘        │             │             │
                              │             │             │
                              └──────┬──────┴──────┬──────┘
                                     │             │
                                     │    I2C      │
                                     │   (GP4,5)   │
                                     │             │
                              ┌──────┴─────────────┴──────┐
                              │      SHARED I2C BUS       │
                              │  PCA9685 (0x40)           │
                              │  LCD     (0x27)           │
                              └───────────────────────────┘


                    ┌─────────────────────────────────────┐
                    │         RASPBERRY PI PICO W         │
                    │              (RP2040)               │
                    └─────────────────────────────────────┘
                              PIN CONNECTIONS:

    ┌─────────────────────────────────────────────────────────────────────┐
    │                                                                     │
    │   POWER:           I2C BUS:          ULTRASONIC:      H-BRIDGE:    │
    │   ┌─────┐          ┌─────┐           ┌─────┐          ┌─────┐      │
    │   │VSYS │←─ 5V     │GP4  │──SDA      │GP6  │──TRIG1   │GP10 │──IN1 │
    │   │GND  │←─ GND    │GP5  │──SCL      │GP7  │──ECHO1   │GP11 │──IN2 │
    │   │3V3  │─→ (out)  └─────┘           │GP8  │──TRIG2   └─────┘      │
    │   └─────┘                            │GP9  │──ECHO2                │
    │                                      └─────┘                       │
    │                                                                     │
    └─────────────────────────────────────────────────────────────────────┘
```

---

## Wire Color Coding Standard

| Color | Function |
|-------|----------|
| 🔴 Red | +5V / +6V / +12V (Power) |
| ⚫ Black | GND (Ground) |
| 🔵 Blue | I2C SDA (Data) |
| 🟡 Yellow | I2C SCL (Clock) |
| 🟠 Orange | Ultrasonic TRIG |
| 🟢 Green | Ultrasonic ECHO |
| 🟣 Purple | Motor Control IN1 |
| ⚪ Gray | Motor Control IN2 |
| 🟤 Brown | Servo Signal |

---

## Component Checklist

| Component | Qty | Voltage | Current (Max) | Verified |
|-----------|-----|---------|---------------|----------|
| Pico W RP2040 | 1 | 5V (VSYS) | 500mA | ☐ |
| PCA9685 | 1 | 5V logic, 6V servo | 25mA + servos | ☐ |
| MG996R Servos | 12 | 4.8-6V | 500mA each stall | ☐ |
| HC-SR04 | 2 | 5V | 15mA each | ☐ |
| LCD 16x2 I2C | 1 | 5V | 20mA | ☐ |
| L298N H-Bridge | 1 | 5-35V | 2A per channel | ☐ |
| DC Motor (Tail) | 1 | 6V | 200mA | ☐ |
| LM2596S (5V) | 1 | 12V in | 3A max | ☐ |
| XL4016 (6V) | 1 | 12V in | 8A max | ☐ |
| 12V Battery | 1 | 12V | 7000mAh | ☐ |
| Rocker Switch | 1 | 250V | 10A | ☐ |

---

## Important Notes

### ⚠️ Voltage Level Warnings

1. **HC-SR04 ECHO Pin**: Outputs 5V, but Pico GPIO is 3.3V!
   - Use voltage divider (2.2kΩ + 3.3kΩ)
   - Or use HC-SR04P (3.3V version)

2. **Servo Power**: 
   - MG996R can draw 500mA+ per servo at stall
   - 12 servos = up to 6A!
   - Use XL4016 (8A capable) for servo power

3. **Common Ground**: 
   - ALL grounds must be connected together
   - Battery GND = Pico GND = PCA9685 GND = L298N GND

### 🔌 I2C Bus Configuration

```c
// I2C Addresses on the bus:
#define PCA9685_ADDRESS  0x40  // Servo driver
#define LCD_I2C_ADDRESS  0x27  // LCD (or 0x3F)

// I2C Pins (shared):
#define I2C_SDA_PIN  4  // GP4
#define I2C_SCL_PIN  5  // GP5
```

### 🔧 Testing Sequence

1. **Power Test**: Verify 5V and 6V rails before connecting components
2. **I2C Scan**: Run I2C scanner to verify PCA9685 and LCD addresses
3. **Single Servo**: Test one servo before connecting all 12
4. **Ultrasonic**: Test each HC-SR04 independently
5. **Motor**: Test L298N with motor before full integration
6. **LCD**: Display test patterns
7. **Integration**: Connect all components

---

## Code Pin Definitions

```c
// ============================================
// SpotMicro Pin Definitions for RP2040-W
// ============================================

// I2C Bus (Shared by PCA9685 and LCD)
#define I2C_PORT        i2c0
#define I2C_SDA_PIN     4       // GP4, Physical Pin 6
#define I2C_SCL_PIN     5       // GP5, Physical Pin 7

// Ultrasonic Sensor 1 (Left)
#define US1_TRIG_PIN    6       // GP6, Physical Pin 9
#define US1_ECHO_PIN    7       // GP7, Physical Pin 10

// Ultrasonic Sensor 2 (Right)
#define US2_TRIG_PIN    8       // GP8, Physical Pin 11
#define US2_ECHO_PIN    9       // GP9, Physical Pin 12

// H-Bridge Motor Control (Tail DC Motor)
#define MOTOR_IN1_PIN   10      // GP10, Physical Pin 14
#define MOTOR_IN2_PIN   11      // GP11, Physical Pin 15

// Smart IMU UART (Nano RP2040 Connect)
#define IMU_UART_PORT   uart0
#define IMU_UART_TX_PIN 16      // GP16, Physical Pin 22 (UART0)
#define IMU_UART_RX_PIN 17      // GP17, Physical Pin 23
#define IMU_UART_BAUD   115200

// I2C Device Addresses
#define PCA9685_ADDR    0x40    // Servo Driver
#define LCD_ADDR        0x27    // LCD Display (or 0x3F)

// Servo Channel Mapping (on PCA9685)
// Front Right Leg
#define FR_SHOULDER     3
#define FR_ELBOW        1
#define FR_WRIST        4
// Front Left Leg
#define FL_SHOULDER     2
#define FL_ELBOW        0
#define FL_WRIST        5
// Rear Right Leg
#define RR_SHOULDER     8
#define RR_ELBOW        10
#define RR_WRIST        7
// Rear Left Leg
#define RL_SHOULDER     9
#define RL_ELBOW        11
#define RL_WRIST        6
```

---

## Physical Pin Quick Reference Card

```
    ╔═══════════════════════════════════════════════════════════════════════╗
    ║                   PICO W - SPOTMICRO WIRING                           ║
    ╠═══════════════════════════════════════════════════════════════════════╣
    ║  Pin 6  (GP4)  ──── I2C SDA ──── PCA9685 + LCD                       ║
    ║  Pin 7  (GP5)  ──── I2C SCL ──── PCA9685 + LCD                       ║
    ║  Pin 9  (GP6)  ──── TRIG ─────── Ultrasonic #1 (Left)               ║
    ║  Pin 10 (GP7)  ──── ECHO ─────── Ultrasonic #1 (Left)               ║
    ║  Pin 11 (GP8)  ──── TRIG ─────── Ultrasonic #2 (Right)              ║
    ║  Pin 12 (GP9)  ──── ECHO ─────── Ultrasonic #2 (Right)              ║
    ║  Pin 14 (GP10) ──── IN1 ──────── L298N Motor Driver                  ║
    ║  Pin 15 (GP11) ──── IN2 ──────── L298N Motor Driver                  ║
    ║  Pin 22 (GP16) ──── UART0 TX ──── Nano GPIO1/RX (Smart IMU)         ║
    ║  Pin 23 (GP17) ──── UART RX ──── Nano RP2040 TX (Smart IMU)          ║
    ║  Pin 39 (VSYS) ──── 5V ───────── From LM2596S                        ║
    ║  Pin 3,8,13... ──── GND ──────── Common Ground                       ║
    ╚═══════════════════════════════════════════════════════════════════════╝

    ╔═══════════════════════════════════════════════════════════════════════╗
    ║                NANO RP2040 CONNECT - SMART IMU                        ║
    ╠═══════════════════════════════════════════════════════════════════════╣
    ║  Pin 15 (VIN)    ──── 5V ──────── From LM2596S                       ║
    ║  Pin 14 (GND)    ──── GND ─────── Common Ground                      ║
    ║  Pin 16 (TX/D1)  ──── TX ──────── To Pico GP17 (RX)                  ║
    ║  Pin 17 (RX/D0)  ──── RX ──────── From Pico GP16 (UART0_TX)          ║
    ╚═══════════════════════════════════════════════════════════════════════╝
```

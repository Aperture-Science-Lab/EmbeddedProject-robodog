# Arduino Nano RP2040 Connect - Wiring Documentation

## Complete Pin Reference

| Component          | Pin(s)      | Wire Color | Notes                              |
|--------------------|-------------|------------|------------------------------------|
| **IMU (LSM6DSOX)** | Onboard     | -          | Internal I2C, no wiring needed     |
| **LDR Module**     | A0          | -          | Digital OUT → A0                   |
| **Green LED**      | A1          | -          | 220Ω resistor in series            |
| **Red LED**        | A2          | -          | 220Ω resistor in series            |
| **IR Front**       | D2          | -          | LOW = Object detected → RGB GREEN  |
| **IR Back**        | D3          | -          | LOW = Object detected → RGB RED    |
| **PIR Front**      | D4          | -          | HIGH = Motion detected             |
| **PIR Back**       | D5          | -          | HIGH = Motion detected             |
| **RGB LED (R)**    | D6          | Red wire   | HW-479 Common cathode              |
| **RGB LED (G)**    | D7          | Green wire | HW-479 Common cathode              |
| **RGB LED (B)**    | D10         | Blue wire  | HW-479 Common cathode              |
| **RGB LED (GND)**  | GND         | Black      | Connect to Ground                  |
| **GPS TX**         | D8          | -          | GPS sends data → Arduino receives  |
| **GPS RX**         | D9          | -          | Arduino sends config → GPS         |
| **Serial1 TX**     | D1          | -          | To main controller RX              |
| **Serial1 RX**     | D0          | -          | To main controller TX              |

---

## RGB LED Behavior (HW-479)

| IR Front | IR Back | RGB Color |
|----------|---------|-----------|
| clear    | clear   | **OFF**   |
| DETECTED | clear   | **GREEN** |
| clear    | DETECTED| **RED**   |
| DETECTED | DETECTED| **BLUE**  |

---

## Wiring Diagrams

### HW-479 RGB LED (D6/D7/D10)
```
HW-479 RGB LED     Arduino Nano RP2040
┌─────────┐        ┌──────────────────┐
│   R     │────────│ D6               │
│   G     │────────│ D7               │
│   B     │────────│ D10              │
│  GND/-  │────────│ GND              │
└─────────┘        └──────────────────┘
```

### NEO-6M GPS Module (D8/D9)
```
NEO-6M GPS         Arduino Nano RP2040
┌─────────┐        ┌──────────────────┐
│   VCC   │────────│ 3.3V             │
│   GND   │────────│ GND              │
│   TX    │────────│ D8  (GPS TX → Arduino RX)
│   RX    │────────│ D9  (Arduino TX → GPS RX)
└─────────┘        └──────────────────┘
```

### IR Sensors (D2 & D3)
```
Front IR (D2)      Arduino Nano RP2040
┌─────────┐        ┌──────────────────┐
│   VCC   │────────│ 3.3V             │
│   GND   │────────│ GND              │
│   OUT   │────────│ D2               │
└─────────┘        └──────────────────┘

Back IR (D3)       Arduino Nano RP2040
┌─────────┐        ┌──────────────────┐
│   VCC   │────────│ 3.3V             │
│   GND   │────────│ GND              │
│   OUT   │────────│ D3               │
└─────────┘        └──────────────────┘
```

### PIR Sensors (D4 & D5)
```
Front PIR (D4)     Arduino Nano RP2040
┌─────────┐        ┌──────────────────┐
│   VCC   │────────│ VIN (5V)         │
│   GND   │────────│ GND              │
│   OUT   │────────│ D4               │
└─────────┘        └──────────────────┘

Back PIR (D5)      Arduino Nano RP2040
┌─────────┐        ┌──────────────────┐
│   VCC   │────────│ VIN (5V)         │
│   GND   │────────│ GND              │
│   OUT   │────────│ D5               │
└─────────┘        └──────────────────┘
```

### LDR + Security LEDs (A0/A1/A2)
```
LDR Module         Arduino Nano RP2040
┌─────────┐        ┌──────────────────┐
│   VCC   │────────│ 3.3V             │
│   GND   │────────│ GND              │
│   DO    │────────│ A0               │
└─────────┘        └──────────────────┘

Security LEDs:
  A1 ──[220Ω]──▶│── GND  (Green LED)
  A2 ──[220Ω]──▶│── GND  (Red LED)
```

---

## Serial Output Formats

**IMU Data (50Hz):**
```
IMU,roll,pitch,yaw,ax,ay,az,gx,gy,gz
```

**GPS Data (1Hz):**
```
GPS,lat,lng,alt,sats          (when valid)
GPS,NO_FIX,Sats:0,Time:12:30  (when searching)
```

**Sensor Status (2Hz):**
```
SENSORS | IR_Front:DETECTED | IR_Back:clear | PIR_Front:MOTION | PIR_Back:clear | LDR:DARK
```

---

## Startup RGB Test

At power-on, the RGB LED will cycle through:
1. **RED** (0.5s)
2. **GREEN** (0.5s)  
3. **BLUE** (0.5s)
4. **OFF**

If you don't see this sequence, check your RGB wiring!

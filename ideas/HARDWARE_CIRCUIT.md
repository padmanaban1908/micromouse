# ⚡ Hardware Circuit & Pinout Architecture: Micromouse (ESP32)

This document provides the exact pinout mappings, circuit architecture, and sensor connections for the **ESP32 Autonomous Micromouse Robot**.

---

## 🔌 1. Exact ESP32 Pin Assignment Table

| Subsystem | Signal Name | ESP32 GPIO Pin | Type / Electrical Notes |
|---|---|---|---|
| **Motors (Left)** | `PWMA` | `GPIO 27` | High-frequency PWM (20 kHz, LEDC Channel 0) |
| | `AIN1` | `GPIO 14` | Digital Output (H-Bridge direction) |
| | `AIN2` | `GPIO 26` | Digital Output (H-Bridge direction) |
| **Motors (Right)** | `PWMB` | `GPIO 32` | High-frequency PWM (20 kHz, LEDC Channel 1) |
| | `BIN1` | `GPIO 25` | Digital Output (H-Bridge direction) |
| | `BIN2` | `GPIO 33` | Digital Output (H-Bridge direction) |
| **Encoders (Left)** | `ENC_L_A` | `GPIO 36` (VP) | Input-only (GPI) + external $4.7\text{ k}\Omega$ pull-up to 3.3V |
| | `ENC_L_B` | `GPIO 39` (VN) | Input-only (GPI) + external $4.7\text{ k}\Omega$ pull-up to 3.3V |
| **Encoders (Right)**| `ENC_R_A` | `GPIO 34` | Input-only (GPI) + external $4.7\text{ k}\Omega$ pull-up to 3.3V |
| | `ENC_R_B` | `GPIO 35` | Input-only (GPI) + external $4.7\text{ k}\Omega$ pull-up to 3.3V |
| **I2C Bus** | `I2C_SDA` | `GPIO 21` | Hardware I2C Data (400 kHz Fast-Mode) |
| | `I2C_SCL` | `GPIO 22` | Hardware I2C Clock (400 kHz Fast-Mode) |
| **ToF Sensors** | `XSHUT_1` (Left 90°) | `GPIO 23` | Dynamic address allocation $\to$ `0x30` |
| | `XSHUT_2` (Front-Left 45°) | `GPIO 19` | Dynamic address allocation $\to$ `0x31` |
| | `XSHUT_3` (Front Center 0°) | `GPIO 18` | Dynamic address allocation $\to$ `0x32` |
| | `XSHUT_4` (Front-Right 45°)| `GPIO 17` | Dynamic address allocation $\to$ `0x33` |
| | `XSHUT_5` (Right 90°) | `GPIO 16` | Dynamic address allocation $\to$ `0x34` |
| **IMU (MPU-6050)** | `SDA/SCL` | `GPIO 21, 22` | Direct I2C register access at address `0x68` |
| **Pushbuttons** | `BTN1` | `GPIO 4` | Active LOW: Starts Search Run 1 |
| | `BTN2` | `GPIO 13` | Active LOW: Starts Speed Run 2 (Smooth 90° Turns) |

---

## 📡 2. 5x Time-of-Flight (VL53L0X) Architecture

All VL53L0X sensors boot up on default address `0x29`. To operate 5 sensors simultaneously without bus contention:
1. All 5 XSHUT lines are pulled LOW at boot to keep sensors in hardware standby.
2. The MCU wakes each sensor one by one by setting its corresponding XSHUT pin HIGH.
3. The sensor is initialized at `0x29` and immediately reassigned a unique runtime I2C address (`0x30` through `0x34`).

```
[ESP32]
  GPIO 23 ---> XSHUT (Sensor 0: Left 90°)      ==> I2C Addr: 0x30
  GPIO 19 ---> XSHUT (Sensor 1: Front-Left 45°) ==> I2C Addr: 0x31
  GPIO 18 ---> XSHUT (Sensor 2: Center 0°)     ==> I2C Addr: 0x32
  GPIO 17 ---> XSHUT (Sensor 3: Front-Right 45°)==> I2C Addr: 0x33
  GPIO 16 ---> XSHUT (Sensor 4: Right 90°)     ==> I2C Addr: 0x34
```

---

## 🔄 3. Continuous Smooth 90° Turn Mechanics

### Why Stop-and-Turn Wastes Time:
1. Move $180\text{ mm} \implies$ full stop: $\approx 250\text{ ms}$
2. Pivot in place $90^\circ \implies$ stop: $\approx 350\text{ ms}$
3. Re-accelerate from zero: $\approx 200\text{ ms}$
- **Total Penalty per Corner**: $\approx 800\text{ ms}$!

### Continuous Arc Turn Solution:
- Maintain forward velocity ($v_{\text{turn}} \approx 160\text{ mm/s}$).
- Inner wheel runs slower, outer wheel runs faster:
  $$v_{\text{outer}} = v_{\text{turn}} + \frac{\omega \cdot W}{2}$$
  $$v_{\text{inner}} = v_{\text{turn}} - \frac{\omega \cdot W}{2}$$
- Gyro Z-axis tracks real angular velocity at $500\text{ Hz}$.
- When rotation reaches $87.5^\circ$, the bot smoothly straightens out and immediately locks onto the next corridor!

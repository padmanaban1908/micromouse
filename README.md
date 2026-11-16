# 🐭 Autonomous Micromouse Robot

An intelligent, high-speed autonomous **Micromouse Robot** designed to map, explore, and solve official $16 \times 16$ competition mazes using the **Left-Hand Rule (LSRB)**, **Queue-based Floodfill Algorithm**, active ToF wall-centering PID control, and closed-loop gyro turning.

Supports both **PlatformIO** (VS Code) and the **Arduino IDE** (ESP32 Core 2.x and 3.x).

---

## 📂 Project Organization

```
micromouse/
├── platformio.ini                     # PlatformIO configuration & automated library management
├── src/
│   └── main.cpp                       # Main setup, loop & LSRB state machine
├── include/                           # Modular embedded C++ headers
│   ├── config.h                       # Pin assignments, speeds, odometry & PID gains
│   ├── motors.h                       # TB6612 motor driver (Core 2.x & 3.x dual compatibility)
│   ├── encoders.h                     # 4x Quadrature encoder ISRs & fast register reads
│   ├── imu.h                          # Direct I2C MPU-6050 gyro yaw rate integration
│   ├── tof.h                          # 5x VL53L0X dynamic XSHUT addressing & reading
│   ├── floodfill.h                    # 16x16 maze matrix & queue-based BFS Floodfill solver
│   └── motion.h                       # Smooth 90° arc turns & ToF wall-centering PID
├── code/
│   ├── firmware/                      # Standalone Arduino IDE sketches (.ino)
│   │   ├── 01_forward_sensors.ino     # Stage 1: Forward driving with sensor wall-centering
│   │   ├── 02_turn_90_right.ino       # Stage 2: Precision 90° right turn with gyro feedback
│   │   ├── 03_encoder_calibration.ino # Stage 3: 4x Quadrature encoder CPR calibration tool
│   │   ├── 04_move_1_metre.ino        # Stage 4: 1-metre precision distance test (PID + L/R sync)
│   │   └── micromouse_robot.ino       # Full autonomous Left-Hand Rule & Floodfill solver
│   └── simulation/
│       └── micromouse_simulator.py    # Python 16x16 maze simulator & visualizer
├── ideas/                             # Mathematical modeling, ideas, and hardware guides
│   ├── ALGORITHMS.md                  # Kinematics, odometry, Floodfill math & velocity profiles
│   ├── FLOODFILL_GUIDE.md             # Complete step-by-step Floodfill & wave propagation guide
│   ├── HARDWARE_CIRCUIT.md            # Circuit schematics, exact pinouts & 5x ToF wiring
│   └── PROJECT_IDEAS.md               # Master engineering notebook & multi-run roadmap
├── others/                            # Calibration and competition specifications
│   ├── CALIBRATION_GUIDE.md           # Sensor curve fitting, encoder CPR & PID tuning
│   └── MAZE_SPECS_RULES.md            # Official IEEE & All-Japan contest rules & maze geometry
└── sync_github.py                     # GitHub REST API direct sync tool
```

---

## ⚡ Verified Hardware Specifications

| Peripheral | Assigned Pins | Description |
|---|---|---|
| **Left Motor (TB6612)** | `PWMA = 27`, `AIN1 = 14`, `AIN2 = 26` | Forward: `AIN1 = LOW, AIN2 = HIGH` |
| **Right Motor (TB6612)**| `PWMB = 32`, `BIN1 = 25`, `BIN2 = 33` | Forward: `BIN1 = HIGH, BIN2 = LOW` |
| **Left Encoder** | `A = 34`, `B = 35` | 4.7 kΩ pull-ups to 3.3V, `L_ENC_DIR = 1` |
| **Right Encoder** | `A = 36`, `B = 39` | 4.7 kΩ pull-ups to 3.3V, `R_ENC_DIR = -1` |
| **Encoder Resolution (CPR)** | **2788.0 counts/rev** | Full 4x quadrature decoding |
| **Wheel Diameter** | **33.5 mm** | Odometry: $\approx 0.03775\text{ mm/count}$ |
| **I2C Bus** | `SDA = 21`, `SCL = 22` | 400 kHz Fast-Mode I2C |
| **5x ToF (VL53L0X)** | `XSHUT: 23, 19, 18, 17, 16` | Dynamic addresses `0x30, 0x31, 0x32, 0x33, 0x34` |
| **IMU (MPU-6050)** | Direct I2C @ `0x68` | Fast Z-gyro rate integration |
| **Pushbuttons** | `BTN1 = 13`, `BTN2 = 4` | `BTN1`: Forward / Start / Verify; `BTN2`: Backward / Zero / Reset |

---

## 🚀 Calibration & Test Sketches

1. **`03_encoder_calibration.ino`**:
   - Counts per revolution (CPR) measurement tool.
   - Turn wheels 10 or 20 full revolutions by hand, type the number in the Serial Monitor (115200 baud), and get averaged CPR.
   - Press `BTN1` or type `v` to verify exactly 1 full spin under motor power.

2. **`04_move_1_metre.ino`**:
   - Drives exactly 1000 mm using encoder feedback and closed-loop speed control.
   - Smooth acceleration, cruise ($200\text{ mm/s}$), and deceleration to target.
   - Active Left/Right wheel synchronization ($K_{\text{straight}} = 2.0$) keeps the bot driving perfectly straight.
   - Press `BTN1` to drive Forward 1 m; press `BTN2` to drive Backward 1 m.
   - Any button press during run acts as an **Emergency Stop**.

3. **`micromouse_robot.ino`**:
   - Full autonomous Left-Hand Rule (LSRB) maze solver.
   - Active wall-centering using 5x ToF sensors.
   - Closed-loop 90° pivot turns and 180° dead-end U-turns using MPU-6050 gyro.
   - Press `BTN1` (GPIO 13) to Start/Pause navigation; press `BTN2` (GPIO 4) to zero heading.

---

## 🛠️ Uploading Code

### Option 1: Arduino IDE (ESP32 Core 2.x & 3.x)
1. Open any sketch from `code/firmware/` in **Arduino IDE**.
2. Select Board: **ESP32 Dev Module**.
3. Select Port: (e.g. `COM10`).
4. Click **Upload**.
   - If the ESP32 doesn't auto-reset: hold `BOOT` (IO0), tap `EN` (RST), then release `BOOT`.

### Option 2: PlatformIO (VS Code)
1. Open the project folder `d:\micromouse` in **VS Code**.
2. Click the PlatformIO **Upload** button in the bottom status bar (`Ctrl + Alt + U`).
3. Open Serial Monitor (`Ctrl + Alt + S`) at **115200 baud**.

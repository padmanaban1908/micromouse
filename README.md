# 🐭 Autonomous Micromouse Robot (PlatformIO)

An intelligent, high-speed autonomous **Micromouse Robot** designed to map, explore, and solve official $16 \times 16$ competition mazes using the **Queue-based Floodfill Algorithm**, wall-centering PID control, and **Continuous Smooth 90° Arc Turns**.

Built and configured for **PlatformIO** on the **ESP32**.

---

## 📂 Project Organization

```
micromouse/
├── platformio.ini                     # PlatformIO configuration & automated library management
├── src/
│   └── main.cpp                       # Main setup, loop & state machine
├── include/                           # Modular embedded C++ headers
│   ├── config.h                       # Pin assignments, speeds, odometry & PID gains
│   ├── motors.h                       # TB6612 motor driver (20 kHz ultrasonic PWM)
│   ├── encoders.h                     # Quadrature encoder ISRs & odometry tracking
│   ├── imu.h                          # Direct I2C MPU-6050 gyro yaw rate integration
│   ├── tof.h                          # 5x VL53L0X dynamic XSHUT addressing & reading
│   ├── floodfill.h                    # 16x16 maze matrix & queue-based BFS Floodfill solver
│   └── motion.h                       # Smooth 90° arc turns & ToF wall-centering PID
├── code/
│   ├── firmware/
│   │   └── micromouse_robot.ino       # Standalone Arduino IDE single-file firmware
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

## ⚡ Hardware Pinout Specification

| Peripheral | Assigned Pins | Description |
|---|---|---|
| **Left Motor (TB6612)** | `PWMA = 27`, `AIN1 = 14`, `AIN2 = 26` | 20 kHz PWM (zero motor hum) |
| **Right Motor (TB6612)**| `PWMB = 32`, `BIN1 = 25`, `BIN2 = 33` | 20 kHz PWM |
| **Left Encoder** | `A = 36 (VP)`, `B = 39 (VN)` | External $4.7\text{ k}\Omega$ pull-up to 3.3V, hardware interrupts |
| **Right Encoder** | `A = 34`, `B = 35` | External $4.7\text{ k}\Omega$ pull-up to 3.3V, hardware interrupts |
| **I2C Bus** | `SDA = 21`, `SCL = 22` | 400 kHz Fast-Mode I2C |
| **5x ToF (VL53L0X)** | `XSHUT: 23, 19, 18, 17, 16` | Dynamic addresses `0x30, 0x31, 0x32, 0x33, 0x34` |
| **IMU (MPU-6050)** | Direct I2C @ `0x68` | Fast Z-gyro rate integration |
| **Pushbuttons** | `BTN1 = 4`, `BTN2 = 13` | `BTN1`: Run 1 (Search), `BTN2`: Run 2 (Smooth 90° Speed Run) |

---

## 🔄 Continuous Smooth 90° Arc Turns (Zero-Stop Turning)

Traditional robots lose up to **800 ms per corner** by stopping in the cell center, pivoting 90° in place, stopping again, and accelerating from rest.

This firmware executes continuous circular arc turns:
- **Zero-Stop Curvature**: The robot maintains linear forward velocity ($v_{\text{turn}} \approx 160\text{ mm/s}$).
- **Differential Wheel Drive**:
  $$v_{\text{outer}} = v_{\text{turn}} + 55, \quad v_{\text{inner}} = v_{\text{turn}} - 65$$
- **Closed-Loop Gyro PID**: Samples the MPU-6050 Z-axis at $500\text{ Hz}$, dynamically regulating angular velocity until exactly $87.5^\circ$ is completed.
- **Immediate Re-Engagement**: Straightens out smoothly and instantly locks onto the next corridor with active ToF wall-centering.

---

## 🛠️ How to Build and Upload with PlatformIO

### In VS Code with PlatformIO IDE:
1. Open the folder `d:\micromouse` in **VS Code**.
2. PlatformIO will automatically recognize `platformio.ini` and download required libraries (`Adafruit_VL53L0X`, `Wire`).
3. Connect your bot via USB.
4. Click the **PlatformIO Upload** icon (checkmark = build, right-arrow = upload) in the bottom blue status bar:
   - Or press: `Ctrl + Alt + U` (Windows)
5. Open Serial Monitor (`Ctrl + Alt + S` or click the plug icon) at baud rate **115200**.

### Via PlatformIO CLI:
```bash
# Build firmware
pio run

# Upload to ESP32
pio run --target upload

# Open Serial Monitor
pio device monitor -b 115200
```

---

## 🎮 How to Operate the Robot

1. **Power On**: Place the robot in the starting cell $(0, 0)$ facing North. The MPU-6050 gyro will calibrate its stationary bias for 1 second.
2. **Press Button 1 (`Pin 4`)**:
   - Executes **Run 1 (Exploration)**.
   - Discovers unknown walls using the 5 ToF sensors, calculates distances with Floodfill, enters the center goal cells $\{(7,7)..(8,8)\}$, and automatically navigates back to $(0,0)$.
3. **Press Button 2 (`Pin 13`)**:
   - Executes **Run 2 (High-Speed Run)**.
   - Navigates the mapped optimal shortest path at top cruise speed using **Smooth 90° Arc Turns**!

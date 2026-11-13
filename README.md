# 🐭 Autonomous Micromouse Robot

An intelligent, high-speed autonomous **Micromouse Robot** designed to map, explore, and solve official $16 \times 16$ competition mazes using the **Floodfill Algorithm**, wall-centering PID control, and dynamic path optimization.

---

## 📂 Project Organization

```
micromouse/
├── README.md                          # Comprehensive project overview and architecture
├── .gitignore                         # Build and environment exclusions
├── sync_github.py                     # GitHub REST API direct synchronization script
├── code/                              # Executable software and simulation
│   ├── firmware/
│   │   └── micromouse_robot.ino       # C++ firmware with Queue-based Floodfill & PID
│   └── simulation/
│       └── micromouse_simulator.py    # Python 16x16 maze simulator & Floodfill visualizer
├── ideas/                             # Mathematical modeling, ideas, and hardware guides
│   ├── ALGORITHMS.md                  # Kinematics, odometry, Floodfill math, & velocity profiles
│   ├── FLOODFILL_GUIDE.md             # Complete step-by-step Floodfill & wave propagation guide
│   ├── HARDWARE_CIRCUIT.md            # Schematics, pinouts, TB6612 driver, & IR sensor circuits
│   └── PROJECT_IDEAS.md               # Master engineering notebook, multi-run strategy & roadmap
└── others/                            # Calibration and competition specifications
    ├── CALIBRATION_GUIDE.md           # IR sensor curve fitting, encoder CPR, and PID tuning
    └── MAZE_SPECS_RULES.md            # Official IEEE & All-Japan contest rules & maze geometry
```

---

## 🌟 Key Features

- **Queue-Based Floodfill Algorithm**: Dynamic real-time distance field recalculation as new walls are perceived.
- **Multi-Run Competition Strategy**:
  - **Run 1 (Search Run)**: Autonomous exploration from $(0, 0)$ to the 4 center destination cells $\{(7, 7), (7, 8), (8, 7), (8, 8)\}$.
  - **Run 2 (Return-to-Start Exploration)**: Reverse floodfill back to $(0, 0)$ to map alternative corridors and discover shorter paths.
  - **Run 3+ (Speed Run)**: High-acceleration execution of the global shortest path with zero dead ends.
- **Pulsed IR Synchronous Demodulation**: Emitters pulsed at high current with ambient-light subtraction to eliminate fluorescent flicker.
- **Wall-Centering PID Control**: Closed-loop steering that keeps the mouse centered between $168\text{ mm}$ corridor walls.
- **16×16 Python Simulator**: High-fidelity benchmark simulator verifying algorithm convergence before hardware deployment.

---

## 📐 Floodfill Mathematical Foundation

Floodfill represents the maze as a potential field $V(x, y)$ where the target cells have potential $0$:

$$V(u) = 1 + \min_{v \in \mathcal{N}_{\text{open}}(u)} V(v)$$

Whenever a new wall blocks an expected path, the cell becomes inconsistent and pushes updates across neighboring cells using a FIFO queue.

### Classical Manhattan Initial Field

$$V_0(x, y) = \min_{gx \in \{7, 8\}, gy \in \{7, 8\}} (|x - gx| + |y - gy|)$$

---

## 🚀 Quick Start Guide

### 1. Run the Python Simulator
Simulate search, return-to-start, and speed runs directly in Python:
```bash
python code/simulation/micromouse_simulator.py
```

### 2. Deploy Physical Robot Firmware
1. Open [code/firmware/micromouse_robot.ino](file:///d:/micromouse/code/firmware/micromouse_robot.ino) in Arduino IDE or PlatformIO.
2. Select your target board (**ESP32**, **STM32 BlackPill**, or **Teensy 4.0**).
3. Follow the pin connections in [ideas/HARDWARE_CIRCUIT.md](file:///d:/micromouse/ideas/HARDWARE_CIRCUIT.md).
4. Perform distance sensor and odometry calibration as described in [others/CALIBRATION_GUIDE.md](file:///d:/micromouse/others/CALIBRATION_GUIDE.md).

### 3. Sync to GitHub
To push or update this project directly on GitHub (`padmanaban1908/micromouse`):
```bash
python sync_github.py
```

---

## 📚 Technical Documentation Directory

- [ideas/ALGORITHMS.md](file:///d:/micromouse/ideas/ALGORITHMS.md) - Odometry kinematics, velocity profiles, and PID mathematics.
- [ideas/FLOODFILL_GUIDE.md](file:///d:/micromouse/ideas/FLOODFILL_GUIDE.md) - Detailed guide to wave propagation and multi-goal handling.
- [ideas/HARDWARE_CIRCUIT.md](file:///d:/micromouse/ideas/HARDWARE_CIRCUIT.md) - Circuit diagrams, motor driver wiring, and BOM.
- [ideas/PROJECT_IDEAS.md](file:///d:/micromouse/ideas/PROJECT_IDEAS.md) - Master engineering ideas and future roadmap.
- [others/CALIBRATION_GUIDE.md](file:///d:/micromouse/others/CALIBRATION_GUIDE.md) - Calibration procedures for IR sensors, encoders, and IMU.
- [others/MAZE_SPECS_RULES.md](file:///d:/micromouse/others/MAZE_SPECS_RULES.md) - IEEE contest maze geometry and rules.
